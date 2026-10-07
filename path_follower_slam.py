#!/usr/bin/env python3
"""
SLAM-based Pure Pursuit Path Follower
- SLAM 오도메트리로 위치 추정
- Pure Pursuit 알고리즘으로 경로 추종
- 고정밀 경로 재현
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
import math
import os
import time


class PathFollowerSLAM(Node):
    def __init__(self):
        super().__init__('path_follower_slam')
        
        # 파라미터
        self.declare_parameter('path_file', '/root/nav2_v2/roboro_nav/paths/latest_path.csv')
        self.declare_parameter('odom_topic', '/liorf/mapping/odometry')
        self.declare_parameter('cmd_topic', '/cmd_vel_main')
        
        # 제어 파라미터 (좁은 공간용 기본값)
        self.declare_parameter('linear_vel', 0.15)        # 기본 선속도 (느리게)
        self.declare_parameter('max_angular_vel', 0.4)    # 최대 각속도
        self.declare_parameter('lookahead_dist', 0.3)     # Lookahead 거리 (짧게)
        self.declare_parameter('goal_tolerance', 0.15)    # 목표 도달 허용치 (정밀)
        self.declare_parameter('angular_kp', 1.5)         # 각도 P 게인 (높게)
        self.declare_parameter('slow_dist', 0.5)          # 감속 시작 거리
        self.declare_parameter('min_vel', 0.05)           # 최소 속도
        
        # 파라미터 로드
        path_file = self.get_parameter('path_file').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        
        self.linear_vel = self.get_parameter('linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.slow_dist = self.get_parameter('slow_dist').value
        self.min_vel = self.get_parameter('min_vel').value
        
        # 상태
        self.path = []
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.closest_idx = 0
        self.is_finished = False
        self.actual_path = []  # 실제 경로 기록
        
        # 경로 로드
        self._load_path(path_file)
        
        # 퍼블리셔
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_path = self.create_publisher(Path, '/teach_path', 10)
        self.pub_status = self.create_publisher(Bool, '/path_following_active', 10)
        
        # 구독
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        
        # 제어 루프 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.path_pub_timer = self.create_timer(1.0, self.publish_path_viz)
        
        self.get_logger().info(f'🚀 SLAM 기반 경로 추종기 시작')
        self.get_logger().info(f'   경로: {len(self.path)} points')
        self.get_logger().info(f'   Lookahead: {self.lookahead_dist}m')
        self.get_logger().info(f'   속도: {self.linear_vel}m/s')
    
    def _load_path(self, path_file):
        """CSV에서 경로 로드"""
        if not os.path.exists(path_file):
            self.get_logger().error(f'❌ 경로 파일 없음: {path_file}')
            return
        
        # 심볼릭 링크면 실제 파일 경로 출력
        if os.path.islink(path_file):
            real_path = os.path.realpath(path_file)
            self.get_logger().info(f'📂 실제 파일: {real_path}')
        
        with open(path_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or not line:
                    continue
                parts = line.split(',')
                if len(parts) >= 3:
                    x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                    self.path.append((x, y, yaw))
        
        if self.path:
            self.get_logger().info(f'📍 시작: ({self.path[0][0]:.2f}, {self.path[0][1]:.2f})')
            self.get_logger().info(f'📍 목표: ({self.path[-1][0]:.2f}, {self.path[-1][1]:.2f})')
    
    def odom_callback(self, msg):
        """SLAM 오도메트리 콜백"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.current_x = pos.x
        self.current_y = pos.y
        
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_yaw = yaw
        
        # 실제 경로 기록
        self.actual_path.append((self.current_x, self.current_y, time.time()))
    
    def control_loop(self):
        """메인 제어 루프"""
        if not self.path or self.current_x is None:
            return
        
        if self.is_finished:
            return
        
        # 상태 퍼블리시
        status_msg = Bool()
        status_msg.data = True
        self.pub_status.publish(status_msg)
        
        # 1. 가장 가까운 경로점 찾기
        min_dist = float('inf')
        for i in range(self.closest_idx, min(self.closest_idx + 50, len(self.path))):
            px, py, _ = self.path[i]
            d = math.hypot(self.current_x - px, self.current_y - py)
            if d < min_dist:
                min_dist = d
                self.closest_idx = i
        
        # 2. 목표 도달 체크
        gx, gy, _ = self.path[-1]
        dist_to_goal = math.hypot(self.current_x - gx, self.current_y - gy)
        
        if dist_to_goal < self.goal_tolerance:
            self._stop()
            self.is_finished = True
            self.get_logger().info('🎯 목표 도달!')
            self._save_actual_path()
            return
        
        # 3. Lookahead 포인트 찾기
        target_idx = self._find_lookahead_point()
        tx, ty, target_yaw = self.path[target_idx]
        
        # 4. Pure Pursuit 제어
        # 목표 방향 계산
        target_heading = math.atan2(ty - self.current_y, tx - self.current_x)
        heading_error = self._wrap_angle(target_heading - self.current_yaw)
        
        # 각속도 계산 (P 제어)
        angular_vel = self.angular_kp * heading_error
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 선속도 계산 (곡선/직선 적응 + 목표 근처 감속)
        # 큰 각도 오차면 속도 줄이기
        vel_factor = 1.0 - min(abs(heading_error) / (math.pi / 3), 0.7)
        
        # 목표 근처 감속
        if dist_to_goal < self.slow_dist:
            vel_factor *= (dist_to_goal / self.slow_dist)
        
        linear_vel = max(self.min_vel, self.linear_vel * vel_factor)
        
        # 5. cmd_vel 퍼블리시
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.pub_cmd.publish(cmd)
        
        # 로그 (1초마다)
        if not hasattr(self, '_log_t') or time.time() - self._log_t > 1.0:
            self.get_logger().info(
                f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
                f'err={math.degrees(heading_error):.0f}° '
                f'v={linear_vel:.2f} ω={angular_vel:.2f} '
                f'goal={dist_to_goal:.1f}m'
            )
            self._log_t = time.time()
    
    def _find_lookahead_point(self):
        """Lookahead 거리에 있는 경로점 찾기"""
        acc_dist = 0.0
        
        for i in range(self.closest_idx, len(self.path) - 1):
            px1, py1, _ = self.path[i]
            px2, py2, _ = self.path[i + 1]
            acc_dist += math.hypot(px2 - px1, py2 - py1)
            
            if acc_dist >= self.lookahead_dist:
                return i + 1
        
        return len(self.path) - 1
    
    def _wrap_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _stop(self):
        """로봇 정지"""
        cmd = Twist()
        self.pub_cmd.publish(cmd)
        
        status_msg = Bool()
        status_msg.data = False
        self.pub_status.publish(status_msg)
    
    def publish_path_viz(self):
        """RViz용 경로 시각화"""
        if not self.path:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'  # 또는 'map'
        
        for x, y, yaw in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.pub_path.publish(path_msg)
    
    def _save_actual_path(self):
        """실제 주행 경로 저장 (분석용)"""
        if not self.actual_path:
            return
        
        save_file = '/root/nav2_v2/roboro_nav/paths/actual_path.csv'
        with open(save_file, 'w') as f:
            f.write('# x,y,timestamp\n')
            for x, y, t in self.actual_path:
                f.write(f'{x:.6f},{y:.6f},{t:.6f}\n')
        
        self.get_logger().info(f'📊 실제 경로 저장: {save_file}')
        
        # Cross-track error 계산
        self._calculate_error()
    
    def _calculate_error(self):
        """경로 추종 오차 계산"""
        if len(self.actual_path) < 2 or len(self.path) < 2:
            return
        
        errors = []
        for ax, ay, _ in self.actual_path[::10]:  # 10개마다 샘플링
            min_dist = float('inf')
            for px, py, _ in self.path:
                d = math.hypot(ax - px, ay - py)
                if d < min_dist:
                    min_dist = d
            errors.append(min_dist)
        
        if errors:
            avg_error = sum(errors) / len(errors)
            max_error = max(errors)
            self.get_logger().info(f'📊 평균 오차: {avg_error:.3f}m, 최대 오차: {max_error:.3f}m')


def main():
    rclpy.init()
    node = PathFollowerSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️ 중단됨')
    finally:
        node._stop()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
