#!/usr/bin/env python3
"""
SLAM-based Path Follower
- V1 pure_pursuit: dense waypoint Pure Pursuit
- V2 p2p_line + drive_dir:=forward : 출발→도착 전진
- V2 p2p_line + drive_dir:=reverse : 도착→출발 후진 복귀
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
from datetime import datetime


class PathFollowerSLAM(Node):
    def __init__(self):
        super().__init__('path_follower_slam')
        
        # 파라미터
        self.declare_parameter('path_file', '~/roboro_nav/paths/latest_path.csv')
        self.declare_parameter('odom_topic', '/liorf/mapping/odometry')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        # pure_pursuit | p2p_line
        self.declare_parameter('follow_mode', 'pure_pursuit')
        # forward: path[-1]로 전진 / reverse: path[0]로 후진 복귀
        self.declare_parameter('drive_dir', 'forward')
        
        # 제어 파라미터 (좁은 공간용 기본값)
        self.declare_parameter('linear_vel', 0.15)        # 순항 선속도 (양수, reverse면 자동 음수)
        self.declare_parameter('max_angular_vel', 0.4)    # 최대 각속도
        self.declare_parameter('lookahead_dist', 0.3)     # Lookahead 거리 (V1)
        self.declare_parameter('goal_tolerance', 0.15)    # 목표 도달 허용치
        self.declare_parameter('angular_kp', 1.5)         # 각도 P 게인
        self.declare_parameter('slow_dist', 0.5)          # 감속 시작 거리
        self.declare_parameter('min_vel', 0.05)           # 최소 속도 (양수)
        
        # V2 P2P 전용
        self.declare_parameter('crawl_dist', 0.35)
        self.declare_parameter('crawl_vel', 0.08)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('save_actual_path', True)
        
        # 파라미터 로드
        path_file = os.path.expanduser(self.get_parameter('path_file').value)
        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        self.follow_mode = self.get_parameter('follow_mode').value
        self.drive_dir = self.get_parameter('drive_dir').value
        self.is_reverse = (self.drive_dir == 'reverse')
        self.vel_sign = -1.0 if self.is_reverse else 1.0
        self.save_actual_path = self.get_parameter('save_actual_path').value
        
        # 속도는 절대값으로 보관, 부호는 vel_sign으로 적용
        self.linear_vel = abs(self.get_parameter('linear_vel').value)
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.slow_dist = self.get_parameter('slow_dist').value
        self.min_vel = abs(self.get_parameter('min_vel').value)
        
        self.crawl_dist = self.get_parameter('crawl_dist').value
        self.crawl_vel = abs(self.get_parameter('crawl_vel').value)
        self.control_rate = max(self.get_parameter('control_rate').value, 10.0)
        self.dt = 1.0 / self.control_rate
        
        # 상태
        self.path = []
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.ref_x = None
        self.ref_y = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.closest_idx = 0
        self.is_finished = False
        self.stop_pulses_left = 0
        self.stop_pulse_count = max(10, int(round(self.control_rate)))
        self.actual_path = []
        self.min_dist_to_goal = float('inf')
        self.locked_yaw = None  # 근접 시 bearing 튐 방지용 고정 헤딩
        
        # 경로 로드
        self._load_path(path_file)
        
        # 퍼블리셔
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_path = self.create_publisher(Path, '/teach_path', 10)
        self.pub_status = self.create_publisher(Bool, '/path_following_active', 10)
        
        # 구독
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        
        # 제어 루프 (기본 50Hz — 1m/s cm급용)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.path_pub_timer = self.create_timer(1.0, self.publish_path_viz)
        
        dir_label = '후진 복귀' if self.is_reverse else '전진'
        self.get_logger().info(f'🚀 SLAM 기반 경로 추종기 시작')
        self.get_logger().info(f'   모드: {self.follow_mode} / {self.drive_dir} ({dir_label})')
        self.get_logger().info(f'   경로: {len(self.path)} points')
        signed_v = self.vel_sign * self.linear_vel
        self.get_logger().info(f'   속도: {signed_v:+.2f}m/s @ {self.control_rate:.0f}Hz')
        self.get_logger().info(f'   도착 허용: {self.goal_tolerance}m')
        if self.follow_mode == 'p2p_line':
            self.get_logger().info(
                f'   P2P: 목표 bearing 추종, slow={self.slow_dist}m '
                f'crawl={self.crawl_dist}m')
        else:
            self.get_logger().info(f'   Lookahead: {self.lookahead_dist}m')
    
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
            if self.is_reverse:
                # 복귀: teach 출발점(path[0])이 목표, 도착점(path[-1])이 기준
                self.goal_x = self.path[0][0]
                self.goal_y = self.path[0][1]
                self.goal_yaw = self.path[0][2]
                self.ref_x = self.path[-1][0]
                self.ref_y = self.path[-1][1]
                self.get_logger().info(
                    f'📍 복귀 목표(출발지): ({self.goal_x:.3f}, {self.goal_y:.3f}) '
                    f'yaw={math.degrees(self.goal_yaw):.1f}°')
                self.get_logger().info(
                    f'📍 현재 기준(도착지): ({self.ref_x:.3f}, {self.ref_y:.3f})')
            else:
                self.goal_x = self.path[-1][0]
                self.goal_y = self.path[-1][1]
                self.goal_yaw = self.path[-1][2]
                self.ref_x = self.path[0][0]
                self.ref_y = self.path[0][1]
                self.get_logger().info(
                    f'📍 시작: ({self.ref_x:.3f}, {self.ref_y:.3f})')
                self.get_logger().info(
                    f'📍 목표: ({self.goal_x:.3f}, {self.goal_y:.3f}) '
                    f'yaw={math.degrees(self.goal_yaw):.1f}°')
            if self.follow_mode == 'p2p_line' and len(self.path) > 2:
                self.get_logger().info(
                    f'   (dense 경로 {len(self.path)}점 → 시작/끝만 사용)')
    def odom_callback(self, msg):
        """SLAM 오도메트리 콜백"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.current_x = pos.x
        self.current_y = pos.y
        
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_yaw = yaw
        
        # 실제 경로 기록 (저장 켤 때만)
        if self.save_actual_path:
            self.actual_path.append((self.current_x, self.current_y, time.time()))
    
    def control_loop(self):
        """메인 제어 루프"""
        if not self.path or self.current_x is None:
            return
        
        # 도착 후 zero cmd_vel을 여러 틱 반복 전송
        if self.is_finished:
            if self.stop_pulses_left > 0:
                self._publish_zero_cmd()
                self.stop_pulses_left -= 1
            return
        
        # 상태 퍼블리시
        status_msg = Bool()
        status_msg.data = True
        self.pub_status.publish(status_msg)
        
        if self.follow_mode == 'p2p_line':
            self._control_p2p_line()
        else:
            self._control_pure_pursuit()
    
    def _finish(self, reason=None):
        """도착 처리 공통"""
        if reason is None:
            reason = '출발지 복귀' if self.is_reverse else '목표 도달'
        dist = math.hypot(self.current_x - self.goal_x, self.current_y - self.goal_y)
        self.is_finished = True
        self.stop_pulses_left = self.stop_pulse_count
        self._stop()
        self.get_logger().info(
            f'🎯 {reason}! 잔여={dist*100:.1f}cm  '
            f'zero cmd_vel × {self.stop_pulse_count}회')
        if self.save_actual_path:
            self._save_actual_path()
    
    # ─── V2: Point-to-Point ───────────────────────────────
    def _control_p2p_line(self):
        """목표 bearing 추종. 근접에서는 yaw 고정(atan2 튐 → 반대방향 정렬 방지)."""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)
        self.min_dist_to_goal = min(self.min_dist_to_goal, dist)

        if dist < self.goal_tolerance:
            self._finish()
            return
        if (self.min_dist_to_goal < self.goal_tolerance * 2.0 and
                dist > self.min_dist_to_goal + 0.05):
            self._finish('오버슈트 감지 — 정지')
            return

        bearing = math.atan2(dy, dx)
        target_yaw = self._wrap_angle(bearing + math.pi) if self.is_reverse else bearing

        # 멀 때: 목표 방향 갱신. 크롤 구간: 마지막 정상 헤딩 고정
        # (도착 직전 atan2가 180° 뒤집히며 반대로 도는 것 방지)
        if dist > self.crawl_dist:
            self.locked_yaw = target_yaw
        elif self.locked_yaw is not None:
            target_yaw = self.locked_yaw

        err = self._wrap_angle(target_yaw - self.current_yaw)

        if dist <= self.crawl_dist:
            speed = self.crawl_vel
        elif dist < self.slow_dist:
            span = max(self.slow_dist - self.crawl_dist, 0.05)
            r = max(0.0, min(1.0, (dist - self.crawl_dist) / span))
            speed = self.crawl_vel + (self.linear_vel - self.crawl_vel) * r
        else:
            speed = self.linear_vel

        if abs(err) > 0.3:
            speed *= max(0.35, 1.0 - abs(err) / math.pi)
        speed = max(self.min_vel, speed)

        # 근접: 선속도 거의 0 + 고정 yaw만 맞춤 (반대방향 강제회전 없음)
        if dist <= self.crawl_dist:
            speed = min(speed, self.crawl_vel)
            # 이미 거의 맞으면 ω 끄기
            if abs(err) < 0.05:
                w = 0.0
            else:
                w = max(-self.max_angular_vel,
                        min(self.max_angular_vel, self.angular_kp * err))
        else:
            w = max(-self.max_angular_vel,
                    min(self.max_angular_vel, self.angular_kp * err))

        cmd = Twist()
        cmd.linear.x = self.vel_sign * speed
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        if not hasattr(self, '_log_t') or time.time() - self._log_t > 0.5:
            self.get_logger().info(
                f'[p2p|{"R" if self.is_reverse else "F"}] '
                f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
                f'err={math.degrees(err):.0f}° '
                f'v={cmd.linear.x:+.2f} ω={w:.2f} goal={dist:.2f}m'
            )
            self._log_t = time.time()
    # ─── V1: Pure Pursuit ─────────────────────────────────────
    def _control_pure_pursuit(self):
        """기존 Pure Pursuit 경로 추종"""
        # 1. 가장 가까운 경로점 찾기
        min_dist = float('inf')
        for i in range(self.closest_idx, min(self.closest_idx + 50, len(self.path))):
            px, py, _ = self.path[i]
            d = math.hypot(self.current_x - px, self.current_y - py)
            if d < min_dist:
                min_dist = d
                self.closest_idx = i
        
        # 2. 목표 도달 체크
        dist_to_goal = math.hypot(self.current_x - self.goal_x,
                                  self.current_y - self.goal_y)
        
        if dist_to_goal < self.goal_tolerance:
            self._finish('목표 도달')
            return
        
        # 3. Lookahead 포인트 찾기
        target_idx = self._find_lookahead_point()
        tx, ty, _ = self.path[target_idx]
        
        # 4. Pure Pursuit 제어
        target_heading = math.atan2(ty - self.current_y, tx - self.current_x)
        heading_error = self._wrap_angle(target_heading - self.current_yaw)
        
        angular_vel = self.angular_kp * heading_error
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        vel_factor = 1.0 - min(abs(heading_error) / (math.pi / 3), 0.7)
        
        if dist_to_goal < self.slow_dist:
            vel_factor *= (dist_to_goal / self.slow_dist)
        
        linear_vel = max(self.min_vel, self.linear_vel * vel_factor)
        
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.pub_cmd.publish(cmd)
        
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
    
    def _publish_zero_cmd(self):
        """cmd_vel 전체 0 퍼블리시 (shutdown 중이면 무시)"""
        try:
            if not rclpy.ok():
                return
            cmd = Twist()
            self.pub_cmd.publish(cmd)
        except Exception:
            pass
    
    def _stop(self, bursts=5):
        """로봇 정지 — zero를 즉시 여러 번 + 이후 타이머로 추가 전송"""
        for _ in range(bursts):
            self._publish_zero_cmd()
            time.sleep(0.02)
        
        try:
            if rclpy.ok():
                status_msg = Bool()
                status_msg.data = False
                self.pub_status.publish(status_msg)
        except Exception:
            pass
        
        # Ctrl+C 등에서도 타이머 반복이 돌도록 예약
        if self.stop_pulses_left <= 0:
            self.stop_pulses_left = self.stop_pulse_count
    
    def publish_path_viz(self):
        """RViz용 경로 시각화"""
        if not self.path:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'  # 또는 'map'
        
        if self.follow_mode == 'p2p_line' and len(self.path) >= 2:
            # 직선 세그먼트만 표시
            viz_pts = [self.path[0], self.path[-1]]
        else:
            viz_pts = self.path
        
        for x, y, yaw in viz_pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.pub_path.publish(path_msg)
    
    def _save_actual_path(self):
        """실제 주행 경로 저장 — results/ 에 실행마다 새 파일"""
        if not self.actual_path:
            return
        
        save_dir = os.path.expanduser('~/roboro_nav/results')
        os.makedirs(save_dir, exist_ok=True)
        
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_file = os.path.join(save_dir, f'actual_path_{stamp}.csv')
        with open(save_file, 'w') as f:
            f.write('# x,y,timestamp\n')
            for x, y, t in self.actual_path:
                f.write(f'{x:.6f},{y:.6f},{t:.6f}\n')
        
        latest = os.path.join(save_dir, 'latest_actual_path.csv')
        if os.path.lexists(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(save_file), latest)
        
        self.get_logger().info(
            f'📊 실제 경로 저장: {save_file} ({len(self.actual_path)} points)')
        
        # 오차 계산
        if self.follow_mode == 'p2p_line':
            self._calculate_p2p_error()
        else:
            self._calculate_error()
    
    def _calculate_p2p_error(self):
        """V2: 직선 이탈 + 최종 도착 오차"""
        if len(self.actual_path) < 2 or self.goal_x is None or self.ref_x is None:
            return
        
        sx, sy = self.ref_x, self.ref_y
        gx, gy = self.goal_x, self.goal_y
        line_len = math.hypot(gx - sx, gy - sy)
        if line_len < 1e-6:
            return
        
        # 직선 cross-track: |(P-S) × (G-S)| / |G-S|
        cte_list = []
        for ax, ay, _ in self.actual_path[::5]:
            cross = abs((ax - sx) * (gy - sy) - (ay - sy) * (gx - sx))
            cte_list.append(cross / line_len)
        
        final_x, final_y, _ = self.actual_path[-1]
        arrival_err = math.hypot(final_x - gx, final_y - gy)
        
        if cte_list:
            avg_cte = sum(cte_list) / len(cte_list)
            max_cte = max(cte_list)
            label = '복귀오차' if self.is_reverse else '도착오차'
            self.get_logger().info(
                f'📊 직선 CTE avg={avg_cte:.3f}m max={max_cte:.3f}m | '
                f'{label}={arrival_err*100:.1f}cm'
            )
    
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
        try:
            node.is_finished = True
            if rclpy.ok():
                node._stop(bursts=10)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
