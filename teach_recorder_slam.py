#!/usr/bin/env python3
"""
SLAM-based Path Recorder (Teach & Repeat)
- SLAM 오도메트리로 경로 기록
- 리모컨 조종 시 위치 저장
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import os
from datetime import datetime


class TeachRecorderSLAM(Node):
    def __init__(self):
        super().__init__('teach_recorder_slam')
        
        # 파라미터
        self.declare_parameter('save_path', '/root/nav2_v2/roboro_nav/paths')
        self.declare_parameter('sample_dist', 0.05)  # 5cm마다 기록 (좁은 공간용)
        self.declare_parameter('sample_angle', 0.05)  # ~3도마다 기록
        self.declare_parameter('odom_topic', '/liorf/mapping/odometry')
        self.declare_parameter('vel_threshold', 0.02)  # 움직임 감지 threshold
        
        self.save_path = self.get_parameter('save_path').value
        self.sample_dist = self.get_parameter('sample_dist').value
        self.sample_angle = self.get_parameter('sample_angle').value
        odom_topic = self.get_parameter('odom_topic').value
        self.vel_threshold = self.get_parameter('vel_threshold').value
        
        # 상태
        self.path = []  # [(x, y, yaw, timestamp), ...]
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.recording = True  # 항상 기록 모드
        
        # 현재 위치
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        
        # 디렉토리 생성
        os.makedirs(self.save_path, exist_ok=True)
        
        # 구독 (오도메트리만 - 속도도 포함되어 있음)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        
        self.get_logger().info(f'📍 SLAM 기반 경로 기록기 시작')
        self.get_logger().info(f'   오도메트리: {odom_topic}')
        self.get_logger().info(f'   샘플링 거리: {self.sample_dist}m (좁은 공간용)')
        self.get_logger().info(f'   저장 경로: {self.save_path}')
        self.get_logger().info('🔴 기록 중... 로봇을 움직이세요! Ctrl+C로 저장')
    
    def odom_callback(self, msg):
        """SLAM 오도메트리 콜백 - 속도로 움직임 감지"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist  # 속도 정보
        
        self.current_x = pos.x
        self.current_y = pos.y
        
        # 쿼터니언 → yaw
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_yaw = yaw
        
        # 오도메트리 속도로 움직임 감지
        linear_vel = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
        angular_vel = abs(twist.angular.z)
        is_moving = linear_vel > self.vel_threshold or angular_vel > self.vel_threshold
        
        # 항상 기록 모드 (움직임 관계없이 위치 변화 감지)
        if self.recording:
            self._try_record_point()
    
    def _try_record_point(self):
        """거리/각도 조건 만족 시 포인트 기록"""
        if self.current_x is None:
            return
        
        x, y, yaw = self.current_x, self.current_y, self.current_yaw
        
        # 첫 포인트
        if self.last_x is None:
            self._record_point(x, y, yaw)
            return
        
        # 거리 체크
        dist = math.hypot(x - self.last_x, y - self.last_y)
        angle_diff = abs(self._wrap_angle(yaw - self.last_yaw))
        
        if dist >= self.sample_dist or angle_diff >= self.sample_angle:
            self._record_point(x, y, yaw)
    
    def _record_point(self, x, y, yaw):
        """포인트 기록"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.path.append((x, y, yaw, timestamp))
        self.last_x, self.last_y, self.last_yaw = x, y, yaw
        
        if len(self.path) % 10 == 0:
            self.get_logger().info(f'📍 {len(self.path)} points recorded')
    
    def _wrap_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def save_recorded_path(self):
        """경로 저장"""
        if not self.path:
            self.get_logger().warn('⚠️ 저장할 경로가 없습니다!')
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'path_{timestamp}.csv'
        filepath = os.path.join(self.save_path, filename)
        
        with open(filepath, 'w') as f:
            f.write('# x,y,yaw,timestamp\n')
            for x, y, yaw, t in self.path:
                f.write(f'{x:.6f},{y:.6f},{yaw:.6f},{t:.6f}\n')
        
        # 최신 경로로 심볼릭 링크
        latest = os.path.join(self.save_path, 'latest_path.csv')
        if os.path.exists(latest):
            os.remove(latest)
        os.symlink(filepath, latest)
        
        self.get_logger().info(f'💾 {len(self.path)} points → {filepath}')
        self.get_logger().info(f'   시작: ({self.path[0][0]:.2f}, {self.path[0][1]:.2f})')
        self.get_logger().info(f'   종료: ({self.path[-1][0]:.2f}, {self.path[-1][1]:.2f})')
        
        total_dist = sum(
            math.hypot(self.path[i+1][0] - self.path[i][0], 
                      self.path[i+1][1] - self.path[i][1])
            for i in range(len(self.path) - 1)
        )
        self.get_logger().info(f'   총 거리: {total_dist:.2f}m')


def main():
    rclpy.init()
    node = TeachRecorderSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️ 중단됨')
    finally:
        node.save_recorded_path()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
