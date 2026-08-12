#!/usr/bin/env python3
"""
SLAM-based Path Recorder (Teach & Repeat)
- dense: 경로 전체를 샘플링 (V1 Pure Pursuit용)
- endpoints: 출발점 + 도착점만 저장 (V2 P2P 직선용)
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
        self.declare_parameter('save_path', '~/roboro_nav/paths')
        self.declare_parameter('sample_dist', 0.05)  # 5cm마다 기록 (좁은 공간용)
        self.declare_parameter('sample_angle', 0.05)  # ~3도마다 기록
        self.declare_parameter('odom_topic', '/liorf/mapping/odometry')
        self.declare_parameter('vel_threshold', 0.02)  # 움직임 감지 threshold
        # dense | endpoints
        self.declare_parameter('record_mode', 'dense')
        
        self.save_path = os.path.expanduser(self.get_parameter('save_path').value)
        self.sample_dist = self.get_parameter('sample_dist').value
        self.sample_angle = self.get_parameter('sample_angle').value
        odom_topic = self.get_parameter('odom_topic').value
        self.vel_threshold = self.get_parameter('vel_threshold').value
        self.record_mode = self.get_parameter('record_mode').value
        
        # 상태
        self.path = []  # [(x, y, yaw, timestamp), ...]
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.recording = True  # 항상 기록 모드
        self.start_point = None  # endpoints 모드용
        self.end_point = None    # endpoints 모드용
        
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
        self.get_logger().info(f'   모드: {self.record_mode}')
        self.get_logger().info(f'   오도메트리: {odom_topic}')
        self.get_logger().info(f'   저장 경로: {self.save_path}')
        if self.record_mode == 'endpoints':
            self.get_logger().info('🔴 endpoints: 출발지에서 시작 → 목적지 이동 → Ctrl+C 저장')
            self.get_logger().info('   (시작점 + Ctrl+C 시점 위치만 저장)')
        else:
            self.get_logger().info(f'   샘플링 거리: {self.sample_dist}m')
            self.get_logger().info('🔴 기록 중... 로봇을 움직이세요! Ctrl+C로 저장')
    
    def odom_callback(self, msg):
        """SLAM 오도메트리 콜백 - 속도로 움직임 감지"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.current_x = pos.x
        self.current_y = pos.y
        
        # 쿼터니언 → yaw
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_yaw = yaw
        
        if self.recording:
            if self.record_mode == 'endpoints':
                self._update_endpoints()
            else:
                self._try_record_point()
    
    def _update_endpoints(self):
        """출발점 고정 + 현재 위치를 도착 후보로 갱신"""
        if self.current_x is None:
            return
        
        x, y, yaw = self.current_x, self.current_y, self.current_yaw
        timestamp = self.get_clock().now().nanoseconds / 1e9
        point = (x, y, yaw, timestamp)
        
        if self.start_point is None:
            self.start_point = point
            self.end_point = point
            self.get_logger().info(
                f'📍 출발점 고정: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}°)')
            return
        
        # 최소 이동 후에만 end 갱신 (노이즈로 출발=도착 되는 것 방지)
        dist = math.hypot(x - self.start_point[0], y - self.start_point[1])
        if dist < 0.05:
            return
        
        prev_end = self.end_point
        self.end_point = point
        
        # 1m마다 진행 로그
        if prev_end is None or abs(dist - math.hypot(
                prev_end[0] - self.start_point[0],
                prev_end[1] - self.start_point[1])) >= 1.0:
            self.get_logger().info(
                f'📍 도착 후보: ({x:.3f}, {y:.3f})  출발로부터 {dist:.2f}m')
    
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
        if self.record_mode == 'endpoints':
            if self.start_point is None or self.end_point is None:
                self.get_logger().warn('⚠️ 저장할 출발/도착점이 없습니다!')
                return
            sx, sy = self.start_point[0], self.start_point[1]
            ex, ey = self.end_point[0], self.end_point[1]
            if math.hypot(ex - sx, ey - sy) < 0.05:
                self.get_logger().warn('⚠️ 출발점과 도착점이 너무 가깝습니다 (<5cm)!')
                return
            self.path = [self.start_point, self.end_point]
        
        if not self.path:
            self.get_logger().warn('⚠️ 저장할 경로가 없습니다!')
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = 'p2p' if self.record_mode == 'endpoints' else 'path'
        filename = f'{prefix}_{timestamp}.csv'
        filepath = os.path.join(self.save_path, filename)
        
        with open(filepath, 'w') as f:
            f.write('# x,y,yaw,timestamp\n')
            for x, y, yaw, t in self.path:
                f.write(f'{x:.6f},{y:.6f},{yaw:.6f},{t:.6f}\n')
        
        # 최신 경로로 심볼릭 링크 (절대경로)
        latest = os.path.join(self.save_path, 'latest_path.csv')
        if os.path.lexists(latest):
            os.remove(latest)
        os.symlink(os.path.abspath(filepath), latest)
        
        self.get_logger().info(f'💾 {len(self.path)} points → {filepath}')
        self.get_logger().info(f'   시작: ({self.path[0][0]:.3f}, {self.path[0][1]:.3f})')
        self.get_logger().info(f'   종료: ({self.path[-1][0]:.3f}, {self.path[-1][1]:.3f})')
        
        total_dist = sum(
            math.hypot(self.path[i+1][0] - self.path[i][0], 
                      self.path[i+1][1] - self.path[i][1])
            for i in range(len(self.path) - 1)
        )
        self.get_logger().info(f'   총 거리: {total_dist:.2f}m')
        if self.record_mode == 'endpoints':
            self.get_logger().info('   (V2 P2P: 출발/도착 2점만 저장됨)')


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
