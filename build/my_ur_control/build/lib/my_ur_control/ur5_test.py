import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class Ur5GoalNode(Node):
    def __init__(self):
        super().__init__('ur5_goal_node')
        # Action Client 생성: 목표를 전달할 대상 컨트롤러 설정
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # UR5의 관절 이름 정의 (URDF와 일치해야 함)
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # 목표 지점 설정
        point1 = JointTrajectoryPoint()
        # 각도 설정 (단위: Radian). 예: shoulder_lift를 -1.0으로 살짝 들기
        point1.positions = [0.0, -1.0, 0.0, -0.5, 1.5, 0.0]
        # 해당 목표까지 도달하는 시간 (5초)
        point1.time_from_start = Duration(sec=3, nanosec=0)

        # 목표 지점 설정
        point2 = JointTrajectoryPoint()
        # 각도 설정 (단위: Radian). 예: shoulder_lift를 -1.0으로 살짝 들기
        point2.positions = [0.0, -1.0, 1.5, -0.5, 1.5, 0.0]
        # 해당 목표까지 도달하는 시간 (5초)
        point2.time_from_start = Duration(sec=5, nanosec=0)
        goal_msg.trajectory.points = [point1, point2]


        # 1. 시작점과 끝점 정의 (Radian)
        start_pos = [0.0, -1.0, 1.5, -0.5, 1.5, 0.0]
        end_pos = [1.0, -1.0, 1.0, -0.5, 1.0, 0.0]
        
        # 2. 보간(Interpolation) 설정
        steps = 10  # 경로를 10개로 쪼갬
        total_time = 5.0 # 총 이동 시간 (초)

        for i in range(steps + 1):
            point = JointTrajectoryPoint()
            
            # 선형 보간 계산: q = start + (end - start) * (i / steps)
            ratio = i / steps
            current_q = [
                s + (e - s) * ratio for s, e in zip(start_pos, end_pos)
            ]
            
            point.positions = current_q
            # 시간도 점진적으로 증가 (중요!)
            point.time_from_start = Duration(
                sec=8+int((total_time * ratio)), 
                nanosec=int((total_time * ratio % 1) * 1e9)
            )
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('UR5 로봇에 목표를 전송합니다...')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

    def send_linear_job(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # 1. 시작점과 끝점 정의 (Radian)
        start_pos = [0.0, -1.0, 1.5, -0.5, 1.5, 0.0]
        end_pos = [1.0, -1.0, 1.0, -0.5, 1.0, 0.0]
        
        # 2. 보간(Interpolation) 설정
        steps = 10  # 경로를 10개로 쪼갬
        total_time = 5.0 # 총 이동 시간 (초)

        for i in range(steps + 1):
            point = JointTrajectoryPoint()
            
            # 선형 보간 계산: q = start + (end - start) * (i / steps)
            ratio = i / steps
            current_q = [
                s + (e - s) * ratio for s, e in zip(start_pos, end_pos)
            ]
            
            point.positions = current_q
            # 시간도 점진적으로 증가 (중요!)
            point.time_from_start = Duration(
                sec=int((total_time * ratio)), 
                nanosec=int((total_time * ratio % 1) * 1e9)
            )
            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'총 {steps}개의 웨이포인트로 직선 모션을 시작합니다...')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Ur5GoalNode()
    future = node.send_goal()
    # future = node.send_linear_job()
    rclpy.spin_until_future_complete(node, future)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()