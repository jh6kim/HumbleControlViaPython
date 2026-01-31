import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
import os

class PinocchioRealtimeNode(Node):
    def __init__(self):
        super().__init__('pinocchio_realtime_node')
        
        # 1. URDF 파일 로드
        urdf_path = os.path.expanduser('~/ros2_ws/dual_franka.urdf')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Pinocchio 모델의 관절 이름 리스트 (순서 확인용)
        # 0번은 보통 "universe"이므로 제외하고 실제 관절 이름만 추출
        self.pin_joint_names = list(self.model.names)[1:] 
        self.get_logger().info(f"Model loaded: {self.model.name}")

        # 2. JointState 구독
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        # 3. 중요: ROS 토픽의 관절 순서를 Pinocchio 모델 순서로 재정렬
        q_dict = dict(zip(msg.name, msg.position))
        try:
            # Pinocchio 모델 순서대로 position 값을 배열로 생성
            q = np.array([q_dict[name] for name in self.pin_joint_names])
        except KeyError as e:
            # 토픽에 아직 모든 관절 정보가 오지 않았을 경우 무시
            return

        # 4. 동역학 계산 (에러 수정 부분)
        # M (Mass Matrix)
        pin.crba(self.model, self.data, q)
        M = self.data.M # crba는 self.data.M에 결과를 저장합니다.

        # G (Gravity Vector) - 에러 수정된 메서드 이름 사용
        tau_gravity = pin.computeGeneralizedGravity(self.model, self.data, q)
        
        # C (Coriolis Matrix)
        v = np.zeros(self.model.nv)
        pin.computeCoriolisMatrix(self.model, self.data, q, v)
        C = self.data.C

        # 5. 출력
        os.system('clear')
        print(f"--- Real-time Dynamics (Dual Franka) ---")
        print(f"Joint Order (Pinocchio): {self.pin_joint_names[:3]} ...")
        print(f"Current Q: {q.round(3)}")
        print(f"\n[Gravity Vector (Nm)]\n{tau_gravity.round(3)}")
        np.set_printoptions(precision=3, suppress=True, linewidth=150) # 출력 포맷 설정
        print(f"\n[Mass Matrix (M) - 14x14 part]\n{M[:14, :14].round(3)}")

def main(args=None):
    rclpy.init(args=args)
    node = PinocchioRealtimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()