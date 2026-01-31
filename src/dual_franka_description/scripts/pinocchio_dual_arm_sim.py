import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
import os

class DualArmSim(Node):
    def __init__(self):
        super().__init__('dual_arm_sim')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 1. 모델 로드
        urdf_path = os.path.expanduser("~/ros2_ws/dual_franka.urdf")
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # 2. 상태 초기화
        self.q = np.full((self.model.nq), 0.1) # 초기 포즈
        self.v = np.zeros(self.model.nv)      # 초기 속도
        self.dt = 0.001  # 1000Hz 시뮬레이션
        
        # 3. 제어 및 어드미턴스 파라미터
        # self.Kp = np.diag([400, 400, 400, 50, 50, 50])
        # self.Kd = np.diag([40, 40, 40, 5, 5, 5])
        self.Kp = np.diag([100, 100, 100, 20, 20, 20])
        self.Kd = np.diag([10, 10, 10, 2, 2, 2])
        self.M_v = np.diag([5.0, 5.0, 5.0, 1.0, 1.0, 1.0])
        self.D_v = np.diag([50.0, 50.0, 50.0, 5.0, 5.0, 5.0])
        
        # 목표 및 레퍼런스 초기화
        pin.framesForwardKinematics(self.model, self.data, self.q)
        self.left_ee_id = self.model.getFrameId("left_fr3_link8")
        self.right_ee_id = self.model.getFrameId("right_fr3_link8")
        
        self.x_left_des = self.data.oMf[self.left_ee_id].translation + np.array([0, 0, 0.1])
        self.x_right_ref = self.data.oMf[self.right_ee_id].translation.copy()
        self.v_right_ref = np.zeros(6)

        # 4. 타이머 설정 (10ms 마다 루프 실행)
        self.timer = self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        # 1. 예외 처리로 감싸서 루프가 죽지 않게 함
        try:
            # (A) 동역학 업데이트
            pin.computeAllTerms(self.model, self.data, self.q, self.v)
            pin.framesForwardKinematics(self.model, self.data, self.q)

            # (B) 제어 토크 계산 (앞서 검증한 로직)
            # Left Arm (Impedance)
            x_L = self.data.oMf[self.left_ee_id].translation
            J_L = pin.getFrameJacobian(self.model, self.data, self.left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            F_L = self.Kp @ np.concatenate([self.x_left_des - x_L, np.zeros(3)]) - self.Kd @ (J_L @ self.v)
            tau_L = self.data.g[:7] + (J_L[:6, :7].T @ F_L)

            # Right Arm (Admittance)
            F_ext = np.array([0, 0, 10.0, 0, 0, 0]) # 가상 외력
            a_admit = np.linalg.inv(self.M_v) @ (F_ext - self.D_v @ self.v_right_ref)
            self.v_right_ref += a_admit * self.dt
            self.x_right_ref += self.v_right_ref[:3] * self.dt

            x_R = self.data.oMf[self.right_ee_id].translation
            J_R = pin.getFrameJacobian(self.model, self.data, self.right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            F_R = self.Kp @ np.concatenate([self.x_right_ref - x_R, np.zeros(3)]) + self.Kd @ (self.v_right_ref - J_R @ self.v)
            tau_R = self.data.g[7:] + (J_R[:6, 7:].T @ F_R)

            tau = np.concatenate([tau_L, tau_R])

            # (C) 정동역학 (Forward Dynamics): tau -> ddq
            # # M(q)ddq + nle = tau  => ddq = M^-1 * (tau - nle)
            # ddq = np.linalg.solve(self.data.M, tau - self.data.nle)
            # M 행렬의 역행렬을 구할 때 수치적으로 불안정할 수 있으므로 미소값을 더함
            M_stable = self.data.M + np.eye(self.model.nv) * 1e-4
            ddq = np.linalg.solve(M_stable, tau - self.data.nle)
            ddq = np.clip(ddq, -100.0, 100.0)

            # (D) 수치 적분 (Semi-implicit Euler)
            self.v += ddq * self.dt
            self.v = np.clip(self.v, -5.0, 5.0) # 속도 제한
            self.q = pin.integrate(self.model, self.q, self.v * self.dt)

            # 5. NaN 체크: 만약 하나라도 NaN이면 업데이트 중단
            if np.any(np.isnan(self.q)):
                self.get_logger().error("Critical: NaN detected! Resetting to initial pose.")
                self.q = np.full((self.model.nq), 0.1)
                self.v = np.zeros(self.model.nv)
                return

            # (E) ROS 2 메시지 발행
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # 조인트 이름 매칭 (모델 로드 시 정해진 순서대로)
            msg.name = [n for n in self.model.names if "joint" in n]
            msg.position = self.q.tolist()
            self.publisher_.publish(msg)
        except Exception as e :
            self.get_logger().error(f"Loop Error: {e}")

def main():
    rclpy.init()
    sim = DualArmSim()
    rclpy.spin(sim)
    sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()