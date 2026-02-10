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
        
        # [추가] 가상 상자 파라미터
        self.obj_mass = 0.5        # 0.5kg 상자
        self.obj_width = 0.4       # 상자 가로 40cm
        self.k_virtual = 1000.0    # 가상 스프링 강성 (로봇과 물체 연결)
        self.d_virtual = 100.0      # 가상 댐핑
        
        # 초기 상자 위치 (두 로봇의 중간 지점)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        self.left_ee_id = self.model.getFrameId("left_fr3_link8")
        self.right_ee_id = self.model.getFrameId("right_fr3_link8")
        
        x_L_init = self.data.oMf[self.left_ee_id].translation
        x_R_init = self.data.oMf[self.right_ee_id].translation
        self.obj_pos = (x_L_init + x_R_init) / 2.0
        self.obj_vel = np.zeros(3)

        # 3. 제어 및 어드미턴스 파라미터
        # self.Kp = np.diag([400, 400, 400, 50, 50, 50])
        # self.Kd = np.diag([40, 40, 40, 5, 5, 5])
        self.Kp = np.diag([80, 80, 80, 5, 5, 5])
        self.Kd = np.diag([60, 60, 60, 3, 3, 3])
        self.M_v = np.diag([2.0, 2.0, 2.0, 1.0, 1.0, 1.0])
        self.D_v = np.diag([20.0, 20.0, 20.0, 2.0, 2.0, 2.0])
        
        # 목표 위치 설정 (왼쪽 팔만 따라가도록)
        self.x_left_des = x_L_init.copy()
        self.x_right_ref = x_R_init.copy()
        self.v_right_ref = np.zeros(6)

        # # 목표 및 레퍼런스 초기화
        # pin.framesForwardKinematics(self.model, self.data, self.q)
        # self.left_ee_id = self.model.getFrameId("left_fr3_link8")
        # self.right_ee_id = self.model.getFrameId("right_fr3_link8")

        # self.x_left_des = self.data.oMf[self.left_ee_id].translation + np.array([0, 0, 0.1])
        # self.x_right_ref = self.data.oMf[self.right_ee_id].translation.copy()
        # self.v_right_ref = np.zeros(6)

        # # 물체의 목표 위치(두 로봇 사이 중앙 위지점)
        # object_target_pos = np.array([0.5, 0.0, 0.5])
        # self.x_left_des = object_target_pos + np.array([0.0, 0.2, 0.0])
        # self.x_right_ref = object_target_pos + np.array([0.0, -0.2, 0.0])
        # self.v_right_ref = np.zeros(6)

        # 4. 타이머 설정 (10ms 마다 루프 실행)
        self.timer = self.create_timer(self.dt, self.control_loop)

    def control_loop(self):
        try:
            self.x_left_des[2] += 0.00005 
            # (A) 동역학 업데이트
            pin.computeAllTerms(self.model, self.data, self.q, self.v)
            pin.framesForwardKinematics(self.model, self.data, self.q)

            x_L = self.data.oMf[self.left_ee_id].translation
            x_R = self.data.oMf[self.right_ee_id].translation
            v_L = pin.getFrameVelocity(self.model, self.data, self.left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            v_R = pin.getFrameVelocity(self.model, self.data, self.right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
            
            # --- (1) 가상 상자 물리 연산 (Interaction Physics) ---
            # 상자의 왼쪽/오른쪽 끝단 위치 계산 (Y축 방향으로 상자가 놓여있다고 가정)
            p_obj_L = self.obj_pos + np.array([0.0, self.obj_width/2, 0.0])
            p_obj_R = self.obj_pos - np.array([0.0, self.obj_width/2, 0.0])

            # 로봇이 상자에 가하는 힘 (Spring-Damper)
            f_L_to_obj = self.k_virtual * (x_L - p_obj_L) + self.d_virtual * (v_L - self.obj_vel)
            f_R_to_obj = self.k_virtual * (x_R - p_obj_R) + self.d_virtual * (v_R - self.obj_vel)

            # 상자의 가속도 (F = ma) - 중력 보상 포함
            obj_acc = (f_L_to_obj + f_R_to_obj) / self.obj_mass + np.array([0, 0, -9.81])
            self.obj_vel += obj_acc * self.dt
            self.obj_pos += self.obj_vel * self.dt

            # --- (2) 협업 제어 로직 ---
            # Left Arm: 임피던스 제어로 목표 궤적 추종 (Leader)
            J_L_full = pin.getFrameJacobian(self.model, self.data, self.left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_L = J_L_full[:6, :7]
            F_L_cmd = self.Kp @ np.concatenate([self.x_left_des - x_L, np.zeros(3)]) - self.Kd @ (J_L @ self.v[:7])
            tau_L = self.data.g[:7] + J_L.T @ np.clip(F_L_cmd, -60, 60)

            # Right Arm: 상자가 밀어내는 힘을 외력으로 인지 (Follower)
            # 상자가 로봇 R을 미는 힘은 로봇 R이 상자를 미는 힘의 반대
            F_ext_R = -np.concatenate([f_R_to_obj, np.zeros(3)])
            
            a_admit = np.linalg.inv(self.M_v) @ (F_ext_R - self.D_v @ self.v_right_ref)
            self.v_right_ref += a_admit * self.dt
            self.x_right_ref += self.v_right_ref[:3] * self.dt

            J_R_full = pin.getFrameJacobian(self.model, self.data, self.right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_R = J_R_full[:6, 7:]
            F_R_cmd = self.Kp @ np.concatenate([self.x_right_ref - x_R, np.zeros(3)]) + self.Kd @ (self.v_right_ref - J_R @ self.v[7:])
            tau_R = self.data.g[7:] + J_R.T @ np.clip(F_R_cmd, -60, 60)

            # --- (3) 통합 및 시뮬레이션 ---
            tau = np.concatenate([tau_L, tau_R]) - 0.5 * self.v # 조인트 댐핑 유지
            
            M_stable = self.data.M + np.eye(self.model.nv) * 1e-2
            ddq = np.linalg.solve(M_stable, tau - self.data.nle)
            
            self.v += np.clip(ddq, -30, 30) * self.dt
            self.q = pin.integrate(self.model, self.q, np.clip(self.v, -2, 2) * self.dt)

            # (E) 메시지 발행 (동일)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [n for n in self.model.names if "joint" in n]
            msg.position = self.q.tolist()
            self.publisher_.publish(msg)
            
            # # # (B) 제어 토크 계산 (앞서 검증한 로직)
            # # # Left Arm (Impedance)
            # # x_L = self.data.oMf[self.left_ee_id].translation
            # # J_L = pin.getFrameJacobian(self.model, self.data, self.left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            # # F_L = self.Kp @ np.concatenate([self.x_left_des - x_L, np.zeros(3)]) - self.Kd @ (J_L @ self.v)
            # # F_L = np.clip(F_L, -50, 50) # 최대 힘 제한
            # # tau_L = self.data.g[:7] + (J_L[:6, :7].T @ F_L)

            # # Right Arm (Admittance)
            # # F_ext = np.array([0, 0, 10.0, 0, 0, 0]) # 가상 외력
            # F_ext = np.zeros(6) # 가상 외력
            # a_admit = np.linalg.inv(self.M_v) @ (F_ext - self.D_v @ self.v_right_ref)
            # self.v_right_ref += a_admit * self.dt
            # self.x_right_ref += self.v_right_ref[:3] * self.dt

            # x_R = self.data.oMf[self.right_ee_id].translation
            # J_R = pin.getFrameJacobian(self.model, self.data, self.right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            # F_R = self.Kp @ np.concatenate([self.x_right_ref - x_R, np.zeros(3)]) + self.Kd @ (self.v_right_ref - J_R @ self.v)
            # F_R = np.clip(F_R, -50, 50) # 최대 힘 제한
            # tau_R = self.data.g[7:] + (J_R[:6, 7:].T @ F_R)

            # # 조인트 마찰 댐핑 추가 : 발산 억제용
            # tau_damping = -0.5 * self.v
            # tau = np.concatenate([tau_L, tau_R]) + tau_damping

            # # (C) 정동역학 (Forward Dynamics): tau -> ddq
            # # # M(q)ddq + nle = tau  => ddq = M^-1 * (tau - nle)
            # # ddq = np.linalg.solve(self.data.M, tau - self.data.nle)
            # # M 행렬의 역행렬을 구할 때 수치적으로 불안정할 수 있으므로 미소값을 더함
            # M_stable = self.data.M + np.eye(self.model.nv) * 1e-2
            # ddq = np.linalg.solve(M_stable, tau - self.data.nle)
            # ddq = np.clip(ddq, -20.0, 20.0)

            # # (D) 수치 적분 (Semi-implicit Euler)
            # self.v += ddq * self.dt
            # self.v = np.clip(self.v, -2.0, 2.0) # 속도 제한
            # # Integration (Pinocchio의 integrate는 쿼터니언 등 안전하게 처리함)
            # self.q = pin.integrate(self.model, self.q, self.v * self.dt)

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
        except Exception as e:
            self.get_logger().error(f"Interaction Loop Error: {e}")

def main():
    rclpy.init()
    sim = DualArmSim()
    rclpy.spin(sim)
    sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()