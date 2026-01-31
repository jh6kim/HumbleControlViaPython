import pinocchio as pin
import numpy as np
import os

# 1. 환경 설정 및 모델 로드
urdf_path = os.path.expanduser("~/ros2_ws/dual_franka.urdf")
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 2. 제어 파라미터 설정 (Gain tuning)
# 작업 공간(Cartesian) 강성(K) 및 감쇠(D) 계수
Kp = np.diag([400, 400, 400, 50, 50, 50])  # [X, Y, Z, Roll, Pitch, Yaw]
Kd = np.diag([40, 40, 40, 5, 5, 5])

# 3. 목표 상태 설정 (Desired State)
# 왼쪽 팔 EE가 현재 위치에서 약간 위(+10cm)로 가길 원한다고 가정
q = np.full((model.nq), 0.1)  # 현재 관절 각도
v = np.zeros(model.nv)        # 현재 관절 속도

# 현재 End-effector 위치 계산 (Forward Kinematics)
left_ee_id = model.getFrameId("left_fr3_link8")
pin.framesForwardKinematics(model, data, q)
x_current = data.oMf[left_ee_id].translation
R_current = data.oMf[left_ee_id].rotation

x_desired = x_current + np.array([0, 0, 0.1])  # 목표: 현재보다 10cm 위
v_desired = np.zeros(6)                        # 목표 속도: 정지

# 4. 제어 연산 (Control Loop 1회 시뮬레이션)
# (1) 중력 및 동역학 파라미터 업데이트
pin.computeAllTerms(model, data, q, v)
g = data.g
# C = pin.computeCoriolisMatrix(model, data, q, v)
C_vector = data.nle - data.g  # 원심력 & 코리올리 토크 (Non-linear effects에서 중력을 뺀 값)

# (2) 자코비안 계산 (프레임 정보 업데이트 후)
pin.framesForwardKinematics(model, data, q)
J = pin.getFrameJacobian(model, data, left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

# (3) 작업 공간 오차 계산 (Position Error)
pos_error = x_desired - x_current
# 회전 오차는 단순화를 위해 여기서는 위치 오차만 반영 (Orientation 제외 시 3x1 사용 가능)
task_error = np.concatenate([pos_error, np.zeros(3)]) 
vel_error = v_desired - (J @ v)

# (4) 가상 힘(Virtual Force) 계산 (Impedance Law)
F_task = Kp @ task_error + Kd @ vel_error

# (5) 최종 토크 계산: 중력 보상 + 임피던스 힘
tau = g + (J.T @ F_task)

print("--- Control Output Summary ---")
print(f"Goal: Move +10cm in Z-axis")
print(f"Calculated Task Force (Fz): {F_task[2]:.2f} N")
print(f"Total Joint Torques (first 7 for Left Arm):\n{tau[:7]}")