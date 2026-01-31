import pinocchio as pin
import numpy as np
import os

# 1. URDF 파일 경로 설정
# urdf_model_path = os.path.expanduser("~/ros2_ws/dual_franka.urdf")
urdf_model_path = os.path.expanduser("~/ros2_ws/dual_franka.urdf")
# 2. 모델 로드
model = pin.buildModelFromUrdf(urdf_model_path)
data = model.createData()

print(f"Model name: {model.name}")
print(f"Number of degrees of freedom: {model.nq}") # 14가 나와야 정상입니다.

# 3. 조인트 이름 확인 (디버깅용)
for i, name in enumerate(model.names):
    print(f"Joint {i}: {name}")

# 4. 임의의 관절 각도 설정 (14차원 벡터)
# 모든 관절이 0.1 라디안만큼 굽혀진 상태 가정
q = np.full((model.nq), 0.1)

# 5. 질량 행렬(Mass Matrix) 계산
M = pin.crba(model, data, q)

# 6. 결과 출력
np.set_printoptions(precision=3, suppress=True)
print("\n--- Mass Matrix (M) ---")
print(f"Shape: {M.shape}")
print(M)

# 7. 중력 보상 토크(Gravity Torque) 계산 (옵션)
g = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
print("\n--- Gravity Torque (g) ---")
print(g)

# 8. 전방 운동학 및 자코비안 업데이트 (프레임 위치 갱신 필수)
pin.framesForwardKinematics(model, data, q)
pin.computeJointJacobians(model, data, q)

# 9. End-effector 프레임 ID 찾기
# URDF에서 생성된 링크 이름 (left_fr3_link8, right_fr3_link8)을 사용합니다.
left_ee_link = "left_fr3_link8"
right_ee_link = "right_fr3_link8"

left_ee_id = model.getFrameId(left_ee_link)
right_ee_id = model.getFrameId(right_ee_link)

# 10. 자코비안 추출 (6x14 행렬)
# ReferenceFrame.LOCAL_WORLD_ALIGNED: 베이스 좌표계 기준의 선속도/각속도를 의미합니다.
J_L = pin.getFrameJacobian(model, data, left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
J_R = pin.getFrameJacobian(model, data, right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

print("\n--- Left Arm Jacobian (6x14) ---")
print(J_L)

print("\n--- Right Arm Jacobian (6x14) ---")
print(J_R)

# 11. 가상 외력에 따른 조인트 토크 계산 (예시: 임피던스 제어의 핵심)
# 왼손 끝(EE)에 Z축 방향으로 10N의 힘이 가해졌을 때 필요한 조인트 토크
force_vector = np.array([0, 0, 10, 0, 0, 0]) # [Fx, Fy, Fz, Mx, My, Mz]
tau_external = J_L.T @ force_vector
print("\n--- Joint Torques for 10N force on Left EE (Z-axis) ---")
print(tau_external)