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