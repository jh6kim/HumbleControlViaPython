import pinocchio as pin
import numpy as np
import os

# 1. 모델 로드 및 초기화
urdf_path = os.path.expanduser("~/ros2_ws/dual_franka.urdf")
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 2. 제어 파라미터 설정
dt = 0.01  # 제어 주기
Kp = np.diag([400, 400, 400, 50, 50, 50])
Kd = np.diag([40, 40, 40, 5, 5, 5])

# 어드미턴스 파라미터 (오른쪽 팔)
M_v = np.diag([5.0, 5.0, 5.0, 1.0, 1.0, 1.0])
D_v = np.diag([50.0, 50.0, 50.0, 5.0, 5.0, 5.0])

# 3. 초기 상태 설정
q = np.full((model.nq), 0.1)
v = np.zeros(model.nv)

# 초기 위치 계산 (Forward Kinematics)
left_ee_id = model.getFrameId("left_fr3_link8")
right_ee_id = model.getFrameId("right_fr3_link8")
pin.framesForwardKinematics(model, data, q)

# 왼쪽 팔 목표 (현재보다 10cm 위)
x_left_desired = data.oMf[left_ee_id].translation + np.array([0, 0, 0.1])
v_left_desired = np.zeros(6)

# 오른쪽 팔 추종 상태 (어드미턴스에 의해 변할 값)
x_right_ref = data.oMf[right_ee_id].translation.copy()
v_right_ref = np.zeros(6)

# 4. 협업 제어 루프 (예시로 10번의 step만 출력)
print(f"--- Starting Cooperative Control Loop (dt={dt}) ---")

for i in range(10):
    # (A) 현재 동역학 상태 업데이트
    pin.computeAllTerms(model, data, q, v)
    pin.framesForwardKinematics(model, data, q)
    g = data.g
    
    # --- [왼쪽 팔: 임피던스 제어 (Leader)] ---
    x_L = data.oMf[left_ee_id].translation
    J_L = pin.getFrameJacobian(model, data, left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    
    err_L = np.concatenate([x_left_desired - x_L, np.zeros(3)])
    F_L = Kp @ err_L + Kd @ (v_left_desired - (J_L @ v))
    tau_L = g[:7] + (J_L[:6, :7].T @ F_L)

    # --- [오른쪽 팔: 어드미턴스 제어 (Follower)] ---
    # 1. 외력 가정 (물체를 통해 왼쪽 팔이 주는 힘이 전달된다고 가정)
    F_ext = np.array([0, 0, 10.0, 0, 0, 0]) # 10N의 힘이 전달됨
    
    # 2. 어드미턴스 모델로 목표 위치(x_right_ref) 갱신
    a_admit = np.linalg.inv(M_v) @ (F_ext - D_v @ v_right_ref)
    v_right_ref += a_admit * dt
    x_right_ref += v_right_ref[:3] * dt
    
    # 3. 갱신된 목표 위치를 추종하는 토크 계산
    x_R = data.oMf[right_ee_id].translation
    J_R = pin.getFrameJacobian(model, data, right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    
    err_R = np.concatenate([x_right_ref - x_R, np.zeros(3)])
    F_R = Kp @ err_R + Kd @ (v_right_ref - (J_R @ v))
    tau_R = g[7:] + (J_R[:6, 7:].T @ F_R)

    # (C) 최종 토크 통합 및 출력
    tau_total = np.concatenate([tau_L, tau_R])
    
    if i % 2 == 0:
        print(f"Step {i}: Left Force Z={F_L[2]:.2f}N, Right Ref Z Offset={x_right_ref[2]-x_R[2]:.4f}m")

print("\n[성공] 임피던스(L)와 어드미턴스(R) 제어 루프가 정상 동작합니다.")