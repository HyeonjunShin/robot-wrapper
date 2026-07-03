import numpy as np

# 1. 하드웨어 치수 정의 (제공해주신 DH 파라미터 값)
D1 = 0.1525   # 베이스 높이
D2 = 0.0345   # 어깨 가로 오프셋
A2 = 0.6200   # 위팔 길이
D4 = 0.5590   # 아래팔 길이
D6 = 0.1210   # 손목 중심 ~ TCP

# 2. DH 파라미터 단일 동차변환 행렬 생성 함수 (Classic DH)
def get_dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])

# 3. 순기구학(FK) 엔진 함수: 6개 관절 각도를 주면 4x4 최종 포즈 행렬 반환
def forward_kinematics(q):
    T1 = get_dh_matrix(q[0], D1, 0, np.pi/2)
    T2 = get_dh_matrix(q[1], D2, A2, 0)
    T3 = get_dh_matrix(q[2], 0, 0, np.pi/2)
    T4 = get_dh_matrix(q[3], D4, 0, -np.pi/2)
    T5 = get_dh_matrix(q[4], 0, 0, np.pi/2)
    T6 = get_dh_matrix(q[5], D6, 0, 0)
    
    # 모든 조인트 행렬 연속 곱 연산
    T_total = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    return T_total

# 4. 고속 수치 미분 기반 6x6 자코비안(Jacobian) 계산 함수
def calculate_jacobian(q):
    h = 1e-5  # 미세 변화량 (미분 간격)
    J = np.zeros((6, 6))
    
    # 기준 포즈의 위치 및 방향 회전행렬 획득
    T_curr = forward_kinematics(q)
    pos_curr = T_curr[:3, 3]
    R_curr = T_curr[:3, :3]
    
    # 6개 관절을 각각 미세하게 움직이며 변화 추적 (각 열 채우기)
    for i in range(6):
        q_perturbed = np.copy(q)
        q_perturbed[i] += h
        
        T_perturbed = forward_kinematics(q_perturbed)
        pos_perturbed = T_perturbed[:3, 3]
        R_perturbed = T_perturbed[:3, :3]
        
        # 선형 위치 미분값 (X, Y, Z 속도 변화율)
        d_pos = (pos_perturbed - pos_curr) / h
        
        # 각속도 미분값 (방향 변화율 축 추출)
        # R_perturbed @ R_curr^T 연산 후 왜곡 비대칭 행렬의 원소를 추출하여 3차원 각속도 벡터 생성
        dR = R_perturbed @ R_curr.T
        d_ori = np.array([
            dR[2, 1] - dR[1, 2],
            dR[0, 2] - dR[2, 0],
            dR[1, 0] - dR[0, 1]
        ]) / (2 * h)
        
        # [버그 수정] 자코비안 행렬의 i번째 열 세팅 (0~2행은 위치, 3~5행은 방향)
        J[:3, i] = d_pos
        J[3:6, i] = d_ori
        
    return J

# 5. 수치해석적 역기구학(IK) 루프 코어 함수 (방향 오차 벡터 연산 수정본)
def numerical_ik(target_pos, target_rot, q_init, max_iter=100, tol=1e-4):
    q = np.copy(q_init)
    damping = 0.01  # 특이점 회피를 위한 감쇠 계수
    
    for item in range(max_iter):
        T_curr = forward_kinematics(q)
        pos_curr = T_curr[:3, 3]
        R_curr = T_curr[:3, :3]
        
        # 위치 오차 계산 [dX, dY, dZ]
        error_pos = target_pos - pos_curr
        
        # 방향 오차 계산 (회전 행렬의 차이 벡터화)
        error_rot_mat = target_rot @ R_curr.T
        error_rot = 0.5 * np.array([
            error_rot_mat[2, 1] - error_rot_mat[1, 2],
            error_rot_mat[0, 2] - error_rot_mat[2, 0],
            error_rot_mat[1, 0] - error_rot_mat[0, 1]
        ])
        
        # 6차원 총 오차 벡터 결합
        error_total = np.hstack((error_pos, error_rot))
        
        # 오차가 허용치(tol)보다 작으면 최적 수렴 성공!
        if np.linalg.norm(error_total) < tol:
            print(f"🎉 역기구학 계산 성공! (반복 횟수: {item}회)")
            return q, True
            
        # 실시간 자코비안 계산
        J = calculate_jacobian(q)
        
        # 감쇠 최소자승법(DLS) 적용하여 안전한 역행렬 연산
        J_JT = J @ J.T
        damping_matrix = (damping ** 2) * np.eye(6)
        inv_part = np.linalg.inv(J_JT + damping_matrix)
        J_pseudo_inv = J.T @ inv_part
        
        # 관절 미세 변화량 계산 및 업데이트 (학습률 0.5 적용하여 안정적 수렴 도모)
        delta_q = J_pseudo_inv @ error_total
        q += 0.5 * delta_q  
        
    print("❌ 최대 반복 횟수 내에 수렴하지 못했습니다.")
    return q, False

# ==========================================================
# 6. 코드 기능 검증 및 시뮬레이션 테스트
# ==========================================================
if __name__ == "__main__":
    # 무작위 정답 관절 각도 정의 (라디안 단위)
    q_true = np.radians([15.0, -30.0, 45.0, 10.0, 60.0, -20.0])
    
    # 정답 각도로부터 가상의 목표 3D 포즈 생성
    T_target = forward_kinematics(q_true)
    print(T_target)
    target_position = T_target[:3, 3]
    target_rotation = T_target[:3, :3]
    
    print("🎯 가상의 목표 XYZ 좌표:", np.round(target_position, 4))
    
    # 아무것도 모르는 초기 상태 설정 (모두 0도에서 탐색 시작)
    q_initial_guess = np.zeros(6)
    
    # 수치해석 IK 연산 실행
    q_result, success = numerical_ik(target_position, target_rotation, q_initial_guess)
    
    if success:
        print("\n[최종 역기구학 계산 결과 (각도 환산)]")
        for idx, angle in enumerate(np.degrees(q_result)):
            print(f"Joint {idx+1}: {angle:.2f} 도")
