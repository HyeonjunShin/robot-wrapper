import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import queue
import sys

# =====================================================================
# 1. 로봇 기구학 파라미터 (Doosan m1013 기반)
# =====================================================================
D1 = 0.1525   # 베이스 높이 (Base to Shoulder)
D2 = 0.0345   # 어깨 가로 오프셋 (Shoulder Offset)
A2 = 0.6200   # 위팔 길이 (Upper Arm)
D4 = 0.5590   # 아래팔 길이 (Forearm: Elbow to Wrist Center)
D6 = 0.1310   # 손목 중심부터 TCP(툴 끝점)까지의 거리 (0.1210 + 0.0100 gripper offset)

# =====================================================================
# 2. 회전 행렬 유틸리티 함수
# =====================================================================
def rot_x(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

def rot_y(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def rot_to_rpy(R):
    """
    회전 행렬 R을 Roll-Pitch-Yaw (X-Y-Z 고정축 회전)로 변환.
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    """
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    if np.abs(np.cos(pitch)) < 1e-6:
        # Gimbal Lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
    return roll, pitch, yaw

# =====================================================================
# 3. 순방향 기구학 (Forward Kinematics)
# =====================================================================
def forward_kinematics(q):
    """
    6개 관절 각도 q에 대해 최종 TCP의 위치와 회전 행렬을 계산합니다.
    """
    q1, q2, q3, q4, q5, q6 = q
    
    T01 = np.eye(4)
    T01[:3, :3] = rot_z(q1)
    T01[:3, 3] = [0, 0, D1]
    
    T12 = np.eye(4)
    T12[:3, :3] = rot_y(q2)
    T12[:3, 3] = [0, D2, 0]
    
    T23 = np.eye(4)
    T23[:3, :3] = rot_y(q3)
    T23[:3, 3] = [0, 0, A2]
    
    T34 = np.eye(4)
    T34[:3, :3] = rot_z(q4)
    T34[:3, 3] = [0, 0, D4]
    
    T45 = np.eye(4)
    T45[:3, :3] = rot_y(q5)
    
    T56 = np.eye(4)
    T56[:3, :3] = rot_z(q6)
    
    T6_tool = np.eye(4)
    T6_tool[:3, 3] = [0, 0, D6]
    
    T0_tool = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T6_tool
    return T0_tool[:3, 3], T0_tool[:3, :3]

# =====================================================================
# 4. 해석적 역운동학 (Analytical Inverse Kinematics)
# =====================================================================
def analytical_ik(target_pos, target_rot):
    """
    목표 위치 target_pos = [x, y, z] 및 회전 행렬 target_rot에 대한
    해석적 역운동학 해(최대 8개)를 계산합니다.
    """
    solutions = []
    
    # Step 1: 손목 중심점 (Wrist Center, P_wc) 계산
    a = target_rot[:, 2]  # 접근 벡터
    p_wc = target_pos - D6 * a
    x_wc, y_wc, z_wc = p_wc
    
    # Step 2: 1번 관절 각도 (q1) 구하기
    r_xy = np.sqrt(x_wc**2 + y_wc**2)
    if r_xy < D2:
        return [None] * 8
        
    offset_angle = np.arctan2(D2, np.sqrt(r_xy**2 - D2**2))
    base_angle = np.arctan2(y_wc, x_wc)
    
    # q1의 두 가지 분기 (Left / Right shoulder)
    q1_candidates = [
        base_angle - offset_angle,         # Left shoulder
        base_angle + offset_angle + np.pi  # Right shoulder
    ]
    
    for s_idx, q1 in enumerate(q1_candidates):
        q1 = np.arctan2(np.sin(q1), np.cos(q1))
        
        R_plane = x_wc * np.cos(q1) + y_wc * np.sin(q1)
        Z_plane = z_wc - D1
        
        s_sq = R_plane**2 + Z_plane**2
        s = np.sqrt(s_sq)
        
        if s > (A2 + D4) or s < np.abs(A2 - D4):
            solutions.extend([None, None, None, None])
            continue
            
        # Step 3: 3번 관절 각도 (q3) 계산 (제2코사인 법칙)
        cos_q3 = (s_sq - A2**2 - D4**2) / (2 * A2 * D4)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        
        # q3의 두 가지 분기 (Elbow Up / Down)
        sin_q3_candidates = [
            np.sqrt(1.0 - cos_q3**2),  # Elbow Up
            -np.sqrt(1.0 - cos_q3**2)  # Elbow Down
        ]
        
        for e_idx, sin_q3 in enumerate(sin_q3_candidates):
            q3 = np.arctan2(sin_q3, cos_q3)
            
            # Step 4: 2번 관절 각도 (q2) 계산
            k1 = A2 + D4 * cos_q3
            k2 = D4 * sin_q3
            
            q2 = np.arctan2(k1 * R_plane - k2 * Z_plane, k2 * R_plane + k1 * Z_plane)
            
            # Step 5: 손목 회전 행렬 계산 및 q4, q5, q6 오일러 각 추출
            R01 = rot_z(q1)
            R12 = rot_y(q2)
            R23 = rot_y(q3)
            R03 = R01 @ R12 @ R23
            
            R36 = R03.T @ target_rot
            
            r11, r12, r13 = R36[0, 0], R36[0, 1], R36[0, 2]
            r21, r22, r23 = R36[1, 0], R36[1, 1], R36[1, 2]
            r31, r32, r33 = R36[2, 0], R36[2, 1], R36[2, 2]
            
            # q5의 두 가지 분기 (Wrist Flip / No-Flip)
            if np.abs(r33) >= 1.0 - 1e-7:
                # Gimbal Lock
                q5 = 0.0 if r33 > 0 else np.pi
                q4 = 0.0
                if r33 > 0:
                    q6 = np.arctan2(r21, r11)
                else:
                    q6 = -np.arctan2(-r21, -r11)
                solutions.append(np.array([q1, q2, q3, q4, q5, q6]))
                solutions.append(np.array([q1, q2, q3, q4, q5, q6]))
            else:
                sin_q5_candidates = [
                    np.sqrt(1.0 - r33**2),  # No-flip
                    -np.sqrt(1.0 - r33**2)  # Flip
                ]
                
                for w_idx, sin_q5 in enumerate(sin_q5_candidates):
                    q5 = np.arctan2(sin_q5, r33)
                    q4 = np.arctan2(r23 / sin_q5, r13 / sin_q5)
                    q6 = np.arctan2(r32 / sin_q5, -r31 / sin_q5)
                    solutions.append(np.array([q1, q2, q3, q4, q5, q6]))
                    
    return solutions

# =====================================================================
# 4.5. 안전 검사 (땅 충돌 방지)
# =====================================================================
def is_solution_safe(sol):
    q1, q2, q3, q4, q5, q6 = sol
    p3_z = D1 + A2 * np.cos(q2)
    p4_z = p3_z + D4 * np.cos(q2 + q3)
    safety_margin = 0.05
    return p3_z >= safety_margin and p4_z >= safety_margin

# =====================================================================
# 5. 환경 설정 및 모델 로드
# =====================================================================
xml_path = '/home/uon/code/robot_control/gui/m1013/m1013_mujoco.xml'
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 관절 인덱스 주소 추출
joint_names = [f"joint_{i+1}" for i in range(6)]
joint_qpos_adr = [model.joint(name).qposadr[0] for name in joint_names]
joint_qvel_adr = [model.joint(name).dofadr[0] for name in joint_names]

# 초기 위치 획득
initial_pos = np.array(data.qpos[joint_qpos_adr])
target_qpos = np.copy(initial_pos)
filtered_target = np.copy(initial_pos)
filtered_target_vel = np.zeros(6)

# 조인트 최대 속도 및 가속도 제한 설정 (라디안 단위)
v_max = np.array([2.0, 2.0, 2.0, 2.5, 2.5, 2.5])      # rad/s
a_max = np.array([4.0, 4.0, 4.0, 5.0, 5.0, 5.0])      # rad/s²

# 궤적 상태 변수
traj_active = False
traj_start_q = np.copy(initial_pos)
traj_target_q = np.copy(initial_pos)
traj_time = 0.0
T_total = 0.0
t_a = 0.0
t_v = 0.0
t_d = 0.0
v_limit = 0.0
a_limit = 0.0
is_trapezoid = False
v_peak = 0.0

# 왕복 운동(Loop Mode) 상태 변수 및 좌표 설정
loop_active = False
loop_target_idx = 0
point_A = (np.array([0.5, 0.3, 0.4]), rot_z(0) @ rot_y(np.radians(90)) @ rot_x(0), "Point A (우측)")
point_B = (np.array([0.5, -0.3, 0.4]), rot_z(0) @ rot_y(np.radians(90)) @ rot_x(0), "Point B (좌측)")
points = [point_A, point_B]

# PD 제어 게인
kp = np.array([2000.0, 2000.0, 1500.0, 800.0, 800.0, 500.0])
kd = np.array([ 250.0,  250.0,  180.0,  80.0,  80.0,  40.0])

# 안전한 스레드 통신 큐
input_queue = queue.Queue()

# =====================================================================
# 6. 터미널 입력 백그라운드 스레드
# =====================================================================
def input_worker():
    while True:
        try:
            print("\n[입력] 목표 TCP 위치/자세 또는 명령(loop/stop)을 입력하세요.")
            print("👉 예시 입력 1: 0.5 0.3 0.4 0 90 0  (X Y Z R P Y)")
            print("👉 예시 입력 2: loop               (지정된 두 지점 왕복 운동 시작)")
            print("👉 예시 입력 3: stop               (왕복 운동 정지)")
            print("👉 ", end="")
            sys.stdout.flush()
            
            line = sys.stdin.readline().strip().lower()
            if not line:
                continue
                
            if line in ["loop", "stop"]:
                input_queue.put(line)
                continue
                
            parts = line.split()
            if len(parts) != 6:
                print(f"❌ 에러: 'loop', 'stop' 또는 6개의 숫자가 필요합니다. (현재 입력: {len(parts)}개)")
                continue
                
            x_target, y_target, z_target, r_deg, p_deg, y_deg = [float(p) for p in parts]
            input_queue.put((x_target, y_target, z_target, r_deg, p_deg, y_deg))
            
        except ValueError:
            print("❌ 에러: 올바른 숫자 형식이 아닙니다.")
        except Exception as e:
            print(f"❌ 스레드 에러: {e}")
            break

# 초기 TCP 위치 및 자세 계산 및 출력
init_xyz, init_R = forward_kinematics(initial_pos)
init_roll, init_pitch, init_yaw = rot_to_rpy(init_R)

print("====================================================")
print("  두산 M1013 역기구학(IK) 안전 추종 제어기 구동 ")
print("  - 입력 시 역기구학 해를 계산하여 부드럽게 이동합니다.")
print("====================================================")
print(f"📍 현재 TCP 위치: X={init_xyz[0]:.4f} m, Y={init_xyz[1]:.4f} m, Z={init_xyz[2]:.4f} m")
print(f"📍 현재 TCP 자세: Roll={np.degrees(init_roll):.2f}°, Pitch={np.degrees(init_pitch):.2f}°, Yaw={np.degrees(init_yaw):.2f}°")

# 입력 스레드 시작
threading.Thread(target=input_worker, daemon=True).start()

# =====================================================================
# 7. 메인 시뮬레이션 및 실시간 제어 루프
# =====================================================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()
        
        # 새로운 목표 입력이 들어왔는지 확인
        if not input_queue.empty():
            try:
                cmd = input_queue.get_nowait()
                if isinstance(cmd, str):
                    if cmd == "loop":
                        loop_active = True
                        loop_target_idx = 0
                        print("\n🔄 [왕복 운동] 왕복 운동 모드가 활성화되었습니다.")
                        # 첫 번째 지점으로 이동 계획 수립
                        target_pos, target_rot, point_name = points[loop_target_idx]
                        solutions = analytical_ik(target_pos, target_rot)
                        valid_sols = [sol for sol in solutions if sol is not None]
                        safe_sols = [sol for sol in valid_sols if is_solution_safe(sol)]
                        if len(safe_sols) == 0:
                            print(f"❌ 에러: {point_name}이 도달 불가능하거나 바닥에 닿습니다. 왕복 모드를 취소합니다.")
                            loop_active = False
                        else:
                            curr_q = data.qpos[joint_qpos_adr]
                            best_sol = min(safe_sols, key=lambda sol: np.linalg.norm(sol - curr_q))
                            traj_start_q = np.copy(filtered_target)
                            traj_target_q = np.copy(best_sol)
                            diff = np.abs(traj_target_q - traj_start_q)
                            max_diff = np.max(diff)
                            if max_diff < 1e-6:
                                traj_active = False
                                target_qpos = best_sol
                                T_total = 0.0
                            else:
                                s_v_limits = [v_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                                s_a_limits = [a_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                                v_limit = min(s_v_limits)
                                a_limit = min(s_a_limits)
                                d_acc = (v_limit**2) / a_limit
                                if d_acc <= 1.0:
                                    t_a = v_limit / a_limit
                                    t_v = (1.0 - d_acc) / v_limit
                                    t_d = t_a
                                    T_total = 2 * t_a + t_v
                                    is_trapezoid = True
                                else:
                                    t_a = 1.0 / np.sqrt(a_limit)
                                    t_v = 0.0
                                    t_d = t_a
                                    T_total = 2 * t_a
                                    is_trapezoid = False
                                    v_peak = np.sqrt(a_limit)
                                traj_time = 0.0
                                traj_active = True
                                target_qpos = best_sol
                            print(f"👉 {point_name}으로 이동을 시작합니다 (예상 시간: {T_total:.2f}초).")
                    elif cmd == "stop":
                        loop_active = False
                        print("\n🛑 [왕복 운동] 왕복 운동 모드가 정지되었습니다. 현재 위치에서 대기합니다.")
                else:
                    # 수동 좌표 입력 수신
                    if loop_active:
                        print("\n🛑 [왕복 운동] 새로운 수동 입력 수신으로 왕복 모드를 해제합니다.")
                        loop_active = False
                        
                    x_t, y_t, z_t, r_d, p_d, y_d = cmd
                    
                    safety_margin = 0.05
                    if z_t < safety_margin:
                        print(f"\n❌ 에러: 목표 TCP 높이(Z={z_t:.3f}m)가 안전 한계({safety_margin}m)보다 낮아 땅에 부딪힙니다.")
                        continue
                    
                    target_pos = np.array([x_t, y_t, z_t])
                    
                    # Roll-Pitch-Yaw를 회전 행렬로 변환
                    yaw_r = np.radians(y_d)
                    pitch_r = np.radians(p_d)
                    roll_r = np.radians(r_d)
                    target_rot = rot_z(yaw_r) @ rot_y(pitch_r) @ rot_x(roll_r)
                    
                    # 역기구학 계산
                    solutions = analytical_ik(target_pos, target_rot)
                    
                    valid_sols = [sol for sol in solutions if sol is not None]
                    safe_sols = [sol for sol in valid_sols if is_solution_safe(sol)]
                    
                    if len(safe_sols) == 0:
                        if len(valid_sols) > 0:
                            print(f"\n⚠️ 경고: 역기구학 해는 존재하지만, 모든 해가 땅에 충돌하는 자세를 취하므로 안전을 위해 차단합니다.")
                        else:
                            print(f"\n❌ 에러: 도달 불가능한 목표 영역입니다 (IK 해 없음).")
                        print(f"👉 입력한 목표: Pos=[{x_t:.3f}, {y_t:.3f}, {z_t:.3f}], Ori=[{r_d:.1f}, {p_d:.1f}, {y_d:.1f}]")
                    else:
                        curr_q = data.qpos[joint_qpos_adr]
                        best_sol = min(safe_sols, key=lambda sol: np.linalg.norm(sol - curr_q))
                        
                        # 궤적 계획 시작
                        traj_start_q = np.copy(filtered_target)
                        traj_target_q = np.copy(best_sol)
                        
                        diff = np.abs(traj_target_q - traj_start_q)
                        max_diff = np.max(diff)
                        if max_diff < 1e-6:
                            traj_active = False
                            target_qpos = best_sol
                            T_total = 0.0
                        else:
                            s_v_limits = [v_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                            s_a_limits = [a_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                            v_limit = min(s_v_limits)
                            a_limit = min(s_a_limits)
                            
                            d_acc = (v_limit**2) / a_limit
                            if d_acc <= 1.0:
                                t_a = v_limit / a_limit
                                t_v = (1.0 - d_acc) / v_limit
                                t_d = t_a
                                T_total = 2 * t_a + t_v
                                is_trapezoid = True
                            else:
                                t_a = 1.0 / np.sqrt(a_limit)
                                t_v = 0.0
                                t_d = t_a
                                T_total = 2 * t_a
                                is_trapezoid = False
                                v_peak = np.sqrt(a_limit)
                                
                            traj_time = 0.0
                            traj_active = True
                            target_qpos = best_sol
                        
                        print(f"\n📌 [IK 성공] 목표 각도 수신 완료. 사다리꼴 속도 프로파일로 이동을 시작합니다 (예상 소요 시간: {T_total:.2f}초).")
                        print(f"   -> q1={np.degrees(best_sol[0]):.2f}°, q2={np.degrees(best_sol[1]):.2f}°, q3={np.degrees(best_sol[2]):.2f}°")
                        print(f"   -> q4={np.degrees(best_sol[3]):.2f}°, q5={np.degrees(best_sol[4]):.2f}°, q6={np.degrees(best_sol[5]):.2f}°")
            except queue.Empty:
                pass
                
        # 외력 초기화
        data.qfrc_applied[:] = 0.0
        
        # 사다리꼴 프로파일 기반 궤적 생성
        if traj_active:
            traj_time += model.opt.timestep
            if traj_time >= T_total:
                s_val = 1.0
                s_vel = 0.0
                traj_active = False
                
                # 왕복 모드 감지 및 다음 지점 예약
                if loop_active:
                    loop_target_idx = 1 - loop_target_idx  # 0 -> 1 또는 1 -> 0 토글
                    target_pos, target_rot, point_name = points[loop_target_idx]
                    solutions = analytical_ik(target_pos, target_rot)
                    valid_sols = [sol for sol in solutions if sol is not None]
                    safe_sols = [sol for sol in valid_sols if is_solution_safe(sol)]
                    if len(safe_sols) == 0:
                        print(f"❌ 에러: {point_name}이 도달 불가능하거나 바닥에 닿습니다. 왕복 모드를 강제 정지합니다.")
                        loop_active = False
                    else:
                        curr_q = data.qpos[joint_qpos_adr]
                        best_sol = min(safe_sols, key=lambda sol: np.linalg.norm(sol - curr_q))
                        traj_start_q = np.copy(filtered_target)
                        traj_target_q = np.copy(best_sol)
                        diff = np.abs(traj_target_q - traj_start_q)
                        max_diff = np.max(diff)
                        if max_diff < 1e-6:
                            traj_active = False
                            target_qpos = best_sol
                            T_total = 0.0
                        else:
                            s_v_limits = [v_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                            s_a_limits = [a_max[i] / diff[i] for i in range(6) if diff[i] > 1e-6]
                            v_limit = min(s_v_limits)
                            a_limit = min(s_a_limits)
                            d_acc = (v_limit**2) / a_limit
                            if d_acc <= 1.0:
                                t_a = v_limit / a_limit
                                t_v = (1.0 - d_acc) / v_limit
                                t_d = t_a
                                T_total = 2 * t_a + t_v
                                is_trapezoid = True
                            else:
                                t_a = 1.0 / np.sqrt(a_limit)
                                t_v = 0.0
                                t_d = t_a
                                T_total = 2 * t_a
                                is_trapezoid = False
                                v_peak = np.sqrt(a_limit)
                            traj_time = 0.0
                            traj_active = True
                            target_qpos = best_sol
                        print(f"\n🔄 [왕복 운동] {point_name}으로 이동을 시작합니다 (예상 시간: {T_total:.2f}초).")
            else:
                if is_trapezoid:
                    if traj_time < t_a:
                        s_val = 0.5 * a_limit * (traj_time**2)
                        s_vel = a_limit * traj_time
                    elif traj_time < t_a + t_v:
                        s_val = 0.5 * a_limit * (t_a**2) + v_limit * (traj_time - t_a)
                        s_vel = v_limit
                    else:
                        t_prime = traj_time - (t_a + t_v)
                        s_val = 1.0 - 0.5 * a_limit * ((T_total - traj_time)**2)
                        s_vel = v_limit - a_limit * t_prime
                else:
                    if traj_time < t_a:
                        s_val = 0.5 * a_limit * (traj_time**2)
                        s_vel = a_limit * traj_time
                    else:
                        t_prime = traj_time - t_a
                        s_val = 1.0 - 0.5 * a_limit * ((T_total - traj_time)**2)
                        s_vel = v_peak - a_limit * t_prime
            
            filtered_target = traj_start_q + s_val * (traj_target_q - traj_start_q)
            filtered_target_vel = s_vel * (traj_target_q - traj_start_q)
        else:
            filtered_target = target_qpos
            filtered_target_vel = np.zeros(6)
        
        # 현재 물리 엔진 상태 계측
        current_qpos = data.qpos[joint_qpos_adr]
        current_qvel = data.qvel[joint_qvel_adr]
        
        # 제어 토크 계산 (PD Control with Feedforward target velocity)
        error = filtered_target - current_qpos
        ctrl_torque = kp * error + kd * (filtered_target_vel - current_qvel)
        
        # 토크 인가
        data.qfrc_applied[joint_qvel_adr] = ctrl_torque
        
        # 시뮬레이션 및 뷰어 갱신
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 2ms 주기 제어
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
