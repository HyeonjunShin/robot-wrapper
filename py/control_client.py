import socket
import struct
import numpy as np
import time

# =====================================================================
# 1. 로봇 기구학 파라미터 (Doosan m1013 기반)
# =====================================================================
D1 = 0.1525   # 베이스 높이 (Base to Shoulder)
D2 = 0.0345   # 어깨 가로 오프셋 (Shoulder Offset)
A2 = 0.6200   # 위팔 길이 (Upper Arm)
D4 = 0.5590   # 아래팔 길이 (Forearm: Elbow to Wrist Center)
D6 = 0.1310   # 손목 중심부터 TCP(툴 끝점)까지의 거리

# 관절 움직임 한계 설정 (XML의 range 속성과 매칭)
JOINT_LIMITS = [
    (-6.283, 6.283),  # joint_1
    (-1.658, 1.658),  # joint_2
    (-2.53,  2.53),   # joint_3
    (-6.283, 6.283),  # joint_4
    (-2.356, 2.356),  # joint_5
    (-6.283, 6.283)   # joint_6
]

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

# 순기구학 (Forward Kinematics)
def forward_kinematics(q):
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

# 6x6 자코비안 수치 해석 계산
def calculate_jacobian(q):
    h = 1e-5
    J = np.zeros((6, 6))
    pos_curr, R_curr = forward_kinematics(q)
    
    for i in range(6):
        q_perturbed = np.copy(q)
        q_perturbed[i] += h
        pos_perturbed, R_perturbed = forward_kinematics(q_perturbed)
        
        d_pos = (pos_perturbed - pos_curr) / h
        
        dR = R_perturbed @ R_curr.T
        d_ori = np.array([
            dR[2, 1] - dR[1, 2],
            dR[0, 2] - dR[2, 0],
            dR[1, 0] - dR[0, 1]
        ]) / (2 * h)
        
        J[:3, i] = d_pos
        J[3:6, i] = d_ori
    return J

# 오프라인 수치 해석 IK 솔버 (초기 정렬용)
def solve_offline_ik(target_pos, target_rot, q_init, max_iter=100, tol=1e-4):
    q = np.copy(q_init)
    damping = 0.02
    for _ in range(max_iter):
        curr_pos, curr_rot = forward_kinematics(q)
        error_pos = target_pos - curr_pos
        
        error_rot_mat = target_rot @ curr_rot.T
        error_rot = 0.5 * np.array([
            error_rot_mat[2, 1] - error_rot_mat[1, 2],
            error_rot_mat[0, 2] - error_rot_mat[2, 0],
            error_rot_mat[1, 0] - error_rot_mat[0, 1]
        ])
        
        error_total = np.hstack((error_pos, error_rot))
        
        if np.linalg.norm(error_total) < tol:
            return q, True
            
        J = calculate_jacobian(q)
        J_JT = J @ J.T
        damping_matrix = (damping ** 2) * np.eye(6)
        inv_part = np.linalg.inv(J_JT + damping_matrix)
        J_damped_inv = J.T @ inv_part
        
        q += 0.5 * J_damped_inv @ error_total
        
        # 물리적 관절 각도 한계(Joint Limits) 내로 클램핑
        for i in range(6):
            q[i] = np.clip(q[i], JOINT_LIMITS[i][0], JOINT_LIMITS[i][1])
        
    return q, False
# =====================================================================
# 2. 로봇 하드웨어 및 시뮬레이터 추상화 인터페이스
# =====================================================================
class RobotInterface:
    def connect(self):
        """로봇 컨트롤러 또는 시뮬레이터에 연결합니다."""
        raise NotImplementedError

    def send_and_receive(self, q_cmd: np.ndarray) -> np.ndarray:
        """목표 관절각을 보내고, 로봇 엔코더로부터 실시간 실제 관절각을 읽어 반환합니다."""
        raise NotImplementedError

    def disconnect(self):
        """연결을 해제하고 리소스를 정리합니다."""
        raise NotImplementedError


class MuJoCoSocketInterface(RobotInterface):
    """
    MuJoCo 시뮬레이터(sim_server.py)와 소켓 통신을 담당하는 인터페이스
    """
    def __init__(self, host='127.0.0.1', port=50005):
        self.host = host
        self.port = port
        self.client = None

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.host, self.port))
            print(f"✅ MuJoCo 시뮬레이터 서버 연결 완료! ({self.host}:{self.port})")
        except Exception as e:
            print(f"❌ 시뮬레이터 서버 연결 실패: {e}")
            print("먼저 py/sim_server.py를 실행해 두었는지 확인하세요.")
            raise e

    def send_and_receive(self, q_cmd: np.ndarray) -> np.ndarray:
        # 목표 각도 전송
        send_bytes = struct.pack('6d', *q_cmd)
        self.client.sendall(send_bytes)
        
        # 실제 각도 피드백 수신
        recv_bytes = self.client.recv(48)
        if not recv_bytes:
            raise ConnectionError("❌ MuJoCo 서버로부터 데이터를 받지 못했습니다. 연결이 강제 종료되었을 수 있습니다.")
        return np.array(struct.unpack('6d', recv_bytes))

    def disconnect(self):
        if self.client:
            self.client.close()
            print("🔌 MuJoCo 소켓 연결을 종료했습니다.")


class RealRobotInterfacePlaceholder(RobotInterface):
    """
    실제 협동 로봇(예: 두산 로봇 M1013)으로 알고리즘을 이식할 때 사용할 드라이버 템플릿
    """
    def __init__(self, ip='192.168.1.100'):
        self.ip = ip
        # self.api = DoosanRobotAPI(ip)  # 실제 로봇 SDK 인스턴스

    def connect(self):
        print(f"🔌 실제 로봇 컨트롤러({self.ip})와 통신을 시작합니다...")
        # self.api.connect()

    def send_and_receive(self, q_cmd: np.ndarray) -> np.ndarray:
        # [실제 로봇 이식 코드 예시]
        # 1. 로봇에 관절 제어 패킷 전송 (예: servoj)
        # self.api.servoj(q_cmd, time=0.01)
        
        # 2. 로봇 엔코더로부터 실제 피드백 각도 수신
        # q_curr = self.api.get_actual_joint_position()
        
        # (시뮬레이션 검증용 임시 가상 데이터 반환)
        q_curr = np.copy(q_cmd)
        return q_curr

    def disconnect(self):
        print("🔌 실제 로봇과 통신을 안전하게 종료하고 모터를 정지합니다.")
        # self.api.disconnect()


# =====================================================================
# 3. 메인 제어 루프
# =====================================================================
def main():
    # ----------------------------------------------------
    # [제어 대상 선택] 아래 두 줄 중 하나만 주석 해제하여 사용하십시오.
    # ----------------------------------------------------
    robot = MuJoCoSocketInterface(host='127.0.0.1', port=50005)      # 시뮬레이터 모드
    # robot = RealRobotInterfacePlaceholder(ip='192.168.1.100')     # 실제 로봇 모드
    # ----------------------------------------------------

    try:
        robot.connect()
    except Exception:
        return

    # 제어 변수 설정
    dt = 0.01  # 제어 루프 주기 (10ms / 100Hz)
    kp = 15.0  # 비례 게인 (오차 피드백 세기)
    damping = 0.02  # 특이점 회피 감쇠 계수
    
    # 궤적 명령어 변수 (YZ 평면의 원 궤적)
    center_pos = np.array([0.4, 0.0, 0.6])   # 원 중심 (X, Y, Z) - 작업 반경 내 안전 위치
    radius = 0.1                            # 원 반지름 (10cm)
    omega = 1.0                             # 회전 속도 (rad/s)
    
    # 목표 방향 (카메라가 바닥을 향하도록 피치 180도 회전 상태 유지)
    target_rot = rot_z(0) @ rot_y(0) @ rot_x(0)
    
    # t = 0 일 때의 시작 타겟 포즈 계산
    start_target_pos = np.array([
        center_pos[0],
        center_pos[1] + radius * np.sin(0),
        center_pos[2] + radius * np.cos(0)
    ])
    
    # 1. 초기 임의의 위치(0도 상태) 전송하여 현재 로봇 실제 각도를 가져옴
    temp_initial_q = np.zeros(6)
    try:
        q_initial_curr = robot.send_and_receive(temp_initial_q)
    except Exception as e:
        print(f"초기 각도 수신 실패: {e}")
        robot.disconnect()
        return
    
    print("🔄 [초기화] 원형 궤적 시작 위치로 로봇 정렬 중...")
    
    # 2. 오프라인 수치 해석 IK를 이용해 원형 궤적의 정확한 시작 조인트 각도 계산
    ik_seed = np.radians([0.0, 15.0, 45.0, 0.0, 30.0, 0.0])
    q_start, success = solve_offline_ik(start_target_pos, target_rot, ik_seed)
    if not success:
        print("⚠️ 시작 위치에 대한 IK를 찾지 못했습니다. 임의의 각도로 진행합니다.")
        q_start = q_initial_curr
        
    # 3. 로봇을 시작 조인트 각도로 천천히 부드럽게 이동 (Joint P2P 선형 보간)
    print("📍 시작 지점으로 조인트 이동 (2초)...")
    steps = 200  # 2초 동안 이동 (10ms * 200)
    for s in range(steps):
        t_frac = s / (steps - 1)
        q_interp = (1 - t_frac) * q_initial_curr + t_frac * q_start
        try:
            robot.send_and_receive(q_interp)
        except Exception as e:
            print(f"정렬 중 에러 발생: {e}")
            robot.disconnect()
            return
        time.sleep(0.01)
        
    print("✅ 정렬 완료. 실시간 원형 궤적 추적 제어를 시작합니다. (RMRC)")
    print("----------------------------------------------------\n")
    
    q_cmd = np.copy(q_start)
    q_curr = np.copy(q_start)
    
    start_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            t = loop_start - start_time
            
            # 1. 실시간 목표 TCP 위치 및 속도 생성 (원형 궤적)
            target_pos = np.array([
                center_pos[0],
                center_pos[1] + radius * np.sin(omega * t),
                center_pos[2] + radius * np.cos(omega * t)
            ])
            # 원형 궤적의 실시간 접선 속도 (Feedforward velocity)
            target_vel = np.array([
                0.0,
                omega * radius * np.cos(omega * t),
                -omega * radius * np.sin(omega * t),
                0.0, 0.0, 0.0 # 방향 속도는 0 유지
            ])
            
            # 2. 현재 로봇 실제 TCP 상태 계산 (Forward Kinematics)
            curr_pos, curr_rot = forward_kinematics(q_curr)
            
            # 3. 작업 공간 오차 계산
            error_pos = target_pos - curr_pos
            
            # 방향 오차 계산
            error_rot_mat = target_rot @ curr_rot.T
            error_rot = 0.5 * np.array([
                error_rot_mat[2, 1] - error_rot_mat[1, 2],
                error_rot_mat[0, 2] - error_rot_mat[2, 0],
                error_rot_mat[1, 0] - error_rot_mat[0, 1]
            ])
            
            error_total = np.hstack((error_pos, error_rot))
            
            # 4. 자코비안 계산
            J = calculate_jacobian(q_cmd)
            
            # 5. DLS (Damped Least Squares) 역행렬 적용
            J_JT = J @ J.T
            damping_matrix = (damping ** 2) * np.eye(6)
            inv_part = np.linalg.inv(J_JT + damping_matrix)
            J_damped_inv = J.T @ inv_part
            
            # 6. 관절속도 계산 (피드포워드 속도 + 피드백 제어) 및 적분을 통한 목표각 생성
            q_vel = J_damped_inv @ (target_vel + kp * error_total)
            
            # 속도 한계 제한 (안전장치: 최대 1.5 rad/s)
            q_vel = np.clip(q_vel, -1.5, 1.5)
            q_cmd += q_vel * dt
            
            # 관절 한계 누적 클램핑 (수치 적분 드리프트 방지)
            for i in range(6):
                q_cmd[i] = np.clip(q_cmd[i], JOINT_LIMITS[i][0], JOINT_LIMITS[i][1])
            
            # 7. 목표 각도 전송 및 현재 실제 각도 수신
            try:
                q_curr = robot.send_and_receive(q_cmd)
            except Exception as e:
                print(f"\n❌ 실시간 제어 루프 통신 실패: {e}")
                break
            
            # 8. 주기적인 실시간 오차 출력 (mm 단위 환산)
            tracking_err_norm = np.linalg.norm(error_pos)
            print(f"⏱️ Time: {t:.2f}s | Target Pos: {np.round(target_pos, 3)} | Track Error: {tracking_err_norm*1000:.2f} mm  ", end='\r')
            
            # 10ms 주기 제어
            elapsed = time.time() - loop_start
            time_to_sleep = dt - elapsed
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
                
    except KeyboardInterrupt:
        print("\n🛑 사용자에 의해 제어가 중지되었습니다.")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()
