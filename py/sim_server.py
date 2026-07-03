import mujoco
import mujoco.viewer
import numpy as np
import time
import socket
import struct
import threading

# =====================================================================
# 1. 환경 설정 및 모델 로드
# =====================================================================
xml_path = '/home/uon/code/robot_control/gui/m1013/m1013_mujoco.xml'
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 관절 주소 인덱스 추출 (m1013 6개 관절)
joint_names = [f"joint_{i+1}" for i in range(6)]
joint_qpos_adr = [model.joint(name).qposadr[0] for name in joint_names]
joint_qvel_adr = [model.joint(name).dofadr[0] for name in joint_names]

# 초기 위치로 타겟 초기화
initial_pos = np.array(data.qpos[joint_qpos_adr])
target_qpos = np.copy(initial_pos)
current_qpos = np.copy(initial_pos)

# 스레드 안전을 위한 락
data_lock = threading.Lock()

# PD 제어 게인 설정 (gui.py와 동일하게 튜닝된 값)
kp = np.array([2000.0, 2000.0, 1500.0, 800.0, 800.0, 500.0])
kd = np.array([ 250.0,  250.0,  180.0,  80.0,  80.0,  40.0])

# =====================================================================
# 2. TCP 소켓 통신 스레드
# =====================================================================
def socket_worker():
    global target_qpos, current_qpos
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 주소 재사용 옵션 설정 (서버 재시작 시 포트 바인딩 에러 방지)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 50005))
    server.listen(1)
    
    print("====================================================")
    print("  MuJoCo 가상 로봇 소켓 서버가 시작되었습니다.")
    print("  포트: 50005 | 클라이언트 연결 대기 중...")
    print("====================================================")
    
    while True:
        try:
            conn, addr = server.accept()
            print(f"👉 제어기 클라이언트 연결 성공: {addr}")
            
            while True:
                # 6개 double-precision float (8 bytes each * 6 = 48 bytes)
                recv_bytes = conn.recv(48)
                if not recv_bytes or len(recv_bytes) < 48:
                    break
                
                # 수신된 바이너리 각도 데이터 언팩 (라디안 단위)
                q_in = np.array(struct.unpack('6d', recv_bytes))
                
                # 목표 각도 안전하게 업데이트
                with data_lock:
                    target_qpos = np.copy(q_in)
                    q_out = np.copy(current_qpos)
                
                # 현재 실제 로봇 각도를 클라이언트에게 전송 (라디안 단위, 48 bytes)
                send_bytes = struct.pack('6d', *q_out)
                conn.sendall(send_bytes)
                
        except Exception as e:
            print(f"⚠️ 소켓 통신 오류 또는 연결 해제: {e}")
        finally:
            print("❌ 클라이언트와의 연결이 끊어졌습니다. 재대기 중...")
            try:
                conn.close()
            except:
                pass

# 백그라운드 스레드로 소켓 서버 실행
threading.Thread(target=socket_worker, daemon=True).start()

# =====================================================================
# 3. 메인 시뮬레이션 및 PD 제어 루프
# =====================================================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🎬 MuJoCo GUI 뷰어가 켜졌습니다.")
    
    while viewer.is_running():
        step_start = time.time()
        
        # 외력 초기화
        data.qfrc_applied[:] = 0.0
        
        # 스레드 락을 걸고 목표 각도 및 현재 실제 각도 복사
        with data_lock:
            local_target = np.copy(target_qpos)
            current_qpos = np.array(data.qpos[joint_qpos_adr])
            local_current_vel = np.array(data.qvel[joint_qvel_adr])
            
        # PD 관절 제어 토크 계산
        error = local_target - current_qpos
        ctrl_torque = kp * error - kd * local_current_vel
        
        # 물리 엔진에 제어 토크 인가
        data.qfrc_applied[joint_qvel_adr] = ctrl_torque
        
        # 물리 엔진 진행 및 뷰어 싱크
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 2ms 주기 (500Hz) 유지
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
