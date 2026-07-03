import mujoco
import mujoco.viewer
import numpy as np
import time

# [두산 로봇 고정 하드웨어 치수]
D2 = 0.0345  

def calculate_doosan_theta1(x_target, y_target):
    """
    두산 로봇 m1013의 실제 하드웨어 축 배치에 맞춘 1번 관절(허리) 역운동학
    """
    r_xy = np.sqrt(x_target**2 + y_target**2)
    if r_xy < D2:
        raise ValueError("과녁이 로봇 몸체 오프셋 안쪽에 있어 조준선이 나오지 않습니다.")
        
    base_angle = np.arctan2(y_target, x_target)
    offset_angle = np.arctan2(D2, np.sqrt(r_xy**2 - D2**2))
    
    # 어깨 오프셋이 왼쪽에 있으므로 오차 각도를 더해줍니다(+)
    theta1 = base_angle + offset_angle
    
    # 각도 범위 정렬 (-pi ~ pi)
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
    return theta1

# 1. 실제 사용하시는 두산 로봇 XML/URDF 파일 경로 지정
xml_path = '/home/uon/code/robot_control/gui/m1013/m1013_mujoco.xml'

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# =====================================================================
# 2. 테스트할 목표 과녁 위치 설정 (X=1m, Y=1m 지점 확인용)
# =====================================================================
target_x = 1
target_y = 1
target_z = 0.3  # 눈으로 잘 보이게 바닥에서 30cm 띄움

try:
    target_theta1 = calculate_doosan_theta1(target_x, target_y)
    print("\n" + "="*50)
    print(f"🎯 목표 지점 설정: X = {target_x}m, Y = {target_y}m")
    print(f"⚙️ 계산된 두산 1번 축 각도: {target_theta1:.4f} rad ({np.degrees(target_theta1):.2f} deg)")
    print("="*50 + "\n")
except ValueError as e:
    print(f"IK 계산 에러: {e}")
    exit()

# 4. MuJoCo passive 뷰어 구동
with mujoco.viewer.launch_passive(model, data) as viewer:
    mujoco.mj_resetData(model, data)
    
    # 초기 상태에서 모든 모터 입력은 0으로 세팅
    data.ctrl[:] = 0.0
    
    while viewer.is_running():
        step_start = time.time()
        
        # [위치 제어] 1번 관절 액추에이터(index 0)에 두산 보정 각도 주입
        data.ctrl[0] = target_theta1
        
        # 물리 엔진 구동
        mujoco.mj_step(model, data)
        
        # =====================================================================
        # ★ [실시간 렌더링 부가 코드] 가상 공간에 빨간색 과녁 공 실시간으로 그리기
        # =====================================================================
        # 가상의 붉은 구체 오브젝트 속성 정의
        geom_id = 9999  # 임의의 가상 지오메트리 ID
        viewer.user_scn.ngeom = 1
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.05, 0.05, 0.05],                    # 지름 10cm의 공
            pos=[target_x, target_y, target_z],        # X=1.0, Y=1.0 지점
            mat=np.eye(3).flatten(),                    # 회전 행렬 (기본값)
            rgba=[1, 0, 0, 1]                           # 빨간색 (불투명)
        )
        
        viewer.sync()
        
        # 시뮬레이션 타임스텝 동기화
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
