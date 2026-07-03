import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons

# =====================================================================
# 1. 로봇 기구학 파라미터 (Doosan m1013 기반)
# =====================================================================
D1 = 0.1525   # 베이스 높이 (Base to Shoulder)
D2 = 0.0345   # 어깨 가로 오프셋 (Shoulder Offset)
A2 = 0.6200   # 위팔 길이 (Upper Arm)
D4 = 0.5590   # 아래팔 길이 (Forearm: Elbow to Wrist Center)
D6 = 0.1210   # 손목 중심부터 TCP(툴 끝점)까지의 거리 (Wrist to Tool offset)

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

def rpy_to_rot(roll, pitch, yaw):
    """Roll-Pitch-Yaw (X-Y-Z 고정축 회전)를 회전 행렬로 변환"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

# =====================================================================
# 3. 순방향 기구학 (Forward Kinematics)
# =====================================================================
def forward_kinematics(q):
    """
    6개 관절 각도 q = [q1, q2, q3, q4, q5, q6] (라디안)에 대해
    각 관절의 3D 좌표와 최종 TCP 포즈를 계산합니다.
    """
    q1, q2, q3, q4, q5, q6 = q
    
    # 각 조인트 프레임 간 변환 행렬 정의
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
    
    # 누적 좌표계 계산
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45
    T06 = T05 @ T56
    T0_tool = T06 @ T6_tool
    
    # 각 링크 조인트의 3차원 위치 추출 (시각화 용도)
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = T01[:3, 3]
    p2 = T02[:3, 3]
    p3 = T03[:3, 3]
    p_wc = T04[:3, 3]  # 4,5,6번 축이 만나는 손목 중심점
    p_tool = T0_tool[:3, 3] # 최종 TCP
    
    joint_positions = [p0, p1, p2, p3, p_wc, p_tool]
    
    return T0_tool, joint_positions

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
    # TCP 위치에서 툴 길이 D6만큼 접근 벡터(Z축) 반대 방향으로 들어옴
    a = target_rot[:, 2]  # Approach vector (3rd column of rotation matrix)
    p_wc = target_pos - D6 * a
    x_wc, y_wc, z_wc = p_wc
    
    # Step 2: 1번 관절 각도 (q1) 구하기
    r_xy = np.sqrt(x_wc**2 + y_wc**2)
    if r_xy < D2:
        # 손목 중심점이 어깨 오프셋 실린더 내부에 있어 해가 존재하지 않음
        return [None] * 8
        
    offset_angle = np.arctan2(D2, np.sqrt(r_xy**2 - D2**2))
    base_angle = np.arctan2(y_wc, x_wc)
    
    # q1의 두 가지 분기 (Left / Right shoulder)
    q1_candidates = [
        base_angle - offset_angle,         # Left shoulder
        base_angle + offset_angle + np.pi  # Right shoulder
    ]
    
    for s_idx, q1 in enumerate(q1_candidates):
        # 각도를 [-pi, pi] 범위로 정규화
        q1 = np.arctan2(np.sin(q1), np.cos(q1))
        
        # 1번 각도 평면 상의 좌표계 변환
        R_plane = x_wc * np.cos(q1) + y_wc * np.sin(q1)
        Z_plane = z_wc - D1
        
        # 가상 직선 s의 제곱 거리 계산
        s_sq = R_plane**2 + Z_plane**2
        s = np.sqrt(s_sq)
        
        # 기하학적 도달 가능 여부 체크
        if s > (A2 + D4) or s < np.abs(A2 - D4):
            # 이 방향으로는 팔을 뻗어도 닿을 수 없음
            solutions.extend([None, None, None, None])
            continue
            
        # Step 3: 3번 관절 각도 (q3) 계산 (제2코사인 법칙)
        cos_q3 = (s_sq - A2**2 - D4**2) / (2 * A2 * D4)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        
        # q3의 두 가지 분기 (Elbow Up / Down)
        sin_q3_candidates = [
            np.sqrt(1.0 - cos_q3**2),  # Elbow Up (양수 사인)
            -np.sqrt(1.0 - cos_q3**2)  # Elbow Down (음수 사인)
        ]
        
        for e_idx, sin_q3 in enumerate(sin_q3_candidates):
            q3 = np.arctan2(sin_q3, cos_q3)
            
            # Step 4: 2번 관절 각도 (q2) 계산
            k1 = A2 + D4 * cos_q3
            k2 = D4 * sin_q3
            
            q2 = np.arctan2(k1 * R_plane - k2 * Z_plane, k2 * R_plane + k1 * Z_plane)
            
            # Step 5: 손목 회전 행렬 계산 및 q4, q5, q6 오일러 각 추출
            # 0번에서 3번까지의 회전 행렬 구하기
            R01 = rot_z(q1)
            R12 = rot_y(q2)
            R23 = rot_y(q3)
            R03 = R01 @ R12 @ R23
            
            # 손목 자체의 회전 행렬
            R36 = R03.T @ target_rot
            
            r11, r12, r13 = R36[0, 0], R36[0, 1], R36[0, 2]
            r21, r22, r23 = R36[1, 0], R36[1, 1], R36[1, 2]
            r31, r32, r33 = R36[2, 0], R36[2, 1], R36[2, 2]
            
            # q5의 두 가지 분기 (Wrist Flip / No-Flip)
            # 특이점 케이스 체크 (|r33| = 1)
            if np.abs(r33) >= 1.0 - 1e-7:
                # Gimbal Lock 특이점 상황: q4와 q6가 동일 축 정렬
                q5 = 0.0 if r33 > 0 else np.pi
                q4 = 0.0 # 임의 고정
                if r33 > 0:
                    q6 = np.arctan2(r21, r11)
                else:
                    q6 = -np.arctan2(-r21, -r11)
                
                # 특이점에서는 Flip 분기 구분이 크게 무의미하므로 두 분기에 동일 복사
                solutions.append(np.array([q1, q2, q3, q4, q5, q6]))
                solutions.append(np.array([q1, q2, q3, q4, q5, q6]))
            else:
                # 일반적인 상황
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
# 5. 대화형 Matplotlib 3D 시각화 구동
# =====================================================================
class RobotVisualizer:
    def __init__(self):
        # 초기 포즈 설정
        self.target_pos = np.array([0.5, 0.3, 0.4])
        self.roll, self.pitch, self.yaw = np.radians(0), np.radians(0), np.radians(0)
        
        # 선택할 해(Solution) 분기 인덱스 (기본값: Left, Up, No-Flip = 0)
        self.shoulder_idx = 0  # 0: Left, 1: Right
        self.elbow_idx = 0     # 0: Up, 1: Down
        self.wrist_idx = 0     # 0: No-Flip, 1: Flip
        
        # 레이아웃 생성
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title("6-Axis Robot Analytical IK 3D Visualizer")
        
        # 3D 축 (로봇 뷰어)
        self.ax = self.fig.add_subplot(121, projection='3d')
        
        # 컨트롤러 축 분배
        self.setup_widgets()
        
        # 초기 그리기 실행
        self.update_plot()
        
    def setup_widgets(self):
        # 슬라이더 영역 정의
        # 위치 슬라이더
        self.ax_x = self.fig.add_axes([0.58, 0.82, 0.35, 0.03])
        self.ax_y = self.fig.add_axes([0.58, 0.77, 0.35, 0.03])
        self.ax_z = self.fig.add_axes([0.58, 0.72, 0.35, 0.03])
        
        # 오리엔테이션 슬라이더 (도 단위)
        self.ax_roll = self.fig.add_axes([0.58, 0.62, 0.35, 0.03])
        self.ax_pitch = self.fig.add_axes([0.58, 0.57, 0.35, 0.03])
        self.ax_yaw = self.fig.add_axes([0.58, 0.52, 0.35, 0.03])
        
        # 슬라이더 객체 생성
        self.slider_x = Slider(self.ax_x, 'Target X (m)', 0.1, 1.2, valinit=self.target_pos[0])
        self.slider_y = Slider(self.ax_y, 'Target Y (m)', -0.8, 0.8, valinit=self.target_pos[1])
        self.slider_z = Slider(self.ax_z, 'Target Z (m)', -0.2, 1.2, valinit=self.target_pos[2])
        
        self.slider_roll = Slider(self.ax_roll, 'Roll (deg)', -180.0, 180.0, valinit=0.0)
        self.slider_pitch = Slider(self.ax_pitch, 'Pitch (deg)', -180.0, 180.0, valinit=0.0)
        self.slider_yaw = Slider(self.ax_yaw, 'Yaw (deg)', -180.0, 180.0, valinit=0.0)
        
        # 해의 조합 라디오 버튼 영역
        self.ax_rad1 = self.fig.add_axes([0.58, 0.32, 0.11, 0.12])
        self.ax_rad2 = self.fig.add_axes([0.70, 0.32, 0.11, 0.12])
        self.ax_rad3 = self.fig.add_axes([0.82, 0.32, 0.11, 0.12])
        
        self.radio_shoulder = RadioButtons(self.ax_rad1, ('Left', 'Right'))
        self.radio_elbow = RadioButtons(self.ax_rad2, ('Up', 'Down'))
        self.radio_wrist = RadioButtons(self.ax_rad3, ('No-Flip', 'Flip'))
        
        # 정보 출력용 텍스트 필드 생성
        self.text_axes = self.fig.add_axes([0.58, 0.05, 0.35, 0.22])
        self.text_axes.axis('off')
        self.info_text = self.text_axes.text(0.01, 0.99, "", fontsize=9.5, family='monospace', verticalalignment='top')
        
        # 이벤트 바인딩
        self.slider_x.on_changed(self.on_slider_update)
        self.slider_y.on_changed(self.on_slider_update)
        self.slider_z.on_changed(self.on_slider_update)
        self.slider_roll.on_changed(self.on_slider_update)
        self.slider_pitch.on_changed(self.on_slider_update)
        self.slider_yaw.on_changed(self.on_slider_update)
        
        self.radio_shoulder.on_clicked(self.on_radio_update)
        self.radio_elbow.on_clicked(self.on_radio_update)
        self.radio_wrist.on_clicked(self.on_radio_update)

    def on_slider_update(self, val):
        self.target_pos[0] = self.slider_x.val
        self.target_pos[1] = self.slider_y.val
        self.target_pos[2] = self.slider_z.val
        self.roll = np.radians(self.slider_roll.val)
        self.pitch = np.radians(self.slider_pitch.val)
        self.yaw = np.radians(self.slider_yaw.val)
        self.update_plot()
        
    def on_radio_update(self, label):
        # 라디오 버튼 상태 읽기
        s_label = self.radio_shoulder.value_selected
        e_label = self.radio_elbow.value_selected
        w_label = self.radio_wrist.value_selected
        
        self.shoulder_idx = 0 if s_label == 'Left' else 1
        self.elbow_idx = 0 if e_label == 'Up' else 1
        self.wrist_idx = 0 if w_label == 'No-Flip' else 1
        
        self.update_plot()
        
    def update_plot(self):
        # 3D 뷰어 비우기
        self.ax.clear()
        self.ax.set_xlabel('X Axis (m)', fontsize=10)
        self.ax.set_ylabel('Y Axis (m)', fontsize=10)
        self.ax.set_zlabel('Z Axis (m)', fontsize=10)
        self.ax.set_xlim3d([-0.8, 1.0])
        self.ax.set_ylim3d([-0.8, 0.8])
        self.ax.set_zlim3d([-0.1, 1.2])
        self.ax.grid(True, linestyle=':', alpha=0.5)
        
        # 목표 회전행렬 계산
        target_rot = rpy_to_rot(self.roll, self.pitch, self.yaw)
        
        # 역운동학 풀기
        solutions = analytical_ik(self.target_pos, target_rot)
        
        # 8가지 해 인덱스 매핑 (이진 트리 구조)
        sol_idx = self.shoulder_idx * 4 + self.elbow_idx * 2 + self.wrist_idx
        q_sol = solutions[sol_idx]
        
        # 시각화 용 타겟 프레임 그리기
        self.draw_coordinate_frame(self.target_pos, target_rot, scale=0.15)
        
        # 상태 확인 및 정보 텍스트 생성
        info = []
        info.append("=== 6-Axis Articulated Robot Status ===")
        info.append(f"Target Pos : X={self.target_pos[0]:.3f}, Y={self.target_pos[1]:.3f}, Z={self.target_pos[2]:.3f}")
        info.append(f"Target Ori : R={np.degrees(self.roll):.1f}, P={np.degrees(self.pitch):.1f}, Y={np.degrees(self.yaw):.1f}")
        info.append(f"Selected Branch : [{self.radio_shoulder.value_selected} Shoulder, {self.radio_elbow.value_selected} Elbow, {self.radio_wrist.value_selected} Wrist] (Sol #{sol_idx + 1})")
        info.append("-" * 43)
        
        if q_sol is None:
            info.append("❌ IK RESULT: Target is UNREACHABLE!")
            info.append("   (Selected branch is out of workspace limits.)")
            self.info_text.set_text("\n".join(info))
            self.info_text.set_color('red')
            self.fig.canvas.draw_idle()
            return
        
        # 역운동학 해가 유효한 경우, 순방향 기구학을 돌려 확인 및 시각화용 노드 좌표 획득
        T_fk, pts = forward_kinematics(q_sol)
        
        # 3D 공간 상에 로봇 뼈대 그리기
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        zs = [float(p[2]) for p in pts]
        
        # 베이스 지지판
        self.ax.plot([0, 0], [0, 0], [-0.1, 0.0], color='#2C3E50', linewidth=10, solid_capstyle='round')
        
        # 로봇 링크 대단 (검은색/파란색 굵은선)
        self.ax.plot(xs[:2], ys[:2], zs[:2], 'o-', color='#34495E', linewidth=6, markersize=8, label='Base Link')
        self.ax.plot(xs[1:3], ys[1:3], zs[1:3], 'o-', color='#34495E', linewidth=5, markersize=8)
        self.ax.plot(xs[2:4], ys[2:4], zs[2:4], 'o-', color='#005588', linewidth=5, markersize=8, label='Upper Arm (A2)')
        self.ax.plot(xs[3:5], ys[3:5], zs[3:5], 'o-', color='#2980B9', linewidth=4, markersize=7, label='Forearm (D4)')
        self.ax.plot(xs[4:6], ys[4:6], zs[4:6], 'o-', color='#D35400', linewidth=3, markersize=6, label='Wrist & Tool')
        
        # 관절 조인트 마킹 강조
        self.ax.scatter([xs[1]], [ys[1]], [zs[1]], color='black', s=80, zorder=10) # Waist
        self.ax.scatter([xs[2]], [ys[2]], [zs[2]], color='#FF9900', s=70, zorder=10) # Shoulder
        self.ax.scatter([xs[3]], [ys[3]], [zs[3]], color='#E74C3C', s=60, zorder=10) # Elbow
        self.ax.scatter([xs[4]], [ys[4]], [zs[4]], color='#9B59B6', s=50, zorder=10) # Wrist Center
        self.ax.scatter([xs[5]], [ys[5]], [zs[5]], color='red', s=60, zorder=10) # TCP (Tool Tip)
        
        # 가상 직각 투영선 점선으로 표기
        p_wc = pts[4]
        self.ax.plot([float(p_wc[0]), float(p_wc[0])], [float(p_wc[1]), float(p_wc[1])], [0.0, float(p_wc[2])], 'k:', alpha=0.4)
        
        # 각 관절 프레임 방향 그리기
        # 베이스 프레임
        self.draw_coordinate_frame([0, 0, 0], np.eye(3), scale=0.1)
        # 손목 중심점 프레임
        R_wc = T_fk[:3, :3] # 편의상 말단자세와 유사한 방향 추출
        self.draw_coordinate_frame(p_wc, R_wc, scale=0.1)
        
        # 수치 검증 오차 계산 (FK 결과와 타겟 포즈 비교)
        pos_err = np.linalg.norm(T_fk[:3, 3] - self.target_pos)
        rot_err = np.linalg.norm(T_fk[:3, :3] - target_rot, 'fro')
        
        # 조인트 값 표시
        info.append("✅ IK RESULT: SUCCESS!")
        info.append(f"q1 : {np.degrees(q_sol[0]):7.2f}° | q4 : {np.degrees(q_sol[3]):7.2f}°")
        info.append(f"q2 : {np.degrees(q_sol[1]):7.2f}° | q5 : {np.degrees(q_sol[4]):7.2f}°")
        info.append(f"q3 : {np.degrees(q_sol[2]):7.2f}° | q6 : {np.degrees(q_sol[5]):7.2f}°")
        info.append("-" * 43)
        info.append(f"FK Validation Error:")
        info.append(f" -> Position Error   : {pos_err * 1e6:.4f} um")
        info.append(f" -> Orientation Error: {rot_err:.4e} (Frobenius)")
        
        self.info_text.set_text("\n".join(info))
        self.info_text.set_color('black')
        
        self.ax.legend(loc='upper left', fontsize=8)
        self.ax.set_title("3D Robot Arm Analytical Configuration", fontsize=11, fontweight='bold')
        self.fig.canvas.draw_idle()

    def draw_coordinate_frame(self, origin, rotation, scale=0.1):
        """3D 공간상에 X(Red), Y(Green), Z(Blue) 직교 프레임을 축으로 렌더링"""
        o = np.array(origin, dtype=float)
        x_axis = o + scale * np.array(rotation[:, 0], dtype=float)
        y_axis = o + scale * np.array(rotation[:, 1], dtype=float)
        z_axis = o + scale * np.array(rotation[:, 2], dtype=float)
        
        # X축 (빨간색)
        self.ax.plot([float(o[0]), float(x_axis[0])], [float(o[1]), float(x_axis[1])], [float(o[2]), float(x_axis[2])], color='red', linewidth=2.5)
        # Y축 (초록색)
        self.ax.plot([float(o[0]), float(y_axis[0])], [float(o[1]), float(y_axis[1])], [float(o[2]), float(y_axis[2])], color='green', linewidth=2.5)
        # Z축 (파란색)
        self.ax.plot([float(o[0]), float(z_axis[0])], [float(o[1]), float(z_axis[1])], [float(o[2]), float(z_axis[2])], color='blue', linewidth=2.5)

if __name__ == "__main__":
    vis = RobotVisualizer()
    plt.show()
