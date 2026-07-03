import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 로봇 위팔/아래팔 길이 상수
A2 = 0.620  # 위팔
D4 = 0.559  # 아래팔

# 각도 아크(호)를 그리기 위한 헬퍼 함수
def draw_arc(ax, start_angle, end_angle, radius, color, label):
    # 각도 범위에 대해 부드러운 호 생성
    theta = np.linspace(start_angle, end_angle, 50)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    ax.plot(x, y, color=color, linewidth=2, linestyle='-')
    
    # 아크 중앙 부근에 각도 명칭 라벨 텍스트 배치
    mid_theta = (start_angle + end_angle) / 2
    tx = float((radius + 0.05) * np.cos(mid_theta))
    ty = float((radius + 0.05) * np.sin(mid_theta))
    ax.text(tx, ty, label, color=color, fontsize=10, ha='center', va='center', fontweight='bold')

def update(val):
    # 슬라이더에서 타겟의 R(가로), Z(세로) 좌표 획득
    R_target = s_r.val
    Z_target = s_z.val
    
    # 1. 기하 계산 시작
    s_sq = R_target**2 + Z_target**2
    s = np.sqrt(s_sq)
    
    # 기하 도달 가능 검증
    if s > (A2 + D4) or s < np.abs(A2 - D4):
        ax.clear()
        ax.text(0.1, 0.5, "❌ OUT OF WORKSPACE\n(도달할 수 없는 지점입니다)", 
                color='red', fontsize=14, fontweight='bold', ha='center', va='center')
        ax.set_xlim(-0.1, 1.0)
        ax.set_ylim(-0.2, 1.0)
        fig.canvas.draw_idle()
        return
        
    # Elbow 각도 계산 (theta3)
    cos_q3 = (s_sq - A2**2 - D4**2) / (2 * A2 * D4)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    theta3 = np.arccos(cos_q3) # Elbow Up 해선택
    
    # Shoulder 각도 계산 (theta2)
    k1 = A2 + D4 * cos_q3
    k2 = D4 * np.sin(theta3)
    theta2 = np.arctan2(k1 * R_target - k2 * Z_target, k2 * R_target + k1 * Z_target)
    
    # beta_plane 계산
    beta_plane = np.arctan2(k2, k1)
    
    # alpha_plane 계산
    alpha_plane = np.arctan2(Z_target, R_target)
    
    # 각도 라디안을 도(Degree) 단위로 변환
    t2_deg = np.degrees(theta2)
    beta_deg = np.degrees(beta_plane)
    alpha_deg = np.degrees(alpha_plane)
    total_sum = t2_deg + beta_deg + alpha_deg
    
    # 2. 그래픽 좌표 설정
    p_origin = np.array([0.0, 0.0])
    p_elbow = np.array([A2 * np.sin(theta2), A2 * np.cos(theta2)])
    p_wrist = np.array([R_target, Z_target])
    
    # 3. 드로잉 업데이트
    ax.clear()
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # 기준축 좌표선 그리기 (Z축: 세로, R축: 가로)
    ax.axhline(0, color='black', linewidth=1.2, alpha=0.7)
    ax.axvline(0, color='black', linewidth=1.2, alpha=0.7)
    
    # 기준 Z축(수직선), R축(수평선) 가이드 점선
    ax.plot([0.0, 0.0], [0.0, 0.9], 'k:', alpha=0.5, linewidth=1.5)
    ax.plot([0.0, 0.9], [0.0, 0.0], 'k:', alpha=0.5, linewidth=1.5)
    
    # 조준선 (어깨 -> 손목)
    ax.plot([0.0, R_target], [0.0, Z_target], 'r--', linewidth=2, label='Target Line (s)')
    
    # 로봇 위팔 (어깨 -> 팔꿈치)
    ax.plot([0.0, p_elbow[0]], [0.0, p_elbow[1]], 'o-', color='#2980B9', linewidth=5, markersize=10, label='Upper Arm (a2)')
    
    # 로봇 아래팔 (팔꿈치 -> 손목)
    ax.plot([p_elbow[0], R_target], [p_elbow[1], Z_target], 'o-', color='#27AE60', linewidth=5, markersize=10, label='Forearm (d4)')
    
    # 각 조인트 라벨 마커 (float 캐스팅 처리 적용)
    ax.scatter(0, 0, color='black', s=150, zorder=5)
    ax.text(-0.02, -0.05, "Shoulder (0,0)", fontsize=9, fontweight='bold', ha='right')
    ax.scatter(float(p_elbow[0]), float(p_elbow[1]), color='#FF9900', s=120, zorder=5)
    ax.scatter(R_target, Z_target, color='red', s=150, zorder=5)
    ax.text(float(R_target) + 0.02, float(Z_target), f"Wrist Center ({R_target:.2f}, {Z_target:.2f})", color='red', fontsize=9, fontweight='bold')
    
    # -----------------------------------------------------------------
    # 각도 아크 시각화 (기울기 순으로 호 덧칠)
    # Z축(90도 = pi/2)에서 출발하여 아래로 차례로 내려옴
    # -----------------------------------------------------------------
    # 1) theta2 아크 (Z축에서 위팔선까지)
    # Z축은 수학적으로 90도(pi/2) 방향이고, 위팔선 방향은 (pi/2 - theta2) 입니다.
    z_rad = np.pi / 2
    u_rad = z_rad - theta2
    draw_arc(ax, u_rad, z_rad, radius=0.20, color='#9B59B6', label=rf"$\theta_2$ ({t2_deg:.1f}°)")
    
    # 2) beta 아크 (위팔선에서 조준선까지)
    # 조준선 방향은 alpha_plane 입니다.
    # 위팔선(u_rad)에서 조준선(alpha_plane)까지의 사잇각
    draw_arc(ax, alpha_plane, u_rad, radius=0.32, color='#E67E22', label=rf"$\beta_{{plane}}$ ({beta_deg:.1f}°)")
    
    # 3) alpha 아크 (조준선에서 R축까지)
    # R축은 0도 방향이고, 조준선은 alpha_plane 입니다.
    draw_arc(ax, 0.0, alpha_plane, radius=0.45, color='#E74C3C', label=rf"$\alpha_{{plane}}$ ({alpha_deg:.1f}°)")
    
    # 4) 90도(pi/2) 직각 표시 박스 (원점 위치)
    square_box = plt.Rectangle((0, 0), 0.05, 0.05, facecolor='none', edgecolor='black', linewidth=1.5, alpha=0.7)
    ax.add_patch(square_box)
    
    # 타이틀 및 수식 박스 갱신
    title_str = (
        r"$\theta_2 + \beta_{plane} + \alpha_{plane} = \mathbf{" + f"{total_sum:.1f}" + r"^\circ} \equiv 90^\circ$ (Constant)"
    )
    ax.set_title(title_str, fontsize=13, fontweight='bold', pad=15, color='#2C3E50')
    
    # 정보 텍스트박스
    info_box = (
        f"1. Theta2 (purple) : {t2_deg:6.1f}°  [Angle from Z-axis to Upper Arm]\n"
        f"2. Beta   (orange) : {beta_deg:6.1f}°  [Angle between Upper Arm and Target Line]\n"
        f"3. Alpha  (red)    : {alpha_deg:6.1f}°  [Angle from Target Line to R-axis]\n"
        f"--------------------------------------------------\n"
        f"👉 Sum: {t2_deg:.1f}° + {beta_deg:.1f}° + {alpha_deg:.1f}° = {total_sum:.1f}° (Exactly 90°)"
    )
    ax.text(0.02, 0.95, info_box, transform=ax.transAxes, fontsize=10, family='monospace',
            verticalalignment='top', bbox=dict(facecolor='white', alpha=0.9, edgecolor='gray', boxstyle='round,pad=0.5'))
    
    ax.set_xlabel("Plane R Axis (Horizontal) [m]")
    ax.set_ylabel("Plane Z Axis (Vertical) [m]")
    ax.set_aspect('equal')
    ax.set_xlim(-0.05, 1.0)
    ax.set_ylim(-0.05, 1.0)
    ax.legend(loc='upper right', fontsize=9)
    fig.canvas.draw_idle()

# =====================================================================
# 메인 윈도우 생성 및 레이아웃 배치
# =====================================================================
fig, ax = plt.subplots(figsize=(9, 9))
plt.subplots_adjust(bottom=0.20)

# 슬라이더 생성 (R: 가로거리, Z: 세로높이)
ax_r = plt.axes([0.15, 0.10, 0.70, 0.03])
ax_z = plt.axes([0.15, 0.05, 0.70, 0.03])

s_r = Slider(ax_r, 'Wrist R (m)', 0.1, 0.9, valinit=0.5)
s_z = Slider(ax_z, 'Wrist Z (m)', 0.1, 0.9, valinit=0.5)

# 이벤트 콜백 연동
s_r.on_changed(update)
s_z.on_changed(update)

# 초기 그리기 실행
update(None)

plt.show()
