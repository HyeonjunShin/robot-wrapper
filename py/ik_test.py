import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 1. 로봇 하드웨어 고정 상수 (어깨 오프셋)
# 시각적으로 삼각형이 더 잘 보이도록 오프셋 크기를 약간 키웠습니다 (0.08m = 8cm)
D2 = 0.08  

def update(val):
    # 슬라이더로부터 목적지 P_wc의 X, Y 좌표 가져오기
    x_wc = s_x.val
    y_wc = s_y.val
    
    # 수평 거리 r_xy 및 기본 각도 alpha 계산
    r_xy = np.sqrt(x_wc**2 + y_wc**2)
    
    if r_xy < D2:
        text_info.set_text("목표 도달 불가능 (특이점 영역)")
        fig.canvas.draw_idle()
        return
    else:
        text_info.set_text("")
        
    alpha = np.arctan2(y_wc, x_wc)
    beta = np.arctan2(D2, np.sqrt(r_xy**2 - D2**2))
    theta1 = alpha - beta
    
    # 가상의 직각삼각형 밑변 길이 계산 (피타고라스)
    base_length = np.sqrt(r_xy**2 - D2**2)
    
    # --- 가상 삼각형의 세 꼭짓점 좌표 계산 ---
    # 1. 원점 (0,0)
    p_origin = np.array([0, 0])
    # 2. 가상의 수직 교점 (모터 축선방향으로 밑변 길이만큼 간 점)
    p_intersect = np.array([base_length * np.cos(theta1), base_length * np.sin(theta1)])
    # 3. 목적지 P_wc
    p_target = np.array([x_wc, y_wc])
    
    # --- 그래픽 업데이트 ---
    ax.clear()
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    
    # [1] 가상의 직각삼각형 내부 채우기 (노란색 영역)
    triangle_pts = np.vstack([p_origin, p_intersect, p_target])
    poly = plt.Polygon(triangle_pts, facecolor='yellow', alpha=0.2, edgecolor='orange', linestyle='--', label='Virtual Right Triangle')
    ax.add_patch(poly)
    
    # [2] 기준점 및 목적지 마커
    ax.plot(0, 0, 'ko', markersize=10, label='Motor Center (0,0)')
    ax.plot(x_wc, y_wc, 'ro', markersize=8, label='Target (P_wc)')
    
    # [3] 순수 목적지 방향선 (Alpha 선 - 빗변)
    ax.plot([0, x_wc], [0, y_wc], 'r--', alpha=0.6, label='Target Dist (r_xy, Hypotenuse)')
    
    # [4] 모터 축 선 (Theta1 방향 - 밑변이 놓인 선)
    ax.plot([0, p_intersect[0]], [0, p_intersect[1]], 'b-', linewidth=2, label='Motor Axis (Theta1, Base)')
    
    # [5] 가상의 높이 선 (D2 수선발)
    ax.plot([p_intersect[0], x_wc], [p_intersect[1], y_wc], 'm-', linewidth=2, label='Virtual Height (D2 Offset)')
    
    # [6] 실제 로봇 팔 (어깨 관절 위치에서 출발하는 선)
    # 실제 어깨 관절 위치 계산
    x_shoulder = D2 * np.sin(theta1)
    y_shoulder = -D2 * np.cos(theta1)
    ax.plot([x_shoulder, x_wc], [y_shoulder, y_wc], 'g-', linewidth=3, label='Real Robot Arm')
    # 어깨 오프셋 링크 구조 표현
    ax.plot([0, x_shoulder], [0, y_shoulder], 'g--', linewidth=1.5)

    # 레이블 및 타이틀
    ax.legend(loc='upper left', fontsize=8)
    ax.set_title(f"Alpha: {np.degrees(alpha):.1f}° | Beta: {np.degrees(beta):.1f}°\nTheta1 (Alpha - Beta): {np.degrees(theta1):.1f}°", fontsize=10)
    
    fig.canvas.draw_idle()

# --- 메인 윈도우 구성 ---
fig, ax = plt.subplots(figsize=(7, 7))
plt.subplots_adjust(bottom=0.25)

# 초기 목적지 위치
initial_x = 1
initial_y = 1

# 슬라이더 추가
ax_x = plt.axes([0.15, 0.1, 0.7, 0.03])
ax_y = plt.axes([0.15, 0.05, 0.7, 0.03])
s_x = Slider(ax_x, 'Target X', -0.4, 0.4, valinit=initial_x)
s_y = Slider(ax_y, 'Target Y', -0.4, 0.4, valinit=initial_y)

text_info = ax.text(-0.45, -0.42, "", color='red', fontsize=12)

s_x.on_changed(update)
s_y.on_changed(update)

update(None)
plt.show()