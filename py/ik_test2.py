import numpy as np
import matplotlib.pyplot as plt

# =====================================================================
# [기구학 상수] 두산 로봇 m1013 실제 치수 (m)
# =====================================================================
D1 = 0.1525   # 베이스 높이
D2 = 0.0345   # 어깨 가로 오프셋
A2 = 0.6200   # 위팔 길이
D4 = 0.5590   # 아래팔 길이

def calculate_and_plot_ultimate_ik(x_target, y_target, z_target):
    # --- 역운동학(IK) 수학 연산 과정 ---
    r_xy = np.sqrt(x_target**2 + y_target**2)
    R = np.sqrt(r_xy**2 - D2**2)
    Z = z_target - D1
    s_sq = R**2 + Z**2
    s = np.sqrt(s_sq)
    
    # 제2코사인 법칙
    cos_phi3 = (A2**2 + D4**2 - s_sq) / (2 * A2 * D4)
    sin_phi3 = np.sqrt(1 - cos_phi3**2)
    phi3 = np.arctan2(sin_phi3, cos_phi3)
    theta3 = phi3 - np.pi/2 
    
    # 이중 각도 결합
    alpha = np.arctan2(Z, R)
    beta = np.arctan2(D4 * np.sin(theta3 + np.pi/2), A2 + D4 * np.cos(theta3 + np.pi/2))
    theta2 = np.pi/2 - (alpha + beta) 

    # --- 꼭짓점 좌표 계산 ---
    shoulder = np.array([0.0, 0.0])
    elbow = np.array([A2 * np.sin(theta2), A2 * np.cos(theta2)])
    target = np.array([R, Z])
    
    # --- 삼각비 투영선 수선의 발 좌표 계산 ---
    u_s = target / s 
    proj_dist = A2 * np.cos(beta)
    proj_foot = u_s * proj_dist  

    # =====================================================================
    # [Matplotlib 시각화 도면 레이아웃 설정]
    # =====================================================================
    plt.figure(figsize=(16, 12))
    plt.grid(True, linestyle='--', alpha=0.4)
    
    # 1. 로봇 링크선 및 가상 조준선 (A2, D4, s)
    plt.plot([shoulder[0], elbow[0], target[0]], [shoulder[1], elbow[1], target[1]], 
             'o-', linewidth=5, color='#005588', label='Robot Linkage (A2, D4)', zorder=4)
    plt.plot([shoulder[0], target[0]], [shoulder[1], target[1]], 
             'k--', alpha=0.5, linewidth=2, label=r'Virtual Line ($s$)')
    
    # 2. 삼각비 직각 투영 가이드선 (높이선, 밑변선)
    plt.plot([elbow[0], proj_foot[0]], [elbow[1], proj_foot[1]], 
             color='#E74C3C', linewidth=3, linestyle='-', label=r'Height Line ($D_4 \cdot \sin\phi_3$)', zorder=3)
    plt.plot([shoulder[0], proj_foot[0]], [shoulder[1], proj_foot[1]], 
             color='#2ECC71', linewidth=3, linestyle='-', label=r'Base Line ($A_2 + D_4 \cdot \cos\phi_3$)', zorder=3)

    # 3. 월드 및 로봇 기준선 좌표축 레이아웃
    plt.axhline(0, color='black', linewidth=1.5, alpha=0.6)
    plt.axvline(0, color='black', linewidth=1.5, alpha=0.6)
    plt.plot([target[0], target[0]], [0, target[1]], 'r:', alpha=0.5, linewidth=1.5) # R 투영선
    plt.plot([0, target[0]], [target[1], target[1]], 'r:', alpha=0.5, linewidth=1.5) # Z 투영선
    
    # 4. 관절 조인트 노드 마킹 (시각화 강조)
    plt.scatter(shoulder[0], shoulder[1], color='black', s=250, zorder=5)
    plt.scatter(elbow[0], elbow[1], color='#FF9900', s=200, zorder=5)
    plt.scatter(target[0], target[1], color='red', s=250, zorder=5)
    
    # =====================================================================
    # 5. [수학식의 총망라] 도면 내 수식 정보 박스 배치 (문법 에러 완벽 차단)
    # =====================================================================
    
    # ① 좌측 상단 박스: 2D 평면 변환 공식
    formulas_step1 = (
        r"$\mathbf{1.\ Plain\ Transformation\ (평면\ 변환)}$" + "\n" +
        r"$\bullet\ R = \sqrt{x_{target}^2 + y_{target}^2 - d_2^2} = " + f"{R:.4f}" + r"\ m$" + "\n" +
        r"$\bullet\ Z = z_{target} - d_1 = " + f"{Z:.4f}" + r"\ m$" + "\n" +
        r"$\bullet\ s = \sqrt{R^2 + Z^2} = " + f"{s:.4f}" + r"\ m$"
    )
    plt.text(-0.12, 0.62, formulas_step1, fontsize=11, bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5'))

    # ② 중앙 주황색 박스: Joint 3 제2코사인 법칙과 atan2 우회 공식
    formulas_step2 = (
        r"$\mathbf{2.\ Joint\ 3\ (Elbow)\ Kinematics}$" + "\n" +
        r"$\bullet\ \cos(\phi_3) = \frac{A_2^2 + D_4^2 - s^2}{2 \cdot A_2 \cdot D_4} = " + f"{cos_phi3:.4f}" + r"$" + "\n" +
        r"$\bullet\ \sin(\phi_3) = \sqrt{1 - \cos^2(\phi_3)} = " + f"{sin_phi3:.4f}" + r"$" + "\n" +
        r"$\bullet\ \phi_3 = \text{atan2}(\sin\phi_3, \cos\phi_3) = " + f"{np.degrees(phi3):.2f}" + r"^\circ$" + "\n" +
        r"$\bullet\ \theta_3 = \phi_3 - \frac{\pi}{2} = \mathbf{" + f"{np.degrees(theta3):.2f}" + r"^\circ}$"
    )
    plt.text(elbow[0] - 0.12, elbow[1] + 0.08, formulas_step2, fontsize=11, color='#D35400',
             bbox=dict(facecolor='#FFF2E6', alpha=0.95, edgecolor='#FF9900', boxstyle='round,pad=0.5'))

    # ③ 좌측 하단 파란색 박스: Joint 2 이중각도 결합과 삼각비 역원 회복 공식
    formulas_step3 = (
        r"$\mathbf{3.\ Joint\ 2\ (Shoulder)\ Kinematics}$" + "\n" +
        r"$\bullet\ \alpha = \text{atan2}(Z, R) = " + f"{np.degrees(alpha):.2f}" + r"^\circ\ (조준선\ 각도)$" + "\n" +
        r"$\bullet\ \beta = \text{atan2}(\text{Height}, \text{Base}) = " + f"{np.degrees(beta):.2f}" + r"^\circ\ (들림\ 사잇각)$" + "\n" +
        r"$\quad\ \text{Where,}\ \text{Height} = D_4 \cdot \sin(\theta_3 + \pi/2) = " + f"{D4 * np.sin(theta3 + np.pi/2):.3f}" + r"\ m$" + "\n" +
        r"$\quad\ \phantom{\text{Where,}}\ \text{Base} = A_2 + D_4 \cdot \cos(\theta_3 + \pi/2) = " + f"{A2 + D4 * np.cos(theta3 + np.pi/2):.3f}" + r"\ m$" + "\n" +
        r"$\bullet\ \theta_2 = \frac{\pi}{2} - (\alpha + \beta) = \mathbf{" + f"{np.degrees(theta2):.2f}" + r"^\circ}$"
    )
    plt.text(-0.12, -0.22, formulas_step3, fontsize=11, color='#005588',
             bbox=dict(facecolor='#E6F2FF', alpha=0.95, edgecolor='#005588', boxstyle='round,pad=0.5'))

    # =====================================================================
    # 6. 각 노드 및 뼈대 위에 수학적 기호 라벨 매칭 링킹 (Labeling)
    # =====================================================================
    # 꼭짓점 명칭 표기
    plt.text(shoulder[0] - 0.04, shoulder[1] + 0.03, r"$\mathbf{Shoulder\ J_2\ (0,0)}$", fontsize=10, fontweight='bold')
    plt.text(elbow[0] + 0.02, elbow[1] + 0.02, r"$\mathbf{Elbow\ J_3}$", fontsize=10, color='#D35400', fontweight='bold')
    plt.text(target[0] + 0.02, target[1] - 0.02, r"$\mathbf{Target\ WC\ (R, Z)}$", fontsize=10, color='red', fontweight='bold')

    # 링크 길이 기호 라벨링
    plt.text(elbow[0]/2 - 0.05, elbow[1]/2 + 0.05, r"$A_2 = 0.620m$", color='#005588', rotation=np.degrees(theta2), fontsize=10)
    mid_d4 = (elbow + target) / 2
    plt.text(mid_d4[0] + 0.03, mid_d4[1] + 0.03, r"$D_4 = 0.559m$", color='#005588', fontsize=10)

    # 삼각비 높이/밑변 꼬리표 직접 매칭
    mid_h = (elbow + proj_foot) / 2
    plt.text(mid_h[0] + 0.01, mid_h[1], r"$\mathbf{Height\ (D_4\sin\phi_3)}$", color='#C0392B', fontsize=9, fontweight='bold')
    mid_b = (shoulder + proj_foot) / 2
    plt.text(mid_b[0] - 0.05, mid_b[1] - 0.04, r"$\mathbf{Base\ (A_2 + D_4\cos\phi_3)}$", color='#27AE60', fontsize=9, fontweight='bold')

    # 7. 그래프 표시 옵션 튜닝
    plt.title('Doosan m1013 Robot Kinematics: Mathematical Blueprint', fontsize=16, fontweight='bold', pad=20)
    plt.xlabel('Horizontal Distance (R) [meter]', fontsize=12)
    plt.ylabel('Vertical Height (Z) [meter]', fontsize=12)
    
    # 1:1 완벽한 기하 비율 강제 및 축 한계 정의
    plt.axis('equal')
    plt.xlim(-0.2, 1.0)
    plt.ylim(-0.28, 0.85)
    plt.legend(loc='upper right', fontsize=11)
    
    plt.show()

# --- 실행 테스팅 ---
if __name__ == "__main__":
    calculate_and_plot_ultimate_ik(x_target=0.5, y_target=0.5, z_target=0.4)
