import numpy as np
import matplotlib.pyplot as plt

# =====================================================================
# 1. 로봇 기구학 파라미터 (Doosan m1013 기반 고정 상수)
# =====================================================================
D1 = 0.1525   # 베이스 높이
D2 = 0.0345   # 어깨 가로 오프셋
A2 = 0.6200   # 위팔 길이
D4 = 0.5590   # 아래팔 길이 (Elbow to Wrist Center)
D6 = 0.1210   # 툴 오프셋

def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def plot_step_by_step(x_target, y_target, z_target, r_deg, p_deg, y_deg):
    # --- 1. 목표 포즈 정의 ---
    target_pos = np.array([x_target, y_target, z_target])
    target_rot = rot_z(np.radians(y_deg)) @ rot_y(np.radians(p_deg)) @ rot_x(np.radians(r_deg))
    
    # --- 2. 역운동학 수학 계산 ---
    # 손목 중심점 (Wrist Center) 계산
    a = target_rot[:, 2]  # 접근 벡터
    p_wc = target_pos - D6 * a
    x_wc, y_wc, z_wc = p_wc
    
    # [Step 1] theta1 계산 (Left-shoulder 기준)
    r_xy = np.sqrt(x_wc**2 + y_wc**2)
    offset_angle = np.arctan2(D2, np.sqrt(r_xy**2 - D2**2))
    base_angle = np.arctan2(y_wc, x_wc)
    theta1 = base_angle - offset_angle
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
    
    # [Step 2] theta2, theta3 계산 (Elbow Up 기준)
    R_plane = x_wc * np.cos(theta1) + y_wc * np.sin(theta1)
    Z_plane = z_wc - D1
    s_sq = R_plane**2 + Z_plane**2
    s = np.sqrt(s_sq)
    
    cos_q3 = (s_sq - A2**2 - D4**2) / (2 * A2 * D4)
    sin_q3 = np.sqrt(1.0 - cos_q3**2)
    theta3 = np.arctan2(sin_q3, cos_q3)
    
    k1 = A2 + D4 * cos_q3
    k2 = D4 * sin_q3
    theta2 = np.arctan2(k1 * R_plane - k2 * Z_plane, k2 * R_plane + k1 * Z_plane)
    
    # --- 3. 순방향 기구학을 통한 점 좌표 획득 ---
    # 각 조인트 위치 계산
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, D1])
    
    # 어깨 위치
    R01 = rot_z(theta1)
    p2 = p1 + R01 @ np.array([0.0, D2, 0.0])
    
    # 팔꿈치 위치
    R12 = rot_y(theta2)
    p3 = p2 + R01 @ R12 @ np.array([0.0, 0.0, A2])
    
    # 손목 중심 위치 (실제 연산 검증용)
    R23 = rot_y(theta3)
    p4 = p3 + R01 @ R12 @ R23 @ np.array([0.0, 0.0, D4])
    
    # =====================================================================
    # [Matplotlib 시각화 레이아웃 생성]
    # =====================================================================
    fig = plt.figure(figsize=(18, 6.5))
    fig.canvas.manager.set_window_title("6-Axis Analytical IK Step-by-Step Explanation")
    
    # -----------------------------------------------------------------
    # Subplot 1: XY 평면상의 1번 관절각 (theta1) 계산 가이드
    # -----------------------------------------------------------------
    ax1 = fig.add_subplot(131)
    ax1.grid(True, linestyle='--', alpha=0.5)
    
    # 원점, 손목 중심점, 어깨 위치 투영
    ax1.plot([0.0, x_wc], [0.0, y_wc], 'r--', label='Target Line (r_xy)')
    ax1.plot([0.0, -D2 * np.sin(theta1)], [0.0, D2 * np.cos(theta1)], 'g-', linewidth=3, label='Shoulder Offset (d2)')
    ax1.plot([-D2 * np.sin(theta1), x_wc], [D2 * np.cos(theta1), y_wc], 'b-', linewidth=3, label='Arm Plane Vector (R)')
    
    # 점 표시
    ax1.scatter(0.0, 0.0, color='black', s=150, zorder=5, label='Base Center')
    ax1.scatter(-D2 * np.sin(theta1), D2 * np.cos(theta1), color='#FF9900', s=100, zorder=5, label='Shoulder Joint')
    ax1.scatter(x_wc, y_wc, color='red', s=150, zorder=5, label='Wrist Center (P_wc)')
    
    # 가이드 선 및 텍스트 매핑 (float 캐스팅 처리 완료)
    ax1.text(0.0, -0.08, "Base (0,0)", fontsize=9, fontweight='bold', ha='center')
    ax1.text(float(x_wc) + 0.02, float(y_wc) + 0.02, f"P_wc\n({x_wc:.3f}, {y_wc:.3f})", color='red', fontsize=9, fontweight='bold')
    
    # 각도 아크 가이드 텍스트
    ax1.text(float(x_wc)/2 - 0.05, float(y_wc)/2 + 0.05, "r_xy", color='red', fontsize=10)
    ax1.text(-float(D2 * np.sin(theta1))/2 - 0.04, float(D2 * np.cos(theta1))/2 + 0.02, "d2", color='green', fontsize=10)
    
    # 수학식 텍스트 박스 배치
    formula1 = (
        r"$\mathbf{Step\ 1:\ Base\ Waist\ Angle\ (\theta_1)}$" + "\n" +
        r"$\bullet\ r_{xy} = \sqrt{x_{wc}^2 + y_{wc}^2} = " + rf"{r_xy:.4f}" + r"\ m$" + "\n" +
        r"$\bullet\ \alpha = \text{atan2}(y_{wc}, x_{wc}) = " + rf"{np.degrees(base_angle):.1f}^\circ$" + "\n" +
        r"$\bullet\ \beta = \text{atan2}(d_2, \sqrt{r_{xy}^2 - d_2^2}) = " + rf"{np.degrees(offset_angle):.1f}^\circ$" + "\n" +
        r"$\bullet\ \theta_1 = \alpha - \beta = \mathbf{" + rf"{np.degrees(theta1):.1f}^\circ" + "}$"
    )
    ax1.text(0.05, 0.05, formula1, transform=ax1.transAxes, fontsize=10,
             bbox=dict(facecolor='#E8F8F5', alpha=0.9, edgecolor='#16A085', boxstyle='round,pad=0.5'))
    
    ax1.set_title("1. XY-Plane Projection & Waist Angle", fontsize=12, fontweight='bold')
    ax1.set_xlabel("X Axis (m)")
    ax1.set_ylabel("Y Axis (m)")
    ax1.set_aspect('equal')
    ax1.set_xlim([-0.2, 0.9])
    ax1.set_ylim([-0.2, 0.9])
    ax1.legend(loc='upper right', fontsize=8)
    
    # -----------------------------------------------------------------
    # Subplot 2: 로봇 가상 평면 상의 2/3번 관절각 (theta2, theta3) 유도
    # -----------------------------------------------------------------
    ax2 = fig.add_subplot(132)
    ax2.grid(True, linestyle='--', alpha=0.5)
    
    # 링크 그리기 (원점(어깨) -> 엘보우 -> 손목)
    elbow_plane = np.array([A2 * np.sin(theta2), A2 * np.cos(theta2)])
    target_plane = np.array([R_plane, Z_plane])
    
    ax2.plot([0.0, elbow_plane[0]], [0.0, elbow_plane[1]], 'o-', color='#34495E', linewidth=4, label='Upper Arm (a2)')
    ax2.plot([elbow_plane[0], target_plane[0]], [elbow_plane[1], target_plane[1]], 'o-', color='#005588', linewidth=4, label='Forearm (d4)')
    ax2.plot([0.0, target_plane[0]], [0.0, target_plane[1]], 'k--', alpha=0.6, label='Virtual Target Line (s)')
    
    # 수선 및 높이선 그리기 (투영 가이드)
    u_s = target_plane / s
    proj_dist = A2 * np.cos(theta2 - np.arctan2(Z_plane, R_plane)) # projection of a2 onto s
    proj_foot = u_s * proj_dist
    ax2.plot([elbow_plane[0], proj_foot[0]], [elbow_plane[1], proj_foot[1]], 'r:', linewidth=2, label='Height Line')
    
    # 노드 구체 마킹
    ax2.scatter(0.0, 0.0, color='black', s=120, zorder=5)
    ax2.scatter(elbow_plane[0], elbow_plane[1], color='#FF9900', s=100, zorder=5)
    ax2.scatter(target_plane[0], target_plane[1], color='red', s=120, zorder=5)
    
    # 라벨 표기 (float 캐스팅 처리 완료)
    ax2.text(0.0, -0.05, "Shoulder (0,0)", fontsize=9, fontweight='bold', ha='center')
    ax2.text(float(elbow_plane[0]) - 0.05, float(elbow_plane[1]) + 0.03, "Elbow", color='#D35400', fontsize=9, fontweight='bold')
    ax2.text(float(target_plane[0]) + 0.02, float(target_plane[1]) - 0.02, f"P_wc\n({R_plane:.3f}, {Z_plane:.3f})", color='red', fontsize=9, fontweight='bold')
    
    # 수학식 텍스트 박스 배치
    formula2 = (
        r"$\mathbf{Step\ 2:\ 2D\ Arm\ Plane\ (\theta_2,\ \theta_3)}$" + "\n" +
        r"$\bullet\ R_{plane} = " + rf"{R_plane:.4f}" + r"\ m,\ Z_{plane} = " + rf"{Z_plane:.4f}" + r"\ m$" + "\n" +
        r"$\bullet\ s = \sqrt{R_{plane}^2 + Z_{plane}^2} = " + rf"{s:.4f}" + r"\ m$" + "\n" +
        r"$\bullet\ \cos(\theta_3) = \frac{s^2 - A_2^2 - D_4^2}{2 \cdot A_2 \cdot D_4} = " + rf"{cos_q3:.4f}$" + "\n" +
        r"$\bullet\ \theta_3 = \mathbf{" + rf"{np.degrees(theta3):.1f}^\circ\ (Elbow\ Up)" + "}$" + "\n" +
        r"$\bullet\ \theta_2 = \mathbf{" + rf"{np.degrees(theta2):.1f}^\circ\ (Shoulder)" + "}$"
    )
    ax2.text(0.05, 0.05, formula2, transform=ax2.transAxes, fontsize=10,
             bbox=dict(facecolor='#EBF5FB', alpha=0.9, edgecolor='#2980B9', boxstyle='round,pad=0.5'))
    
    ax2.set_title("2. Local Arm Plane & Shoulder/Elbow", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Plane R Axis (m)")
    ax2.set_ylabel("Plane Z Axis (m)")
    ax2.set_aspect('equal')
    ax2.set_xlim([-0.1, 0.9])
    ax2.set_ylim([-0.2, 0.8])
    ax2.legend(loc='upper right', fontsize=8)
    
    # -----------------------------------------------------------------
    # Subplot 3: 3D 기구학 분리 및 최종 로봇 형상 렌더링
    # -----------------------------------------------------------------
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.grid(True, linestyle=':', alpha=0.5)
    
    # 3D 뼈대 데이터 준비 (float 변환 처리 완료)
    xs = [float(p[0]) for p in [p0, p1, p2, p3, p4, target_pos]]
    ys = [float(p[1]) for p in [p0, p1, p2, p3, p4, target_pos]]
    zs = [float(p[2]) for p in [p0, p1, p2, p3, p4, target_pos]]
    
    # 로봇 팔 조인트 플로팅
    ax3.plot(xs[:2], ys[:2], zs[:2], 'o-', color='#34495E', linewidth=4, label='Base Link')
    ax3.plot(xs[1:3], ys[1:3], zs[1:3], 'o-', color='#34495E', linewidth=4)
    ax3.plot(xs[2:4], ys[2:4], zs[2:4], 'o-', color='#005588', linewidth=4, label='Upper Arm')
    ax3.plot(xs[3:5], ys[3:5], zs[3:5], 'o-', color='#2980B9', linewidth=3, label='Forearm')
    ax3.plot(xs[4:], ys[4:], zs[4:], 'o-', color='#D35400', linewidth=3, label='Wrist & Tool')
    
    # 관절 노드 구체 표기
    ax3.scatter([xs[0]], [ys[0]], [zs[0]], color='black', s=80)
    ax3.scatter([xs[2]], [ys[2]], [zs[2]], color='#FF9900', s=70)
    ax3.scatter([xs[3]], [ys[3]], [zs[3]], color='#E74C3C', s=60)
    ax3.scatter([xs[4]], [ys[4]], [zs[4]], color='#9B59B6', s=60)
    ax3.scatter([xs[5]], [ys[5]], [zs[5]], color='red', s=80)
    
    # 손목 중심점 가상 조준 및 툴 연장 라인 표기
    ax3.plot([xs[4], xs[4]], [ys[4], ys[4]], [0.0, zs[4]], 'k:', alpha=0.4)
    
    # 로봇 말단 회전축 시각화 (Z-axis approach vector 그리기)
    scale = 0.15
    o = target_pos
    z_axis = o + scale * a
    ax3.plot([float(o[0]), float(z_axis[0])], [float(o[1]), float(z_axis[1])], [float(o[2]), float(z_axis[2])],
             color='blue', linewidth=3, label='TCP Approach (a)')
    
    # 3D 수학식 설명 박스
    formula3 = (
        r"$\mathbf{Step\ 3:\ 3D\ Wrist\ Decoupling}$" + "\n" +
        r"$\bullet\ P_{wc} = P_{target} - d_6 \cdot \mathbf{a}$" + "\n" +
        r"$\bullet\ P_{wc} = [" + f"{x_wc:.3f}, {y_wc:.3f}, {z_wc:.3f}" + r"]^T$" + "\n" +
        r"$\bullet\ R_3^6 = (R_0^3)^T R_{target}$" + "\n" +
        r"$\bullet\ \theta_4, \theta_5, \theta_6 = \text{Euler}_{ZYZ}(R_3^6)$"
    )
    # 3D 캔버스 상단 혹은 빈곳에 텍스트 표기
    ax3.text2D(0.05, 0.05, formula3, transform=ax3.transAxes, fontsize=9.5,
               bbox=dict(facecolor='#FDEDEC', alpha=0.9, edgecolor='#CD6155', boxstyle='round,pad=0.5'))
    
    ax3.set_title("3. 3D Decoupling & Config", fontsize=12, fontweight='bold')
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_zlabel("Z (m)")
    ax3.set_xlim3d([-0.5, 0.8])
    ax3.set_ylim3d([-0.5, 0.8])
    ax3.set_zlim3d([-0.1, 1.0])
    ax3.legend(loc='upper right', fontsize=8)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 검증 목표 포즈 입력: X=0.5m, Y=0.3m, Z=0.4m, R=10도, P=30도, Y=45도
    plot_step_by_step(
        x_target=0.50, 
        y_target=0.30, 
        z_target=0.40, 
        r_deg=10.0, 
        p_deg=30.0, 
        y_deg=45.0
    )
