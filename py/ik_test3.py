import numpy as np
import matplotlib.pyplot as plt

# [Kinematic Constants] Doosan m1013 robot dimensions (m)
D1 = 0.1525   # Base height
D2 = 0.0345   # Shoulder offset
A2 = 0.6200   # Upper arm length
D4 = 0.5590   # Forearm length

def plot_pure_complementary_proof(x_target, y_target, z_target):
    # --- Inverse Kinematics (IK) Calculation ---
    r_xy = np.sqrt(x_target**2 + y_target**2)
    R = np.sqrt(r_xy**2 - D2**2)
    Z = z_target - D1
    s_sq = R**2 + Z**2
    s = np.sqrt(s_sq)
    
    # Law of Cosines for Joint 3
    cos_phi3 = (A2**2 + D4**2 - s_sq) / (2 * A2 * D4)
    sin_phi3 = np.sqrt(1 - cos_phi3**2)
    phi3 = np.arctan2(sin_phi3, cos_phi3)
    theta3 = phi3 - np.pi/2 
    
    # Joint 2 Angle
    alpha = np.arctan2(Z, R)
    beta = np.arctan2(D4 * np.sin(theta3 + np.pi/2), A2 + D4 * np.cos(theta3 + np.pi/2))
    theta2 = np.pi/2 - (alpha + beta) 

    # --- Joint Coordinate Points ---
    shoulder = np.array([0.0, 0.0])
    elbow = np.array([A2 * np.sin(theta2), A2 * np.cos(theta2)])
    target = np.array([R, Z])
    
    # --- Perpendicular Foot Calculation ---
    u_s = target / s   # Unit vector of target line
    proj_dist = A2 * np.cos(beta)
    proj_foot = u_s * proj_dist  

    # =====================================================================
    # [Matplotlib Visualization Plot]
    # =====================================================================
    plt.figure(figsize=(14, 10))
    plt.grid(True, linestyle='--', alpha=0.4)
    
    # 1. Robot Linkage (A2, D4)
    plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'o-', linewidth=5, color='#34495E', label='Upper Arm (A2)')
    plt.plot([elbow[0], target[0]], [elbow[1], target[1]], 'o-', linewidth=5, color='#005588', label='Forearm (D4)')
    
    # 2. Virtual Target Line (s)
    plt.plot([shoulder[0], target[0]], [shoulder[1], target[1]], 'k--', alpha=0.5, linewidth=2, label='Virtual Target Line (s)')
    
    # 3. Extended Upper Arm Line (Purple Dotted Line)
    extended_point = elbow + (elbow / A2) * 0.35 
    plt.plot([elbow[0], extended_point[0]], [elbow[1], extended_point[1]], color='#9B59B6', linewidth=3, linestyle=':', label='Extended Upper Arm Axis')
    
    # 4. Trigonometric Projection Lines
    # ① Red Height Line ➡️ D4 * sin(phi_3)
    plt.plot([elbow[0], proj_foot[0]], [elbow[1], proj_foot[1]], color='#E74C3C', linewidth=3, linestyle='-', label='Height Line (D4 * sin)')
    # ② Green Base Line
    plt.plot([shoulder[0], proj_foot[0]], [shoulder[1], proj_foot[1]], color='#2ECC71', linewidth=3, linestyle='-', alpha=0.7, label='Base Line')

    # 5. Coordinate Axis & Node Markers
    plt.axhline(0, color='black', linewidth=1, alpha=0.4)
    plt.scatter(shoulder[0], shoulder[1], color='black', s=220, zorder=5)
    plt.scatter(elbow[0], elbow[1], color='#FF9900', s=180, zorder=5)
    plt.scatter(target[0], target[1], color='red', s=220, zorder=5)
    
    # =====================================================================
    # 6. Twin Angles Symbol Labeling (배열 인덱스 개별 분리 완료)
    # =====================================================================
    # ① Joint 3 external angle (phi_3)
    plt.text(elbow[0] + 0.04, elbow[1] - 0.01, r"$\phi_3$", fontsize=15, color='#9B59B6', fontweight='bold',
             bbox=dict(facecolor='white', alpha=0.8, boxstyle='circle,pad=0.3'))
    plt.text(extended_point[0] + 0.01, extended_point[1] + 0.01, "Extended Axis", color='#9B59B6', fontsize=10)
    
    # ② Right triangle corner angle (psi = phi_3)
    plt.text(target[0] - 0.08, target[1] + 0.04, r"$\psi = \phi_3$", fontsize=14, color='#E74C3C', fontweight='bold',
             bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.3'))
    
    # Right angle marker (L)
    plt.text(proj_foot[0] - 0.01, proj_foot[1] + 0.01, "L", fontsize=12, color='black', fontweight='bold')

    # =====================================================================
    # 7. Pure English Explanation Box (f-string/r-string 완전 분리)
    # =====================================================================
    explanation = (
        "1. Extend the upper arm (A2) line to make the purple dotted line (Extended Axis).\n"
        "2. The external angle between Extended Axis and Forearm (D4) is 'phi_3'.\n"
        "3. Look at the right triangle formed by dropping a perpendicular line (Red Height Line).\n"
        r"4. By the geometric parallel line rule, the corner angle 'psi' at Target position" + "\n"
        r"   becomes a 'Twin Angle' that perfectly matches the external angle 'phi_3' (\psi = \phi_3)." + "\n\n"
        "Therefore, using the trigonometric ratio (sin = Height / Hypotenuse):\n"
        "-> Height = D4 * sin(phi_3)\n"
        "You can instantly calculate the height using the entire angle phi_3!"
    )
    plt.text(0.12, -0.22, explanation, fontsize=11, color='#111111', fontweight='normal',
             bbox=dict(facecolor='#FAFAFA', alpha=0.95, edgecolor='#9B59B6', boxstyle='round,pad=0.6'))

    # Node Labels (배열 인덱스 명시 적용)
    plt.text(shoulder[0] - 0.04, shoulder[1] + 0.03, "Shoulder Joint", fontsize=10, fontweight='bold')
    plt.text(elbow[0] - 0.05, elbow[1] + 0.03, "Elbow Joint", fontsize=10, color='#D35400', fontweight='bold')
    plt.text(target[0] + 0.02, target[1] - 0.02, "Target (Wrist Center)", fontsize=10, color='red', fontweight='bold')
    plt.text(proj_foot[0] + 0.02, proj_foot[1] + 0.02, "Perpendicular Foot", fontsize=9, color='black')

    # Value Link Labels
    mid_h = (elbow + proj_foot) / 2
    mid_b = (shoulder + proj_foot) / 2
    plt.text(mid_h[0] + 0.01, mid_h[1], "Height = D4 * sin(phi_3)", color='#C0392B', fontsize=10, fontweight='bold')
    plt.text(mid_b[0] - 0.05, mid_b[1] - 0.04, "Base = A2 + D4 * cos(phi_3)", color='#27AE60', fontsize=10, fontweight='bold')

    # Title & Axis Options
    plt.title('Geometric Visualization: Complementary Angle Proof for Joint 3', fontsize=14, fontweight='bold', pad=15)
    plt.xlabel('Horizontal Distance (R)', fontsize=12)
    plt.ylabel('Vertical Height (Z)', fontsize=12)
    
    # 1:1 Aspect ratio handling safely
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(-0.15, 0.9)
    plt.ylim(-0.25, 0.8)
    plt.legend(loc='upper right', fontsize=11)
    
    plt.show()

if __name__ == "__main__":
    plot_pure_complementary_proof(x_target=0.5, y_target=0.5, z_target=0.4)
