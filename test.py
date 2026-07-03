import numpy as np
import matplotlib.pyplot as plt
import time

# 1. 가상의 비선형 로봇 함수 정의: y = x^3 - 3x^2 + 2x + 5
def f(x):
    return x**3 - 3*x**2 + 2*x + 5

# 2. 미분 함수 (접선의 기울기 = 자코비안 J): J = 3x^2 - 6x + 2
def jacobian(x):
    return 3*x**2 - 6*x + 2

# 3. 환경 설정
x_start = 0.2        # 초기 추정 각도 (시작점)
y_target = 8.0       # 우리가 도달하고 싶은 목표 위치
tolerance = 1e-3     # 허용 오차
max_iter = 10        # 최대 반복 횟수

# 시각화를 위한 배경 곡선 데이터 생성
x_curve = np.linspace(-0.5, 3.5, 500)
y_curve = f(x_curve)

# 대화형 그래프 모드 켜기 (실시간 업데이트 애니메이션용)
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

x_curr = x_start

print("--- 뉴턴-랩슨(자코비안 반복법) 시각화 시작 ---")

for item in range(max_iter):
    y_curr = f(x_curr)
    error = y_target - y_curr
    
    # 4. 그래프 그리기 (매 단계 초기화 후 다시 그리기)
    ax.clear()
    ax.plot(x_curve, y_curve, label='Robot Function $y=f(x)$ (Non-linear)', color='blue', lw=2)
    ax.axhline(y=y_target, color='red', linestyle='--', label=f'Target $y={y_target}$')
    
    # 현재 탐색 점 표시
    ax.scatter([x_curr], [y_curr], color='black', s=100, zorder=5)
    ax.text(x_curr + 0.05, y_curr - 0.5, f'Step {item}\n(x={x_curr:.3f}, y={y_curr:.3f})', fontsize=10, fontweight='bold')
    
    # 오차가 기준치 이하이면 종료
    if abs(error) < tolerance:
        ax.scatter([x_curr], [y_curr], color='green', s=200, marker='*', zorder=6, label='Success!')
        print(f"🎉 [성공] {item}번째 단계에서 목표값에 도달했습니다. 최종 x = {x_curr:.4f}")
        break
        
    # 5. 자코비안(기울기) 계산 및 접선 시각화
    J = jacobian(x_curr)
    
    # 접선(Tangent Line) 방정식 유도: y - y_curr = J * (x - x_curr)
    # 접선이 목표선(y_target)과 만나는 다음 x점 계산: x_next = x_curr + (y_target - y_curr) / J
    if J == 0:
        print("❌ [특이점 발생] 기울기가 0이 되어 더 이상 계산할 수 없습니다.")
        break
        
    delta_x = error / J
    x_next = x_curr + delta_x
    
    # 접선 그리기용 선 데이터
    x_tangent = np.linspace(x_curr - 0.5, x_next + 0.5, 100)
    y_tangent = J * (x_tangent - x_curr) + y_curr
    ax.plot(x_tangent, y_tangent, color='orange', linestyle=':', label='Jacobian (Tangent Line)')
    
    # 다음 목표 이동 방향 화살표 표시
    ax.annotate('', xy=(x_next, y_target), xytext=(x_curr, y_curr),
                arrowprops=dict(arrowstyle="->", color='purple', lw=1.5))
    
    # 그래프 꾸미기
    ax.set_title("Numerical IK Mechanism Visualization (Newton-Raphson)", fontsize=14, fontweight='bold')
    ax.set_xlabel("Joint Angle (x)", fontsize=12)
    ax.set_ylabel("End-Effector Position (y)", fontsize=12)
    ax.set_xlim(-0.5, 3.5)
    ax.set_ylim(2, 12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')
    
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    # 다음 단계 진행을 위해 잠시 대기 (애니메이션 효과)
    time.sleep(2.0)
    
    # 값 업데이트
    x_curr = x_next
else:
    print("❌ 최대 반복 횟수 내에 수렴하지 못했습니다.")

# 애니메이션 종료 후 그래프 창 유지
plt.ioff()
plt.show()
