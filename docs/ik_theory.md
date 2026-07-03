# 6축 산업용 로봇의 해석적 역운동학 (Analytical Inverse Kinematics) 가이드

이 문서는 구형 손목(Spherical Wrist) 구조를 가진 6자유도(6-DOF) 수직 다관절 로봇의 역운동학(Inverse Kinematics, IK)을 기하학적 및 대수학적 방법으로 유도하는 과정을 다룹니다.

---

## 1. 개요 및 기구학적 분리 (Kinematic Decoupling)

6축 로봇의 역운동학을 수치해석적(Numerical) 방법이 아닌 **해석적(Analytical, Closed-form) 방법**으로 풀기 위해서는 로봇의 구조적인 대칭성이나 기하학적 특성이 필요합니다. 대표적인 조건이 바로 **구형 손목(Spherical Wrist)** 조건입니다.

> [!IMPORTANT]
> **구형 손목 (Spherical Wrist) 조건**
> 로봇의 마지막 3개 축(4번, 5번, 6번 축)의 회전 중심선이 **공간상의 한 점(손목 중심점, Wrist Center)**에서 만나는 구조를 의미합니다.
> 이 조건이 만족되면, 6자유도 역운동학 문제를 **3자유도 위치 문제**와 **3자유도 자세(배향) 문제**로 완벽하게 분리하여 순차적으로 해결할 수 있습니다 (Pieper's Method).

### 기구학적 분리의 수학적 원리
로봇 말단 장치(End-Effector, Tool)의 목표 포즈 $T_{target}$가 다음과 같이 주어졌다고 가정합니다.
$$T_{target} = T_0^6 = \begin{bmatrix} R_{target} & P_{target} \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} n_x & s_x & a_x & p_x \\ n_y & s_y & a_y & p_y \\ n_z & s_z & a_z & p_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
여기서:
- $R_{target} = [n, s, a]$는 말단 장치의 회전 행렬(방향)을 나타내며, $a = [a_x, a_y, a_z]^T$는 말단 장치가 지향하는 **접근 벡터(Approach Vector)**입니다.
- $P_{target} = [p_x, p_y, p_z]^T$는 말단 장치 끝점(TCP, Tool Center Point)의 3차원 위치입니다.

손목 중심점(Wrist Center, $P_{wc}$)은 6번 관절 축 상에 존재하므로, 말단 장치 끝점 $P_{target}$로부터 접근 벡터 $a$의 반대 방향으로 마지막 링크의 길이(오프셋 $d_6$)만큼 이동한 위치에 존재합니다.
$$\mathbf{P_{wc}} = P_{target} - d_6 \cdot \mathbf{a}$$
$$\begin{bmatrix} x_{wc} \\ y_{wc} \\ z_{wc} \end{bmatrix} = \begin{bmatrix} p_x - d_6 \cdot a_x \\ p_y - d_6 \cdot a_y \\ p_z - d_6 \cdot a_z \end{bmatrix}$$

이 식을 통해 말단 장치의 목표 포즈로부터 **손목 중심점 $P_{wc}$의 좌표를 먼저 직접 계산**해낼 수 있습니다. 이 점의 위치는 오직 앞의 3개 관절 각도($\theta_1, \theta_2, \theta_3$)에 의해서만 결정되므로, 문제를 다음과 같이 나눕니다.

1. **위치 역운동학**: $P_{wc} = [x_{wc}, y_{wc}, z_{wc}]^T$를 만족하는 $\theta_1, \theta_2, \theta_3$를 기하학적으로 계산합니다.
2. **자세 역운동학**: 구해진 $\theta_1, \theta_2, \theta_3$를 바탕으로 앞쪽 링크들의 회전 행렬 $R_0^3$를 구하고, 남은 자세 차이를 채우기 위한 손목 회전 행렬 $R_3^6 = (R_0^3)^T R_{target}$를 대수적으로 풀어 $\theta_4, \theta_5, \theta_6$를 계산합니다.

---

## 2. 위치 역운동학: $\theta_1, \theta_2, \theta_3$ 구하기

두산 m1013 등 일반적인 6축 협동 로봇의 링크 구성을 모델로 삼아 유도를 진행합니다.
- $d_1$: 베이스 높이 (Base to Shoulder Joint 2)
- $d_2$: 어깨 가로 오프셋 (Shoulder Offset)
- $a_2$: 위팔 링크 길이 (Upper Arm Length)
- $d_4$: 아래팔 링크 길이 (Forearm Length - Elbow Joint 3 to Wrist Center 5)

### 2.1 1번 관절 각도 ($\theta_1$) - 허리 회전
로봇을 위에서 내려다본 $XY$ 평면 투영도를 생각합니다.
손목 중심점 $P_{wc} = (x_{wc}, y_{wc})$와 어깨 오프셋 $d_2$에 의해 직각삼각형이 형성됩니다.
- 원점과 손목 중심점 사이의 거리: $r_{xy} = \sqrt{x_{wc}^2 + y_{wc}^2}$
- 어깨 오프셋 $d_2$를 뺀 실제 유효 회전 반경: $R_{plane} = \sqrt{r_{xy}^2 - d_2^2}$

이때 직각삼각형의 성질에 의해 다음과 같은 두 각도를 정의합니다.
$$\alpha = \text{atan2}(y_{wc}, x_{wc})$$
$$\beta = \text{atan2}(d_2, R_{plane}) = \text{atan2}(d_2, \sqrt{r_{xy}^2 - d_2^2})$$

이때 로봇의 구성(어깨의 왼쪽/오른쪽 배치)에 따라 두 가지 해가 존재합니다.
- **Left-shoulder (좌측 구성)**: $\theta_1 = \alpha + \beta$
- **Right-shoulder (우측 구성)**: $\theta_1 = \alpha - \beta + \pi$ (혹은 부호에 맞게 범위 조정)

일반적으로 어깨 오프셋 방향에 따라 다음과 같이 계산합니다.
$$\theta_1 = \text{atan2}(y_{wc}, x_{wc}) \pm \text{atan2}(d_2, \sqrt{x_{wc}^2 + y_{wc}^2 - d_2^2})$$

---

### 2.2 2번 및 3번 관절 각도 ($\theta_2, \theta_3$) - 어깨와 팔꿈치
$\theta_1$에 의해 결정된 평면 상에서 문제를 해결합니다. 이 평면의 가로축을 $R_{plane}$, 세로축을 $Z_{plane}$이라고 합니다.
- $R_{plane} = \sqrt{x_{wc}^2 + y_{wc}^2 - d_2^2}$ (1번 축에서 손목 중심까지의 수평 거리)
- $Z_{plane} = z_{wc} - d_1$ (베이스 높이 $d_1$을 제외한 수직 높이)

이 평면 상에서 어깨 관절(Joint 2)을 원점 $(0,0)$으로 하고, 손목 중심점의 좌표는 $(R_{plane}, Z_{plane})$가 됩니다.
여기에 길이 $a_2$인 위팔(Upper arm)과 길이 $d_4$인 아래팔(Forearm) 링크가 연결되어 삼각형을 이룹니다.

#### 1) 3번 관절 각도 ($\theta_3$) 유도
어깨 조인트와 손목 중심점 사이의 직선 거리를 $s$라고 하면 피타고라스 정리에 의해:
$$s^2 = R_{plane}^2 + Z_{plane}^2$$

위팔($a_2$), 아래팔($d_4$), 가상 조준선($s$)이 이루는 삼각형에 대해 **제2코사인 법칙(Law of Cosines)**을 적용합니다. 위팔과 아래팔 사이의 사잇각을 $\phi_3$라고 하면:
$$s^2 = a_2^2 + d_4^2 - 2 a_2 d_4 \cos(\phi_3)$$
$$\cos(\phi_3) = \frac{a_2^2 + d_4^2 - s^2}{2 a_2 d_4}$$

만약 $\cos(\phi_3)$의 값이 $[-1, 1]$ 범위를 벗어나면, 목표 지점이 로봇이 닿을 수 없는 영역(Workspace 영역 밖)에 있음을 의미합니다.

$\sin(\phi_3)$는 삼각함수 항등식에 의해 두 가지 부호로 계산됩니다.
$$\sin(\phi_3) = \pm \sqrt{1 - \cos^2(\phi_3)}$$
여기서 $\sin(\phi_3) > 0$이면 **Elbow-down (팔꿈치 아래)**, $\sin(\phi_3) < 0$이면 **Elbow-up (팔꿈치 위)** 자세가 됩니다.

$$\phi_3 = \text{atan2}(\sin\phi_3, \cos\phi_3)$$

로봇 설계 사양(DH 프레임 설정)에 따라 $\theta_3$ 각도를 조정합니다. 예를 들어 위팔과 아래팔이 일직선일 때 $\theta_3 = 0$이 되도록 정의하는 경우:
$$\theta_3 = \phi_3 - \frac{\pi}{2}$$
(실제 좌표계 오프셋에 맞게 조정이 필요합니다.)

#### 2) 2번 관절 각도 ($\theta_2$) 유도
$\theta_2$는 전체 목표 방향각 $\alpha_{plane}$에서 링크 굽힘에 의한 사잇각 $\beta_{plane}$를 빼서 구합니다.
- 가상 조준선이 가로축 $R_{plane}$과 이루는 각도:
  $$\alpha_{plane} = \text{atan2}(Z_{plane}, R_{plane})$$
- 링크 $a_2$와 가상 조준선 $s$ 사이의 사잇각 $\beta_{plane}$는 직각삼각형 투영 기하를 이용해 구합니다:
  $$\text{Height} = d_4 \sin(\phi_3)$$
  $$\text{Base} = a_2 + d_4 \cos(\phi_3)$$
  $$\beta_{plane} = \text{atan2}(\text{Height}, \text{Base}) = \text{atan2}(d_4 \sin\phi_3, a_2 + d_4 \cos\phi_3)$$

최종적으로 어깨 관절각 $\theta_2$는 다음과 같습니다.
$$\theta_2 = \frac{\pi}{2} - (\alpha_{plane} + \beta_{plane})$$
(이 역시 로봇의 기준 축 방향에 맞춰 부호 및 기하학적 오프셋을 더하거나 뺍니다.)

---

## 3. 자세 역운동학: $\theta_4, \theta_5, \theta_6$ 구하기

앞의 3개 관절 각도 $\theta_1, \theta_2, \theta_3$를 모두 구했으므로, 0번 프레임(베이스)에서 3번 프레임(팔꿈치 이후 링크)까지의 회전 행렬 $R_0^3$를 순방향 기구학으로 계산할 수 있습니다.
$$R_0^3 = R_0^1(\theta_1) R_1^2(\theta_2) R_2^3(\theta_3)$$

전체 목표 자세 회전 행렬이 $R_{target}$이므로, 다음과 같은 관계식이 성립합니다.
$$R_{target} = R_0^6 = R_0^3 R_3^6$$

우리가 구하고자 하는 손목 관절의 회전 행렬 $R_3^6$는 양변에 $(R_0^3)^T$를 왼쪽에 곱해 고립시킬 수 있습니다.
$$R_3^6 = (R_0^3)^T R_{target}$$

행렬 곱의 결과를 다음과 같은 원소들로 정의합니다.
$$R_3^6 = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}$$

---

### 3.1 오일러 각 추출을 통한 손목 각도 유도

구형 손목은 일반적으로 **ZYZ** 또는 **YZY** 오일러 각 회전 형태를 띱니다. 로봇의 4번 축, 5번 축, 6번 축 회전 방향에 따라 결정됩니다.
여기서는 표준적인 **ZYZ 오일러 각** 구성을 기준으로 설명합니다.

ZYZ 회전 행렬의 대수적 표현은 다음과 같습니다.
$$R_{ZYZ}(\theta_4, \theta_5, \theta_6) = R_z(\theta_4) R_y(\theta_5) R_z(\theta_6)$$
$$R_{ZYZ} = \begin{bmatrix}
\cos\theta_4\cos\theta_5\cos\theta_6 - \sin\theta_4\sin\theta_6 & -\cos\theta_4\cos\theta_5\sin\theta_6 - \sin\theta_4\cos\theta_6 & \cos\theta_4\sin\theta_5 \\
\sin\theta_4\cos\theta_5\cos\theta_6 + \cos\theta_4\sin\theta_6 & -\sin\theta_4\cos\theta_5\sin\theta_6 + \cos\theta_4\cos\theta_6 & \sin\theta_4\sin\theta_5 \\
-\sin\theta_5\cos\theta_6 & \sin\theta_5\sin\theta_6 & \cos\theta_5
\end{bmatrix}$$

이 행렬을 우리가 구한 $R_3^6$의 원소들과 일대일 매칭하여 각도를 추출합니다.

#### 1) 5번 관절 각도 ($\theta_5$)
$r_{33}$ 원소를 비교합니다.
$$\cos\theta_5 = r_{33}$$
$$\sin\theta_5 = \pm \sqrt{1 - r_{33}^2}$$
$$\theta_5 = \text{atan2}(\sin\theta_5, \cos\theta_5)$$

여기서도 두 가지 해가 존재합니다.
- **정방향 자세 (No-flip)**: $\sin\theta_5 > 0$
- **역방향 자세 (Wrist-flip)**: $\sin\theta_5 < 0$

#### 2) 4번 및 6번 관절 각도 ($\theta_4, \theta_6$)
$\sin\theta_5 \neq 0$인 일반적인 상황(특이점이 아닌 경우)에서는 다음 원소들을 나눗셈하여 구할 수 있습니다.

- **$\theta_4$ 구하기**: $r_{13}$와 $r_{23}$ 비교
  $$r_{13} = \cos\theta_4\sin\theta_5$$
  $$r_{23} = \sin\theta_4\sin\theta_5$$
  따라서,
  $$\theta_4 = \text{atan2}\left(\frac{r_{23}}{\sin\theta_5}, \frac{r_{13}}{\sin\theta_5}\right)$$

- **$\theta_6$ 구하기**: $r_{31}$와 $r_{32}$ 비교
  $$r_{31} = -\sin\theta_5\cos\theta_6$$
  $$r_{32} = \sin\theta_5\sin\theta_6$$
  따라서,
  $$\theta_6 = \text{atan2}\left(\frac{r_{32}}{\sin\theta_5}, \frac{-r_{31}}{\sin\theta_5}\right)$$

---

### 3.2 짐벌 락 특이점 (Gimbal Lock Singularity) 해결

만약 $\sin\theta_5 = 0$ 이라면(즉, $\theta_5 = 0$ 또는 $\pi$ 라면), 4번 관절축과 6번 관절축이 완벽히 일직선 상에 놓이게 되어 **짐벌 락(Gimbal Lock) 특이 상태**가 됩니다. 이 상태에서는 4번 각도와 6번 각도의 합(또는 차)만 결정되며, 개별 각도는 무한히 많은 해를 가집니다.

#### Case 1: $\theta_5 = 0$ 인 경우 ($\cos\theta_5 = 1$)
회전 행렬 $R_{ZYZ}$는 다음과 같이 단순화됩니다.
$$R_{ZYZ} = \begin{bmatrix}
\cos(\theta_4 + \theta_6) & -\sin(\theta_4 + \theta_6) & 0 \\
\sin(\theta_4 + \theta_6) & \cos(\theta_4 + \theta_6) & 0 \\
0 & 0 & 1
\end{bmatrix}$$
따라서:
$$\theta_4 + \theta_6 = \text{atan2}(r_{21}, r_{11})$$
이 경우, 무수히 많은 해 중 하나를 임의로 선택합니다. 일반적으로 계산 편의성을 위해 한쪽 관절 값을 고정합니다.
- 예: $\theta_4 = 0$ (현재 각도 유지 등)
- 그러면 $\theta_6 = \text{atan2}(r_{21}, r_{11})$가 됩니다.

#### Case 2: $\theta_5 = \pi$ 인 경우 ($\cos\theta_5 = -1$)
회전 행렬 $R_{ZYZ}$는 다음과 같이 단순화됩니다.
$$R_{ZYZ} = \begin{bmatrix}
-\cos(\theta_4 - \theta_6) & -\sin(\theta_4 - \theta_6) & 0 \\
-\sin(\theta_4 - \theta_6) & \cos(\theta_4 - \theta_6) & 0 \\
0 & 0 & -1
\end{bmatrix}$$
따라서:
$$\theta_4 - \theta_6 = \text{atan2}(-r_{21}, -r_{11})$$
이 역시 무수히 많은 해가 존재하므로 한 축을 고정하여 풉니다.
- 예: $\theta_4 = 0$
- 그러면 $\theta_6 = -\text{atan2}(-r_{21}, -r_{11})$가 됩니다.

---

## 4. 다중 해(Multiple Solutions) 분기 요약

6축 로봇의 순방향 기구학 포즈 하나에 대해, 역운동학은 대칭성에 의해 최대 **8가지 조합의 서로 다른 관절 해(Solutions)**를 가질 수 있습니다.

| 번호 | Shoulder (J1) | Elbow (J3) | Wrist (J5) | 특징 |
|---|---|---|---|---|
| 1 | Left | Up | No-Flip | 일반적인 표준 자세 |
| 2 | Left | Up | Flip | 손목을 180도 뒤집은 자세 |
| 3 | Left | Down | No-Flip | 팔을 아래로 굽힌 자세 |
| 4 | Left | Down | Flip | 팔을 아래로 굽히고 손목을 뒤집은 자세 |
| 5 | Right | Up | No-Flip | 몸체를 뒤로 돌려 접근하는 자세 |
| 6 | Right | Up | Flip | 몸체를 돌리고 손목을 뒤집은 자세 |
| 7 | Right | Down | No-Flip | 몸체를 돌려 아래로 굽힌 자세 |
| 8 | Right | Down | Flip | 몸체를 돌려 아래로 굽히고 손목 뒤집음 |

제어기 설계 시에는 이 8가지 해 중에서 **현재 관절 위치와 물리적으로 가장 가까운 해** 또는 **관절 제한 범위(Limit)를 벗어나지 않는 해**를 최적의 해로 선택하여 추종하도록 구현하게 됩니다.
