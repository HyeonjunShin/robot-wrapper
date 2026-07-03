# 로봇 기구학 기초 수학 및 역운동학 입문 가이드

이 문서는 6축 다관절 로봇의 역운동학(Inverse Kinematics, IK)을 이해하기 위해 꼭 필요한 기초 수학 개념(역삼각함수, 제2코사인 법칙)부터 시작하여 좌표계 변환, 회전 행렬, 그리고 구형 손목을 이용한 6축 분리 연산의 핵심 개념을 초보자의 시선에 맞춰 상세히 설명합니다.

---

## 1. 필수 기초 수학 개념

로봇 관절의 회전 각도를 기하학적으로 풀기 위해서는 고등학교 수준의 삼각함수와 벡터 기하학 개념을 로봇공학 관점에서 재해석해야 합니다.

### 1.1 탄젠트($\tan$)와 역탄젠트($\arctan$)의 이해
직각삼각형에서 밑변의 길이를 $x$, 높이를 $y$라고 할 때, 그 사잇각 $\theta$의 탄젠트 비율은 다음과 같습니다:
$$\tan(\theta) = \frac{y}{x}$$

우리가 알고 싶은 것은 변의 비율이 아니라 실제 각도 $\theta$이므로, 탄젠트의 역함수인 아크탄젠트(Arctangent)를 사용하여 각도를 구합니다:
$$\theta = \arctan\left(\frac{y}{x}\right)$$

#### ⚠️ 아크탄젠트 사용 시의 치명적인 함정
컴퓨터 프로그래밍에서 단순히 $\arctan(y/x)$를 계산하면 로봇의 관절이 엉뚱한 방향으로 회전하는 버그가 발생합니다.
1. **사분면 구별 불가능**:
   - $x=1, y=1$ (1사분면, $45^\circ$): $y/x = 1.0 \implies \arctan(1.0) = 45^\circ$
   - $x=-1, y=-1$ (3사분면, $-135^\circ$): $y/x = 1.0 \implies \arctan(1.0) = 45^\circ$
   - 수학적으로 탄젠트 함수의 치역은 $-\pi/2 \sim \pi/2$ ($-90^\circ \sim 90^\circ$)로 제한되어 있어, $180^\circ$ 뒤쪽 방향을 구별하지 못합니다.
2. **분모가 0인 경우 (나눗셈 오류)**:
   - $x=0$ (예: $90^\circ$나 $-90^\circ$ 방향)인 경우, $y/0$ 연산이 발생하여 프로그램이 다운됩니다.

#### 💡 해결책: `atan2(y, x)` 함수
이러한 한계를 극복하기 위해 로봇공학과 대부분의 프로그래밍 언어(Python, C++ 등)에서는 `atan2(y, x)` 함수를 제공합니다.
- 이 함수는 나눗셈 결과가 아니라 **$y$값과 $x$값을 독립된 인자**로 받습니다.
- 내부적으로 $x$와 $y$의 **부호(Plus/Minus)**를 분석하여 $-\pi \sim \pi$ ($-180^\circ \sim 180^\circ$) 전 사분면의 정확한 각도를 찾아냅니다.
- $x=0$인 수직 정렬 상태도 안전하게 연산합니다.
- **규칙**: 역운동학 알고리즘을 코드로 짤 때는 일반 `atan`을 절대 쓰지 않고, 무조건 `atan2`를 사용합니다.

---

### 1.2 제2코사인 법칙 (Law of Cosines)
직각삼각형이 아닌 일반 삼각형에서 세 변의 길이($a, b, c$)와 한 각도($C$)의 관계를 나타내는 공식입니다.

```
          C (Elbow Joint)
           /\
          /  \
     a   /    \   b
        /      \
       /________\
      A    c     B (Wrist Center)
```

위 삼각형에서 사이각 $C$와 마주 보는 변 $c$ 사이에는 다음과 같은 공식이 성립합니다:
$$c^2 = a^2 + b^2 - 2ab \cos(C)$$

이를 코사인에 대해 정리하면 다음과 같습니다:
$$\cos(C) = \frac{a^2 + b^2 - c^2}{2ab}$$

#### 로봇 공학에서의 활용
로봇의 위팔(길이 $a$)과 아래팔(길이 $b$)의 길이, 그리고 어깨에서 손목 중심점까지의 거리 $c$를 알고 있다면, **제2코사인 법칙을 통해 팔꿈치 관절이 꺾여야 하는 사잇각 $C$를 즉시 구할 수 있습니다.**

---

## 2. 3D 공간의 표현: 좌표계와 변환 행렬

로봇은 고정된 바닥(Base) 위에 여러 개의 회전축이 쌓여 있는 구조입니다. 각 관절의 움직임을 수학적으로 추적하기 위해 각 링크마다 **좌표계(Coordinate Frame)**를 붙입니다.

### 2.1 회전 행렬 (Rotation Matrix)
3차원 공간에서 기준 좌표계 대비 특정 좌표계가 얼마나 회전했는지를 나타내는 3x3 행렬입니다. 행렬의 각 열은 회전된 좌표계의 $X, Y, Z$ 단위원 벡터가 기준 좌표계에서 바라보았을 때의 좌표 성분을 나타냅니다.

로봇 관절 회전에 쓰이는 대표적인 기본 회전 행렬은 다음과 같습니다:

#### 1) Z축 기준 회전 ($R_z(\theta)$)
로봇의 1축(허리 회전), 4축, 6축 회전에 사용됩니다.
$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

#### 2) Y축 기준 회전 ($R_y(\theta)$)
로봇의 2축(어깨 들림), 3축(팔꿈치 꺾임), 5축 회전에 사용됩니다.
$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

---

### 2.2 동차 변환 행렬 (Homogeneous Transformation Matrix)
회전 행렬($R$, 3x3)과 위치 벡터($P$, 3x1)를 하나의 행렬로 묶어 연산을 편리하게 만든 4x4 행렬입니다.
$$T = \begin{bmatrix} R_{3\times3} & P_{3\times1} \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & x_{pos} \\ r_{21} & r_{22} & r_{23} & y_{pos} \\ r_{31} & r_{32} & r_{33} & z_{pos} \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

- **결합 법칙**: 변환 행렬을 차례대로 곱하면, 좌표 변환이 꼬리를 물고 연결됩니다.
  $T_0^2 = T_0^1 \cdot T_1^2$ (0번 좌표계에서 본 2번 좌표계의 위치와 자세)
- **순방향 기구학 (Forward Kinematics)**: 베이스에서 말단 장치(Tool)까지의 변환 행렬을 순서대로 곱하는 과정입니다.
  $$T_0^{tool} = T_0^1(\theta_1) \cdot T_1^2(\theta_2) \cdot T_2^3(\theta_3) \cdot T_3^4(\theta_4) \cdot T_4^5(\theta_5) \cdot T_5^6(\theta_6) \cdot T_6^{tool}$$

---

## 3. 구형 손목(Spherical Wrist)과 기구학적 분리

6축 로봇의 역운동학(IK)은 원래 6개의 미지수($\theta_1 \sim \theta_6$)가 복잡한 비선형 삼각함수로 꼬여 있어 해를 구하기 매우 어렵습니다. 

하지만 현대의 거의 모든 6축 로봇은 **4축, 5축, 6축의 회전 중심선이 3차원 공간상의 한 점(손목 중심점, Wrist Center)에서 교차하는 구형 손목** 구조를 가지고 있습니다.

> [!TIP]
> **기구학적 분리 (Kinematic Decoupling)**
> 손목 중심선이 한 점에서 만나기 때문에, **4, 5, 6번 관절의 회전은 손목 중심점의 위치($P_{wc}$)에 아무런 영향을 주지 못합니다.**
> 따라서 6축 역운동학 문제를 아래의 두 단계로 나누어 아주 쉽게 해결할 수 있습니다.
> 1. **위치 문제 ($\theta_1, \theta_2, \theta_3$)**: 베이스에서 손목 중심점($P_{wc}$)까지 도달하기 위한 해 구하기.
> 2. **자세 문제 ($\theta_4, \theta_5, \theta_6$)**: 손목 조인트들을 회전시켜 목표 오리엔테이션 채우기.

---

## 4. 단계별 역운동학 풀이 과정 (Geometric Approach)

### [Step 1] 손목 중심점 ($P_{wc}$) 역추적
최종 말단 장치(TCP)의 목표 위치 $P_{target}$와 방향(회전 행렬 $R_{target}$)이 주어집니다. 
회전 행렬 $R_{target}$의 3번째 열 벡터는 툴이 향하는 **접근 벡터(Approach Vector, $\mathbf{a}$)**입니다.

손목 중심점 $P_{wc}$는 최종 목표 위치에서 툴 길이 $d_6$만큼 접근 벡터의 반대 방향으로 들어온 지점입니다.
$$\mathbf{P_{wc}} = P_{target} - d_6 \cdot \mathbf{a}$$
$$\begin{bmatrix} x_{wc} \\ y_{wc} \\ z_{wc} \end{bmatrix} = \begin{bmatrix} p_x - d_6 \cdot a_x \\ p_y - d_6 \cdot a_y \\ p_z - d_6 \cdot a_z \end{bmatrix}$$

이제 이 $P_{wc} = (x_{wc}, y_{wc}, z_{wc})$를 도달하기 위한 앞의 3축 각도를 기하학적으로 해결합니다.

---

### [Step 2] 1번 관절각 ($\theta_1$) 계산 ($XY$ 평면 투영)
로봇을 위에서 바라본 평면도 상에서 원점 $(0,0)$에서 손목 중심점 $W(x_{wc}, y_{wc})$를 바라봅니다. 
이때 어깨 조인트가 몸체 중심에서 옆으로 $d_2$만큼 튀어나와 있는 오프셋 구조를 가집니다.

```
       Y축
        ^           Wrist Center W (x_wc, y_wc)
        |             /|
        |            / | 
        |      r_xy /  | R_plane
        |          /   |
        |         /    | 
        |        /     |
        |       / beta |
        |      /_______|
        |    (0,0) d2  Shoulder S
```

1. 평면상의 총 거리: $r_{xy} = \sqrt{x_{wc}^2 + y_{wc}^2}$
2. 어깨 오프셋 $d_2$에 직교하는 유효 도달 거리: $R_{plane} = \sqrt{r_{xy}^2 - d_2^2}$
3. 기본 방향각 $\alpha$와 오프셋 각도 $\beta$를 구합니다:
   $$\alpha = \text{atan2}(y_{wc}, x_{wc})$$
   $$\beta = \text{atan2}(d_2, R_{plane}) = \text{atan2}(d_2, \sqrt{r_{xy}^2 - d_2^2})$$
4. 최종 1번 관절 각도 $\theta_1$:
   - **Left-shoulder**: $\theta_1 = \alpha - \beta$
   - **Right-shoulder**: $\theta_1 = \alpha + \beta + \pi$

---

### [Step 3] 3번 관절각 ($\theta_3$) 계산 (제2코사인 법칙)
1번 축 회전각 $\theta_1$에 의해 결정된 평면 상으로 좌표계를 눕힙니다. 
가로축은 $R_{plane}$, 세로축은 $Z_{plane} = z_{wc} - d_1$ (베이스 높이 제외)이 됩니다.

어깨 관절 $(0,0)$에서 손목 중심점 $(R_{plane}, Z_{plane})$까지의 거리를 $s$라고 정의합니다.
$$s^2 = R_{plane}^2 + Z_{plane}^2$$

위팔($a_2$), 아래팔($d_4$), 가상 조준선($s$)이 삼각형을 이룹니다. 제2코사인 법칙에 의해:
$$\cos(\theta_3) = \frac{s^2 - a_2^2 - d_4^2}{2 a_2 d_4}$$

만약 $\cos(\theta_3)$의 값이 $[-1, 1]$ 범위를 벗어나면 로봇 팔의 길이 한계를 초과한 도달 불가능 영역(Workspace Out)입니다.
- **Elbow Up (정자세)**: $\sin(\theta_3) = \sqrt{1 - \cos^2(\theta_3)}$
- **Elbow Down (역자세)**: $\sin(\theta_3) = -\sqrt{1 - \cos^2(\theta_3)}$

최종 3번 관절각 $\theta_3$:
$$\theta_3 = \text{atan2}(\sin\theta_3, \cos\theta_3)$$

---

### [Step 4] 2번 관절각 ($\theta_2$) 계산 (삼각비 투영)
2번 관절 $\theta_2$는 어깨에서 조준선이 이루는 각도 $\alpha_{plane}$와 두 링크가 꺾이며 생기는 사잇각 $\beta_{plane}$를 조합하여 구합니다.

1. 조준선 전체 기울기 각도:
   $$\alpha_{plane} = \text{atan2}(Z_{plane}, R_{plane})$$
2. 링크 내부 기하 사잇각:
   $$\text{Height} = d_4 \sin\theta_3$$
   $$\text{Base} = a_2 + d_4 \cos\theta_3$$
   $$\beta_{plane} = \text{atan2}(\text{Height}, \text{Base}) = \text{atan2}(d_4 \sin\theta_3, a_2 + d_4 \cos\theta_3)$$
3. 2번 관절각 $\theta_2$:
   $$\theta_2 = \frac{\pi}{2} - (\alpha_{plane} + \beta_{plane})$$

#### 💡 [보충 설명] Height, Base, $\beta_{plane}$, $\theta_2$의 기하학적 의미
어깨 조인트에서 손목 중심점까지 뻗어 있는 링크들의 기하학적 각도 관계가 왜 저런 공식으로 정의되는지 단계별로 쉽게 이해해 봅시다.

##### A. Height와 Base는 어떻게 유도되었을까?
위팔($a_2$, 어깨에서 팔꿈치까지)을 하나의 **가상의 바닥 기준선**으로 생각하는 영리한 아이디어를 사용합니다. 위팔선을 팔꿈치($E$) 방향 너머로 일직선 연장하고, 손목 중심점($W$)에서 이 연장선에 **수직인 선**을 내려 수선의 발($H$)을 만듭니다.

그러면 다음과 같은 가상의 직각삼각형 $S-H-W$가 형성됩니다:

```
 어깨 (0,0)             팔꿈치 (E)            연장선 방향 (H)
   *======================*----------------------
    \        위팔 a2       \  d4 * cos(theta3)  |
     \                      \                   | Height = d4 * sin(theta3)
   s  \                      \                  |
       \                      \                 |
        \                      \                v
         \======================================* 손목 W (R_plane, Z_plane)
         <-------------- Base ----------------->
         <--     a2     --><--  d4 * cos(theta3)-->
```

- **높이 (Height)**: 팔꿈치 관절 꺾임각이 $\theta_3$이므로, 아래팔($d_4$)이 위팔 연장선에 비해 수직으로 높이 솟은 거리는 삼각비에 의해 다음과 같습니다.
  $$\text{Height} = d_4 \sin\theta_3$$
- **밑변 (Base)**: 어깨 관절에서 수선의 발 $H$까지의 총 직선거리입니다. 이는 위팔 길이($a_2$)에 아래팔이 위팔 방향으로 연장된 투영 길이($d_4 \cos\theta_3$)를 더한 값입니다.
  $$\text{Base} = a_2 + d_4 \cos\theta_3$$

##### B. $\beta_{plane}$ (기하 사잇각)의 의미
직각삼각형 $S-H-W$에서 **어깨 부위에 생기는 예각**을 $\beta_{plane}$라고 정의합니다. 즉, **"실제 위팔($a_2$)의 축선"**과 **"어깨에서 손목으로 최단거리로 연결한 가상 조준선($s$)"** 사이의 사잇각입니다.
$$\tan(\beta_{plane}) = \frac{\text{Height}}{\text{Base}}$$
$$\beta_{plane} = \text{atan2}(\text{Height}, \text{Base}) = \text{atan2}(d_4 \sin\theta_3, a_2 + d_4 \cos\theta_3)$$

##### C. $\theta_2$ (2번 관절각) 공식의 최종 유도
이제 시선을 로봇 팔 전체가 속한 2D 평면 좌표계($R_{plane}-Z_{plane}$)로 옮겨 봅니다.

```
       Z축 (세로축)
        |
        | t2 (위팔선이 Z축과 이루는 각도)
        |   \
        |    \  위팔선 (Shoulder -> Elbow)
        |     \
        |      \  beta (위팔선과 조준선 사이의 사잇각)
        |       \
        |        \  조준선 (Shoulder -> Wrist Center)
        |         \
        |__________\ alpha (조준선이 가로 R축과 이루는 각도)
     Shoulder(0,0)--------------------------------> R_plane (가로축)
```

1. 세로축(Z축)과 가로축(R축)이 이루는 사잇각은 직교하므로 정확히 $90^\circ$ ($\frac{\pi}{2}$ rad) 입니다.
2. Z축부터 R축까지 채워진 전체 각도는 기하학적으로 세 각도의 합으로 구성됩니다:
   $$\text{Z축에서 R축까지의 각도} = \theta_2 + \beta_{plane} + \alpha_{plane} = \frac{\pi}{2}$$
3. 우리가 구하고자 하는 값은 세로 Z축을 기준으로 기울어진 관절각 $\theta_2$이므로, 이항하여 정리하면 최종 공식이 도출됩니다.
   $$\theta_2 = \frac{\pi}{2} - (\alpha_{plane} + \beta_{plane})$$

##### 📌 [이해가 안 될 때 보는 1초 피드백] 구체적인 각도 숫자로 대입해 보기
이 공식이 직관적으로 이해되지 않는다면, 실제 각도 수치를 대입해서 각도들의 합을 시각적으로 확인해 봅시다.

예를 들어, 목표점(손목)이 **가로축(R) 대비 $45^\circ$** 방향에 있다고 가정하겠습니다.
1. **$\alpha_{plane}$**: 조준선이 가로 R축과 이루는 각도이므로 **$45^\circ$** 입니다.
2. **남은 각도**: 세로 Z축($90^\circ$)과 가로 R축($0^\circ$)은 직각($90^\circ$)이므로, 세로 Z축에서 조준선까지의 남은 영역은 **$90^\circ - 45^\circ = 45^\circ$** 입니다.
3. **$\theta_2$ 와 $\beta_{plane}$**:
   - 세로 Z축을 기준으로 위팔이 앞으로 기울어지는 각도를 **$\theta_2 = 15^\circ$** 라고 해봅시다.
   - 그렇다면 위팔선과 조준선 사이의 벌어진 각도 **$\beta_{plane}$**는 당연히 남은 사잇각에서 $\theta_2$를 뺀 **$45^\circ - 15^\circ = 30^\circ$**가 됩니다.
4. **전체 합산**:
   - $\theta_2(15^\circ) + \beta_{plane}(30^\circ) + \alpha_{plane}(45^\circ) = 90^\circ$
   - 따라서, $\theta_2 = 90^\circ - (\alpha_{plane} + \beta_{plane})$ 공식이 완벽하게 성립합니다.

---

### [Step 5] 4, 5, 6번 관절각 ($\theta_4, \theta_5, \theta_6$) 계산 (자세 추출)
앞서 구한 $\theta_1, \theta_2, \theta_3$를 조합하여 베이스에서 3축까지의 회전 성분 $R_0^3$를 행렬 곱으로 계산합니다.
$$R_0^3 = R_0^1(\theta_1) \cdot R_1^2(\theta_2) \cdot R_2^3(\theta_3)$$

손목이 담당해야 할 순수 회전 행렬 $R_3^6$는 전체 목표 자세 $R_{target}$에서 앞부분 회전 성분을 나눠서(전치행렬 곱) 고립시킵니다:
$$R_3^6 = (R_0^3)^T \cdot R_{target} = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}$$

이 회전 행렬은 손목 관절의 구성인 ZYZ 오일러 회전 행렬의 대수적 모양과 일대일로 비교하여 각도를 추출합니다:
1. **$\theta_5$ 계산**:
   $$\cos\theta_5 = r_{33} \implies \theta_5 = \text{atan2}(\pm\sqrt{1 - r_{33}^2}, r_{33})$$
   (여기서 $+$ 부호 선택 시 Wrist No-Flip, $-$ 부호 선택 시 Wrist Flip 해가 나옵니다.)
2. **$\theta_4$ 및 $\theta_6$ 계산** (일반적인 경우, $\sin\theta_5 \neq 0$):
   $$\theta_4 = \text{atan2}\left(\frac{r_{23}}{\sin\theta_5}, \frac{r_{13}}{\sin\theta_5}\right)$$
   $$\theta_6 = \text{atan2}\left(\frac{r_{32}}{\sin\theta_5}, \frac{-r_{31}}{\sin\theta_5}\right)$$

#### 💡 [보충 설명] 손목 회전(ZYZ) 고립과 각도 추출의 수학적/물리적 의미
4, 5, 6번 관절(손목)은 로봇 말단 장치(Tool)의 '방향(Orientation)'만 담당합니다. 이 관절들이 전체 자세 $R_{target}$에 부합하도록 정렬되는 과정을 단계별로 알기 쉽게 풀어봅시다.

##### A. 왜 전치행렬 $(R_0^3)^T$을 목표 자세 앞에 곱할까?
로봇의 총 회전 행렬 $R_{target}$은 베이스에서 3축까지의 회전 $R_0^3$와, 3축에서 최종 6축(손목)까지의 회전 $R_3^6$가 순차적으로 곱해진 결과입니다:
$$R_0^3 \cdot R_3^6 = R_{target}$$

우리의 목표는 이미 알고 있는 $R_0^3$(앞서 구한 $\theta_1, \theta_2, \theta_3$로 만든 수치 행렬)과 $R_{target}$(최종 목표 자세)을 가지고 **미지수가 포함된 손목 행렬 $R_3^6$만 고립**시키는 것입니다.

행렬 곱의 순서를 유지하면서 $R_0^3$를 없애기 위해, 양변의 **왼쪽**에 역행렬 $(R_0^3)^{-1}$을 곱합니다:
$$(R_0^3)^{-1} \cdot R_0^3 \cdot R_3^6 = (R_0^3)^{-1} \cdot R_{target}$$
$$I \cdot R_3^6 = (R_0^3)^{-1} \cdot R_{target}$$

이때, **회전 행렬은 '직교 행렬(Orthogonal Matrix)'**이므로 역행렬이 곧 전치 행렬(Transpose)이라는 매우 편리한 성질을 가집니다:
$$(R_0^3)^{-1} = (R_0^3)^T$$

따라서, 아래와 같이 손목이 담당해야 하는 순수 회전 성분 $R_3^6$을 구해낼 수 있습니다:
$$R_3^6 = (R_0^3)^T \cdot R_{target}$$

##### B. ZYZ 오일러 회전 행렬의 대수적 구조
로봇의 4축(Z축 회전), 5축(Y축 회전), 6축(Z축 회전)은 순차적으로 연결되어 ZYZ 오일러 각을 형성합니다. 이를 기본 회전 행렬의 곱으로 나타내면 다음과 같습니다:
$$R_3^6 = R_z(\theta_4) \cdot R_y(\theta_5) \cdot R_z(\theta_6)$$

세 개의 행렬을 순서대로 곱해 전개하면 다음과 같은 대수적 형태가 도출됩니다:
$$R_3^6 = \begin{bmatrix} \cos\theta_4\cos\theta_5\cos\theta_6 - \sin\theta_4\sin\theta_6 & -\cos\theta_4\cos\theta_5\sin\theta_6 - \sin\theta_4\cos\theta_6 & \cos\theta_4\sin\theta_5 \\ \sin\theta_4\cos\theta_5\cos\theta_6 + \cos\theta_4\sin\theta_6 & -\sin\theta_4\cos\theta_5\sin\theta_6 + \cos\theta_4\cos\theta_6 & \sin\theta_4\sin\theta_5 \\ -\sin\theta_5\cos\theta_6 & \sin\theta_5\sin\theta_6 & \cos\theta_5 \end{bmatrix}$$

이 문자식 행렬을 우리가 앞서 수치적으로 계산해 둔 행렬 $R_3^6 = \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}$ 과 원소별로 비교하며 각도들을 하나씩 구하게 됩니다.

##### C. $\theta_5$ 계산과 Wrist Flip / No-Flip의 물리적 의미
행렬의 맨 우측 하단($3$행 $3$열) 원소를 비교하면 다음과 같은 식을 얻습니다:
$$r_{33} = \cos\theta_5$$

각도를 구하기 위해 삼각함수 항등식 $\sin^2\theta_5 + \cos^2\theta_5 = 1$을 활용하여 $\sin\theta_5$를 계산합니다:
$$\sin\theta_5 = \pm\sqrt{1 - \cos^2\theta_5} = \pm\sqrt{1 - r_{33}^2}$$

이를 통해 `atan2` 함수에 대입해 $\theta_5$를 결정합니다:
$$\theta_5 = \text{atan2}(\pm\sqrt{1 - r_{33}^2}, r_{33})$$

이때 **$\pm$ 부호의 선택**은 물리적으로 로봇 손목이 꺾이는 구조를 정의합니다:

```
    [Wrist No-Flip (+)]                  [Wrist Flip (-)]
         5축 (꺾임)                           5축 (반대로 꺾임)
           /\                                     \/
          /  \                                   /  \
     4축 /    \ 6축                         4축 /    \ 6축
        /      \                               /      \
      (정방향 정렬)                           (4, 6축이 180도 회전하여
                                               말단 방향을 맞춤)
```

1. **Wrist No-Flip ( $+$ 부호 선택 )**: 5번 관절각 $\theta_5$가 양의 각도($0 \sim 180^\circ$)를 가지며 가장 자연스럽고 직관적인 형태로 손목을 정방향 정렬합니다.
2. **Wrist Flip ( $-$ 부호 선택 )**: 5번 관절각 $\theta_5$가 음의 각도($-180^\circ \sim 0$)로 반대로 꺾입니다. 이 경우 말단 장치(Tool)의 최종 방향을 No-Flip과 동일하게 유지하기 위해, **4축($\theta_4$)과 6축($\theta_6$)이 각각 $180^\circ$ 근처로 반전**하게 됩니다. 좁은 공간이나 구조물 간섭이 있을 때 회피 자세로 유용하게 쓰입니다.

##### D. $\theta_4$ 와 $\theta_6$ 각도의 수학적 유도
$\theta_5$를 알고 나면, 남은 원소들의 식에서 $\sin\theta_5$를 활용하여 $\theta_4$와 $\theta_6$를 독점적으로 떼어낼 수 있습니다. (일반적인 경우, $\sin\theta_5 \neq 0$)

* **$\theta_4$ 구하기 (3열 성분 이용)**:
  $$r_{13} = \cos\theta_4\sin\theta_5 \implies \cos\theta_4 = \frac{r_{13}}{\sin\theta_5}$$
  $$r_{23} = \sin\theta_4\sin\theta_5 \implies \sin\theta_4 = \frac{r_{23}}{\sin\theta_5}$$
  이를 `atan2(y, x)` 형태인 `atan2(sin, cos)`에 대입하면:
  $$\theta_4 = \text{atan2}\left(\frac{r_{23}}{\sin\theta_5}, \frac{r_{13}}{\sin\theta_5}\right)$$

* **$\theta_6$ 구하기 (3행 성분 이용)**:
  $$r_{31} = -\sin\theta_5\cos\theta_6 \implies \cos\theta_6 = \frac{-r_{31}}{\sin\theta_5}$$
  $$r_{32} = \sin\theta_5\sin\theta_6 \implies \sin\theta_6 = \frac{r_{32}}{\sin\theta_5}$$
  동일하게 대입하면:
  $$\theta_6 = \text{atan2}\left(\frac{r_{32}}{\sin\theta_5}, \frac{-r_{31}}{\sin\theta_5}\right)$$

##### E. [특수한 예외] 특이점(Singularity) 상황 처리
만약 $\sin\theta_5 = 0$ 이라면(즉, $\theta_5 = 0$ 또는 $\theta_5 = \pi$), 분모가 0이 되어 위의 $\theta_4, \theta_6$ 공식은 수학적으로 정의되지 않습니다.

물리적으로는 **4축의 회전축선과 6축의 회전축선이 완벽히 일직선 상에 겹치는 상태**가 됩니다. 이때는 두 축이 따로 돌 필요 없이 둘이 합쳐서 하나의 큰 축처럼 동작하기 때문에, 두 관절의 각도 합(또는 차)만 결정되고 개별 각도는 무한히 많은 조합이 존재하게 됩니다. (Gimbal Lock 현상)

실제 제어에서는 이 특이점을 해결하기 위해 다음과 같이 예외 처리를 수행합니다.

1. **$\theta_5 \approx 0$ 인 경우 (일직선 상태)**:
   이때 회전 행렬은 $R_3^6 = R_z(\theta_4 + \theta_6)$ 의 형태가 됩니다.
   $$\theta_4 + \theta_6 = \text{atan2}(r_{21}, r_{11})$$
   따라서 관절의 급격한 회전을 방지하기 위해 **4축을 현재 각도로 고정(예: $\theta_4 = 0$)**하고, 나머지 회전량을 6축에 전부 몰아줍니다:
   $$\theta_4 = 0, \quad \theta_6 = \text{atan2}(r_{21}, r_{11})$$

2. **$\theta_5 \approx \pi$ (또는 $-\pi$) 인 경우 (접힌 일직선 상태)**:
   이때 회전 행렬은 $R_3^6 = R_z(\theta_4 - \theta_6)R_y(\pi)$ 의 형태가 됩니다.
   $$\theta_4 - \theta_6 = \text{atan2}(r_{21}, -r_{11})$$
   마찬가지로 **4축을 고정(예: $\theta_4 = 0$)**하고 해결합니다:
   $$\theta_4 = 0, \quad \theta_6 = -\text{atan2}(r_{21}, -r_{11})$$

---

## 5. 결론 및 요약

이와 같이 수식의 복잡함 뒤에는 아주 단순하고 우아한 기하학적 법칙들이 숨겨져 있습니다.

- **기구학적 분리(구형 손목)** 덕분에 $3\text{축 위치} + 3\text{축 자세}$로 나누어 풀 수 있으며,
- 사분면 예외를 해결하기 위해 항상 **`atan2(y, x)`**를 기저에 깔고 계산하고,
- 링크가 이루는 임의의 삼각형은 **제2코사인 법칙**으로 각도를 풀어냅니다.
- 마지막으로 남은 오리엔테이션 오차 행렬에서 **ZYZ 오일러 각 추출**을 통해 손목의 회전각들을 차례로 뽑아냄으로써 6축 분석적 역운동학 알고리즘이 마침내 완결됩니다.
