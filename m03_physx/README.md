# Module 03 — PhysX 물리 심화: 4대 증상 직접 재현하기

이 문서는 Isaac Sim GUI에서 로봇/오브젝트의 물리 이상 증상 4가지(떨림·통과·반전·무게중심 쏠림)를 **직접 손으로 재현하고 원인을 눈으로 확인**하는 실습 가이드다. 이 문서를 따라 하면 [`03-physx-deep-dive.md`](../03-physx-deep-dive.md) 3.5절 진단표를 암기가 아니라 경험으로 익힐 수 있다.

## 준비물

- Isaac Sim 5.1.0, `isaacsim-run` alias (ROS2 불필요)
- Module 2에서 저장한 `practicum/m02_python_api/robot_scene.usd` 열기 — Denso `cobotta_pro_1300` 로봇이 `/World/cobotta_pro_1300`에 배치되어 있음
- 아래 실습 중 진단 1·4는 이 로봇의 `joint_3`을 그대로 쓰고, 진단 2·3은 별도의 간단한 오브젝트(Cube, Torus)를 새로 만들어 사용한다 — Collision/무게 관련 실험은 복잡한 로봇 메시보다 단순 도형이 원인을 훨씬 명확하게 보여주기 때문

`joint_3` 기본값: Axis=X, stiffness=`537889`, damping=`248`, Lower/Upper Limit=`-150°`/`150°`. Articulation 기본 Solver Position/Velocity Iteration Count=`32`/`16`.

---

## 진단 1 — 관절 떨림(Jitter)

### 배울 것
Position Drive(스프링-댐퍼 모델)에서 stiffness/damping/solver iteration이 각각 안정성에 미치는 영향.

### ⚠️ 시작 전에 꼭 알아야 할 함정
`joint_3`을 선택하고 stiffness/damping 값만 극단으로 바꾼 뒤 Play를 누르면 **아무 일도 안 일어난다.** 이유는 Position Drive의 힘이 다음 식으로 계산되기 때문이다.

```
force = stiffness × (target - 현재값) - damping × 속도
```

Target Position을 안 바꾸면 오차(target - 현재값)가 계속 0이라, 게인을 아무리 극단적으로 설정해도 **힘 자체가 0**이다. 그래서 이 실습은 반드시 Target Position을 원래값에서 **+30도 스텝**으로 강제로 바꿔서 오차를 만드는 것부터 시작한다.

### 관찰 방법
Target을 30도 스텝으로 주면 웬만한 조합은 육안으로 "출렁이는 것처럼" 비슷해 보여 구분이 안 된다. 대신 Play 중 `joint_3`을 선택해두고 Property 패널의 **실시간 현재 Position 값**을 관찰해서, 진동 폭(몇 도~몇 도 사이)과 정착 시간(몇 초 만에 멎는지)을 숫자로 비교한다.

### 단계별 실습
1. `joint_3` 선택, Target Position을 원래값 + 30도로 변경
2. 아래 표의 조합 순서대로 stiffness/damping/iteration을 바꿔가며 Play → Stop → 다음 조합, 매번 진폭·정착시간을 기록
3. Solver Position/Velocity Iteration Count는 `/World/cobotta_pro_1300` 루트(또는 base) prim의 Articulation 항목에서 조정

### 결과

| # | stiffness | damping | iteration (pos/vel) | 결과 |
|---|---|---|---|---|
| 1 | 5378890 (×10) | 0 | 32/16 (기본) | 28~32도 사이(±2도) 진동, ~2초 정착 |
| 2 | 537889 (원래) | 0 | 32/16 (기본) | 28~32도 사이(±2도) 진동, ~2초 정착 — **1번과 사실상 동일** |
| 3 | 5378890 (×10) | 248 (원래) | 32/16 (기본) | 거의 안 흔들리고 즉시 정착 |
| 4 | 5378890 (×10) | 0 | **1**/16 | 진폭 불규칙하게 큼, 4~5초간 지속 |
| 5 | 5378890 (×10) | 0 | **128**/16 | 1초 이내로 빠르게 정착 |
| 6 | 53789 (×0.1) | 248 (원래) | 32/16 (기본) | 오버슈트 없이 천천히 처지듯 목표 도달 |

### 핵심 정리
- **damping이 지배적 요인**: 조합 1·2는 stiffness가 10배 차이 나도 결과가 동일했다. 반면 조합 3처럼 damping만 원래값(248)으로 복원하면 stiffness가 10배여도 거의 안 흔들린다. 이 구간에서는 stiffness 크기보다 damping의 유무가 훨씬 결정적이다.
- **Solver Position Iteration Count도 독립적으로 큰 영향**: iteration을 1로 낮추면 진동이 불규칙하고 오래 지속되고, 128로 올리면 기본값(32)보다도 훨씬 빨리 정착한다. PhysX는 implicit solver라 iteration을 늘릴수록 constraint를 매 스텝 더 정확히 풀어내고, 이게 사실상 "수치적 damping"처럼 작용한다.
- **stiffness를 낮추면 진동이 아니라 처짐**: damping을 원래대로 두고 stiffness만 0.1배로 낮추면 오버슈트 없이 느리게 목표에 도달한다 — stiffness가 너무 작으면 중력/외력을 못 버티고 처진다는 이론과 일치.

---

## 진단 2 — Collision 유무에 따른 통과 현상

### 배울 것
RigidBody API와 Collision API가 서로 다른 역할을 한다는 것 — 하나만 있으면 어떤 일이 생기는지.

### 단계별 실습
1. `Create > Mesh > Cube`로 두 개 생성, Ground Plane 위 공중(Z=2)에 서로 겹치지 않게 배치
2. **Cube 1**: Property 패널에서 `Add > Physics > Rigid Body`만 추가 (Collider는 추가하지 않음)
3. **Cube 2**: `Add > Physics > Collider`만 추가 (Rigid Body는 추가하지 않음)
4. Play

### 결과
- **Cube 1** (RigidBody만): 중력을 받아 떨어지다가 Ground Plane을 **그대로 통과**함
- **Cube 2** (Collider만): Play 전후로 **Transform이 완전히 동일** — 정적 배경 취급되어 중력 자체를 안 받음

RigidBody 없이 Collision만 있으면 "힘을 받아도 안 움직이는 정적 배경", Collision 없이 RigidBody만 있으면 "다른 물체를 그대로 통과하는 동적 물체"가 된다는 걸 그대로 확인할 수 있다.

---

## 진단 3 — Collision Approximation과 오목 형상

### 배울 것
Convex Hull은 오목한 부분(구멍, 홈)을 무시하고 볼록 껍질로 감싸버린다는 것.

### 단계별 실습
1. `Create > Mesh > Torus`(도넛)를 Ground Plane 위(Z=1)에, 구멍이 위-아래로 뚫린 방향으로 배치
2. Torus에 `Add > Physics > Collider`만 추가 (RigidBody는 추가하지 않음 — 고정 장애물 역할)
3. Torus 구멍 중심 바로 위(같은 X/Y, Z=5)에 `Create > Mesh > Sphere` 생성, RigidBody + Collider 둘 다 추가 (Sphere 지름은 구멍보다 확실히 작게)
4. Torus의 Collider Approximation을 **Convex Hull**로 설정하고 Play → 결과 관찰
5. Stop, Sphere를 원래 높이로 되돌린 뒤 Torus Approximation을 **Convex Decomposition**으로 바꿔서 다시 Play

### 결과

| Torus Approximation | 결과 |
|---|---|
| Convex Hull | Sphere가 도넛 윗면에 얹힘 — 구멍이 막힌 것처럼 처리됨 |
| Convex Decomposition | Sphere가 구멍을 통과해 Ground Plane 바닥까지 도달 |

같은 형상, 같은 Sphere인데 Approximation 설정만 바꿔도 충돌 정확도가 완전히 달라지는 걸 직접 확인할 수 있다.

---

## 진단 4 — 관절 Limit 반전

### 배울 것
USD Physics의 RevoluteJoint는 axis가 X/Y/Z 토큰으로만 지정되어, URDF처럼 부호로 축 방향을 뒤집는 조작이 GUI에서 직접 되지 않는다. 대신 **Lower/Upper Limit 값을 서로 맞바꾸는 방식**으로 "반전된 limit" 버그를 재현한다.

### 단계별 실습
1. `joint_3` 선택, Lower/Upper Limit이 원래값(-150°/150°)인지 확인
2. Lower를 `150`, Upper를 `-150`으로 바꾼다 (숫자를 서로 맞바꿔 Lower > Upper인 모순 상태를 만듦)
3. Target Position에 +30도 스텝을 주고 Play

### 결과
관절이 **반대 방향으로 꺾이는 게 아니라, 완전히 정지**했다. Lower > Upper처럼 모순된 limit 조합에서는 PhysX가 움직임 자체를 차단하는 것으로 보인다.

> **교안과 다른 점**: [`03-physx-deep-dive.md`](../03-physx-deep-dive.md) 표 3.5는 이 증상을 "관절이 반대로 꺾임"으로 서술하지만, 실제로 Limit을 뒤집어 재현해보면 "완전히 멈춤"으로 나타났다. 실전에서 관절이 안 움직일 때, 힘/드라이브 문제가 아니라 limit 설정 자체를 의심해봐야 하는 이유다.

4. 실습 후 Lower/Upper를 반드시 원래값(-150°/150°)으로 복원할 것

---

## (미해결) 무게중심 쏠림 재현 — 도전해보고 싶다면

Module 3.4절 이론은 "무게중심이 형상과 어긋나면 계속 한쪽으로 쓰러진다"고 설명하지만, 이번 세션에서는 이걸 재현하는 데 **끝내 실패**했다. 시도한 것과 배제한 원인들을 정리해둔다 — 같은 실수를 반복하지 않도록, 혹은 직접 원인을 찾아보고 싶은 사람을 위해.

**먼저 확인한 것**: `/World/cobotta_pro_1300/world` 링크가 Module 2에서 "음수 질량" 경고를 냈던 것을 다시 조사해보니, 이 링크는 Mesh/Collision이 아예 없는 빈 Xform(더미 좌표계)이었다. 부피가 0이라 질량 계산(밀도×부피) 자체가 0으로 나누기가 되어 Center of Mass가 `-inf`로 나온 것 — 애초에 "재계산해서 고칠 대상"이 아니라 계산이 불가능한 케이스였다.

**실제 시도**: `joint_3`에 연결된 진짜 메시 링크(mass=1, 원래 Inertia는 "ignore"=자동계산, COM=(0,0,0))로 대상을 바꿔서 시도했다.

| 시도 | 조건 | 결과 |
|---|---|---|
| 1 | Center of Mass (0.5, 0, 0) | 정지 — joint_3 axis가 X라 오프셋이 회전축과 평행, `torque = r×F`에서 그 축 성분이 0이 됨 (물리적으로 타당한 결과) |
| 2 | Center of Mass (0, 0.5, 0) (축과 수직으로 변경) | 여전히 정지 |
| 3 | 2 + stiffness/damping을 0으로 낮춰 드라이브 비활성화 | 여전히 정지 |
| 4 | 3 + Inertia를 "ignore"(자동)에서 수동입력((0.1,0.1,0.1))으로 전환 | 여전히 정지 |

이론상 각가속도(`torque/inertia` ≈ 4.905/0.1 ≈ 49 rad/s²)는 상당히 커야 하는데도 전혀 반응이 없었다. 아래 원인들은 확인해서 **배제**했다:

- Limit이 진단 4 실습에서 되돌려지지 않은 것 아니냐 → 확인 결과 정상 복원(-150°/150°)되어 있었음
- Disable Gravity, Kinematic 플래그 → 둘 다 꺼져 있었음
- Diagonal Inertia가 0이라 계산 불능 → 0.1이었으므로 해당 없음 (참고: 값을 0,0,0으로 입력하면 자동으로 "ignore" 모드로 되돌아가는 UI 동작을 발견함)
- 별도 Velocity Drive가 브레이크처럼 작동 → 이 관절엔 Velocity Drive 섹션 자체가 없었음

**끝까지 확인하지 못한 것**: Play 전후 실제 Position 값을 비교하지 못했다 — 순간적으로 확 돌아서 Limit(±150°)에 걸려 멈췄을 가능성이 남아있다. 이걸 가장 먼저 확인해볼 것을 추천한다.

---

## 체크리스트

- [x] Position Drive가 오차 기반으로 힘을 계산한다는 것을 실제로 확인했는가
- [x] stiffness/damping/iteration 최소 6개 조합을 테스트했는가 (과제 요건)
- [x] 진동 폭/정착시간을 육안이 아니라 실시간 숫자값으로 비교했는가
- [x] Collision 유무에 따른 통과/무반응 재현
- [x] Collision Approximation 비교 (Convex Hull vs Decomposition)
- [x] 관절 반전(axis flip) 재현
- [ ] Mass/Inertia 무게중심 쏠림 재현 — 미해결
- [ ] 미니 진단 챌린지 — 이번 세션에서는 진행하지 않기로 결정

---
이전: [`m02_python_api/README.md`](../m02_python_api/README.md) · 참고: [`03-physx-deep-dive.md`](../03-physx-deep-dive.md)
