# Module 03 — PhysX 물리 심화

**권장 소요: 이론 4h · 실습 7h**

---

## 1. 학습 목표

- Rigid Body / Collision / Articulation API가 각각 무엇을 담당하는지 설명할 수 있다.
- Collision Approximation 종류(convex hull, convex decomposition, triangle mesh, SDF)의 차이와 선택 기준을 설명할 수 있다.
- Joint Drive(stiffness/damping)와 solver iteration이 안정성에 미치는 영향을 원리 수준에서 이해한다.
- 임의의 메시로부터 mass/inertia를 직접 계산하거나 자동 계산 도구로 부여할 수 있다.
- "일부러 망가뜨린" 오브젝트를 처음 보고도 원인을 진단할 수 있다.

## 2. 선수 지식 확인

- 이전 과정(ROS2 3주차)에서 이미 "떨림/바닥 통과/관절 반전/무게중심" 4종 문제를 진단해본 적이 있다면, 이번 모듈은 그 경험을 "왜 그런 현상이 생기는가"의 원리 레벨로 재구성하는 것임을 인지 (`~/Documents/ros2/03-isaac-sim-urdf-import-and-gait-video.md` 3장 참고 가능).
- 없다면 모듈 2의 standalone 스크립트 작성 능력만 있으면 충분.

---

## 3. 이론 세션

### 3.1 물리 관련 API 세 갈래

| API | 역할 | 없으면 생기는 일 |
|---|---|---|
| RigidBodyAPI | Prim이 물리 엔진에 의해 움직이는 강체가 됨 | 물리적으로 정적인 배경 취급(충돌은 하지만 힘을 받아도 안 움직임) |
| CollisionAPI (+ Approximation) | 충돌 형상 정의 | 다른 물체를 그대로 통과함 |
| ArticulationRootAPI + Joint APIs | 여러 강체를 관절로 묶어 하나의 제어 가능한 기구로 만듦 | 개별 링크가 따로 노는 자유 강체 뭉치가 됨 |

Rigid Body만 있고 Collision이 없는 프리미티브를 만들어 다른 오브젝트를 그대로 통과하는 것을 직접 재현해보는 것이 가장 빠른 체득 방법이다.

### 3.2 Collision Approximation 선택 기준

| 종류 | 특징 | 언제 쓰나 |
|---|---|---|
| Convex Hull | 원본 메시를 감싸는 볼록 껍질 하나 | 가장 빠름. 대략적인 형상이면 충분한 링크 |
| Convex Decomposition | 여러 개의 convex hull 조합으로 오목한 형상 근사 | 오목한 부분이 중요한 형상(그리퍼, 손 등) |
| Triangle Mesh | 원본 메시 그대로 | 정적 배경(지면, 벽)에만 권장 — 동적 강체끼리 쓰면 불안정/성능 저하 |
| SDF (Signed Distance Field) | 정밀 충돌, 계산 비용 높음 | 얇은 형상, 정밀 접촉이 중요한 경우(파지 등) |

**강의 포인트**: "동적으로 움직이는 두 물체 모두 Triangle Mesh로 충돌시키면 불안정해진다"는 것은 실습에서 직접 재현해 확인한다(의도적으로 Triangle Mesh 설정 후 물체가 서로 통과하거나 튕기는 것을 관찰).

### 3.3 Joint Drive와 안정성

- Position Drive는 목표 각도까지 stiffness(강성)와 damping(감쇠)으로 도달하는 스프링-댐퍼 모델이다.
- stiffness가 너무 크고 damping이 부족하면 목표값 주변에서 진동(jitter)한다. 반대로 stiffness가 너무 작으면 중력/외력을 못 버티고 처진다.
- Solver Position/Velocity Iteration Count가 부족하면 복잡한 관절 체인(다리처럼 링크가 여러 개 이어진 구조)에서 발산하기 쉽다 — 링크 수가 많을수록 iteration을 올려야 하는 경향이 있다는 것을 실습으로 확인.

### 3.4 Mass와 Inertia

- 기본적으로 형상+밀도로부터 자동 계산되지만, 임포트 과정에서 대충 들어간 값이 남아있으면(예: 모든 링크가 동일한 기본 inertia) 무게중심이 실제 형상과 어긋나 계속 한쪽으로 쓰러진다.
- Isaac Sim의 mass properties 자동 계산 도구(메시 기반)로 재계산하는 방법과, 필요시 Python으로 `PhysicsMassAPI`를 직접 읽고 쓰는 방법 둘 다 실습.

### 3.5 진단 방법론 정리 (표로)

| 증상 | 원인 후보 | 확인 방법 |
|---|---|---|
| 심하게 떨림(jitter) | stiffness/damping 불균형, solver iteration 부족 | Property 패널에서 drive 값 확인, iteration count 올려보고 재현 여부 확인 |
| 바닥/다른 물체를 통과 | Collision API 누락, approximation 부적절, 초기 침투 | `get_prim_at_path`로 CollisionAPI 존재 확인, bbox로 초기 겹침 확인 |
| 관절이 반대로 꺾임 | joint axis 부호/limit 반전 | Property 패널의 axis, upper/lower limit 값 직접 확인 |
| 계속 한쪽으로 쓰러짐 | 무게중심(inertia)이 형상과 불일치 | mass properties 자동 계산 재적용 후 비교 |

---

## 4. 실습 가이드 (7h)

1. **RigidBody 없이/Collision 없이 재현 (1.5h)** — Cube 두 개를 만들어 각각 API를 하나씩 빼고 Play, 통과/무반응 현상을 직접 재현.
2. **Collision Approximation 비교 (2h)** — 오목한 메시(예: U자 모양) 오브젝트에 Convex Hull vs Convex Decomposition을 각각 적용해 충돌 정확도 차이를 시각적으로 비교.
3. **관절 체인 안정성 튜닝 (2h)** — 링크 3~4개짜리 간단한 팔 모양을 만들어(또는 기존 biped 재사용) stiffness/damping/iteration 값을 의도적으로 극단으로 설정해 발산을 재현한 뒤 정상 범위로 되돌리는 과정을 기록.
4. **Mass/Inertia 재계산 (1h)** — 기본값이 부여된 오브젝트의 무게중심을 확인 → 자동 계산 도구 적용 → 안정성 변화 비교.
5. **미니 진단 챌린지 (0.5h)** — 위 스크립트들 중 하나를 골라 파라미터를 무작위로 하나 망가뜨린 뒤, 시간을 재며 표 3.5를 활용해 스스로 진단.

결과와 진단 기록은 `practicum/m03_physx/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. Convex Hull로 바꿨는데도 여전히 통과해요.**
A. 재계산(bake)이 필요한 경우가 있다 — approximation 타입을 바꾼 뒤 collision mesh를 다시 생성하는 옵션/버튼을 확인.

**Q. Solver iteration을 올려도 여전히 떨려요.**
A. 이 경우 iteration 부족이 아니라 drive stiffness가 형상 대비 과도한 경우가 많다 — iteration을 올리기 전에 먼저 stiffness를 낮춰서 재현되는지 확인하는 순서를 권장(진단 순서 자체가 중요).

**Q. 물체 두 개가 서로 미세하게 겹친 채 시작하면 어떻게 되나요?**
A. PhysX가 겹침을 강하게 밀어내려 하면서 순간적으로 튕겨나가는 현상(explosion)이 생길 수 있다. 초기 배치 시 약간의 여유(margin)를 두는 습관을 들인다.

---

## 6. 체크포인트 & 과제

**체크포인트**: 처음 보는 "일부러 망가뜨린" 오브젝트를 표 3.5만 참고해 원인 후보를 좁히고 실제로 수정할 수 있다.

**과제**: 임의의 다관절 오브젝트(로봇 팔/다리 등)를 골라 stiffness/damping/solver iteration 3개 파라미터를 각각 바꿔가며 안정성이 어떻게 변하는지 최소 6개 조합(2×3 등)을 표로 정리 — 다음 모듈에서 실제 매니퓰레이터를 다룰 때 이 표의 직관을 재사용한다.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "Physics" / "Rigid Body Simulation" / "Articulations" 섹션
- PhysX 공식 문서 — Solver iteration, joint drive 파라미터 설명
- `~/Documents/isaacsim/inspect_physics_schema.py`, `inspect_collision.py`, `diag_1_jitter.py` — 이전 과정에서 쓴 진단 스크립트, 그대로 재사용 가능

---
이전: [`02-python-scripting-api.md`](./02-python-scripting-api.md) · 다음: [`04-robot-import-and-manipulation.md`](./04-robot-import-and-manipulation.md)
