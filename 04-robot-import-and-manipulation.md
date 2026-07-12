# Module 04 — 로봇 임포트와 매니퓰레이션

**권장 소요: 이론 3h · 실습 8h**

---

## 1. 학습 목표

- URDF/MJCF Importer가 내부적으로 무엇을 하는지(link→Xform, joint→PhysX Joint 변환) 설명할 수 있다.
- 제공되는 Franka 매니퓰레이터 에셋으로 articulation 제어와 간단한 pick-and-place를 구현할 수 있다.
- Articulation Controller의 position/velocity/effort 제어 모드 차이를 이해한다.
- 기본적인 IK(역기구학) 도구를 사용해 엔드이펙터 목표 pose로 팔을 움직일 수 있다.

## 2. 선수 지식 확인

- 모듈 3에서 정리한 joint drive 안정성 직관(stiffness/damping/iteration)을 매니퓰레이터에도 그대로 적용한다.
- 이전 과정에서 biped 다리 URDF를 임포트해봤다면, 이번엔 "다리(위치 제어 위주, 접지)"가 아니라 "팔(정밀한 엔드이펙터 pose 제어)"이라는 다른 종류의 문제라는 점을 미리 짚는다.

---

## 3. 이론 세션

### 3.1 URDF/MJCF Importer가 하는 일

- `<link>` → Xform + RigidBody + Collision Prim
- `<joint>` → PhysX Joint(revolute/prismatic/fixed) + JointDriveAPI
- Import 옵션이 결과에 미치는 영향(모듈 3에서 다룬 것과 동일한 축): Fix Base Link, Self Collision, Joint Drive 초기값
- MJCF(Mujoco) 임포터도 존재하며, Isaac Lab의 다수 예제 로봇이 MJCF 기반이라는 점을 언급 (다리/사족 로봇 계열에서 자주 보임)

### 3.2 매니퓰레이터를 다루는 이유

지금까지(이전 과정 포함)는 계속 "다리 → 이동/보행"만 다뤘다. 로보틱스의 다른 큰 축은 "팔 → 조작(manipulation)"이며, 접근 방식이 근본적으로 다르다:
- 다리: 접지/균형이 핵심, 목표는 주로 관절 각도 시퀀스
- 팔: 엔드이펙터(그리퍼 끝)가 3D 공간의 목표 pose에 도달하는 것이 핵심 → 역기구학(IK) 필요

### 3.3 Franka 에셋과 Articulation Controller

Isaac Sim은 Franka Emika Panda 로봇 팔 예제 에셋을 기본 제공한다.

```python
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation

add_reference_to_stage(usd_path=FRANKA_USD_PATH, prim_path="/World/Franka")
franka = world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
world.reset()

franka.get_articulation_controller().apply_action(
    ArticulationAction(joint_positions=target_positions)
)
```

- Articulation Controller가 position/velocity/effort 세 가지 제어 모드를 제공하며, pick-and-place처럼 정밀한 위치 제어가 중요한 경우 기본적으로 position 제어를 쓴다는 것을 확인.
- 그리퍼는 별도의 2-DOF(또는 커플링된 1-DOF) 관절로 존재 — 팔 관절과 분리해서 다뤄야 함을 `dof_names`로 확인.

### 3.4 기본 IK

- Isaac Sim/Isaac Lab 계열에서 제공하는 IK 솔버(예: Lula Kinematics 또는 differential IK)를 사용해 "엔드이펙터를 이 pose로 보내라"는 목표를 관절 각도로 변환.
- 이번 모듈에서는 IK를 처음부터 구현하지 않고 **제공되는 IK 솔버를 사용**한다는 것을 명확히 안내 — 목적은 IK 자체의 수학이 아니라 "엔드이펙터 목표 → 관절 명령"이라는 파이프라인 흐름 이해.

### 3.5 Pick-and-Place 파이프라인 구조

```
1. 물체 위치 파악 (이번 모듈은 GT 위치를 코드에서 직접 읽음 — 인식은 모듈 5/6에서 다룸)
2. 목표 pose 계산 (물체 위 approach pose → grasp pose)
3. IK로 관절 각도 계산
4. Articulation Controller로 이동
5. 그리퍼 닫기 (effort/position 제어)
6. 목표 위치로 이동 → 그리퍼 열기
```

---

## 4. 실습 가이드 (8h)

1. **Franka 에셋 불러오기 및 dof_names 확인 (1h)** — 팔 7개 관절 + 그리퍼 관절을 이름으로 구분, 모듈 2에서 만든 이름-인덱스 매핑 헬퍼 재사용.
2. **Position 제어로 팔 흔들어보기 (1.5h)** — 임의의 관절 각도 시퀀스로 팔을 움직여보고, stiffness/damping을 모듈 3의 직관으로 튜닝(팔은 다리보다 정밀도 요구가 높다는 것을 체감).
3. **큐브 하나 Stage에 배치 + IK로 접근 (2h)** — 큐브 위 approach pose를 좌표로 지정, IK 솔버로 관절 각도 계산, 팔이 실제로 그 위치에 도달하는지 확인(엔드이펙터 pose를 FK로 재확인).
4. **그리퍼 제어 및 pick (2h)** — 그리퍼 닫기 → 큐브에 물리적으로 붙잡히는지(마찰/힘 확인) → 들어올리기.
5. **Place 및 전체 파이프라인 영상화 (1.5h)** — 목표 위치로 이동 후 그리퍼 열기까지 전체 시퀀스를 실행하고 영상으로 녹화.

결과 스크립트/영상은 `practicum/m04_manipulation/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. IK 결과 관절 각도로 이동시켰는데 엔드이펙터가 목표 위치에서 살짝 벗어나요.**
A. IK는 근사해를 찾는 경우가 많고(특히 여유자유도가 많은 7-DOF 팔), 여러 해 중 하나를 고를 수 있다. 완전히 일치하지 않는 것이 정상 범주인지, 아니면 좌표계(월드 vs 로봇 베이스 기준)를 착각한 것인지부터 구분한다.

**Q. 그리퍼를 닫아도 물체가 안 붙잡혀요(뚫고 지나감).**
A. 모듈 3의 Collision Approximation 문제와 동일한 계열 — 그리퍼 손가락과 물체 모두에 적절한 Collision이 있는지, 마찰(friction) 값이 너무 낮지 않은지 확인.

**Q. 팔이 목표에 도달하기 전에 진동해요.**
A. 모듈 3에서 정리한 표를 그대로 적용 — position drive의 stiffness/damping을 재조정.

---

## 6. 체크포인트 & 과제

**체크포인트**: Franka 팔로 Stage 위 임의 위치의 큐브를 pick해서 다른 위치에 place하는 전체 시퀀스가 안정적으로 동작한다.

**과제**: 큐브의 시작 위치를 매 실행마다 무작위로 바꿔도(코드에서 랜덤 좌표 생성) pick-and-place가 성공하도록 스크립트를 일반화 — 다음 모듈에서 카메라로 물체 위치를 "인식"하는 것으로 이 하드코딩된 GT 위치 읽기를 대체하게 된다.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "Franka" 예제, "Manipulators" 섹션
- Isaac Lab의 `isaaclab_assets`, `isaaclab_tasks` 내 manipulation 태스크 예제 코드 (`~/IsaacLab/source/isaaclab_tasks`)
- Lula Kinematics / Differential IK 문서

---
이전: [`03-physx-deep-dive.md`](./03-physx-deep-dive.md) · 다음: [`05-sensors-and-omnigraph.md`](./05-sensors-and-omnigraph.md)
