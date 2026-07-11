# Module 04 — 로봇 임포트와 매니퓰레이션: Franka로 Pick-and-Place

이 문서는 Franka Emika Panda 로봇 팔을 GUI로 배치하고, standalone 스크립트로 관절 제어 → IK → pick-and-place → 영상 녹화까지 진행하는 실습 가이드다. Module 2·3에서 다진 Python API/PhysX 직관을 매니퓰레이터에 그대로 적용한다.

---

## 준비물

- Isaac Sim 5.1.0, `isaacsim-run` alias
- Content Browser에서 Franka 에셋을 배치할 새 Stage

## 1. Franka 배치하기 (GUI)

### 단계별 실습
1. Content Browser에서 Franka(Panda) 에셋을 찾아 뷰포트로 드래그
2. Stage 패널에서 실제 prim 이름 확인 (드래그 위치에 따라 `/World/Franka`가 아니라 `/World/franka`처럼 대소문자가 다를 수 있으니 반드시 직접 확인)
3. `practicum/m04_manipulation/franka_scene.usd`로 저장

### ⚠️ 반드시 확인: 로봇 베이스가 월드 원점에 있는가
Content Browser에서 드래그한 위치에 그대로 놓이기 때문에, **원점이 아닌 임의 좌표에 배치될 수 있다.** 이후 모든 단계(IK, RMPFlow)에서 좌표를 "로봇 베이스 기준"인지 "월드 기준"인지 헷갈리면 큰 삽질로 이어진다 (실제로 이번 세션에서 이것 때문에 4~5단계를 통째로 다시 진단해야 했다 — [5절](#5-그리퍼로-큐브-집기--실패의-진짜-원인) 참고).

GUI에서 로봇 prim을 선택해 Property 패널의 **Translate가 (0, 0, 0)인지** 확인하고, 아니라면 지금 바로 (0, 0, 0)으로 옮겨서 저장해두는 것을 강력히 권장한다.

---

## 2. dof_names 확인하기

### 단계별 실습
`inspect_franka.py`를 직접 실행:
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u inspect_franka.py
```

### 결과
```
dof_names (9): ['panda_joint1', ..., 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
```
팔 7축 + 그리퍼 2축(별도 DOF, mimic 여부는 이후 Lula 경고로 확인). `isaacsim.core.prims.Articulation`(batched 뷰 클래스)로 읽으면 `get_joint_positions()`가 `(1, 9)` shape의 2차원 배열로 나온다 — Module 2의 `Robot` 클래스(1차원 반환)와 다른 점.

> **API 함정**: `Articulation(prim_path=...)`가 아니라 `Articulation(prim_paths_expr=...)`를 써야 한다 (batched 뷰라 여러 prim을 와일드카드로 지정하는 용도라서 파라미터 이름이 다르다).

---

## 3. Position 제어로 팔 흔들어보기

### 단계별 실습
`control_franka.py`로 팔 7축에 +0.3 rad 오프셋을 주고 200스텝 실행.

> **API 함정**: batched `Articulation` 뷰의 `apply_action()`은 `isaacsim.core.utils.types.ArticulationAction`(단수)이 아니라 **`ArticulationActions`(복수)**를 요구한다. 소스(`isaacsim.core.prims.impl.articulation.py`)를 직접 읽어서 확인했다 — 에러 메시지(`no attribute 'joint_names'`)만으로는 어느 클래스를 써야 하는지 알기 어려웠다.

### 결과
7개 관절 중 6개는 목표(+0.3)에 잘 수렴했는데, **`panda_joint4`만 시작값에서 전혀 안 움직였다.**

### 핵심 정리
GUI에서 `panda_joint4`의 Lower/Upper Limit을 확인하니 대략 `-176°`/`-3°`였고, 시작 위치가 이미 upper limit에서 1도 정도밖에 여유가 없는 상태였다. +0.3 rad(약 17°) 목표는 이 한계를 크게 초과하는 값이었는데, 물리적으로는 "한계까지만 이동하고 멈춰야" 정상인데도 **그 1도조차 전혀 안 움직이고 완전히 고정**됐다 — [Module 3의 axis-flip 실습](../m03_physx/README.md#진단-4--관절-limit-반전)에서 봤던 것과 같은 패턴("limit을 벗어나는 target은 부분 이동이 아니라 완전 정지로 처리되는 경향")이 매니퓰레이터에서도 재현된 것.

---

## 4. IK로 큐브에 접근하기

### 준비물
Franka용 Lula IK 설정 파일은 Isaac Sim 설치 경로에 이미 포함되어 있다:
- `.../isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml`
- `.../isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf`

### 단계별 실습
1. Ground Plane 위에 5cm Cube를 만들고 Rigid Body + Collider 추가, `(0.5, 0, 0.025)`에 배치 (바닥에 정확히 닿는 높이 = half-height)
2. `LulaKinematicsSolver(robot_description_path, urdf_path)` + `ArticulationKinematicsSolver(..., end_effector_frame_name="right_gripper")`로 IK 솔버 구성
3. `ik_reach_cube.py`로 큐브 위 approach pose `[0.5, 0, 0.25]`에 대해 IK 계산 → 적용 → FK로 재확인

### 결과
목표 `[0.5, 0, 0.25]` vs 실제 도달(FK) `[0.4995, -0.0001, 0.2488]` — **오차 약 1.3mm**로 성공.

실행 로그에 뜨는 아래 경고는 그리퍼 관절이 mimic으로 묶여있다는 뜻이며 무시해도 된다:
```
[Lula] Joint 'panda_finger_joint2' is specified as a mimic joint, but its control chain [...] terminates with a joint [...] that is not a c-space coordinate.
```

---

## 5. 그리퍼로 큐브 집기 — 실패의 진짜 원인

이 단계는 세 번의 실패를 거쳐 원인을 찾았다. 최종 해결책만 원하면 [5.3절](#53-진짜-원인--로봇-베이스가-원점이-아니었다)로 건너뛰어도 되지만, 진단 과정 자체가 이 모듈의 핵심 교훈이라 순서대로 남겨둔다.

### 5.1 첫 번째 실패: 큐브가 바닥에 떠 있었다

수동으로 작성한 `pick_cube.py`(approach → 하강 → 그리퍼 닫기 → 들어올리기)를 실행했더니, 그리퍼가 거의 완전히(0에 가깝게) 닫혔고 큐브는 오히려 아래로 "떨어진" 것처럼 기록됐다.

원인: 5cm 큐브(half-height 0.025)를 Translate Z=0.05로 배치하면 바닥과 2.5cm 간격이 뜬다. `world.reset()` 직후 읽은 좌표는 이 "뜬 상태"였고, 물리 스텝이 돌자마자 큐브가 바닥까지 떨어져 정착했다. 그 결과 팔은 (떠 있던 시절의) Z=0.05를 목표로 내려가 큐브보다 2.5cm 위 허공을 잡았다.

**해결**: 큐브 Translate Z를 0.025(half-height, 바닥에 정확히 닿는 값)로 수정.

### 5.2 두 번째 실패: 방향을 안 정했더니 여전히 헛손질

높이를 고쳐도 그리퍼는 여전히 큐브를 못 잡았다(finger가 저항 없이 거의 0까지 닫힘). 원인을 좁히려고 Isaac Sim이 공식 제공하는 `PickPlaceController`(`isaacsim.robot.manipulators`) 소스를 읽어보니, grasp 시 엔드이펙터 방향을 `euler_angles_to_quat([0, π, 0])`(수직 아래를 보도록 고정)로 명시적으로 지정하고 있었다 — 우리 스크립트는 방향을 지정하지 않고 IK가 자유롭게 고르게 뒀던 것이 문제로 의심됐다.

**시도한 해결**: 직접 방향을 튜닝하는 대신, Isaac Sim이 이미 제공하는 검증된 조합(`Franka` 래퍼 + `RMPFlowController` + `PickPlaceController`, 10단계 pick-and-place 상태 머신)으로 전환. 이게 교안이 원래 권장하는 "제공되는 IK 솔버를 사용" 방식이기도 하다.

### 5.3 진짜 원인 — 로봇 베이스가 원점이 아니었다

공식 컨트롤러로 바꿔도 여전히 실패했다(917스텝을 다 돌았는데 큐브가 한 번도 안 움직임). 그런데 엔드이펙터의 실제 world pose를 매 phase마다 찍어보니 큐브 근처에도 못 가고 있었다. `franka.get_world_pose()`로 로봇 베이스 자체를 확인하자 원인이 드러났다:

```
franka 베이스 world pose - position: [0.88157123 0.83373755 0.8714629] / orientation: [1. 0. 0. 0.]
```

**Content Browser에서 드래그한 로봇이 원점이 아니라 (0.88, 0.83, 0.87)에 배치되어 있었다.** `LulaKinematicsSolver`는 로봇 베이스가 원점에 있다고 가정하고 IK/FK를 계산하기 때문에, 4단계의 "IK 성공"(target과 FK가 거의 일치)은 사실 **같은 잘못된 가정 안에서 자기 자신과 비교한 결과라 항상 일치해 보였을 뿐**, 실제 월드 좌표로는 전혀 다른 곳을 짚고 있었던 것이다.

**해결**: GUI에서 로봇 Translate를 (0, 0, 0)으로 수정 → 저장. ([1절의 함정 콜아웃](#-반드시-확인-로봇-베이스가-월드-원점에-있는가)이 바로 이 사고를 막기 위한 것이다.)

### 5.4 최종 성공

로봇을 원점으로 옮긴 뒤 `pick_place_official.py`(`Franka` + `RMPFlowController` + `PickPlaceController`)를 재실행:

- 그리퍼가 `[0.0250, 0.0250]`에서 정확히 멈춤 — 큐브 half-width와 일치, 진짜로 잡힌 것
- 집는 동안 도달한 최대 높이 `0.254` (시작 `0.025` 대비 +23cm)
- 최종 위치 `[0.468, 0.332, 0.025]` vs place 목표 `[0.5, 0.3, 0.025]` — 오차 3~4cm로 근접 성공

---

## 6. 영상 녹화 — headless 카메라 캡처 실패, GUI Movie Capture로 전환

### 6.1 첫 시도: headless 카메라 스크립트 (실패)

`isaacsim.sensors.camera.Camera`로 씬 밖에서 대각선으로 바라보는 카메라를 만들고, 매 스텝 `get_rgba()`로 프레임을 모아 `imageio`로 mp4를 저장하는 방식을 시도했다. 결과물은 로봇/큐브가 전혀 안 보이고 항상 똑같은 회색-파란 격자 이미지만 나왔다.

디버깅 과정(카메라 world pose를 직접 확인 → 요청값과 정확히 일치, look-at 계산을 공식 `isaacsim.core.utils.rotations.lookat_to_quatf`로 교체, `simulation_app.update()`/`rep.orchestrator.step()`으로 렌더 파이프라인을 강제로 펌핑 등)로도 원인을 못 찾았다 — 카메라 prim의 transform은 정확한데도 렌더된 이미지가 그 transform과 무관하게 항상 동일했다. headless RTX 렌더러/synthetic-data 파이프라인 쪽의 더 깊은 설정 문제로 추정되며, 이 세션에서는 원인 규명을 포기했다.

### 6.2 최종: GUI Movie Capture로 전환

대신 `run_for_recording.py`(headless=False)로 GUI 창을 띄우고, 15초 카운트다운 동안 사람이 직접 뷰포트 카메라 각도를 잡은 뒤, Isaac Sim 내장 **Movie Capture** 툴로 화면을 녹화했다. pick-and-place 시퀀스 자체는 동일한 `PickPlaceController` 로직으로 자동 실행된다.

### 결과
`practicum/m04_manipulation/pick_and_place.webm` — 픽업부터 플레이스까지 전체 시퀀스가 담긴 영상, GUI로 직접 확인 완료.

**교훈**: headless synthetic-data 캡처가 안 풀릴 때, GUI Movie Capture(사람이 직접 보면서 녹화)로 전환하는 것도 합리적인 선택지다 — 특히 이 커리큘럼처럼 "사람이 직접 보고 확인"하는 것을 원칙으로 하는 경우 더더욱 그렇다.

---

## 체크리스트

- [x] Franka를 GUI로 배치하고 베이스가 원점에 있는지 확인했는가
- [x] `dof_names`로 팔/그리퍼 관절 구성을 확인했는가
- [x] Position 제어로 관절을 흔들어보고, 관절 limit에 의한 정지 현상을 진단했는가
- [x] IK로 엔드이펙터를 목표 pose에 도달시키고 FK로 재확인했는가
- [x] 그리퍼로 큐브를 실제로 집어서 들어올렸는가 (물리적 접촉 확인)
- [x] Pick-and-place 전체 시퀀스를 영상으로 기록했는가

## 알려진 문제와 해결

| 관찰 | 원인 | 비고 |
|---|---|---|
| `Articulation(prim_path=...)` TypeError | batched 뷰 클래스는 `prim_paths_expr` 파라미터를 씀 | `isaacsim.core.api.robots.Robot`(Module 2)과 `isaacsim.core.prims.Articulation`은 다른 클래스 |
| `ArticulationAction`에 `joint_names` 없다는 에러 | batched `Articulation.apply_action()`은 단수형이 아니라 복수형 `ArticulationActions`를 요구 | 소스 직접 확인(`isaacsim.core.prims.impl.articulation.py`)으로 해결 |
| `panda_joint4`만 목표에 안 움직임 | 시작 위치가 실제 joint limit(upper ≈ -3°) 바로 근처였고, target이 limit을 초과 | limit 초과 target은 "한계까지 이동"이 아니라 "완전 정지"로 처리됨 — Module 3 axis-flip과 동일 패턴 |
| 큐브를 집으려 해도 그리퍼가 저항 없이 닫힘 (1차) | 큐브가 바닥에서 2.5cm 뜬 채로 배치되어 있었고, 물리 시작 후 조용히 떨어져 정착 — 팔은 옛 좌표를 목표로 삼음 | 바닥에 놓는 오브젝트는 Translate Z = half-height로 배치 |
| 그리퍼가 저항 없이 닫힘 (2차, 방향 미지정 IK) | grasp 시 엔드이펙터 방향을 지정하지 않아 IK가 임의 방향을 선택 | 공식 `PickPlaceController`는 `euler_angles_to_quat([0, π, 0])`로 방향을 고정함 |
| 공식 컨트롤러로 바꿔도 여전히 실패, ee 위치가 큐브 근처에도 안 감 | **로봇 베이스가 원점이 아니라 (0.88, 0.83, 0.87)에 배치되어 있었음** | `LulaKinematicsSolver`가 베이스=원점을 가정해서, "IK 성공"이 실제로는 잘못된 좌표계 안에서의 자기 일치였을 뿐이었음 |
| headless 카메라로 찍은 영상에 로봇/큐브가 안 보이고 항상 같은 격자 이미지만 나옴 | 원인 불명 (카메라 transform은 정확했는데도 렌더 결과가 무관했음) | GUI Movie Capture로 전환해서 해결 — [6절](#6-영상-녹화--headless-카메라-캡처-실패-gui-movie-capture로-전환) 참고 |

## 남은 과제 (아직 안 한 것)

- 큐브 시작 위치를 매 실행마다 무작위로 바꿔도 pick-and-place가 성공하도록 일반화 (교안 과제) — 다음 모듈에서 카메라 인식으로 GT 좌표 읽기를 대체하기 전에 해볼 만함

---
이전: [`03-physx-deep-dive.md`](../03-physx-deep-dive.md) · 참고: [`04-robot-import-and-manipulation.md`](../04-robot-import-and-manipulation.md) · 다음: [`05-sensors-and-omnigraph.md`](../05-sensors-and-omnigraph.md)
