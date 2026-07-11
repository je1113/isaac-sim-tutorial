# Module 02 — Python 스크립팅 API 핵심

**권장 소요: 이론 3h · 실습 6h**

---

## 1. 학습 목표

- `SimulationApp` 부트스트랩 순서와 이유를 설명할 수 있다.
- `isaacsim.core.api.World`로 씬을 구성하고, `world.step()`/`world.reset()`의 역할을 설명할 수 있다.
- `isaacsim.core.utils` 계열 유틸리티(prim 생성, stage 참조 추가, USD attribute 읽기/쓰기)를 코드로 사용할 수 있다.
- standalone 스크립트와 Script Editor(Extension) 방식의 차이를 이해한다.

## 2. 선수 지식 확인

- 모듈 1에서 GUI로 파악한 Prim 트리 구조(`/World/...`)를 그대로 코드로 재현하는 것이 이번 모듈의 목표임을 인지.
- 이전 과정(ROS2 3주차)에서 `control_biped.py` 같은 standalone 스크립트를 이미 짜본 경험이 있다면, 그때는 결과 위주였다면 이번엔 각 API 호출이 정확히 무엇을 하는지 원리 수준으로 다시 본다.

---

## 3. 이론 세션

### 3.1 SimulationApp이 먼저 와야 하는 이유

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 이 아래에서만 omni.*, isaacsim.* 모듈 import 가능
from isaacsim.core.api import World
```

Isaac Sim의 Python 모듈들은 Kit 앱 컨텍스트가 생성된 뒤에만 정상적으로 import된다. `SimulationApp` 인스턴스화가 곧 Kit 확장 로딩이며, 이 순서를 어기면 import 에러가 나는 것이 초심자가 가장 흔히 겪는 실수다. 헤드리스(`headless: True`) 여부에 따라 GUI 렌더링 없이 물리만 빠르게 돌릴 수 있다는 점도 짚는다.

### 3.2 World, Scene, Stage의 관계

| 객체 | 역할 |
|---|---|
| `World` | 물리 시뮬레이션 루프(step/reset/play)를 감싸는 최상위 객체 |
| `World.scene` | World 안에 등록된 오브젝트(Robot, RigidObject 등)의 컬렉션 — `world.scene.add(...)` |
| Stage | USD 씬 그 자체 (모듈 1에서 다룬 Prim 트리). World는 내부적으로 현재 열린 Stage에 대해 동작 |

```python
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()  # 물리 핸들 초기화 — add 이후 반드시 호출
```

`world.reset()`을 빼먹으면 방금 추가한 오브젝트의 물리 핸들이 아직 준비되지 않아 `set_joint_positions` 등이 조용히 무시되거나 에러가 나는 경우가 많다 — 반드시 "씬 구성 완료 → reset → 제어 루프" 순서를 지킨다.

### 3.3 Prim 생성/참조 유틸리티

```python
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim, get_prim_at_path
from isaacsim.core.utils import prims

add_reference_to_stage(usd_path="/path/asset.usd", prim_path="/World/MyAsset")
create_prim(prim_path="/World/Cube", prim_type="Cube", position=(0, 0, 1))
prim = get_prim_at_path("/World/Cube")
```

모듈 1에서 GUI로 했던 "Reference로 에셋 불러오기"가 코드 한 줄(`add_reference_to_stage`)과 정확히 대응된다는 것을 실습에서 직접 비교한다.

### 3.4 Robot / Articulation 객체로 제어

```python
from isaacsim.core.api.robots import Robot

add_reference_to_stage(usd_path="/path/robot.usd", prim_path="/World/Robot")
robot = world.scene.add(Robot(prim_path="/World/Robot", name="my_robot"))
world.reset()

print(robot.dof_names)  # 관절 이름 순서 — 하드코딩 금지, 이름으로 매핑할 것
robot.set_joint_positions([0.3, -0.6, 0.0, 0.0])

for i in range(500):
    world.step(render=True)
```

`dof_names`로 관절 순서를 확인하지 않고 인덱스를 하드코딩하면, 임포트 방식이나 USD 파일이 바뀔 때 관절이 뒤바뀌는 사고가 난다(이전 과정에서 실제로 hip/knee가 인터리브된 순서였음을 확인한 바 있다) — 항상 이름 기반으로 매핑하는 습관을 들인다.

### 3.5 standalone 스크립트 vs Script Editor

- **standalone**: 터미널에서 `python3 script.py`로 실행, `SimulationApp`을 직접 생성. 자동화/배치 실행/CI에 적합.
- **Script Editor(Extension 내 콘솔)**: GUI가 이미 떠 있는 상태에서 코드를 그때그때 실행. 이미 열려 있는 Stage에 바로 작업할 수 있어 탐색/디버깅에 유리하지만 `SimulationApp`을 다시 만들 필요는 없다(이미 존재).
- 이번 과정은 재현성과 자동화가 중요하므로 이후 모듈에서도 기본적으로 standalone 스크립트를 우선한다.

---

## 4. 실습 가이드 (6h)

1. **최소 스크립트 (1h)** — `SimulationApp` → `World` → `add_default_ground_plane` → `world.reset()` → 100스텝 실행 → `simulation_app.close()`. `python3 -u`로 실행해 버퍼링 문제 없이 로그 확인.
2. **모듈 1에서 만든 씬을 코드로 재현 (2h)** — 모듈 1 과제에서 캡처한 Prim 트리(로봇 에셋)를 그대로 `add_reference_to_stage` + `Robot`으로 재현. GUI로 만든 결과와 코드로 만든 결과가 동일한 Prim 경로 구조를 갖는지 Stage 패널로 대조.
3. **관절 제어 + dof_names 검증 (2h)** — `dof_names` 출력 후 각 인덱스에 다른 값을 넣어 어느 링크가 움직이는지 실제로 확인, 이름-인덱스 매핑 딕셔너리를 만들어 이후 모듈에서 재사용할 헬퍼 함수로 정리.
4. **standalone vs Script Editor 비교 (1h)** — 동일한 코드를 Script Editor 안에서 한 줄씩 실행해보고, standalone 실행과 어떤 점이 다른지(재시작 필요 여부, 상태 유지 여부) 직접 비교.

결과 스크립트는 `practicum/m02_python_api/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. `ModuleNotFoundError: No module named 'omni'` 또는 `isaacsim`이 나요.**
A. `SimulationApp`을 생성하기 전에 다른 Isaac Sim/omni 모듈을 import한 경우. import 순서를 다시 확인.

**Q. `set_joint_positions`를 호출했는데 아무 변화가 없어요.**
A. `world.reset()`을 호출하기 전이거나, 관절 배열 길이가 `dof_names` 개수와 다른 경우가 흔하다. 항상 `len(dof_names)`와 입력 배열 길이를 맞춘다.

**Q. 스크립트가 끝나기 직전에 출력한 print가 안 보여요.**
A. `simulation_app.close()`가 표준출력 버퍼를 날리는 경우가 있다. `python3 -u`(unbuffered)로 실행하거나 `sys.stdout.flush()`를 명시적으로 호출.

---

## 6. 체크포인트 & 과제

**체크포인트**: standalone 스크립트로 임의의 로봇 에셋을 불러와 `dof_names` 기반으로 관절을 제어하고, 이름-인덱스 매핑을 코드로 검증할 수 있다.

**과제**: 모듈 1 과제의 Prim 트리 문서와, 이번 모듈에서 그 씬을 재현한 코드를 나란히 놓고 "GUI 조작 ↔ API 호출" 대응표를 만들어 정리 (예: "Content Browser 드래그" ↔ `add_reference_to_stage`). 이 대응표는 앞으로 GUI에서 뭔가 본 것을 코드로 옮길 때 계속 참고할 개인 치트시트가 된다.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "Python Environment" / `isaacsim.core.api` API 레퍼런스
- `isaacsim.core.utils.stage`, `isaacsim.core.utils.prims` 소스 (`~/isaacsim_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.utils/`) — docstring이 실질적인 최신 레퍼런스가 되는 경우가 많음

---
이전: [`01-usd-stage-basics.md`](./01-usd-stage-basics.md) · 다음: [`03-physx-deep-dive.md`](./03-physx-deep-dive.md)
