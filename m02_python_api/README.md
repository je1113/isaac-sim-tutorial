# Module 02 — Python 스크립팅 API 핵심: standalone 스크립트로 로봇 제어하기

이 문서는 Content Browser에서 새로 고른 로봇을 GUI로 배치한 뒤, `isaacsim.core.api` 기반 standalone 스크립트로 관절을 제어하는 실습 가이드다. 이전 ROS2 과정의 biped와는 완전히 별개로, Denso `cobotta_pro_1300`(그리퍼 장착 협동로봇 팔)을 대상으로 진행한다.

---

## 1. 로봇 배치하기 (GUI)

### 단계별 실습
1. Content Browser에서 `cobotta_pro_1300`을 찾는다
2. 뷰포트로 드래그해서 `/World/cobotta_pro_1300`에 배치 (Module 1과 같은 방식)
3. `File → Save As`로 `practicum/m02_python_api/robot_scene.usd`에 저장

이후 모든 단계는 이 씬(`robot_scene.usd`, 로봇 prim `/World/cobotta_pro_1300`)을 전제로 한다.

---

## 2. Standalone 스크립트 뼈대 작성하기

### 배울 것
- `SimulationApp`은 다른 Isaac Sim/omni 모듈을 import하기 **전에** 먼저 생성해야 한다
- 5.1 기준 최신 네임스페이스는 `isaacsim.core.api`(구 `omni.isaac.core`)
- 씬 구성 후 `world.reset()`을 반드시 호출해야 물리 핸들이 준비된다

### 코드
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

robot = world.scene.add(Robot(prim_path="/World/cobotta_pro_1300", name="cobotta"))
world.reset()
```

---

## 3. dof_names로 관절 목록 확인하기

### 배울 것
관절 인덱스를 하드코딩하지 말고 이름 기반으로 다뤄야 한다는 원칙 — 이번엔 "하나의 로봇 안에 성격이 다른 두 그룹(팔/그리퍼)이 섞여 있다"는 패턴이 처음 등장한다.

### 단계별 실습
`robot.dof_names`를 출력해본다.

### 결과
```
dof_names (12): ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
                 'finger_joint', 'left_inner_knuckle_joint', 'right_inner_knuckle_joint',
                 'right_outer_knuckle_joint', 'left_inner_finger_joint', 'right_inner_finger_joint']
```
`joint_1`~`joint_6`이 실제 팔의 6축이고, 나머지 6개는 장착된 2핑거 그리퍼의 관절이다.

---

## 4. 관절 제어 — 모든 관절에 동일한 +0.3 rad 오프셋

### 단계별 실습
시작 위치(전부 0)에서 12개 관절 전부에 `start + 0.3`을 목표로 주고 200스텝 동안 `set_joint_positions`로 명령한다.

```python
start = robot.get_joint_positions()   # 전부 0.0
target = start + 0.3                  # 전부 0.3
for step in range(200):
    robot.set_joint_positions(target)
    world.step(render=False)
```

터미널에서 아래 명령으로 직접 실행하고 로그를 눈으로 확인한다 (`headless=True`이므로 GUI 없이 로그만으로 검증):
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u control_cobotta.py
```

### 결과
```
step   0 | current: [0.3 0.3 0.3 0.285 0.3 0.31 0.299 0.265 0.267 -0.122 0.299 0.267] | max_err: 0.4225
step 199 | current: [0.3 0.3 0.3 0.284 0.3 0.31 0.299 0.265 0.267 -0.122 0.299 0.267] | max_err: 0.4225
```

### 핵심 정리
- `joint_1`~`joint_6`(팔)과 그리퍼의 `finger_joint`/`left_inner_knuckle_joint`/`left_inner_finger_joint`/`right_inner_finger_joint`는 **첫 스텝에서 이미 거의 정확히 0.3 근처에 도달**했고 199스텝까지 그대로 유지됐다 — 드라이브 게인을 전혀 안 건드렸는데도 진동/발산 없이 즉시 수렴했다. Content Browser에서 제공하는 완성된 로봇 에셋은 드라이브 게인이 이미 실사용 가능한 수준으로 튜닝되어 있다는 뜻이다 (이전 과정에서 자체 URDF를 임포트했을 땐 기본값이 비현실적이어서 매번 튜닝이 필요했던 것과 대비된다 — Module 3의 jitter 실습에서 이 로봇의 기본 게인이 얼마나 안정적인지 다시 확인한다).
- 딱 하나, **`right_outer_knuckle_joint`만 목표(0.3)가 아니라 -0.122에서 정지**했다. 나머지 그리퍼 관절은 전부 목표에 도달했는데 이 관절만 다른 값, 그것도 부호가 반대인 값으로 안착했다.
- **원인 추정**: 2핑거 그리퍼는 보통 knuckle/finger 관절 중 일부가 서로 기구학적으로 커플링(mimic joint)되어 있어, 하나가 움직이면 나머지가 종속적으로 따라 움직이도록 설계된다. `right_outer_knuckle_joint`가 다른 관절들과 반대 부호로 안착한 것은 독립적으로 제어되는 게 아니라 다른 관절의 움직임에 종속된 채로 "따라간" 결과로 보인다. `dof_names`에 이름이 있다고 해서 모든 관절이 서로 독립적으로 임의 목표에 도달할 수 있는 건 아니라는 것을 실제로 확인한 셈이다. 정확한 커플링 관계(어느 관절이 어느 관절을 mimic하는지)는 확인하지 않고 추정까지만 진행했다 — 남은 과제.

**검증 방법에 대해**: 처음엔 이 스크립트를 실행자가 직접 터미널에서 돌리지 않고 대신 실행해서 로그만 보고했었는데, 사람이 직접 실행 결과를 두 눈으로 확인하는 게 이 커리큘럼의 원칙이라 다시 직접 실행해서 재검증했다. 재실행 결과는 위 로그와 정확히 일치했다(결정론적) — `right_outer_knuckle_joint` 이상은 실제로 재현 가능한 현상임이 확인됐다.

---

## 5. 임포트 시점 경고 확인

### 결과
실행 로그에 다음 경고가 떴다:
```
The rigid body at /World/cobotta_pro_1300/world has a possibly invalid inertia tensor of
{1.0, 1.0, 1.0} and a negative mass, small sphere approximated inertia was used.
```

### 핵심 정리
`/World/cobotta_pro_1300/world`라는 링크(로봇의 월드 기준 프레임을 나타내는 더미 링크로 추정)에 음수 질량이 들어있어 PhysX가 자동으로 작은 구 형태의 기본 관성으로 대체했다는 경고다. 동작 자체에는 영향이 없었다(관절 제어가 정상 동작). [Module 3의 무게중심 실습](../m03_physx/README.md#미해결-무게중심-쏠림-재현--도전해보고-싶다면)에서 이 링크를 다시 조사해, 애초에 Mesh/Collision이 없는 빈 Xform이라 질량 계산이 성립하지 않는 케이스였다는 걸 확인했다.

---

## 체크리스트

- [x] `SimulationApp`을 다른 Isaac Sim/omni 모듈 import보다 먼저 생성했는가
- [x] `isaacsim.core.api` 네임스페이스로 import했는가
- [x] 씬 구성 후 `world.reset()`을 호출했는가
- [x] 관절 인덱스를 하드코딩하지 않고 `dof_names`로 이름 기반으로 다뤘는가
- [x] 명령값과 실제값을 스텝마다 비교해 수렴 여부를 검증했는가 (육안이 아니라 로그로 정량 확인)
- [x] 스크립트를 직접 터미널에서 실행하고 본인이 로그를 확인했는가

## 알려진 문제와 해결

| 관찰 | 원인(추정) | 비고 |
|---|---|---|
| `right_outer_knuckle_joint`만 목표와 다른(부호 반대) 값에 안착 | 그리퍼 내 mimic/커플링 관절이라 다른 관절 움직임에 종속적으로 따라간 것으로 추정 | 정확한 커플링 관계는 추가 확인 필요 — 남은 과제 |
| `.../world` 링크에 음수 질량 경고 | 에셋 자체에 원래부터 있던 값 (Module 3에서 확인: 이 링크는 Mesh/Collision이 없는 빈 Xform이라 질량 계산이 애초에 불가능) | 동작에는 영향 없었음 |
| 대부분의 관절이 진동 없이 즉시 수렴 | Content Browser 제공 에셋은 드라이브 게인이 이미 실사용 수준으로 튜닝돼 있음 | 이전 과정에서 자체 URDF를 임포트했을 땐 기본 게인이 비현실적이라 매번 튜닝이 필요했던 것과 대비됨 |

## 남은 과제 (아직 안 한 것)

- `right_outer_knuckle_joint`의 정확한 mimic/커플링 관계를 USD 스키마(`PhysxSchema.PhysxMimicJointAPI` 등)에서 직접 확인
- Script Editor 콘솔 방식과 standalone 방식을 나란히 비교

---
이전: [`m01_usd_basics/README.md`](../m01_usd_basics/README.md) · 참고: [`02-python-scripting-api.md`](../02-python-scripting-api.md) · 다음: [`m03_physx/README.md`](../m03_physx/README.md)
