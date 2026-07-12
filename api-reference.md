# Isaac Sim Python API 레퍼런스 (Module 1~5 실습 기준)

Module 1~5를 진행하며 실제로 사용한 Python API를 카테고리별로 정리한 것. 각 항목은 실습에서 실제로 쓴 형태 그대로 예시를 남기고, 걸렸던 함정은 해당 모듈 README로 링크했다. 이론 정리가 아니라 **직접 실행해서 검증한 것만** 담았다.

버전: Isaac Sim 5.1.0 (`~/isaacsim_env`, Python 3.11) — `isaacsim.*` 네임스페이스 기준 (구버전 `omni.isaac.core` 아님).

---

## 0. 앱 부트스트랩

모든 standalone 스크립트의 첫 두 줄. `SimulationApp`을 만들기 **전에는** 다른 `isaacsim`/`omni` 모듈을 import할 수 없다.

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})  # GUI 필요하면 False

# ... 여기서부터 나머지 import ...

simulation_app.close()  # 스크립트 끝에서 반드시 호출
```

---

## 1. World & Scene — `isaacsim.core.api`

```python
from isaacsim.core.api import World

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()      # 주의: USD 파일에 저장되지 않음, 매번 런타임에 추가
robot = world.scene.add(SomePrimWrapper(prim_path="/World/robot", name="robot"))
world.reset()                                # prim 추가/센서 부착 후에는 다시 호출해야 물리 핸들 갱신됨
world.step(render=True)                      # render=False면 렌더링 없이 물리만 진행 (빠름)
```

- `world.reset()`은 씬을 바꿀 때마다(로봇 추가, 센서 부착 등) 다시 호출해야 한다 — [Module 5](m05_sensors/README.md#3-imu--접촉-센서--python-api)에서 센서 추가 후 재-리셋.
- `add_default_ground_plane()`은 USD 파일에 저장되지 않는다 — GUI로 직접 연 `.usd` 파일에는 바닥이 없다는 걸 잊지 말 것 ([Module 5](m05_sensors/README.md#4-omnigraph로-imu-재구성-gui)).

## 2. Stage/Prim 저수준 조작 — `omni.usd`, `pxr`, `isaacsim.core.utils.stage`

```python
import omni.usd
omni.usd.get_context().open_stage(STAGE_PATH)   # standalone 스크립트에서 기존 .usd 파일 열기

from isaacsim.core.utils.stage import get_current_stage
stage = get_current_stage()
prim = stage.GetPrimAtPath("/World/franka")

from pxr import Usd, UsdGeom, Gf
for p in Usd.PrimRange(prim):                    # 하위 전체 순회
    if p.IsInstanceable():
        p.SetInstanceable(False)                 # instanceable 해제 (semantic label 전파 안 될 때)

geom = UsdGeom.Gprim(cube_prim)
geom.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.1, 0.1)])  # 눈에 띄는 색 지정 (색 마스킹용)
```

- `Usd.PrimRange` + `SetInstanceable(False)`는 instanceable 레퍼런스가 semantic label 전파를 막을 때의 우회법 — [Module 5](m05_sensors/README.md#1-카메라-rgb--depth--segmentation).

## 3. Prims & Objects — `isaacsim.core.prims`, `isaacsim.core.api.objects`

| 클래스 | 용도 | 핵심 파라미터 |
|---|---|---|
| `SingleRigidPrim` | 이미 존재하는 단일 리지드 바디를 다루는 뷰 | `prim_path` (단수) |
| `FixedCuboid` / `VisualCuboid` | 코드로 큐브 생성 (물리 유무 선택) | `prim_path`, `position`, `scale`, `color` |
| `Articulation` | **배치(batched)** 뷰 — 여러 prim을 와일드카드로 다룸 | `prim_paths_expr` (복수형 파라미터명 주의) |
| `SingleArticulation` | 단일 articulation 뷰 | `prim_path` (단수) |

```python
from isaacsim.core.prims import SingleRigidPrim, Articulation, SingleArticulation
from isaacsim.core.api.objects import FixedCuboid

cube = world.scene.add(SingleRigidPrim(prim_path="/World/Cube", name="cube"))
wall = world.scene.add(FixedCuboid(prim_path="/World/wall_1", name="wall_1",
                                    position=np.array([3.0, 0.0, 0.5]), scale=np.array([0.2, 4.0, 1.0])))
franka_batched = world.scene.add(Articulation(prim_paths_expr="/World/franka", name="franka"))
franka_single  = world.scene.add(SingleArticulation(prim_path="/World/franka", name="franka"))
```

**액션 타입도 배치/단일이 다르다** (`isaacsim.core.utils.types`):

```python
from isaacsim.core.utils.types import ArticulationAction, ArticulationActions

# SingleArticulation / Robot / Franka 등 단일 래퍼 -> 단수형
robot.apply_action(ArticulationAction(joint_positions=target))
# 배치 Articulation -> 복수형
franka_batched.apply_action(ArticulationActions(joint_positions=target))
```
> 헷갈리면 `no attribute 'joint_names'` 같은 알기 어려운 에러가 난다 — [Module 4](m04_manipulation/README.md#2-dof_names-확인하기).

`get_joint_positions()`도 클래스에 따라 shape가 다르다: `Robot`(단일)은 1차원, 배치 `Articulation`은 `(1, N)` 2차원.

## 4. 로봇 / 매니퓰레이터

```python
from isaacsim.core.api.robots import Robot                          # 범용 단일 로봇 wrapper
from isaacsim.robot.manipulators.examples.franka import Franka       # 그리퍼 포함 Franka 전용 wrapper
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController

robot = world.scene.add(Robot(prim_path="/World/cobotta_pro_1300", name="cobotta"))
print(robot.dof_names)
robot.set_joint_positions(target)   # 즉시 텔레포트 (Position 제어와 다름, 진단용으로 유용)

franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
controller = PickPlaceController(name="pick_place", gripper=franka.gripper, robot_articulation=franka)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

while not controller.is_done():
    actions = controller.forward(
        picking_position=cube_pos, placing_position=place_pos,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(actions)
    world.step(render=False)
    controller.get_current_event()   # 0~9 상태 머신 진행 단계 (approach/close/lift/... )
```

IK 직접 계산 (`isaacsim.robot_motion.motion_generation`):

```python
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver

kinematics_solver = LulaKinematicsSolver(robot_description_path=YAML_PATH, urdf_path=URDF_PATH)
ik = ArticulationKinematicsSolver(robot_articulation=franka, kinematics_solver=kinematics_solver,
                                   end_effector_frame_name="right_gripper")
pos, rot = ik.compute_end_effector_pose()                       # FK
action, success = ik.compute_inverse_kinematics(target_position=target)  # IK
franka.apply_action(action)
```
> `LulaKinematicsSolver`는 **로봇 베이스가 world 원점에 있다고 가정**한다 — 아니면 IK/FK가 자기 자신과는 일치하는데 실제 좌표는 완전히 틀어진다. 배치 직후 `franka.get_world_pose()`로 베이스 위치를 꼭 확인할 것 ([Module 4](m04_manipulation/README.md#53-진짜-원인--로봇-베이스가-원점이-아니었다)).

## 5. 센서 — `isaacsim.sensors.*`

### 카메라 (`isaacsim.sensors.camera.Camera`)

```python
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.rotations import lookat_to_quatf
from pxr import Gf

camera = Camera(prim_path="/World/Camera", position=np.array([1.5, 1.5, 1.5]), resolution=(640, 480))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()             # get_depth()에 필요한 애노테이터 (distance_to_camera 아님!)
camera.add_instance_segmentation_to_frame(init_params={"colorize": True})

quat = lookat_to_quatf(target, eye, up)                    # 인자 순서: (target, eye, up) — camera_utils.py 내부 관례와 동일
orientation = np.array([quat.GetReal(), *quat.GetImaginary()])   # (w, x, y, z)
camera.set_world_pose(position=eye, orientation=orientation, camera_axes="usd")

for _ in range(30):
    world.step(render=True)     # RTX 워밍업 필수 — 초기 프레임은 비어있거나 노이즈

rgb = camera.get_rgba()          # (H, W, 4) uint8
depth = camera.get_depth()       # (H, W) 또는 (H, W, 1) float32
seg = camera.get_current_frame()["instance_segmentation"]   # colorize=True면 (H,W,4) uint8

# 픽셀 좌표 + depth -> 3D 월드 좌표 역투영 (물체 인식 파이프라인에 사용)
world_pt = camera.get_world_points_from_image_coords(np.array([[u, v]]), np.array([depth_at_uv]))[0]
```
> `get_depth()`가 `None`을 반환하면 반드시 `add_distance_to_image_plane_to_frame()`을 붙였는지 확인 — 잘못된 애노테이터를 붙이면 `AttributeError`를 넘어 **segfault**까지 난다 ([Module 5](m05_sensors/README.md#1-카메라-rgb--depth--segmentation)).

### IMU / 접촉 센서 (`isaacsim.sensors.physics`)

```python
from isaacsim.sensors.physics import IMUSensor, ContactSensor

imu = IMUSensor(prim_path="/World/franka/panda_hand/imu")   # 반드시 실제 물리 바디의 자식으로!
frame = imu.get_current_frame()   # {"ang_vel": [x,y,z], "lin_acc": [x,y,z], "orientation": [w,x,y,z], ...}

contact = ContactSensor(prim_path="/World/franka/panda_leftfinger/contact")  # radius 기본값(-1)=바디 전체 기준
cframe = contact.get_current_frame()   # {"in_contact": bool, "force": float, ...} (키는 "value"가 아니라 "force")
```
> `/World/franka` 같은 컨테이너 Xform에 붙이면 값이 절대 안 바뀐다 — 반드시 실제로 움직이는 링크의 자식으로 부착 ([Module 5](m05_sensors/README.md#3-imu--접촉-센서--python-api)).

### RTX Lidar (`isaacsim.sensors.rtx.LidarRtx`)

```python
from isaacsim.sensors.rtx import LidarRtx

lidar = LidarRtx(prim_path="/World/Lidar", position=np.array([0, 0, 0.5]), config_file_name="Debug_Rotary")
lidar.initialize()
lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

frame = lidar.get_current_frame()
data = frame["IsaacExtractRTXSensorPointCloudNoAccumulator"]
xyz = np.asarray(data["data"]).reshape(-1, 3)   # 실제 포인트클라우드 xyz
```
> 기본 GroundPlane과 함께 쓰면 수평 스캔이 grazing angle로 바닥과 만나 원거리 히트가 대량 오염된다 — 실내 장애물 스캔이 목적이면 바닥 없이 진행. `Debug_Rotary`는 헤드리스에서 render 스텝과 회전 타이밍이 결정론적으로 안 맞물려서, 프레임을 늘려도 특정 방위각이 안 잡힐 수 있다 ([Module 5](m05_sensors/README.md#2-rtx-lidar)).

## 6. Semantic Label — `isaacsim.core.utils.semantics`

```python
from isaacsim.core.utils.semantics import add_labels, get_labels

add_labels(prim, ["franka"], instance_name="class")
get_labels(prim)   # {"class": ["franka"]} — 등록 여부 확인용, 실제 렌더링 반영 여부는 별개(instanceable 함정 주의)
```

---

## 7. OmniGraph (GUI, Action Graph) — 참고용

Python API가 아니라 GUI로 조립하지만, 같은 데이터를 얻는 대체 경로로 같이 알아둘 것 ([Module 5](m05_sensors/README.md#4-omnigraph로-imu-재구성-gui)):

- **On Playback Tick** — 매 프레임 실행 트리거 (Event 카테고리)
- **Isaac Read IMU Node** — `Inputs > IMU Prim`에 센서 prim 지정, `Outputs`에 `Ang Vel`/`Lin Acc`/`Orientation` 실시간 표시
- Play 중 노드를 선택하면 Property 패널에서 출력값이 실시간 갱신됨 (ROS2 없이도 값 확인 가능)

---

## 함정 색인 (모듈별)

| 함정 | 모듈 |
|---|---|
| 로봇 베이스가 원점이 아닐 수 있음 (IK 가정 깨짐) | [Module 4](m04_manipulation/README.md#1-franka-배치하기-gui) |
| `Articulation`(배치) vs `SingleArticulation`/`Robot`(단일) 파라미터·액션 타입 차이 | [Module 4](m04_manipulation/README.md#2-dof_names-확인하기) |
| 바닥에 놓는 오브젝트는 Translate Z = half-height로 | [Module 4](m04_manipulation/README.md#51-첫-번째-실패-큐브가-바닥에-떠-있었다) |
| headless 카메라(`get_rgba()`)가 항상 같은 이미지만 반환 (원인 불명, GUI Movie Capture로 우회) | [Module 4](m04_manipulation/README.md#6-영상-녹화--headless-카메라-캡처-실패-gui-movie-capture로-전환) |
| `get_depth()` 애노테이터 이름 오류 → segfault | [Module 5](m05_sensors/README.md#1-카메라-rgb--depth--segmentation) |
| instanceable USD 레퍼런스가 semantic label 상속을 막음 | [Module 5](m05_sensors/README.md#1-카메라-rgb--depth--segmentation) |
| 기본 GroundPlane + 수평 Lidar 스캔 = grazing angle 오염 | [Module 5](m05_sensors/README.md#2-rtx-lidar) |
| IMU/접촉 센서를 컨테이너 Xform에 붙이면 값이 고정됨 | [Module 5](m05_sensors/README.md#3-imu--접촉-센서--python-api) |
| `ContactSensor` 힘 값 키는 `"force"` (`"value"` 아님) | [Module 5](m05_sensors/README.md#3-imu--접촉-센서--python-api) |

---
[Module 1](m01_usd_basics/README.md) · [Module 2](m02_python_api/README.md) · [Module 3](m03_physx/README.md) · [Module 4](m04_manipulation/README.md) · [Module 5](m05_sensors/README.md)
