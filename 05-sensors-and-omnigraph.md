# Module 05 — 센서 스위트 & OmniGraph

**권장 소요: 이론 4h · 실습 8h**

---

## 1. 학습 목표

- 카메라(RGB/Depth/Segmentation), RTX Lidar, IMU, 접촉 센서를 각각 Python API로 구성할 수 있다.
- 동일한 센서를 OmniGraph(Action Graph) 노드 조합으로도 구성할 수 있고, 두 경로의 장단점을 설명할 수 있다.
- 여러 센서를 동시에 로깅하는 파이프라인을 만들 수 있다.

## 2. 선수 지식 확인

- 이전 과정(ROS2 4주차)에서 IMU/접촉 센서를 OmniGraph로 이미 구성해 `/imu`, `/contact_L`, `/contact_R` 토픽까지 뽑아본 경험이 있다 — 이번 모듈은 그 경험을 (a) ROS2 없이 Python API로도 직접 읽는 법, (b) 카메라/라이다까지 확장하는 것에 초점.
- "컨테이너 Xform vs 실제 물리 바디" 함정(이전 과정에서 반복적으로 겪음 — 센서는 반드시 실제 바디에 붙여야 함)을 다시 상기.

---

## 3. 이론 세션

### 3.1 두 가지 경로: Python API vs OmniGraph

| 경로 | 특징 | 언제 쓰나 |
|---|---|---|
| Python API (`isaacsim.sensors.*`) | 코드로 직접 센서 생성/데이터 획득. 로직이 명시적, 디버깅 쉬움 | standalone 스크립트, 합성 데이터 생성(모듈 6), RL 관측(모듈 7) |
| OmniGraph (Action Graph) | 노드 그래프로 시각적 조립. ROS2 브릿지처럼 "매 프레임 자동 퍼블리시"가 필요한 경우 편리 | ROS2 연동, GUI에서 빠르게 프로토타이핑 |

이번 과정은 ROS2 의존이 없으므로 Python API를 기본으로 삼되, OmniGraph도 최소 한 번은 직접 구성해 두 경로의 결과가 동일함을 확인한다.

### 3.2 카메라: RGB / Depth / Segmentation

```python
from isaacsim.sensors.camera import Camera
import numpy as np

camera = Camera(prim_path="/World/Camera", resolution=(640, 480))
camera.initialize()
camera.add_distance_to_camera_to_frame()      # depth
camera.add_instance_segmentation_to_frame()   # segmentation

world.step(render=True)  # 워밍업 필요 — 초기 몇 프레임은 노출/디노이즈 미수렴
rgb = camera.get_rgba()
depth = camera.get_depth()
seg = camera.get_current_frame()["instance_segmentation"]
```

- RTX 렌더러는 초기 프레임이 어둡게/노이즈 있게 나온다는 것을 이전 과정에서 이미 확인했음 — 캡처 전 워밍업 스텝이 여전히 필요.
- Segmentation은 "인스턴스별 고유 ID"로 나온다는 점, 클래스 라벨과의 매핑은 별도 관리가 필요하다는 점을 짚는다(모듈 6 Replicator에서 이 매핑을 자동화).

### 3.3 RTX Lidar

```python
from isaacsim.sensors.rtx import LidarRtx
```

- RTX Lidar는 카메라와 유사하게 RTX 렌더 파이프라인을 공유하는 센서로, point cloud를 프레임 단위로 얻는다.
- 실제 라이다 스펙(FOV, 채널 수, 스캔 패턴)에 맞는 프리셋 설정 파일이 제공되며, 커스텀 스펙도 만들 수 있다는 것만 개념적으로 소개(깊은 커스터마이징은 이번 과정 범위 밖).

### 3.4 IMU와 접촉 센서 (Python API로 직접 읽기)

```python
from isaacsim.sensors.physics import IMUSensor, ContactSensor

imu = IMUSensor(prim_path="/World/Robot/base_link/imu", ...)
contact = ContactSensor(prim_path="/World/Robot/left_shin/contact", ...)

imu_data = imu.get_current_frame()
contact_data = contact.get_current_frame()
```

- 이전 과정에서 겪은 "센서는 컨테이너 Xform이 아니라 실제 물리 바디 자식 prim에 붙여야 한다"는 함정을 Python API 경로에서도 동일하게 확인.
- OmniGraph 경로와 값이 일치하는지 대조하는 것을 실습 포인트로 삼는다.

### 3.5 멀티센서 동기화

- 여러 센서를 같은 `world.step()` 루프 안에서 동시에 읽을 때, "이 프레임의 카메라 이미지와 이 프레임의 IMU 값이 같은 시각을 가리키는가"를 보장하는 것이 실제 로보틱스 파이프라인에서 중요한 문제라는 것을 짚는다 — 이번 모듈에서는 `world.step()` 한 번 = 한 타임스텝이라는 단순한 동기화만 다루고, 센서별 업데이트 주기가 다른 경우(예: 카메라는 저주파, IMU는 고주파)는 심화 주제로 언급만 한다.

---

## 4. 실습 가이드 (8h)

1. **카메라 RGB/Depth/Segmentation (2h)** — 모듈 4의 Franka+큐브 씬에 카메라를 배치, 세 종류 데이터를 각각 이미지 파일로 저장해 시각적으로 비교.
2. **RTX Lidar (1.5h)** — 간단한 씬(벽/장애물 몇 개)에 Lidar를 배치, point cloud를 얻어 2D 평면에 투영해 플롯으로 확인(장애물 형상이 보이는지).
3. **IMU/접촉 센서를 Python API로 (2h)** — 모듈 3~4에서 쓴 로봇에 IMU+접촉 센서를 Python API로 직접 붙여 값 로깅. "컨테이너 Xform 함정"을 의도적으로 재현(컨테이너에 붙였다가 잘못된 값이 나오는 것 확인) 후 수정.
4. **OmniGraph로 동일 센서 재구성 (1.5h)** — 3번과 동일한 IMU를 OmniGraph Action Graph로 구성해 Python API 결과와 값이 일치하는지 대조.
5. **멀티센서 로깅 스크립트 (1h)** — 카메라+IMU+접촉을 한 루프에서 동시에 로깅하는 스크립트로 통합, N 스텝 동안의 로그를 CSV/이미지 시퀀스로 저장.

결과는 `practicum/m05_sensors/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. Segmentation 이미지에 모든 물체가 같은 색(ID)으로 나와요.**
A. Instance segmentation은 Prim의 semantic label이 설정되어 있어야 의미있는 ID가 부여된다 — 라벨이 없으면 기본값으로 뭉뚱그려질 수 있다. 각 Prim에 semantic label을 붙였는지 확인.

**Q. Lidar point cloud가 텅 비어 있어요.**
A. 씬에 실제로 충돌 가능한(Collision API가 있는) 지오메트리가 있는지 확인 — 모듈 3의 "Collision 누락" 문제와 같은 계열.

**Q. Python API로 읽은 IMU 값과 OmniGraph로 읽은 값이 달라요.**
A. 두 경로가 참조하는 prim_path가 실제로 같은 prim인지, 좌표 기준(월드 vs 로컬)이 같은지부터 확인 — 컨테이너 Xform 함정의 변형인 경우가 많다.

---

## 6. 체크포인트 & 과제

**체크포인트**: 카메라(RGB+Depth+Seg), Lidar, IMU, 접촉 센서를 모두 최소 한 번씩 Python API로 구성하고 값을 확인했다. IMU는 OmniGraph 경로와 교차검증했다.

**과제**: 모듈 4의 pick-and-place 씬에 카메라를 추가해, 지금까지처럼 GT 좌표를 코드에서 직접 읽는 대신 "카메라 이미지에서 물체 위치를 (간단하게, 예: 컬러 마스크의 중심점으로) 추정"해서 그 추정 위치로 pick을 시도해볼 것 — 완벽한 인식 파이프라인은 아니지만 "센서 → 인식 → 제어" 흐름을 처음 경험하는 것이 목적.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "Sensors" 섹션 (Camera, RTX Lidar, Physics Sensors)
- `isaacsim.sensors.camera`, `isaacsim.sensors.physics`, `isaacsim.sensors.rtx` 소스/예제 (`~/isaacsim_env/lib/python3.11/site-packages/isaacsim/exts/`)
- `~/Documents/ros2/04-ros2-isaac-bridge.md` — 이전 과정에서 IMU/접촉센서 OmniGraph 구성 시 겪은 트러블슈팅 기록

---
이전: [`04-robot-import-and-manipulation.md`](./04-robot-import-and-manipulation.md) · 다음: [`06-replicator-synthetic-data.md`](./06-replicator-synthetic-data.md)
