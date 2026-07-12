# Module 05 — 센서 스위트 & OmniGraph

이 문서는 카메라(RGB/Depth/Segmentation), RTX Lidar, IMU, 접촉 센서를 Python API와 OmniGraph 양쪽 경로로 구성하고, 마지막으로 카메라 인식만으로 pick-and-place를 시도해보는 실습 가이드다. Module 4의 `franka_scene.usd`(Franka + Cube, 베이스가 원점에 있음)를 그대로 재사용한다.

---

## 준비물

- Isaac Sim 5.1.0, `isaacsim-run` alias (Python API 실습은 headless 스크립트, OmniGraph 실습은 GUI)
- `practicum/m04_manipulation/franka_scene.usd`

## 1. 카메라: RGB / Depth / Segmentation

### 단계별 실습
`capture_camera.py`로 Franka+Cube 씬에 카메라를 배치하고 세 종류 데이터를 각각 이미지로 저장:
```
cd practicum/m05_sensors
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u capture_camera.py
```

### 결과
`rgb.png`, `depth.png`, `segmentation.png` — 셋 다 정상 캡처됨.

### 핵심 정리
1. **`get_depth()`는 `distance_to_image_plane` 애노테이터를 요구**한다. `add_distance_to_camera_to_frame()`(다른 애노테이터: `distance_to_camera`)를 붙이면 `get_depth()`가 `None`을 반환해 `AttributeError`가 나고, 그 처리되지 않은 예외 때문에 Isaac Sim이 종료 과정에서 **segfault**까지 났다. `add_distance_to_image_plane_to_frame()`으로 고쳐야 한다.
2. **instanceable USD 레퍼런스는 semantic label 상속을 막는다.** Franka의 각 링크 메시(`panda_link0/geometry` 등)는 전부 `instanceable=True`인 USD 레퍼런스라, 조상 prim(`/World/franka`)에 `add_labels()`로 라벨을 걸어도(`get_labels()`로 확인하면 정상 등록된 것처럼 보인다) 실제 렌더링에는 반영되지 않는다 — USD 인스턴싱은 prototype을 공유하기 때문에 인스턴스 경계를 넘는 상속이 안 된다. `colorize=True`로 segmentation을 시각화해보면 라벨이 있는 Cube만 색이 나오고 라벨이 있어야 할 Franka는 배경과 같은 색(안 잡힘)으로 나와서 발견했다. **해결**: 캡처 전에 `Usd.PrimRange`로 순회하며 `prim.SetInstanceable(False)`로 instanceable을 꺼주면 라벨이 정상적으로 전파된다.

---

## 2. RTX Lidar

### 단계별 실습
`capture_lidar.py`가 벽 2개(ㄱ자)와 작은 pillar로 이루어진 씬을 만들고, `Debug_Rotary` 라이다 프로파일로 스캔한 뒤 top-down 2D 평면에 투영:
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u capture_lidar.py
```

### 결과
`lidar_topdown.png` — 벽 2개(ㄱ자)와 pillar의 근접 모서리가 뚜렷하게 잡히고, pillar가 벽 일부를 가리는 occlusion 그림자까지 물리적으로 정확하게 재현됐다.

### 핵심 정리
1. **기본 GroundPlane을 깔면 안 된다.** 라이다를 z=0.5 높이에서 수평으로 스캔하면 광선이 바닥과 거의 평행(grazing angle)으로 만나면서, 실제 장애물과 무관한 원거리(최대 ~10m) 히트가 대량으로 섞여 들어온다. 플롯에 동심원처럼 반지름이 점점 커지는 패턴으로 나타났는데, 바닥을 빼자 즉시 사라져서 원인을 확인했다. 이 실습은 실내 라이다 데모 목적이라 바닥 없이 진행.
2. **`Debug_Rotary`는 에미터 1개(azimuthDeg=[0])가 기계적으로 회전하며 360도를 스캔**하는 방식이다. `IsaacExtractRTXSensorPointCloudNoAccumulator` 애노테이터는 프레임마다 이미 몇 만 개 단위의 점을 반환하지만, 헤드리스 환경에서는 render 스텝과 회전 타이밍이 완벽히 맞물리지 않아서 프레임을 아무리 많이 누적해도(90프레임 이상) 특정 방위각 구간이 거의 스캔되지 않는 현상을 겪었다. 재현성을 위해 실측으로 확인한 "안정적으로 스캔되는" 방위각 범위(대략 -35°~118°) 안에 물체를 배치했다.
3. 위 이유로 프레임을 누적할수록 같은 표면이 반복해서 잡혀 포인트가 기하급수적으로 늘어난다(90프레임에 300만 개 이상). 시각화/저장용으로 랜덤 서브샘플링(2만 개)해서 정리했다.

---

## 3. IMU / 접촉 센서 — Python API

### 단계별 실습
`capture_imu_contact.py`가 IMU 두 개(하나는 일부러 잘못된 위치에)와 접촉 센서 두 개를 붙이고, `PickPlaceController`로 실제 pick-and-place를 실행하며 값을 로깅:
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u capture_imu_contact.py
```

### 결과
- **컨테이너 Xform 함정 재현**: `/World/franka`(articulation을 담는 컨테이너, 실제 물리 바디가 아님)에 붙인 `imu_wrong`은 팔이 격렬하게 움직이는 동안에도 각속도가 항상 `[0, 0, 0]`. 실제로 움직이는 링크 `panda_hand`에 붙인 `imu_correct`는 그리퍼를 닫고 들어올리는 동작에서 각속도 z축이 최대 2.6까지 튀는 등 실제 움직임을 정확히 반영.
- **접촉 센서**: 그립을 시작한 스텝부터 놓기 직전까지 좌우 손가락 모두 안정적으로 ~4N의 접촉힘 유지, 놓는 순간 힘이 0으로 떨어짐 — pick-and-place 물리와 정확히 일치. 최대 접촉힘 left 6.06N / right 5.14N.

### 핵심 정리
`ContactSensor`를 처음에 `radius=0.02`로 좁게 설정했더니 그립을 성공했는데도(module 4에서 이미 검증된 시퀀스) 접촉이 한 번도 안 잡혔다 — 센서의 로컬 기준 위치가 정확한 접촉 지점(핑거팁)과 안 맞았기 때문으로 보인다. `radius`를 기본값(`-1`, 바디 전체 기준)으로 두면 문제없이 잡힌다. `ContactSensor.get_current_frame()`의 접촉힘 키는 `"value"`가 아니라 **`"force"`**.

---

## 4. OmniGraph로 IMU 재구성 (GUI)

### 단계별 실습
1. Isaac Sim GUI로 `franka_scene.usd`를 열고, Stage에서 `/World/franka/panda_hand`를 선택
2. `Create > Isaac > Sensors > IMU Sensor`로 IMU를 실제 바디(`panda_hand`)의 자식으로 생성
3. `Window > Graph Editors > Action Graph`에서 새 그래프 생성, **On Playback Tick → Isaac Read IMU Node** 연결, IMU Prim에 방금 만든 센서 지정
4. Play 후 관절을 GUI에서 직접 돌려보며 **Isaac Read IMU Node**의 `Ang Vel` 출력이 튀는지 확인

### 결과
`Ang Vel`이 관절 조작에 반응해 튀는 것을 GUI에서 직접 확인 — Python API(`imu_correct`)와 같은 종류의 반응을 OmniGraph 경로로도 재현.

### 참고
`franka_scene.usd` 파일 자체에는 바닥이 저장되어 있지 않다(Python 스크립트들이 매번 `world.scene.add_default_ground_plane()`으로 런타임에 추가할 뿐). GUI로 직접 열면 바닥 없이 로드되어 큐브가 계속 떨어지는데, 이 실습(IMU 확인)과는 무관하므로 무시해도 된다.

---

## 5. 멀티센서 로깅 스크립트

### 단계별 실습
`multisensor_log.py`가 카메라(RGB 20스텝마다) + IMU + 접촉 센서를 pick-and-place 루프 한 번에 동시 로깅, CSV + 이미지 시퀀스로 저장:
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u multisensor_log.py
```

### 결과
`multisensor_log/log.csv`(총 917행) + `multisensor_log/images/`(46장). `step_0320.png`가 정확히 그리퍼가 큐브를 문 순간의 이미지고, 같은 스텝의 CSV 행에서 `contact_left_in_contact`/`contact_right_in_contact`가 동시에 `True`로 찍혀 카메라·접촉센서 타이밍이 정확히 일치함을 확인.

---

## 6. 과제 — 카메라 인식으로 pick 시도

교안 과제: GT 좌표를 코드에서 직접 읽는 대신, 카메라 이미지에서 물체 위치를 추정해 그 추정 위치로 pick을 시도.

### 단계별 실습
`perception_pick.py`:
1. 큐브를 눈에 띄는 빨간색으로 지정 (기본은 무채색이라 마스킹이 애매함)
2. RGB에서 빨간 픽셀 마스크 추출 → 중심점(centroid) 계산
3. 그 픽셀의 depth 값 + `Camera.get_world_points_from_image_coords()`로 3D 월드 좌표 역투영
4. 추정 좌표를 `PickPlaceController`의 `picking_position`으로 사용해 pick-and-place 실행
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u perception_pick.py
```

### 결과
- 빨간 마스크 픽셀 수 1,692개, 마스크 중심 `(u=265.6, v=328.1)`
- 카메라 인식 추정 위치 vs GT 오차: **1.65cm**
- 그립 중 양쪽 손가락 동시 접촉: **성공**, 큐브가 place 방향으로 실제 이동

완벽한 인식 파이프라인은 아니지만(색 마스크 기반, 조명/배경색 바뀌면 깨짐), "센서 → 인식 → 제어" 흐름이 실제로 동작하는 것을 확인했다.

---

## 체크리스트

- [x] 카메라(RGB+Depth+Seg)를 Python API로 구성하고 값을 확인했다
- [x] RTX Lidar로 point cloud를 얻어 2D 평면에 투영해 장애물 형상을 확인했다
- [x] IMU/접촉 센서를 Python API로 구성했다 (컨테이너 Xform 함정 재현 포함)
- [x] 동일 IMU를 OmniGraph로 재구성해 Python API 결과와 대조했다
- [x] 카메라+IMU+접촉을 한 루프에서 동시 로깅하는 스크립트를 만들었다
- [x] 카메라 인식만으로 pick-and-place를 시도했다 (과제)

## 알려진 문제와 해결

| 관찰 | 원인 | 해결 |
|---|---|---|
| `get_depth()`가 `None` 반환 → `AttributeError` → segfault | `add_distance_to_camera_to_frame()`은 잘못된 애노테이터(`distance_to_camera`)를 붙임 | `add_distance_to_image_plane_to_frame()` 사용 |
| Segmentation에서 라벨을 걸어둔 Franka가 배경과 같은 색으로 뭉개짐 | Franka 링크 메시가 instanceable USD 레퍼런스라 조상 prim의 라벨이 인스턴스 경계를 못 넘음 | 캡처 전 `Usd.PrimRange`로 순회하며 `SetInstanceable(False)` |
| Lidar point cloud가 반지름이 커지는 동심원 패턴으로 오염 | 기본 GroundPlane과 수평 스캔 광선의 grazing angle 교차 | 이 실습은 바닥 없이 진행 |
| Lidar 특정 방위각 구간이 프레임을 아무리 늘려도 안 잡힘 | 헤드리스에서 render 스텝과 `Debug_Rotary`의 회전 타이밍이 결정론적으로 안 맞물림 | 실측으로 확인한 안정적 스캔 범위 안에 물체 배치 |
| `radius=0.02`로 설정한 접촉 센서가 실제 그립 중에도 접촉을 못 잡음 | 센서 기준 위치가 실제 접촉 지점과 안 맞음 | `radius` 기본값(-1, 바디 전체 기준) 사용 |
| `ContactSensor.get_current_frame()`에서 힘 값을 못 찾음 | 키 이름이 `"value"`가 아니라 `"force"` | 키 이름 수정 |
| IMU를 `/World/franka`에 붙이면 팔이 움직여도 값이 항상 0 | 컨테이너 Xform이지 실제 물리 바디가 아님 (Module 4/ROS2 과정에서 반복된 패턴) | 실제로 움직이는 링크(`panda_hand`)의 자식으로 부착 |

---
이전: [`04-robot-import-and-manipulation.md`](../04-robot-import-and-manipulation.md) · 참고: [`05-sensors-and-omnigraph.md`](../05-sensors-and-omnigraph.md) · 다음: [`06-replicator-synthetic-data.md`](../06-replicator-synthetic-data.md)
