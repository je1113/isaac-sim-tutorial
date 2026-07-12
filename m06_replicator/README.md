# Module 06 — Replicator와 합성 데이터 생성

이 문서는 `omni.replicator.core`로 Domain Randomization(무작위화)이 적용된 씬을 만들고, RGB + Bounding Box + Segmentation 라벨이 자동으로 붙은 합성 데이터셋을 생성하는 실습 가이드다. 최소 스크립트에서 시작해 Module 4/5의 Franka+Cube 씬까지 단계적으로 확장했다.

---

## 준비물

- Isaac Sim 5.1.0, `isaacsim-run` alias 아님 — 전부 headless standalone 스크립트
- `practicum/m04_manipulation/franka_scene.usd` (4단계에서 재사용)

## 1. 최소 Replicator 스크립트

### 단계별 실습
`min_replicator.py`: 큐브 하나에 pose/color 랜덤화를 걸고 `BasicWriter`로 RGB 100장 생성.
```
cd practicum/m06_replicator
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u min_replicator.py
```

### 결과
`out_min/rgb_0000.png` ~ `rgb_0099.png` — 큐브가 매 프레임 다른 위치/회전/색상으로 나온다 (파란 격자 바닥은 `World().scene.add_default_ground_plane()`의 기본 텍스처).

### 핵심 정리 — 세 가지 함정을 순서대로 만났다

1. **`rep.orchestrator.run()`은 시작 명령만 던지고 바로 리턴한다.** Standalone 스크립트에서 결과를 기다리려면 `run_until_complete()`를 써야 한다 — 안 그러면 출력 폴더가 텅 빈 채로 스크립트가 "완료"라고 찍고 끝난다.
2. **조명이 없으면 완전히 새까맣게 나온다.** `rep.create.cube()`/`rep.create.camera()`만으로는 광원이 자동으로 생기지 않는다 — `rep.create.light(light_type="Dome", ...)`를 반드시 추가해야 한다.
3. **(가장 오래 걸린 함정) `World()`를 먼저 만들지 않으면 조명 intensity를 아무리 올려도(1000 → 30000까지 시도) 계속 거의 새까만 이미지만 나온다.** 실제 raw 애노테이터 데이터를 직접 찍어보니 dtype은 정상(uint8)인데 값이 계속 `(1, 1, 1)` 근처였다 — 노출 수렴 문제(rt_subframes 부족)로 의심해 20까지 올려봐도 소용없었다. `isaacsim.core.api.World()`를 생성하고 나서야 정상적인 픽셀 값(`(25, 58, 87)` 등)이 나왔다. **Replicator 단독 스크립트에서도 `World()`는 사실상 필수** — RTX 렌더러의 기본 환경/조명 파이프라인이 `World()` 생성 시점에 초기화되는 것으로 보인다.

---

## 2. Bounding Box + Segmentation 라벨 추가

### 단계별 실습
`bbox_seg_replicator.py`: `BasicWriter`에 `bounding_box_2d_tight=True`, `semantic_segmentation=True`를 추가.
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u bbox_seg_replicator.py
```

### 결과
`out_bbox_seg/`에 RGB 30장 + bbox + segmentation. bbox를 RGB 위에 직접 그려서 검증(`bbox_check.png`) — 큐브를 딱 맞게 감싸는 빨간 사각형, segmentation도 큐브만 정확히 초록색으로 분리됨.

### 핵심 정리
- Bounding box는 JSON이 아니라 **`.npy`**(structured array: `semanticId, x_min, y_min, x_max, y_max, occlusionRatio`)로 저장된다.
- 클래스 이름 매핑은 별도 파일 **`bounding_box_2d_tight_labels_NNNN.json`**(`{"0": {"class": "cube"}}`)에 있다 — 이미지/npy 파일명과 프레임 번호로 짝을 맞춰야 한다.
- `semantic_segmentation`은 `colorize_semantic_segmentation=True`로 두면 바로 눈으로 볼 수 있는 컬러 PNG로 나온다.

---

## 3. 조명/배경 랜덤화 확장

### 단계별 실습
`light_bg_replicator.py`: DomeLight의 intensity/색온도, 바닥 색을 매 프레임 랜덤화.
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u light_bg_replicator.py
```

### 결과
`out_light_bg/` — 바닥 색이 청록→분홍→연두로, 조명 색온도/강도도 프레임마다 바뀐다.

### 핵심 정리
1. **`rep.create.light(intensity=1000)`처럼 정수 리터럴로 만들면 USD 속성이 `int` 타입으로 고정된다.** 나중에 `rep.modify.attribute()`로 float 분포를 덮어쓰려 하면 `Type mismatch: expected 'int', got 'TfPyObjWrapper'` 에러가 난다. `intensity=1000.0`처럼 처음부터 float로 만들어야 한다.
2. **`rep.modify.attribute()`에 넘기는 속성 이름은 네임스페이스를 포함한 실제 USD 스키마 이름**(`inputs:intensity`, `inputs:colorTemperature`)이어야 한다. `rep.create.light`의 생성자 파라미터명(`intensity`, `temperature`)과 런타임 수정 시 필요한 속성명이 다르다는 걸 헷갈리기 쉽다.

---

## 4. Module 4/5 씬에 적용 — 미니 데이터셋 생성

### 단계별 실습
`franka_dr_dataset.py`: Module 4의 `franka_scene.usd`(Franka+Cube)를 열어서, 카메라 위치/각도·큐브 위치/색·조명을 모두 랜덤화하며 300장 생성.
```
OMNI_KIT_ACCEPT_EULA=Y /home/pw/isaacsim_env/bin/python3 -u franka_dr_dataset.py
```

랜덤화 대상:
- 카메라 위치(테이블을 내려다보는 반구 영역 안에서 랜덤) + 항상 큐브 근처를 바라보도록 `look_at` 고정
- 큐브 위치(로봇이 닿는 범위 내) + 색상
- DomeLight intensity/색온도

### 결과
`out_franka_dr/` — RGB 300장 + bbox + segmentation, 51MB. `franka_bbox_check.png`로 검증: Franka(빨강)와 Cube(초록) 두 클래스가 각각 정확한 bbox로 잡히고, segmentation도 두 클래스가 뚜렷이 분리됨.

### 핵심 정리
Module 5에서 발견한 **instanceable 함정이 여기서도 그대로 재현**된다 — Franka 링크 메시가 instanceable USD 레퍼런스라 `/World/franka`에 건 라벨이 전파되지 않는다. `Usd.PrimRange`로 순회하며 `SetInstanceable(False)`로 끈 뒤에 라벨을 걸어야 Franka가 segmentation/bbox에 제대로 잡힌다.

---

## 5. 라벨 검증

4단계에서 이미 함께 진행: `rgb_0100.png` 위에 `bounding_box_2d_tight_0100.npy`의 두 bbox(franka, cube)를 그려서(`franka_bbox_check.png`) 육안으로 대조 — 둘 다 실제 물체 경계와 정확히 일치했다.

---

## 체크포인트 & 과제 (회고)

**체크포인트**: 도메인 랜덤화가 적용된 씬에서 RGB+BBox+Segmentation 라벨이 함께 저장되는 데이터셋을 생성했고, 샘플의 라벨 정확도를 육안으로 검증했다. ✅

**과제 — 이 데이터셋으로 무엇을 할 수 있을까?**

- **학습 가능한 모델**: `out_franka_dr/`은 2-클래스(franka, cube) object detection 또는 instance segmentation 모델의 학습 데이터로 바로 쓸 수 있는 형태다(RGB + tight bbox + colorized/raw segmentation 마스크). 실제로 Module 5 과제(카메라 색 마스크로 큐브 위치 추정)를 이런 학습된 detector로 대체하면, 큐브 색이 매번 달라져도(도메인 랜덤화가 이미 그 경우를 커버함) 인식이 깨지지 않는 모델을 만들 수 있다는 것이 이 모듈 전체의 핵심 동기였다.
- **부족한 랜덤화 항목**:
  - **카메라 파라미터(focal length, FOV, 노이즈)를 안 건드렸다** — 실제 배포 카메라마다 렌즈 특성이 다르므로, 학습 시 이것도 랜덤화해야 특정 카메라에 과적합하지 않는다.
  - **배경/테이블 텍스처를 다양화하지 않았다** — 지금은 바닥이 항상 같은 파란 격자다. 실제 환경은 나무 책상, 다른 색 바닥 등 다양하므로, 3단계에서 해본 것처럼 바닥 재질/텍스처 교체 랜덤화를 Franka 씬에도 적용했어야 더 견고한 데이터셋이 됐을 것.
  - **다중 물체/occlusion 시나리오가 없다** — 큐브가 항상 하나뿐이라, 물체가 여러 개 겹쳐 있을 때의 부분 가려짐(occlusion) 학습이 안 된다. `occlusionRatio` 필드가 bbox 데이터에 이미 있으니(이번 데이터셋에서는 대부분 0에 가까움), 물체 수를 늘리면 자연스럽게 이 값도 다양해질 것.
  - **Franka의 자세(관절 각도)가 항상 고정**이다 — 로봇이 다양한 포즈를 취한 상태에서도 인식이 되어야 실제 pick-and-place 도중의 프레임에서도 안정적으로 동작할 텐데, 지금 데이터셋은 정지 자세만 담고 있다.
- 실제 모델 학습(bbox 검출기 파인튜닝 등)은 이번 세션 범위 밖 — 선택 심화 과제로 남겨둔다.

## 알려진 문제와 해결

| 관찰 | 원인 | 해결 |
|---|---|---|
| `orchestrator.run()` 호출 직후 출력 폴더가 비어있음 | `run()`은 시작 명령만 던지고 즉시 리턴 (블로킹 안 함) | `run_until_complete()` 사용 |
| RGB 이미지가 완전히 새까맣게 나옴 | 씬에 조명이 없음 | `rep.create.light(light_type="Dome", ...)` 추가 |
| 조명 intensity를 1000→30000까지 올려도 픽셀 값이 계속 `(1,1,1)` 근처 | `isaacsim.core.api.World()`를 안 만듦 — RTX 렌더 파이프라인 초기화가 안 됨 | `World()`를 `rep.new_layer()` 전에 생성 |
| `rep.modify.attribute("intensity", ...)`에서 `Type mismatch: expected 'int'` | 생성 시 정수 리터럴(`intensity=1000`)을 써서 USD 속성이 int로 고정됨, 속성명도 네임스페이스 누락 | 생성 시 `1000.0`(float)로, 수정 시 `"inputs:intensity"`로 |
| Franka가 segmentation/bbox에 안 잡힘 | Franka 링크 메시가 instanceable이라 조상 prim의 라벨이 전파 안 됨 (Module 5와 동일 함정) | 라벨 걸기 전에 `Usd.PrimRange` + `SetInstanceable(False)` |

---
이전: [`05-sensors-and-omnigraph.md`](../05-sensors-and-omnigraph.md) · 참고: [`06-replicator-synthetic-data.md`](../06-replicator-synthetic-data.md) · 다음: [`07-isaac-lab-custom-task.md`](../07-isaac-lab-custom-task.md)
