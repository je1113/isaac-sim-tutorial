# Module 06 — Replicator와 합성 데이터 생성

**권장 소요: 이론 3h · 실습 7h**

---

## 1. 학습 목표

- Replicator의 randomizer/writer 구조를 설명할 수 있다.
- Domain Randomization(조명, 텍스처, 물체 pose, 배경)을 적용한 씬을 구성할 수 있다.
- Writer를 이용해 RGB + Bounding Box + Segmentation 라벨이 자동으로 붙은 합성 데이터셋을 생성할 수 있다.
- 왜 합성 데이터에 "무작위성"이 필요한지(sim-to-real gap, 과적합 방지) 설명할 수 있다.

## 2. 선수 지식 확인

- 모듈 5의 카메라/세그멘테이션 API 경험을 그대로 재사용 — Replicator는 그 위에 "자동 랜덤화 + 자동 라벨링"을 얹은 레이어라고 이해하면 빠르다.
- 모듈 5 과제에서 시도한 "카메라로 물체 위치 추정" pick-and-place가, 왜 애초에 다양한 조명/배경에서 학습된 인식 모델이 필요한지에 대한 동기부여로 연결된다.

---

## 3. 이론 세션

### 3.1 왜 합성 데이터인가

- 실제 카메라로 라벨링된 데이터를 모으는 것은 비싸고 느리다(사람이 직접 bounding box를 그려야 함).
- 시뮬레이터에서는 물체의 정확한 3D pose를 이미 알고 있으므로 **라벨이 자동으로, 완벽하게 정확하게** 나온다 — bounding box, segmentation mask, depth 등 사람이 만들면 오류가 나기 쉬운 라벨일수록 이득이 크다.
- 문제는 "시뮬레이션에서 학습한 모델이 실제 카메라 이미지에서도 통하는가"(sim-to-real gap) — 이를 줄이는 핵심 기법이 **Domain Randomization**: 매 샘플마다 조명/텍스처/배경/카메라 각도를 무작위로 바꿔, 모델이 특정 렌더링 디테일에 과적합하지 않고 "진짜 중요한 특징"만 배우도록 강제한다.

### 3.2 Replicator 핵심 구조

| 개념 | 역할 |
|---|---|
| Randomizer | 매 프레임(또는 매 N프레임)마다 특정 속성을 무작위로 바꾸는 함수 (위치, 회전, 색상, 조명 강도 등) |
| Trigger | Randomizer를 언제 실행할지(매 프레임 `on_frame`, N번째마다 등) |
| Writer | 렌더링 결과 + 라벨을 디스크에 저장하는 방식 정의 (기본 제공 `BasicWriter` 등) |
| Annotator | RGB 외에 bounding box, segmentation, depth 등 추가로 뽑을 데이터 종류 |

```python
import omni.replicator.core as rep

with rep.new_layer():
    camera = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
    cube = rep.create.cube(semantics=[("class", "cube")])

    with rep.trigger.on_frame(num_frames=100):
        with cube:
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
            )
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir="out/", rgb=True, bounding_box_2d_tight=True, semantic_segmentation=True)
    writer.attach([rep.render_product.create(camera, (640, 480))])

rep.orchestrator.run()
```

### 3.3 Semantic Label과 자동 라벨링

- 모듈 5에서 "segmentation ID와 클래스 매핑이 별도로 필요하다"고 짚었던 부분을, Replicator에서는 `semantics=[("class", "cube")]`처럼 물체 생성 시점에 라벨을 못박아 자동으로 매핑되게 한다.
- Writer가 저장하는 결과물에는 이미지뿐 아니라 각 물체의 클래스/인스턴스 ID/bounding box 좌표가 JSON 등으로 함께 저장된다는 것을 실제 출력 파일로 확인.

### 3.4 무엇을 랜덤화할 것인가

일반적으로 랜덤화 대상:
- 물체 pose(위치/회전), 색상/텍스처
- 조명 강도/색온도/방향(DomeLight, distant light 등)
- 카메라 위치/각도
- 배경(다른 배경 USD를 매 샘플마다 교체)

**강의 포인트**: 무엇을 랜덤화할지는 "실제 배포 환경에서 실제로 변할 수 있는 것"을 기준으로 고른다 — 예를 들어 물체의 물리적 크기가 실제로는 고정이라면 크기를 랜덤화하는 것은 오히려 학습을 방해할 수 있다.

---

## 4. 실습 가이드 (7h)

1. **최소 Replicator 스크립트 (1.5h)** — 큐브 하나 + pose/color 랜덤화 + BasicWriter로 RGB만 100장 생성, 실제로 매 이미지가 다른지 육안 확인.
2. **Bounding Box + Segmentation 라벨 추가 (1.5h)** — Annotator를 추가해 bounding box/segmentation까지 저장, 출력 JSON을 열어 라벨 구조 확인.
3. **조명/배경 랜덤화 확장 (2h)** — DomeLight 강도/색온도 랜덤화, 배경 텍스처 또는 다른 배경 에셋 교체 랜덤화 추가.
4. **모듈 4/5 씬에 적용 (1.5h)** — pick-and-place에서 쓴 큐브+테이블 씬에 도메인 랜덤화를 적용해 "인식 모델 학습용" 미니 데이터셋(수백 장) 생성.
5. **간단한 검증 (0.5h)** — 생성된 데이터셋에서 임의의 샘플 몇 장을 골라 bounding box를 이미지 위에 그려 라벨이 실제로 정확한지 시각적으로 검증.

결과 데이터셋(샘플 일부만, 전체는 용량 문제로 blog에만)과 스크립트는 `practicum/m06_replicator/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. Randomizer가 매 프레임 적용이 안 되고 첫 프레임 값으로 고정돼요.**
A. `rep.trigger.on_frame` 컨텍스트 밖에서 randomizer를 호출했거나, `rep.orchestrator.run()`을 호출하지 않고 `world.step()`만 돌린 경우 — Replicator의 트리거 체계는 일반 `world.step()` 루프와 별개로 동작한다는 점을 확인.

**Q. Bounding box가 물체보다 훨씬 크거나 작게 나와요.**
A. `bounding_box_2d_tight`(실제 렌더링된 형상 기준)와 `bounding_box_2d_loose`(3D bbox를 2D로 투영, 가려진 부분 포함) 등 종류가 다르면 결과가 다르게 나온다 — 용도에 맞는 종류를 선택했는지 확인.

**Q. 생성 속도가 너무 느려요.**
A. 해상도, 워밍업 스텝 수, Annotator 개수가 많을수록 프레임당 렌더링 비용이 커진다 — 우선 저해상도로 파이프라인을 검증한 뒤 최종 해상도로 올리는 순서를 권장.

---

## 6. 체크포인트 & 과제

**체크포인트**: 도메인 랜덤화가 적용된 씬에서 RGB+BBox+Segmentation 라벨이 함께 저장되는 데이터셋을 생성했고, 샘플 몇 장의 라벨 정확도를 육안으로 검증했다.

**과제**: 모듈 6에서 만든 합성 데이터셋을 이용해(직접 모델을 학습시키지 않더라도) "이 데이터셋으로 어떤 인식 모델을 학습시킬 수 있을지, 어떤 랜덤화 항목이 부족해 보이는지"를 블로그용 회고로 정리 — 실제 모델 학습(예: 간단한 bbox 검출기 파인튜닝)은 선택 심화 과제로 남겨둔다.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "Replicator" 섹션, Randomizer/Writer API 레퍼런스
- `isaacsim.replicator.examples`, `isaacsim.replicator.domain_randomization` 소스/예제
- NVIDIA Replicator 튜토리얼 (공식 문서 내 "Synthetic Data Generation")

---
이전: [`05-sensors-and-omnigraph.md`](./05-sensors-and-omnigraph.md) · 다음: [`07-isaac-lab-custom-task.md`](./07-isaac-lab-custom-task.md)
