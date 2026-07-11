# Module 01 — USD와 Stage 기초 (GUI 중심)

**권장 소요: 이론 3h · 실습 5h**

---

## 1. 학습 목표

- USD의 Stage/Prim/Layer/Attribute 개념을 설명할 수 있다.
- Isaac Sim GUI의 Stage/Property/Content Browser 패널을 능숙하게 다룰 수 있다.
- 코드 없이 GUI만으로 간단한 씬(지면 + 물체 + 조명 + 카메라)을 조립하고 `.usd`로 저장할 수 있다.
- Reference/Payload로 다른 USD 파일을 씬에 불러오는 방법을 이해한다.

## 2. 선수 지식 확인

- 이전 과정(ROS2 3주차)에서 URDF Importer로 이미 로봇을 임포트해본 경험이 있다면, "그때 자동으로 생성된 Prim 트리"를 이번 모듈에서 원리 수준으로 다시 뜯어본다는 점을 미리 인지한다.
- 없다면 아무 사전 지식 없이 시작해도 무방 — 이 모듈이 진짜 입문이다.

---

## 3. 이론 세션

### 3.1 왜 URDF가 아니라 USD인가

URDF는 로봇 하나를 기술하기 위한 좁은 포맷(link/joint만 표현)이다. USD는 픽사가 영화 프로덕션(수만 개 오브젝트, 조명, 카메라, 애니메이션, 여러 아티스트의 동시 작업)을 위해 만든 범용 씬 포맷이며, Isaac Sim은 이 USD 위에서 동작한다. 로봇, 지면, 카메라, 라이트, 물리 속성이 전부 동일한 Prim 트리 안에 존재한다.

### 3.2 핵심 개념

| 개념 | 설명 |
|---|---|
| Stage | 씬 전체. 하나 이상의 `.usd`/`.usda`/`.usdc` 파일(Layer)로 구성됨 |
| Prim(Primitive) | Stage 안의 노드 하나. 경로로 접근(`/World/Robot/leg/thigh`) — 트리 구조 |
| Xform | 위치/회전/스케일 변환을 갖는 Prim 타입. 대부분의 그룹/로봇 링크가 이 타입 |
| Attribute | Prim이 갖는 값 (translate, color, mass 등) |
| Layer | Attribute를 겹쳐 쓰는 레이어. 포토샵 레이어처럼 위 레이어가 아래 레이어 값을 덮어씀 |
| Reference | 다른 USD 파일을 현재 Stage의 한 Prim 아래로 "불러와 붙이는" 것 (복사가 아니라 링크) |
| Payload | Reference와 비슷하지만 지연 로딩(무거운 에셋을 필요할 때만 로드) |

Prim 경로가 트리 구조라는 점은 ROS 2의 tf 트리(부모-자식 좌표계)와 유사하다 — 익숙하다면 그 직관을 그대로 가져와도 된다.

### 3.3 GUI 투어

라이브로 직접 열어보며 확인:
- 뷰포트 네비게이션 (Isaac Sim의 카메라 조작은 Blender/Maya와 단축키가 다르므로 각 버튼의 역할을 먼저 확인)
- **Stage 패널**: Prim 트리 뷰. 클릭하면 뷰포트에서 해당 Prim이 하이라이트됨
- **Property 패널**: 선택한 Prim의 모든 Attribute (Transform, 물리 속성, 렌더링 속성 등)
- **Content Browser**: 로컬/Nucleus 에셋 브라우저. 제공 예제 로봇/오브젝트 에셋 위치 확인
- Play/Stop 버튼 — Stop 상태에서는 물리가 비활성화됨. Attribute를 직접 편집하는 것은 대부분 Stop 상태에서, 관절을 실시간으로 흔드는 것은 Play 상태에서 한다는 차이를 명확히 구분

### 3.4 Layer와 편집 대상(Edit Target)

- 여러 Layer가 겹쳐 있을 때 지금 편집이 "어느 Layer에 쓰이는지"(Edit Target)가 항상 명시적으로 설정돼 있다. 잘못된 Layer에 편집하면 저장 후 사라지거나 의도와 다른 곳에 값이 써질 수 있다.
- Layer 패널에서 현재 Edit Target을 확인하는 법을 실습으로 확인.

### 3.5 Reference로 씬 조립하기

```
/World (Xform)
 ├── GroundPlane
 ├── Light (DomeLight)
 ├── Camera
 └── Props (Xform)
      ├── Cube_01 → Reference: /Isaac/Props/some_asset.usd
      └── Cube_02 → Reference: 동일 파일, 다른 prim path (인스턴스처럼 재사용)
```

같은 USD 파일을 여러 Prim 경로에서 Reference로 재사용하는 것이 "복사-붙여넣기"보다 저장 용량과 관리 측면에서 나은 이유를 짚는다.

---

## 4. 실습 가이드 (5h)

1. **빈 Stage에서 시작 (1h)** — `isaacsim-run`으로 GUI 실행, 빈 Stage에 GroundPlane + DomeLight 추가, Play를 눌러 아무 일도 안 일어나는 것(물리 바디가 없으므로)을 확인.
2. **기본 프리미티브로 물리 오브젝트 만들기 (1.5h)** — Cube/Sphere Prim 추가 → Property 패널에서 Rigid Body 물리 속성 부여(체크박스 하나로 활성화되는 것 확인) → Play 눌러 낙하 확인. Stop 후 위치를 바꾸고 다시 Play해서 Stop 상태 편집이 유지되는지 확인.
3. **Content Browser에서 제공 에셋 Reference로 불러오기 (1h)** — Isaac Assets 브라우저에서 로봇 또는 소품 에셋 하나를 Stage로 드래그, Stage 패널에서 생성된 Prim 트리를 펼쳐 실제 구조(Xform 계층, 물리 속성이 어디 붙어있는지)를 확인.
4. **Layer 실습 (0.5h)** — Layer 패널을 열어 현재 Edit Target 확인, 새 Layer 추가 후 그 위에서 오브젝트 색상만 바꿔보고 Layer를 껐다 켜서 값이 사라지는 것을 확인.
5. **저장 및 재확인 (1h)** — 전체 씬을 `.usd`로 저장 → Isaac Sim 재시작 → 다시 열어서 동일하게 복원되는지 확인. `practicum/m01_usd_basics/scene.usd`로 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. Property 패널에 물리 관련 항목이 안 보여요.**
A. Prim에 아직 Physics API(RigidBodyAPI, CollisionAPI 등)가 적용되지 않은 상태. Property 패널 하단의 "Add" 메뉴에서 Physics 관련 API를 추가해야 항목이 나타난다.

**Q. Reference로 불러온 에셋을 수정했는데 원본 파일도 바뀌나요?**
A. 기본적으로 수정 사항은 현재 편집 중인 Layer에만 쓰인다(원본 Reference 파일은 불변으로 유지되는 것이 일반적인 워크플로우). 원본 자체를 바꾸려면 해당 파일을 직접 열어 편집해야 한다.

**Q. Stage 패널에 있는 Prim이 뷰포트에 안 보여요.**
A. Prim의 Visibility 속성이 꺼져 있거나, 카메라 클리핑 범위 밖에 있거나, 실제로 Payload가 아직 로드되지 않은 상태일 수 있다. Stage 패널에서 아이콘으로 로드 상태를 구분할 수 있다.

---

## 6. 체크포인트 & 과제

**체크포인트**: GUI만으로 지면+조명+오브젝트+Reference 에셋이 포함된 씬을 조립하고 저장/재로드가 정상 동작한다. Stage 패널에서 임의의 Prim을 골라 그 Attribute를 Property 패널에서 설명할 수 있다.

**과제**: Content Browser의 제공 로봇 에셋 하나(URDF Importer를 쓰지 않고 이미 USD로 제공되는 것)를 Stage에 불러와 Prim 트리를 캡처하고, 어느 Prim이 Xform이고 어느 Prim에 물리 속성이 붙어있는지 표로 정리. 다음 모듈(Python API)에서 이 구조를 코드로 그대로 다시 만들어볼 것이다.

## 7. 참고자료

- `docs.isaacsim.omniverse.nvidia.com` — "User Interface" / "Stage" 섹션
- `openusd.org` — USD 공식 개념 문서 (Pixar)
- Isaac Sim Content Browser 내 제공 예제 에셋 목록

---
이전: [`00-syllabus.md`](./00-syllabus.md) · 다음: [`02-python-scripting-api.md`](./02-python-scripting-api.md)
