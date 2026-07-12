# Isaac Sim 5.1 실습 튜토리얼 — USD부터 매니퓰레이션까지

Isaac Sim 5.1을 GUI와 Python standalone 스크립트 양쪽으로 처음부터 파고드는 실습 기록이다. 이론만 읽고 넘어가는 대신, **직접 문제를 만들고, 재현하고, 원인을 소스 코드까지 파고들어 진단**하는 방식으로 진행했다 — 그래서 각 모듈 README에는 정답 코드뿐 아니라 실제로 삽질한 과정과 원인 분석이 그대로 남아있다. Isaac Sim을 처음 시작하는 사람이 "왜 안 되지?"에 부딪혔을 때 참고할 수 있도록 쓴 문서다.

## 대상 독자

- Python에는 능숙하지만 Isaac Sim은 처음이거나 기초 정도만 다뤄본 사람
- ROS 2 지식은 필요 없음
- 이론 설명(`0N-*.md`)과 실습 기록(`mNN_*/README.md`)을 같이 보면서 따라 하는 것을 권장

## 환경

- Isaac Sim 5.1.0 (`~/isaacsim_env`, Python 3.11)
- NVIDIA GeForce RTX 4070 (VRAM 12GB) — 이 스펙 기준으로 대용량 시뮬레이션(병렬 환경 수 등)을 다룰 때 주의사항을 남겨둠
- GUI 실행: `isaacsim-run` alias (ROS 2 관련 확장이 필요 없는 모듈은 이걸로 충분)

## 구조

```
00-syllabus.md              전체 커리큘럼 개요 (8개 모듈)
01-usd-stage-basics.md      모듈별 이론 교안
02-python-scripting-api.md
03-physx-deep-dive.md
04-robot-import-and-manipulation.md
...

m01_usd_basics/README.md        모듈별 실습 기록 (튜토리얼 형식)
m02_python_api/README.md
m03_physx/README.md
m04_manipulation/README.md
...
```

`0N-*.md`는 "무엇을 배울지"를 정리한 이론/실습 가이드이고, `mNN_*/README.md`는 그 가이드를 따라 **실제로 실행한 결과, 마주친 에러, 원인 분석**을 담은 튜토리얼이다. 코드/에셋은 각 `mNN_*/` 폴더 안에 있다.

## 모듈 목록

| # | 주제 | 이론 | 실습 기록 | 상태 |
|---|---|---|---|---|
| 1 | USD/Stage 기초 — GUI로 빈 씬 조립 | [01-usd-stage-basics.md](01-usd-stage-basics.md) | [m01_usd_basics/README.md](m01_usd_basics/README.md) | 완료 |
| 2 | Python 스크립팅 API — standalone 로봇 제어 | [02-python-scripting-api.md](02-python-scripting-api.md) | [m02_python_api/README.md](m02_python_api/README.md) | 완료 |
| 3 | PhysX 심화 — 떨림/통과/반전/무게중심 4대 진단 | [03-physx-deep-dive.md](03-physx-deep-dive.md) | [m03_physx/README.md](m03_physx/README.md) | 완료 (무게중심 실습은 원인 불명으로 미해결 기록) |
| 4 | 로봇 임포트와 매니퓰레이션 — Franka Pick-and-Place | [04-robot-import-and-manipulation.md](04-robot-import-and-manipulation.md) | [m04_manipulation/README.md](m04_manipulation/README.md) | 완료 |
| 5 | 센서 & OmniGraph — 카메라/Lidar/IMU/접촉, 카메라 인식 pick 과제 | [05-sensors-and-omnigraph.md](05-sensors-and-omnigraph.md) | [m05_sensors/README.md](m05_sensors/README.md) | 완료 |
| 6 | Replicator 합성 데이터 | 예정 | — | 예정 |
| 7 | Isaac Lab 커스텀 태스크 | 예정 | — | 예정 |
| 8 | 캡스톤 | 예정 | — | 예정 |

전체 커리큘럼 목표와 평가 기준은 [00-syllabus.md](00-syllabus.md) 참고.

지금까지 실습에서 실제로 사용한 Python API를 카테고리별로 정리한 문서: [api-reference.md](api-reference.md)

## 이 튜토리얼의 특징

- **직접 실행 원칙**: GUI 클릭이든 Python 스크립트든, 실습 주체가 직접 실행하고 눈으로 본 결과만 기록했다. 대신 실행해보고 로그만 읽는 방식은 지양했다.
- **실패도 기록**: 원인을 못 찾고 끝난 실험(Module 3의 무게중심 재현)이나 다른 방법으로 우회한 사례(Module 4의 headless 카메라 캡처 실패 → GUI Movie Capture 전환)도 "왜 안 됐는지"와 "무엇을 배제했는지"까지 그대로 남겼다. Isaac Sim을 익히는 과정에서 실제로 부딪히는 문제들이라고 판단해서다.
- **API 시그니처는 소스에서 확인**: 에러 메시지만으로 원인이 불명확할 때는 `~/isaacsim_env/lib/python3.11/site-packages/isaacsim/` 아래 실제 설치된 소스를 직접 읽어서 확인했다 (예: `Articulation` vs `SingleArticulation`, `ArticulationAction` vs `ArticulationActions`의 차이).

## 몇 가지 반복적으로 마주친 함정

다음 모듈을 진행하기 전에 알아두면 삽질을 줄일 수 있는 것들:

- **로봇을 Content Browser로 드래그해 배치하면 원점(0,0,0)이 아닌 임의 위치에 놓일 수 있다.** IK/모션 생성 라이브러리 상당수가 로봇 베이스=원점을 가정하므로, 배치 직후 반드시 베이스 좌표를 확인할 것 ([Module 4](m04_manipulation/README.md#1-franka-배치하기-gui) 참고).
- **PhysX Position Drive는 오차(target - 현재값) 기반으로 힘을 계산한다.** Target을 안 바꾸고 stiffness/damping만 바꾸면 아무 일도 안 일어난다 ([Module 3](m03_physx/README.md#진단-1--관절-떨림jitter) 참고).
- **바닥에 놓는 오브젝트는 Translate Z를 half-height로 맞춰야 한다.** 살짝이라도 떠 있으면 물리 스텝이 시작되자마자 조용히 떨어져서, 이후 좌표 계산이 전부 틀어진다 ([Module 4](m04_manipulation/README.md#51-첫-번째-실패-큐브가-바닥에-떠-있었다) 참고).
- **Isaac Sim의 다양한 `Articulation` 관련 클래스는 배치(batched) 버전과 단일(single) 버전이 파라미터 이름과 액션 타입이 다르다** (`prim_path` vs `prim_paths_expr`, `ArticulationAction` vs `ArticulationActions`) — 섞어 쓰면 헷갈리는 에러가 난다.
- **IMU/접촉 센서는 반드시 실제 물리 바디의 자식으로 붙여야 한다.** 컨테이너 Xform(예: articulation root)에 붙이면 값이 절대 안 바뀐다 — Module 4/5에서 반복 확인 ([Module 5](m05_sensors/README.md#3-imu--접촉-센서--python-api) 참고).
- **instanceable USD 레퍼런스는 semantic label 상속을 막는다.** 조상 prim에 라벨을 걸어도 instanceable 자식 메시까지는 전파되지 않는다 — segmentation에서 물체가 안 잡히면 의심해볼 것 ([Module 5](m05_sensors/README.md#1-카메라-rgb--depth--segmentation) 참고).

## 라이선스 / 참고

개인 학습 기록이며, Isaac Sim 자체의 라이선스 및 NVIDIA 공식 문서 정책을 따른다.
