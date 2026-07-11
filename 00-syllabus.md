# Isaac Sim 심화 학습 교안 — 총론

**과정명**: Isaac Sim 단독 심화 트랙 (USD → 물리 → 센서 → 합성 데이터 → Isaac Lab)
**대상**: Python에 능숙하고, Isaac Sim은 기초 임포트/조인트 제어 정도만 겪어본 학습자
**전제 조건**: 없음 (ROS 2 지식 불필요) — 단, `~/Documents/ros2/` 6주 과정의 3~5주차에서 URDF 임포트·PhysX 진단·ROS 2 브릿지·Isaac Lab 보행 RL을 이미 겪었으므로, 필요할 때 그 결과물을 참고자료로만 재사용한다.
**기간**: 8개 모듈, 자기 주도 학습 (모듈당 권장 1~3일)
**실습 환경**: 로컬 워크스테이션, NVIDIA GeForce RTX 4070 (VRAM 12GB), Isaac Sim 5.1.0 (`~/isaacsim_env`, Python 3.11), Isaac Lab (`~/IsaacLab`)

이 문서는 총론이며, 각 모듈의 상세 교안은 `01-*.md` ~ `08-*.md` 파일을 참고한다. 실습 코드는 이 폴더(`~/Documents/isaacsim/`) 아래 `practicum/mNN_주제/` 하위 폴더에 모듈별로 쌓는다. 기존 최상위의 스크립트/영상들(`gait_isaac.py`, `rl_baseline.mp4` 등)은 이전 ROS2 과정 3~5주차의 결과물이며 이번 과정에서는 참고자료로만 취급하고 그대로 둔다.

---

## 1. 과정 목표

이 과정을 마치면:

1. USD(Universal Scene Description)의 Stage/Prim/Layer 구조를 이해하고 GUI와 Python 양쪽에서 씬을 다룰 수 있다.
2. `isaacsim.core.api` 기반 standalone 스크립트로 World/Robot/Articulation을 처음부터 구성할 수 있다.
3. PhysX 물리 속성(rigid body, articulation drive, collision approximation, solver 설정)을 원리 수준에서 이해하고 임의의 로봇/오브젝트에 대해 스스로 튜닝할 수 있다.
4. URDF/MJCF 임포터의 동작 원리를 이해하고, 매니퓰레이터(Franka)를 이용한 기본 조작(pick-and-place) 파이프라인을 구성할 수 있다.
5. 카메라/뎁스/세그멘테이션/라이다/IMU/접촉 센서를 Python API와 OmniGraph(Action Graph) 두 경로로 모두 구성할 수 있다.
6. Replicator로 도메인 랜덤화 기반 합성 데이터셋을 생성할 수 있다.
7. Isaac Lab의 manager-based workflow로 커스텀 태스크(관측/행동/보상/종료 조건)를 처음부터 설계하고, 제한된 VRAM 환경에서 병렬 환경 수를 조정하며 학습시킬 수 있다.
8. 위 요소를 조합한 캡스톤 프로젝트를 설계·기록할 수 있다.

## 2. 전체 모듈표

| 모듈 | 주제 | 형태 | 산출물 |
|---|---|---|---|
| 1 | USD와 Stage 기초 (GUI 중심) | 이론+GUI 실습 | 손으로 조립한 간단한 씬(.usd) |
| 2 | Python 스크립팅 API 핵심 | 이론+코드 | World/Robot 제어 standalone 스크립트 |
| 3 | PhysX 물리 심화 | 이론+진단 실습 | 처음부터 안정화한 커스텀 오브젝트 |
| 4 | 로봇 임포트와 매니퓰레이션 | 이론+코드 | Franka pick-and-place 데모 영상 |
| 5 | 센서 스위트 & OmniGraph | 이론+코드 | 카메라/라이다/IMU 멀티센서 로그 |
| 6 | Replicator와 합성 데이터 | 이론+코드 | 도메인 랜덤화 합성 데이터셋 |
| 7 | Isaac Lab 심화 (커스텀 태스크) | 이론+코드 | 직접 설계한 태스크의 학습 곡선 |
| 8 | 캡스톤 | 통합 프로젝트 | 블로그용 최종 데모 + 리포트 |

## 3. 사전 준비물

- Isaac Sim 5.1.0 (`~/isaacsim_env`), Isaac Lab (`~/IsaacLab`) 설치 확인 완료 (이미 되어 있음)
- GUI 실행: `isaacsim-ros2` 또는 `isaacsim-run` alias (`~/.bashrc` 참고) — ROS 2 관련 env가 필요 없는 모듈에서는 `isaacsim-run`으로 충분
- `conda` 환경을 쓰는 터미널이 아니라 plain 터미널에서 작업 (4주차 트러블슈팅 기록 참고: conda 활성화 시 툴체인 충돌 이력 있음)
- VRAM 12GB 제약 — 모듈 7에서 `num_envs`를 공식 예제 기본값보다 낮춰야 할 가능성 높음 (모듈 7에서 구체적으로 다룸)

## 4. 진행 방식

각 모듈 교안은 다음 구조를 따른다.

1. **학습 목표**
2. **선수 지식 확인** — 직전 모듈에서 반드시 필요한 것
3. **이론 세션** — 개념 설명 + 코드/설정 예제
4. **실습 가이드** — 단계별 진행 순서와 확인 포인트
5. **자주 나오는 질문 / 트러블슈팅**
6. **체크포인트 & 과제**
7. **참고자료**

실습이 끝나면 이전 과정과 같은 패턴으로, 실제로 겪은 문제와 삽질 기록을 담은 컴패니언 문서(`practicum/mNN_주제/README.md` 또는 블로그 원고)를 별도로 작성한다. 이 총론/모듈 교안 자체는 "계획"이고, 실제 실행 기록이 "진짜 자료"라는 원칙은 이전 과정과 동일하게 유지한다.

## 5. 평가 및 체크포인트

각 모듈의 체크포인트를 통과하지 못하면 다음 모듈로 넘어가기 전 보강한다. 특히 모듈 2(Python API)와 모듈 3(PhysX)은 이후 모든 모듈의 기반이므로 반드시 확실히 하고 넘어간다.

## 6. 공통 참고자료

| 자료 | 용도 |
|---|---|
| `docs.isaacsim.omniverse.nvidia.com` | Isaac Sim 5.x 공식 문서 — Python API 레퍼런스, Replicator, 센서 |
| `docs.omniverse.nvidia.com/usd` | USD 개념 공식 문서 |
| `github.com/isaac-sim/IsaacLab` | Isaac Lab 소스, manager-based 태스크 예제 |
| `~/Documents/ros2/03-isaac-sim-urdf-import-and-gait-video.md`, `04-ros2-isaac-bridge.md`, `05-isaac-lab-rl.md` | 이전 과정에서 실제로 겪은 PhysX/센서/RL 트러블슈팅 기록 (재사용) |

---
다음: [`01-usd-stage-basics.md`](./01-usd-stage-basics.md)
