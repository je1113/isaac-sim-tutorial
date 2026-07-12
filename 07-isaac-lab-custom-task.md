# Module 07 — Isaac Lab 심화: 커스텀 태스크 설계

**권장 소요: 이론 4h · 실습 10h**

---

## 1. 학습 목표

- Isaac Lab의 manager-based workflow(Observation/Action/Reward/Termination/Event Manager) 구조를 설명할 수 있다.
- 제공 예제가 아닌, 직접 관측/행동/보상/종료 조건을 설계한 커스텀 태스크를 처음부터 만들 수 있다.
- RTX 4070(VRAM 12GB) 환경에서 `num_envs`를 조정해 OOM 없이 학습을 돌릴 수 있다.
- direct workflow와 manager-based workflow의 차이를 이해하고 상황에 맞게 고를 수 있다.

## 2. 선수 지식 확인

- 이전 과정(ROS2 5주차)에서 이미 제공 locomotion 예제로 baseline 학습, reward 변형 2종(`rl_smooth_gait_failed.mp4`의 실패 사례 포함), reward hacking 재현(`rl_reward_hacking.mp4`)까지 경험했다 — 이번 모듈은 "제공된 태스크의 reward만 수정"에서 "태스크 자체를 설계"하는 단계로 올라가는 것이 핵심 차이.
- reward hacking, 물리 불안정이 학습 실패로 이어지는 것을 이미 겪었으므로, 이번엔 새 태스크를 만들 때 처음부터 그 실패 패턴을 피하도록 설계에 반영한다.

---

## 3. 이론 세션

### 3.1 Manager-based vs Direct Workflow

| Workflow | 특징 | 언제 쓰나 |
|---|---|---|
| Manager-based | Observation/Action/Reward/Termination/Event를 각각 독립된 "Manager"와 "Term" 함수로 선언적으로 조합 | 항목을 자주 갈아끼우며 실험하는 연구/학습 단계 — 이번 모듈에서 이 방식을 사용 |
| Direct | 환경 클래스 하나에 `step`/`reset`을 직접 명령형으로 구현 (Gym 스타일에 더 가까움) | 성능 최적화가 중요하거나 이미 로직이 확정된 프로덕션 태스크 |

이전 과정 5주차에서 다룬 제공 locomotion 태스크는 이미 완성된 manager-based 태스크의 reward 항만 바꾼 것이었다 — 이번엔 Observation/Action/Reward/Termination Term을 전부 직접 선언한다.

### 3.2 커스텀 태스크의 5요소

```python
@configclass
class MyTaskEnvCfg(ManagerBasedRLEnvCfg):
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()  # 리셋 시 랜덤화 등
```

| Manager | 역할 | 이번 태스크 예 |
|---|---|---|
| Observation | 정책이 받는 입력 | 관절 각도/속도, 몸통 자세 |
| Action | 정책이 내리는 출력 | 목표 관절 위치(또는 delta) |
| Reward | 각 스텝의 보상 항 합산 | 목표 도달 + 안정성 − 에너지 |
| Termination | 에피소드 종료 조건 | 낙상, 시간 초과, 목표 도달 |
| Event | 리셋 시 초기 상태 랜덤화 | 시작 pose를 매 에피소드 조금씩 다르게 |

**강의 포인트**: Termination을 너무 관대하게 설계하면(예: 웬만해선 안 끝남) 학습이 다양한 실패 사례를 못 겪어 오히려 비효율적이라는 것 — 이전 과정에서 본 reward hacking과는 다른 종류의 설계 실수임을 구분해서 짚는다.

### 3.3 태스크 선정: 이전 locomotion과 다른 것으로

이번 모듈은 이전 과정에서 이미 다룬 "다리 보행"을 반복하지 않고, 새로운 종류의 태스크를 하나 골라 처음부터 설계한다. 예:
- 모듈 4의 Franka 팔로 "목표 지점에 엔드이펙터 도달"(reach) 태스크를 강화학습으로 (IK 없이 직접 관절 명령을 학습)
- 또는 큐브를 특정 지점까지 밀어 옮기는(push) 태스크

이렇게 하면 모듈 4~5의 매니퓰레이션 자산을 재사용하면서 RL 관점에서 다시 접근하게 되어, "직접 궤적 설계(모듈 4의 IK)" vs "학습으로 획득(이번 모듈)"을 같은 태스크로 직접 비교할 수 있다는 이점이 있다.

### 3.4 VRAM 제약과 num_envs

- RTX 4070의 VRAM은 12GB로, 공식 문서/예제가 기본값으로 제시하는 `num_envs`(수천 단위, 종종 4096)는 그대로 돌리면 OOM이 날 가능성이 높다.
- 대응 원칙: **먼저 낮은 `num_envs`(예: 256~512)로 파이프라인이 정상 동작하는지 확인 → 점진적으로 올리며 OOM 임계값 탐색 → 그 아래로 여유를 두고 확정**. 관측/렌더링을 켜는지(headless 여부)도 VRAM 사용량에 영향을 준다는 것을 실습으로 확인.
- 참고: 이전 과정 5주차 트러블슈팅에서도 "GPU 메모리 부족 시 num_envs를 줄인다"는 원칙을 이미 확인한 바 있음 — 이번엔 그 값을 실제로 이 하드웨어에 맞게 캘리브레이션한다.

### 3.5 Event Manager로 Domain Randomization

- 모듈 6에서 다룬 도메인 랜덤화 개념이 여기서도 등장한다 — 다만 목적이 다르다: 모듈 6은 "인식 모델을 위한 시각적 다양성", 여기서는 "정책이 물리적 변동(질량, 마찰, 초기 pose)에 강건해지도록" 리셋마다 물리 파라미터를 조금씩 흔드는 것.

---

## 4. 실습 가이드 (10h)

1. **기존 예제 태스크 코드 구조 정독 (1.5h)** — Isaac Lab 제공 예제 하나(`~/IsaacLab/source/isaaclab_tasks`)를 열어 Observation/Action/Reward/Termination/Event Cfg가 각각 어디 정의됐는지 대응시켜 정리.
2. **num_envs 캘리브레이션 (1h)** — 기존 예제를 num_envs만 바꿔가며(예: 4096→1024→256) 실행해 이 GPU에서 안정적으로 돌아가는 상한을 실측.
3. **커스텀 태스크 설계 (3h)** — 3.3절에서 고른 태스크(예: Franka reach)의 Observation/Action/Reward/Termination을 처음부터 작성. 보상은 단순하게 시작(거리 기반 reward 하나)한 뒤 3.2 표의 나머지 요소를 순차적으로 추가.
4. **학습 실행 및 TensorBoard 분석 (2h)** — 캘리브레이션한 num_envs로 학습, reward curve/episode length curve 확인.
5. **Event Manager로 리셋 랜덤화 추가 후 재학습 (1.5h)** — 초기 pose/목표 위치를 리셋마다 랜덤화하도록 Event를 추가하고, 랜덤화 전후 학습 난이도/수렴 속도 비교.
6. **정책 rollout 검증 및 영상화 (1h)** — 학습된 정책을 Isaac Sim에서 재생, 모듈 4에서 IK로 직접 만든 동작과 시각적으로 비교.

결과와 학습 로그는 `practicum/m07_isaaclab_custom/`에 저장.

---

## 5. 자주 나오는 질문 / 트러블슈팅

**Q. num_envs를 줄였더니 학습이 아예 안 돼요(reward가 안 오름).**
A. 병렬 환경 수가 너무 적으면 한 배치당 경험(experience) 양이 부족해 학습이 불안정해질 수 있다 — 이 경우 배치 크기/학습률 등 다른 하이퍼파라미터를 함께 조정해야 하는 트레이드오프가 있음을 확인.

**Q. Reward는 오르는데 Termination 조건에 거의 안 걸려요.**
A. Termination이 너무 관대하면(3.2절 강의 포인트) 정책이 실패 회복을 배울 기회가 적다 — 조건을 좀 더 엄격하게 만들어 재학습 비교.

**Q. Manager-based 태스크에서 특정 Term 하나만 바꾸고 싶은데 클래스 전체를 다시 써야 하나요?**
A. 대부분 기존 Cfg를 상속해 해당 Term만 override하는 방식이 가능하다 — 전체를 처음부터 다시 쓰기보다 상속 구조를 먼저 확인.

---

## 6. 체크포인트 & 과제

**체크포인트**: 제공 예제를 복사/수정한 것이 아니라 Observation/Action/Reward/Termination을 스스로 선언한 커스텀 태스크가 이 GPU에서 OOM 없이 수렴까지 학습되고, rollout으로 결과를 확인했다.

**과제**: 3.4절의 num_envs 캘리브레이션 결과(어느 값에서 OOM이 나고 어느 값이 안전한지)와, Event Manager 랜덤화 적용 전후의 학습 곡선 비교를 정리 — 다음 캡스톤 모듈에서 이 태스크를 프로젝트의 한 축으로 재사용할 수 있다.

## 7. 참고자료

- `github.com/isaac-sim/IsaacLab` — `isaaclab_tasks`(manager-based 예제), `isaaclab.envs.mdp`(Observation/Reward/Termination Term 라이브러리)
- Isaac Lab 문서 내 "Manager-Based Environments" / "Direct Workflow" 비교 섹션
- `~/Documents/ros2/05-isaac-lab-rl.md` — 이전 과정의 reward shaping/트러블슈팅 기록 (`rl_baseline.mp4`, `rl_reward_hacking.mp4`, `rl_smooth_gait_failed.mp4` 포함)

---
이전: [`06-replicator-synthetic-data.md`](./06-replicator-synthetic-data.md) · 다음: [`08-capstone.md`](./08-capstone.md)
