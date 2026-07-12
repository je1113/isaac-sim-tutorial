# Module 07 — Isaac Lab 커스텀 태스크: Franka 큐브 밀기(Push)

이 문서는 Isaac Lab의 manager-based workflow(Observation/Action/Reward/Termination/Event)를 활용해, 라이브러리에 없는 태스크(큐브를 목표 지점까지 밀기)를 처음부터 설계하고 학습시킨 실습 기록이다. 기존 예제(`reach`)를 먼저 분석해 구조를 파악한 뒤, 일부러 "관대한 Termination"이라는 설계 허점이 있는 첫 버전을 만들어 학습이 정체되는 것을 직접 확인하고, 이를 개선한 두 번째 버전으로 실제로 풀리는지 비교했다.

---

## 준비물

- Isaac Sim 5.1.0 / Isaac Lab (`~/IsaacLab`), `isaacsim_env` 활성화 (`unset CONDA_PREFIX && source ~/isaacsim_env/bin/activate`)
- GPU: RTX 4070 12GB

## 1. 기존 예제(`Isaac-Reach-Franka-v0`) 구조 분석

새 태스크를 설계하기 전에, Isaac Lab이 이미 제공하는 `manager_based/manipulation/reach` 예제를 열어 5요소가 각각 어디 정의됐는지 대응시켰다.

| Manager | 위치 | 내용 |
|---|---|---|
| Observation | `reach_env_cfg.py` `ObservationsCfg` | joint_pos/vel(noise 포함) + pose_command + last_action |
| Action | `config/franka/joint_pos_env_cfg.py` | `JointPositionActionCfg`, `panda_joint.*` |
| Reward | `RewardsCfg` + `mdp/rewards.py` | 거리 오차(L2) + tanh 커널 + 방향 오차 + action/속도 페널티 |
| Termination | `TerminationsCfg` | **`time_out`만 존재** |
| Event | `EventCfg` | 리셋마다 관절 위치를 기본값의 0.5~1.5배로 랜덤 스케일 |

여기서 눈에 띄는 설계 허점: **Termination이 `time_out`뿐이라 성공/실패 신호가 정책에 전혀 전달되지 않는다.** 이 관찰이 아래 2단계 설계에 직접 영향을 줬다.

## 2. num_envs 캘리브레이션

공식 기본값(`num_envs=4096`)부터 실측했다.

```bash
python3 -u scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-Franka-v0 --num_envs 4096 --headless --max_iterations 20 --run_name calib_4096
```

**결과**: 예상과 달리 OOM 없이 255,343 steps/sec로 문제없이 돌아갔다. 이전 과정(6주 ROS2 코스)의 로코모션 태스크(AnymalC)가 이 GPU에서 `num_envs=64`까지 낮춰야 했던 것과 대조적인데, `reach`는 씬이 훨씬 가볍기 때문으로 보인다(팔 하나 + 테이블뿐, 지형/여러 강체 없음). 실제 OOM 경계(8192, 16384 등)까지는 컴퓨터에 부담을 주지 않기 위해 찾지 않기로 하고, **`num_envs=4096`을 이후 모든 학습에 그대로 사용**했다.

## 3. 태스크 설계 — Push

라이브러리에 이미 `reach`가 있어 그대로 재구현하면 사실상 베끼기에 가깝다고 판단해, 목표를 바꿔 **큐브를 테이블 위 목표 지점까지 밀기(Push)**로 설계했다. 라이브러리에 없는 태스크라 5요소를 전부 직접 작성해야 했고, Module 4(Franka)·Module 6(Replicator에서 쓴 `dex_cube`)의 자산을 재사용할 수 있었다.

코드는 `push_task/`에 있으며, IsaacLab 소스 트리 밖(이 저장소 안)에 있는 **외부 태스크**다. `train_push.py`/`play_push.py`는 IsaacLab 공식 `scripts/reinforcement_learning/rsl_rl/{train,play}.py`를 그대로 복사하고 `import push_task` 한 줄만 추가해 gym 등록이 되도록 만든 실행 스크립트다.

```
push_task/
  __init__.py            gym.register (Isaac-Push-Franka-{Minimal,,-Play}-v0)
  push_env_cfg.py         Scene + 5개 Manager Cfg + 두 버전(Minimal / Full)
  mdp/
    observations.py       cube_position_in_robot_root_frame (커스텀)
    rewards.py             cube_to_target_distance 등 3개 (커스텀)
    terminations.py         cube_reached_target (커스텀)
  agents/rsl_rl_ppo_cfg.py
```

일부러 **두 버전을 순서대로** 만들었다 — Reach 예제에서 짚었던 "관대한 Termination" 문제를 우리 태스크에서도 재현시켜본 뒤, 고치면 실제로 나아지는지 보기 위해서다.

### Phase 1 — `FrankaPushEnvCfg_Minimal` (일부러 단순하게)

| Manager | 설계 |
|---|---|
| Reward | `cube_to_target_distance` 단 하나 (거리 L2, weight=-1.0) |
| Termination | **`time_out`만** — Reach 예제와 똑같은 허점을 의도적으로 재현 |
| Event | 큐브·로봇 관절 모두 리셋 시 고정값(랜덤화 없음) |
| Command(목표 지점) | 고정된 한 점 |

### Phase 2 — `FrankaPushEnvCfg` (Full)

| Manager | 추가된 것 |
|---|---|
| Reward | + tanh 정밀 보상, + ee→큐브 접근 유도 보상, + action_rate/joint_vel 페널티 |
| Termination | + `success`(큐브가 목표 3cm 이내 도달 시 조기 종료), + `cube_fell`(테이블 밖으로 낙하) |
| Event | 큐브 시작 위치 랜덤화(±10~15cm), 목표 지점 랜덤 영역으로 확대 |

## 4. 학습 — Phase 1 vs Phase 2

두 버전 모두 동일 조건(num_envs=4096, headless, 300 iteration)으로 학습해 비교했다.

```bash
python3 ~/Documents/isaacsim/practicum/m07_isaaclab_custom/train_push.py --task Isaac-Push-Franka-Minimal-v0 --num_envs 4096 --headless --max_iterations 300 --run_name minimal_v1
python3 ~/Documents/isaacsim/practicum/m07_isaaclab_custom/train_push.py --task Isaac-Push-Franka-v0 --num_envs 4096 --headless --max_iterations 300 --run_name full_v1
```

| iter | Minimal position_error | Full position_error | Full reward | Full success% |
|---|---|---|---|---|
| 0 | 0.428 | 0.446 | -0.07 | 1% |
| 40 | 0.227 (최저점) | 0.220 | 0.93 | 4% |
| 240 | 0.336 (정체) | 0.084 | 4.04 | 5% |
| 299 | 0.325 (정체) | 0.086 | 4.31 | 3% |

**Phase 1(Minimal)**은 iteration 40 근처에서 0.22m까지 개선되다가 이후 0.32~0.36m 사이에서 정체된 채 끝났다 — `time_out`만 있어 에피소드가 항상 180스텝(=6초)을 꽉 채우고, 정책이 "대충 가까이는 가지만 더 정밀해질 이유가 없는" 지점에서 안주해버린 것으로 보인다.

**Phase 2(Full)**는 계속 개선되어 최종 0.08~0.09m까지(약 4배 더 정밀하게) 수렴했고, `success` termination도 실제로 발생하기 시작했다(3~5%). 1단계에서 Reach 예제를 보며 짚었던 "관대한 Termination + 단순 보상의 한계"가 우리 태스크에서 그대로 재현됐고, Termination·Reward·Event를 보강하니 실제로 풀린 것을 직접 확인한 셈이다.

## 5. 정책 rollout 및 영상 확인

```bash
python3 ~/Documents/isaacsim/practicum/m07_isaaclab_custom/play_push.py --task Isaac-Push-Franka-Play-v0 --num_envs 4
```

`--headless` 없이 GUI 창을 띄우고, Module 4에서 썼던 것과 동일한 방식(GUI 내장 Movie Capture로 직접 녹화 — 이 환경에서 headless 카메라 캡처가 안 되는 문제가 Module 4에서 이미 확인된 바 있어 처음부터 이 방법을 사용)으로 `push_policy_rollout.webm`을 남겼다.

**관찰**: 큐브가 목표 지점 근처까지 밀리는 것은 확인됐지만, Module 4의 IK 기반 pick-and-place와 비교하면 **팔의 움직임이 훨씬 급작스럽다** — IK 궤적은 웨이포인트 사이를 부드럽게 보간하지만, RL 정책은 스텝마다 독립적으로 관절 목표를 내보내다 보니 최적 경로를 "찾아가는" 과정에서 방향 전환이 잦고 덜 매끄럽다. `action_rate_l2` 페널티를 넣었음에도 완전히 매끈한 움직임까지는 만들지 못했다 — 페널티 가중치(`-1e-4`)를 더 키우면 완화될 여지가 있어 보이지만 이번 세션에서는 시도하지 않았다.

---

## 체크포인트 & 과제 (회고)

**체크포인트**: 제공 예제를 복사/수정한 게 아니라 Observation/Action/Reward/Termination/Event를 스스로 선언한 커스텀 태스크(Push)가 이 GPU에서 OOM 없이 수렴까지 학습됐고, rollout으로 결과를 확인했다. ✅

**과제 — num_envs 캘리브레이션 & 랜덤화 전후 비교**
- num_envs: 4096에서 문제없이 동작 확인, 실제 OOM 상한은 컴퓨터 부담을 피하기 위해 찾지 않음 (필요 시 8192부터 순차 시도 권장).
- Event 랜덤화 전후 비교: 위 4절 표 참고 — Phase 1(랜덤화 없음, 단순 보상, time_out만)은 0.32m대에서 정체, Phase 2(랜덤화 추가 + 보상/종료 조건 보강)는 0.08m대로 수렴. 다음 캡스톤 모듈에서 이 태스크를 재사용할 수 있다.

**남은 부분**:
- `action_rate` 페널티를 더 키워 움직임을 부드럽게 만드는 실험은 하지 않음 — 다음에 이어서 해볼 만하다.
- Curriculum Manager(보상 가중치를 학습 중 점진적으로 변경)는 이번 태스크에서 쓰지 않음 — reach/lift 예제는 사용하지만, 5요소(Observation/Action/Reward/Termination/Event)에 집중하기 위해 의도적으로 범위에서 뺐다.

## 알려진 문제와 해결

| 관찰 | 원인 | 해결 |
|---|---|---|
| 리다이렉션(`> file.log`)한 명령이 파일을 안 만들고 화면에 그대로 출력됨, Hydra가 로그 경로를 override 인자로 파싱하려다 에러 | 여러 줄을 백슬래시(`\`)로 이어서 붙여넣을 때 터미널이 줄을 잘못 이어붙임 | 리다이렉션 포함 전체 명령을 한 줄로 작성 |
| 학습 로그 파일에 iteration 57~236 구간 콘솔 출력이 통째로 빠짐(0~56, 237~299만 존재) | 원인 불명 — rsl_rl 콘솔 출력 빈도가 구간에 따라 달라지는 것으로 보이나 정확한 메커니즘은 확인 못함 | exit code/학습 소요 시간으로 300 iteration이 실제로 다 돌았음을 확인 — 학습 결과 자체엔 영향 없음. 콘솔 출력만 필요하면 `python3 -u`(unbuffered) 옵션을 추가해볼 것 |

---
이전: [`06-replicator-synthetic-data.md`](../06-replicator-synthetic-data.md) · 참고: [`07-isaac-lab-custom-task.md`](../07-isaac-lab-custom-task.md) · 다음: [`08-capstone.md`](../08-capstone.md)
