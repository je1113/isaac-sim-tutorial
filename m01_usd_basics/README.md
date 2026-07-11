# Module 01 — USD/Stage 기초: 빈 씬에서 직접 조립하기

이 문서는 **빈 Stage에서 시작해서 GUI만으로** Ground Plane, 물리 오브젝트, Reference 에셋, Layer 오버라이드까지 조립하는 실습 가이드다. 코드는 전혀 쓰지 않는다. 이전 ROS2 과정의 biped/URDF 에셋(`test.usd`)은 참조하지 않고, 완전히 새로운 Stage에서 시작한다.

---

## 1. Ground Plane + Dome Light 만들기

### 배울 것
씬의 최소 구성 요소 — 바닥과 조명 — 를 메뉴 프리셋만으로 만들 수 있다는 것.

### 단계별 실습
1. `Create → Physics → Ground Plane`
2. `Create → Light → Dome Light`
3. Play를 눌러본다

### 결과
아직 물리 오브젝트가 없으므로 Play해도 아무 일도 일어나지 않는다 — 이 상태가 정상이다.

---

## 2. 프리미티브에 물리 속성 부여하기

### 배울 것
Rigid Body API와 Collider(Collision) API가 각각 어떤 역할을 하는지. (통과 현상 자체를 직접 재현하는 실험은 [Module 3 진단 2](../m03_physx/README.md#진단-2--collision-유무에-따른-통과-현상)에서 별도 오브젝트로 다룬다.)

### 단계별 실습
1. `Create → Shape → Cube`로 Ground Plane 위쪽(z≈1)에 큐브 생성
2. 이 상태에서 Property 패널을 보면 물리 관련 항목이 안 보인다 — 아직 Physics API가 안 붙어있기 때문
3. `+ Add → Physics → Rigid Body` 추가
4. `+ Add → Physics → Collider` 추가
5. Play

### 결과
큐브가 자유낙하해 Ground Plane 위에 정확히 멈춘다. Stop 후 큐브 위치를 옮기고 다시 Play하면, Stop 상태에서의 편집이 유지된 채로 새 위치에서 다시 낙하한다.

---

## 3. Content Browser에서 Reference로 에셋 불러오기

### 배울 것
Content Browser 에셋을 드래그해 불러오면 일반 Prim이 아니라 **Reference**로 Stage에 들어온다는 것.

### 단계별 실습
1. Content Browser에서 원하는 에셋을 찾는다
2. 뷰포트 또는 Stage 패널로 드래그해서 불러온다
3. Stage 패널에서 방금 들어온 Prim을 확인한다

### 결과
Reference로 들어온 Prim은 일반 Prim과 다르게 표시되고, Prim 트리가 실제로 생성된다.

---

## 4. Layer로 오버라이드 체감하기

### 배울 것
USD의 Layer가 "포토샵 레이어처럼 위 레이어가 아래 레이어 값을 덮어쓴다"는 개념을, 값을 껐다 켰다 하면서 직접 눈으로 확인한다.

### 단계별 실습
1. Layer 패널에서 Root Layer가 기본 Edit Target으로 잡혀있는 것을 확인
2. `+`로 Sub-layer를 추가하고, 그 Layer를 Edit Target으로 지정
3. 그 상태에서 큐브의 Display Color를 변경
4. Layer 패널에서 방금 만든 Layer를 껐다 켰다 해본다
5. Edit Target을 다시 Root Layer로 되돌린다

### 결과
Layer를 끄고 켤 때마다 큐브 색상이 바뀌었다 사라졌다 한다 — 이 값이 그 Sub-layer에만 저장되어 있다는 뜻이다. 토글 한 번으로 "레이어 오버라이드"라는 추상적 개념이 가장 직관적으로 와닿는 실습이었다.

---

## 5. 저장 → 재시작 → 복원 확인

### 단계별 실습
1. `File → Save As`로 `practicum/m01_usd_basics/scene.usd`에 저장
2. Isaac Sim을 완전히 종료했다가 다시 실행
3. 저장한 파일을 다시 연다

### 결과
Ground Plane, 큐브, Reference 에셋, Layer 색상 오버라이드까지 전부 그대로 복원된다.

---

## 체크리스트

- [x] Ground Plane / Dome Light를 GUI만으로 생성했는가
- [x] 프리미티브에 Rigid Body + Collider를 직접 추가하고 낙하/정지를 확인했는가
- [x] Stop 상태에서의 편집이 Play 후에도 유지되는 것을 확인했는가
- [x] Content Browser 에셋을 Reference로 불러오고 Prim 트리를 확인했는가
- [x] Sub-layer를 만들어 Edit Target을 전환하고, 값이 그 Layer에만 저장된다는 것을 On/Off 토글로 확인했는가
- [x] 저장 후 재시작 → 재로드까지 정상 복원을 확인했는가

## 알려진 문제와 해결

이번 모듈은 5단계 전부 큰 트러블 없이 순조롭게 완료됐다 — 특별히 기록할 오류가 없었다.

## 남은 과제 (아직 안 한 것)

- Rigid Body만 있고 Collider가 없는 프리미티브가 바닥을 그대로 통과하는지 재현하는 실험 → [Module 3 진단 2](../m03_physx/README.md#진단-2--collision-유무에-따른-통과-현상)에서 별도 오브젝트로 완료함

---
참고: [`01-usd-stage-basics.md`](../01-usd-stage-basics.md) · 다음: [`m02_python_api/README.md`](../m02_python_api/README.md)
