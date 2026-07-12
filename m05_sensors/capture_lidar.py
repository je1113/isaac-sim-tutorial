from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.sensors.rtx import LidarRtx

OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m05_sensors"

world = World(stage_units_in_meters=1.0)
# 바닥 없이 진행: 기본 GroundPlane을 깔면 z=0.5 수평 스캔 광선이 거의 평행(grazing angle)으로
# 바닥과 만나면서 실제 벽/장애물과 무관한 원거리(최대 ~10m) 히트가 대량으로 섞여 들어온다
# (동심원처럼 반지름이 점점 커지는 패턴으로 나타남 - 바닥을 빼자 즉시 사라져서 확인됨)

# ㄱ자(L자) 모양 장애물 배치 - 라이다가 형상을 제대로 잡아내는지 확인하기 좋은 형태
# (주의: 이 Debug_Rotary 라이다는 헤드리스에서 render 스텝과 회전 타이밍이 완벽히
# 맞물리지 않아, 프레임을 아무리 많이 누적해도 특정 방위각 구간은 거의 스캔되지 않는
# 현상을 겪었다 - 실측으로 확인한 "안정적으로 스캔되는" 방위각 범위(대략 -35°~118°)
# 안에 모든 물체를 배치해 재현성을 확보)
world.scene.add(FixedCuboid(prim_path="/World/wall_1", name="wall_1",
                             position=np.array([3.0, 0.0, 0.5]), scale=np.array([0.2, 4.0, 1.0])))
world.scene.add(FixedCuboid(prim_path="/World/wall_2", name="wall_2",
                             position=np.array([1.0, 2.0, 0.5]), scale=np.array([4.0, 0.2, 1.0])))
world.scene.add(FixedCuboid(prim_path="/World/pillar", name="pillar",
                             position=np.array([1.4, -0.5, 0.5]), scale=np.array([0.4, 0.4, 1.0])))

world.reset()

lidar = LidarRtx(
    prim_path="/World/Lidar",
    name="lidar",
    position=np.array([0.0, 0.0, 0.5]),
    config_file_name="Debug_Rotary",
)
lidar.initialize()
lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

# 렌더 파이프라인 워밍업 (초기 프레임은 비어있음)
for _ in range(30):
    world.step(render=True)

# Debug_Rotary는 에미터 1개(azimuthDeg=[0])가 기계적으로 회전하며 360도를 스캔하는
# 방식이라, 한 바퀴를 다 채우려면 scanRateBaseHz(6Hz)에 맞춰 충분한 시뮬레이션 시간이
# 필요하다 - 프레임 수가 너무 적으면(3~5) 회전 중 일부 구간만 캡처되어 물체 일부가 누락된다
all_points = []
for _ in range(150):
    world.step(render=True)
    frame = lidar.get_current_frame()
    data = frame.get("IsaacExtractRTXSensorPointCloudNoAccumulator")
    if data is None:
        continue
    xyz = data["data"]
    if xyz is None or len(xyz) == 0:
        continue
    xyz = np.asarray(xyz).reshape(-1, 3)
    all_points.append(xyz)

points = np.concatenate(all_points, axis=0) if all_points else np.zeros((0, 3))
print("원본 누적 포인트 수:", points.shape[0])
if points.shape[0] > 0:
    print("x/y/z 범위:", points.min(axis=0), "~", points.max(axis=0))

# 프레임마다 사실상 같은 표면을 거의 겹쳐서 재방문하기 때문에(회전 중 대부분의 각도가
# 인접 프레임에서도 재관측됨) 원본은 과도하게 중복돼 있다 - 시각화/저장용으로 서브샘플링
MAX_POINTS = 20000
if points.shape[0] > MAX_POINTS:
    idx = np.random.default_rng(0).choice(points.shape[0], MAX_POINTS, replace=False)
    points = points[idx]
print("서브샘플 후 포인트 수:", points.shape[0])

np.save(f"{OUT_DIR}/lidar_points.npy", points)

fig, ax = plt.subplots(figsize=(6, 6))
ax.scatter(points[:, 0], points[:, 1], s=1, c="black", alpha=0.3)
ax.scatter([0], [0], s=40, c="red", marker="x", label="lidar")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("RTX Lidar point cloud (top-down projection)")
ax.set_aspect("equal")
ax.legend()
fig.savefig(f"{OUT_DIR}/lidar_topdown.png", dpi=150)
print("저장 완료:", f"{OUT_DIR}/lidar_topdown.png")

simulation_app.close()
