from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.replicator.core as rep
from pxr import Usd
from isaacsim.core.api import World
from isaacsim.core.utils.semantics import add_labels
from isaacsim.core.utils.stage import get_current_stage

STAGE_PATH = "/home/pw/Documents/isaacsim/practicum/m04_manipulation/franka_scene.usd"
OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m06_replicator/out_franka_dr"
NUM_FRAMES = 300

omni.usd.get_context().open_stage(STAGE_PATH)
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

stage = get_current_stage()

# Module 5에서 확인한 함정: Franka 링크 메시는 instanceable이라 조상 prim의 라벨이
# 안 먹는다 - 라벨을 붙이기 전에 instanceable을 꺼야 한다.
franka_prim = stage.GetPrimAtPath("/World/franka")
for prim in Usd.PrimRange(franka_prim):
    if prim.IsInstanceable():
        prim.SetInstanceable(False)

add_labels(franka_prim, ["franka"], instance_name="class")
add_labels(stage.GetPrimAtPath("/World/Cube"), ["cube"], instance_name="class")

with rep.new_layer():
    dome_light = rep.create.light(light_type="Dome", intensity=1000.0, temperature=6500.0)

    # 큐브 시작 위치(0.5, 0, 0.025) 주변, 로봇이 닿는 범위 안에서 위치/색을 랜덤화
    cube = rep.get.prims(path_pattern="/World/Cube")

    # 테이블 위를 살짝 다른 각도/거리에서 내려다보는 카메라 - 매 프레임 랜덤 시점
    camera = rep.create.camera()
    render_product = rep.create.render_product(camera, (640, 480))

    with rep.trigger.on_frame(num_frames=NUM_FRAMES, rt_subframes=4):
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((0.9, -0.9, 0.5), (1.6, 0.9, 1.4)),
                look_at=(0.5, 0.0, 0.05),
            )
        with cube:
            rep.modify.pose(
                position=rep.distribution.uniform((0.35, -0.2, 0.025), (0.65, 0.2, 0.025)),
            )
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        with dome_light:
            rep.modify.attribute("inputs:intensity", rep.distribution.uniform(300.0, 3000.0), attribute_type="float")
            rep.modify.attribute(
                "inputs:colorTemperature", rep.distribution.uniform(3000.0, 9000.0), attribute_type="float"
            )

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=OUT_DIR,
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=True,
    )
    writer.attach([render_product])

rep.orchestrator.run_until_complete()

print("완료:", OUT_DIR)
simulation_app.close()
