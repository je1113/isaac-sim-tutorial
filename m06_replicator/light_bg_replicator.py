from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World

OUT_DIR = "/home/pw/Documents/isaacsim/practicum/m06_replicator/out_light_bg"

omni.usd.get_context().new_stage()
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

with rep.new_layer():
    dome_light = rep.create.light(light_type="Dome", intensity=1000.0, temperature=6500.0)
    camera = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
    cube = rep.create.cube(semantics=[("class", "cube")])

    ground_plane = rep.get.prims(path_pattern="/World/defaultGroundPlane")

    with rep.trigger.on_frame(num_frames=30, rt_subframes=4):
        with cube:
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
            )
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

        with dome_light:
            rep.modify.attribute("inputs:intensity", rep.distribution.uniform(300.0, 3000.0), attribute_type="float")
            rep.modify.attribute(
                "inputs:colorTemperature", rep.distribution.uniform(3000.0, 9000.0), attribute_type="float"
            )

        with ground_plane:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=OUT_DIR,
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=True,
    )
    render_product = rep.create.render_product(camera, (640, 480))
    writer.attach([render_product])

rep.orchestrator.run_until_complete()

print("완료:", OUT_DIR)
simulation_app.close()
