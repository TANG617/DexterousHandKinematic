"""
Example script demo'ing robot control.

Options for random actions, as well as selection of robot action space
"""

import torch as th

import omnigibson as og
import omnigibson.lazy as lazy
from omnigibson.macros import gm
from omnigibson.robots import REGISTERED_ROBOTS
from omnigibson.utils.ui_utils import KeyboardRobotController, choose_from_options

gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True



def main(random_selection=False, headless=False, short_exec=False, quickstart=False):
    """
    Robot control demo with selection
    Queries the user to select a robot, the controllers, a scene and a type of input (random actions or teleop)
    """
    og.log.info(f"Demo {__file__}\n    " + "*" * 80 + "\n    Description:\n" + main.__doc__ + "*" * 80)
    # og.log.setLevel(og.log.INFO)  # 添加在文件开头
    robot_name = "A1"

    scene_cfg = dict()
    scene_cfg["type"] = "Scene"

    robot0_cfg = dict()
    robot0_cfg["type"] = robot_name
    robot0_cfg["obs_modalities"] = ["rgb"]
    robot0_cfg["action_type"] = "continuous"
    robot0_cfg["action_normalize"] = True

    dex_hand_cfg = dict(
        type = "USDObject",
        name= "dex_hand",
        visual_only=False,
        usd_path= f"/home/timli/Documents/DexterousHandKinematic/DexHand_V1.usd",
        scale= 2.0,
        position = [0.59, -0.1, 0.97],
        orientation =  [0, 0, 0, 1],
        fixed_base = True,
        load_articulation = True,  # 确保加载关节信息
    )

    box_cfg = dict(
        type="PrimitiveObject",
        name="box",
        primitive_type="Cube",
        rgba=[1.0, 0, 0, 1.0],
        size=0.05,
        position=[0.53, -0.1, 0.97],
    )
    cfg = dict(scene=scene_cfg,  objects=[dex_hand_cfg])
    # cfg = dict(scene=scene_cfg, robots=[robot0_cfg], objects=[dex_hand_cfg])

    env = og.Environment(configs=cfg)

    # robot = env.robots[0]
    # controller_choices = {
    #     "base": "DifferentialDriveController",
    #     "arm_0": "InverseKinematicsController",
    #     "gripper_0": "MultiFingerGripperController",
    #     "camera": "JointController",
    # }

    # controller_config = {component: {"name": name} for component, name in controller_choices.items()}
    # robot.reload_controllers(controller_config=controller_config)

    env.scene.update_initial_state()

    og.sim.viewer_camera.set_position_orientation(
        position=th.tensor([1.46949, -3.97358, 2.21529]),
        orientation=th.tensor([0.56829048, 0.09569975, 0.13571846, 0.80589577]),
    )

    env.reset()
    # robot.reset()

    # action_generator = KeyboardRobotController(robot=robot)

    print("Running demo.")
    print("Press ESC to quit")

    max_steps = 10000
    step = 0

    while step != max_steps:
        action = (
            # action_generator.get_random_action()
        )
        env.step(action=action)
        step += 1
        print(step)

    og.clear()


if __name__ == "__main__":
    main()