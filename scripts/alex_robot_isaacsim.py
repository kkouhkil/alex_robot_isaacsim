import argparse
import math

from omni.isaac.lab.app import AppLauncher
import carb
import torch
import numpy as np

# create argparser
parser = argparse.ArgumentParser(description="Creating the isaacsim robot environment")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

def design_scene():

    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    # cfg_light_distant = sim_utils.DistantLightCfg(
    #     intensity=5000.0,
    #     color=(0.75, 0.75, 0.75),
    # )

    # cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(0, 0, 10))

    # spawn dome light
    cfg_light_dome = sim_utils.DomeLightCfg(
        intensity=1500.0,
        color=(1.0, 1.0, 1.0),
    )

    cfg_light_dome.func("/World/lightDome", cfg_light_dome, translation=(0, 0, 10))


    # Origin 1 with alex robot
    prim_utils.create_prim("/World/Origin", "Xform", translation=(0.0, 0.0, 0.0))

    # spawn a usd file of a table into the scene
    cfg_stand = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0))
    cfg_stand.func("/World/Origin/Table", cfg_stand, translation=(0, 0.0, 1.03))

    # spawn a usd file of a robot into the scene
    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/ANYbotics/anymal_d.usd", scale=(1.0, 1.0, 1.0))
    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/KINOVA/kinova_gen3_7_dof.usd", scale=(1.0, 1.0, 1.0))

    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/MODELS/b1.usd", scale=(1.0, 1.0, 1.0))

    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/ALEX/alex_fixed_base_gripper_hands_no_collision_original_urdf.usd", scale=(1.0, 1.0, 1.0))

    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/ALEX/alex_fixed_base_gripper_hands_with_collision_modified_urdf.usd", scale=(1.0, 1.0, 1.0))
    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/ALEX/alex_fixed_base_psyonic_hands_with_collision_modified_urdf.usd", scale=(1.0, 1.0, 1.0))
    cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/ALEX/alex_fixed_base_nub_hands_with_collision_modified_urdf.usd", scale=(1.0, 1.0, 1.0))

    cfg_robot.func("/World/Origin/Robot", cfg_robot, translation=(0.0, 0.0, 1.03))

    # alex_robot_cfg = ALEX_ROBOT_CFG.replace(prim_path="/World/Origin/Robot")
    # alex_robot = Articulation(cfg=alex_robot_cfg)

    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(0.2, 0.5, 0.2),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(),
    )
    # cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.0, 0.0, 2.25))

def main():

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([3.5, 0.0, 3.5], [-30 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180])

    # Design scene by adding assets to it
    design_scene()

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":

    # run the main function
    main()

    # close sim app
    simulation_app.close()