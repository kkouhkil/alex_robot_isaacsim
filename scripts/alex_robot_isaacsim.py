import argparse
from omni.isaac.lab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Creating the isaacsim robot environment")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math
import carb
import torch
import numpy as np

import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils

from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

ALEX_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/Models/Alex_TestStand_FixedHead/Alex_TestStand_FixedHead.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0, fix_root_link=True
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            'LeftShoulderPitch': 0.048357511455654374,
            'LeftShoulderRoll': 0.4872313475856689,
            'LeftShoulderYaw': -0.05178980486407529,
            'LeftElbowPitch': -1.6959062862442926,
            'LeftWristYaw': 0.48159430270696807,
            'LeftWristRoll': -1.4978222113624438,
            'LeftGripperYaw': 1.6203515292517257,
            'RightShoulderPitch': 0.04873066307647937,
            'RightShoulderRoll': -0.4826395065886908,
            'RightShoulderYaw': 0.04678311712508363,
            'RightElbowPitch': -1.6965444788383939,
            'RightWristYaw': -0.47744752865575935,
            'RightWristRoll': 1.5025787437622276,
            'RightGripperYaw': -1.6233965682966376,
        },
    ),
    actuators={
        "LeftShoulderPitch": ImplicitActuatorCfg(
            joint_names_expr=["LeftShoulderPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftShoulderRoll": ImplicitActuatorCfg(
            joint_names_expr=["LeftShoulderRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftShoulderYaw": ImplicitActuatorCfg(
            joint_names_expr=["LeftShoulderYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftElbowPitch": ImplicitActuatorCfg(
            joint_names_expr=["LeftElbowPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftWristYaw": ImplicitActuatorCfg(
            joint_names_expr=["LeftWristYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftWristRoll": ImplicitActuatorCfg(
            joint_names_expr=["LeftWristRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "LeftGripperYaw": ImplicitActuatorCfg(
            joint_names_expr=["LeftGripperYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        
        "RightShoulderPitch": ImplicitActuatorCfg(
            joint_names_expr=["RightShoulderPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightShoulderRoll": ImplicitActuatorCfg(
            joint_names_expr=["RightShoulderRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightShoulderYaw": ImplicitActuatorCfg(
            joint_names_expr=["RightShoulderYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightElbowPitch": ImplicitActuatorCfg(
            joint_names_expr=["RightElbowPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightWristYaw": ImplicitActuatorCfg(
            joint_names_expr=["RightWristYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightWristRoll": ImplicitActuatorCfg(
            joint_names_expr=["RightWristRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "RightGripperYaw": ImplicitActuatorCfg(
            joint_names_expr=["RightGripperYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
    
    },
    soft_joint_pos_limit_factor=1.0,
)
def design_scene():

    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn dome light
    cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Origin 1 with alex robot
    prim_utils.create_prim("/World/Origin", "Xform", translation=(0.0, 0.0, 0.0))

    # spawn a usd file of a table into the scene
    cfg_stand = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0))
    cfg_stand.func("/World/Origin/Stand", cfg_stand, translation=(0, 0.0, 1.03))

    # spawn a usd file of a robot into the scene
    alex_robot_cfg = ALEX_ROBOT_CFG.replace(prim_path="/World/Robot")
    alex_robot_cfg.init_state.pos = (0.0, 0.0, 1.03)
    alex = Articulation(cfg=alex_robot_cfg)
   
    # usd file visualisation 
    # cfg_robot = sim_utils.UsdFileCfg(usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/Models/...", scale=(1.0, 1.0, 1.0))
    # cfg_robot.func("/World/Origin/Robot", cfg_robot, translation=(0.0, 0.0, 1.03))

    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(0.3, 0.7, 0.3),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(),
    )
    cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.5, 0.0, 2.25))

    return alex

def main():

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([3.5, 0.0, 2.5], [-180 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180])

    # Design scene by adding assets to it
    alex_robot = design_scene()
    
    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    print(alex_robot.joint_names)

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":

    # run the main function
    main()

    # close sim app
    simulation_app.close()