import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Alex Robot")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch
import math

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

ALEX_ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/keyhan/Documents/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/Models/Alex_TestStand_FixedHead_nubHands/Alex_TestStand_FixedHead_nubHands.usd",
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

def define_origins(num_origins: int, spacing: float) -> list[list[float]]:
    """Defines the origins of the the scene."""
    # create tensor based on number of environments
    env_origins = torch.zeros(num_origins, 3)

    # create a grid of origins
    num_rows = np.floor(np.sqrt(num_origins))
    num_cols = np.ceil(num_origins / num_rows)

    xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols), indexing="xy")

    env_origins[:, 0] = spacing * xx.flatten()[:num_origins] - spacing * (num_rows - 1) / 2
    env_origins[:, 1] = spacing * yy.flatten()[:num_origins] - spacing * (num_cols - 1) / 2
    env_origins[:, 2] = 0.0

    # return the origins
    return env_origins.tolist()

def design_scene(num_of_origins) -> tuple[dict, list[list[float]]]:

    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Each group will have a mount and a robot on top of it
    # num_of_origins = 1
    origins = define_origins(num_origins = num_of_origins, spacing=2.0)
    
    for i in range(0, num_of_origins):
        # Origin with Alex
        prim_utils.create_prim(f"/World/Origin_{i}", "Xform", translation=origins[i])

        # -- Stand
        cfg = sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0)
        )
        cfg.func(f"/World/Origin_{i}/Stand_{i}", cfg, translation=(0.0, 0.0, 1.03))

    alex_robot_cfg_dic = {}
    alex_dict = {}

    for i in range(0, num_of_origins):
        alex_robot_cfg_dic[f'alex_robot_cfg_{i}'] = ALEX_ROBOT_CFG.replace(prim_path=f"/World/Origin_{i}/Robot_{i}")
        alex_robot_cfg_dic[f'alex_robot_cfg_{i}'].init_state.pos = (0, 0, 1.03)
        alex_dict[f'alex_{i}'] = Articulation(cfg=alex_robot_cfg_dic[f'alex_robot_cfg_{i}'])

    # return the scene information
    scene_entities_dic = {}
    for i in range(0, num_of_origins):
        scene_entities_dic[f"alex_{i}"] = alex_dict[f"alex_{i}"]    

    return scene_entities_dic, origins

def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origins: torch.Tensor):

    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 200 == 0:

            # reset counters
            sim_time = 0.0
            count = 0

            # reset the scene entities
            for index, robot in enumerate(entities.values()):

                # root state
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                robot.write_root_state_to_sim(root_state)

                # set joint positions
                joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
                robot.write_joint_state_to_sim(joint_pos, joint_vel)

                # clear internal buffers
                robot.reset()

            print("[INFO]: Resetting robot state...")

        # apply random actions to the robots
        for robot in entities.values():

            # generate random joint positions
            joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
            joint_pos_target = joint_pos_target.clamp_(
                robot.data.soft_joint_pos_limits[..., 0], robot.data.soft_joint_pos_limits[..., 1]
            )

            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)

            # write data to sim
            robot.write_data_to_sim()

        # perform step
        sim.step()

        # update sim-time
        sim_time += sim_dt
        count += 1

        # update buffers
        for robot in entities.values():
            robot.update(sim_dt)


def main():

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg()
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([3.5, 0.0, 2.5], [-180 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180])

    # number of loaded robots
    num_of_origins = 16

    # design scene
    scene_entities, scene_origins = design_scene(num_of_origins)
    scene_origins = torch.tensor(scene_origins, device=sim.device)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)

if __name__ == "__main__":

    # run the main function
    main()

    # close sim app
    simulation_app.close()