import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="using the interactive scene interface.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import math

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

##
# Pre-defined configs
##

from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg

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

            # 'LeftShoulderPitch': 0 * math.pi/180,
            # 'LeftShoulderRoll': 0 * math.pi/180,
            # 'LeftShoulderYaw': 0 * math.pi/180,
            # 'LeftElbowPitch': 0 * math.pi/180,
            # 'LeftWristYaw': 0 * math.pi/180,
            # 'LeftWristRoll': 0 * math.pi/180,
            # 'LeftGripperYaw': 0 * math.pi/180,
            # 'RightShoulderPitch': 0 * math.pi/180,
            # 'RightShoulderRoll': 0 * math.pi/180,
            # 'RightShoulderYaw': 0 * math.pi/180,
            # 'RightElbowPitch': 0 * math.pi/180,
            # 'RightWristYaw': 0 * math.pi/180,
            # 'RightWristRoll': 0 * math.pi/180,
            # 'RightGripperYaw': 0 * math.pi/180,
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


@configclass
class AlexRobotSceneCfg(InteractiveSceneCfg):
    """Configuration for alex robot scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    alex: ArticulationCfg = ALEX_ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):

    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["alex"]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # Simulation loop
    while simulation_app.is_running():

        # Reset
        if count % 500 == 0:

            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            # clear internal buffers

            scene.reset()
            print(f"[INFO]: Resetting robot state...")

        # Perform step
        sim.step()

        # Increment counter
        count += 1

        # Update buffers
        scene.update(sim_dt)


def main():

    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device="cpu")
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_cfg = AlexRobotSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    # Now Setup is complete!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":

    # run the main function
    main()

    # close sim app
    simulation_app.close()