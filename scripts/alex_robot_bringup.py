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

ALEX_ROBOT_NUB_HANDS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/keyhan/Documents/alex_usd_files/Alex_Nub_Hands/Alex_NubHands.usd",
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
            'SpineYaw':             0.0 * math.pi/180,
            'SpineRoll':            0.0 * math.pi/180,
            'SpinePitch':           0.0 * math.pi/180,
            'NeckYaw':              0.0 * math.pi/180,
            'NeckPitch':            0.0 * math.pi/180,
            'LeftShoulderPitch':    0.0 * math.pi/180,
            'LeftShoulderRoll':     0.0 * math.pi/180,
            'LeftShoulderYaw':      0.0 * math.pi/180,
            'LeftElbowPitch':       0.0 * math.pi/180,
            'LeftWristYaw':         0.0 * math.pi/180,
            'LeftWristRoll':        0.0 * math.pi/180,
            'LeftGripperYaw':       0.0 * math.pi/180,
            'RightShoulderPitch':   0.0 * math.pi/180,
            'RightShoulderRoll':    0.0 * math.pi/180,
            'RightShoulderYaw':     0.0 * math.pi/180,
            'RightElbowPitch':      0.0 * math.pi/180,
            'RightWristYaw':        0.0 * math.pi/180,
            'RightWristRoll':       0.0 * math.pi/180,
            'RightGripperYaw':      0.0 * math.pi/180
        },
    ),
    actuators={
        "SpineYaw": ImplicitActuatorCfg(
            joint_names_expr=["SpineYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpineRoll": ImplicitActuatorCfg(
            joint_names_expr=["SpineRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpinePitch": ImplicitActuatorCfg(
            joint_names_expr=["SpinePitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckYaw": ImplicitActuatorCfg(
            joint_names_expr=["NeckYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckPitch": ImplicitActuatorCfg(
            joint_names_expr=["NeckPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
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

ALEX_ROBOT_SAKE_HANDS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/keyhan/Documents/alex_usd_files/Alex_Sake_Hands/Alex_SakeHands.usd",
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
            'SpineYaw':             0.0 * math.pi/180,
            'SpineRoll':            0.0 * math.pi/180,
            'SpinePitch':           0.0 * math.pi/180,
            'NeckYaw':              0.0 * math.pi/180,
            'NeckPitch':            0.0 * math.pi/180,
            'LeftShoulderPitch':    0.0 * math.pi/180,
            'LeftShoulderRoll':     0.0 * math.pi/180,
            'LeftShoulderYaw':      0.0 * math.pi/180,
            'LeftElbowPitch':       0.0 * math.pi/180,
            'LeftWristYaw':         0.0 * math.pi/180,
            'LeftWristRoll':        0.0 * math.pi/180,
            'LeftGripperYaw':       0.0 * math.pi/180,
            'Left_GRIPPER_X1':      0.0 * math.pi/180,
            'Left_GRIPPER_X2':      0.0 * math.pi/180,
            'RightShoulderPitch':   0.0 * math.pi/180,
            'RightShoulderRoll':    0.0 * math.pi/180,
            'RightShoulderYaw':     0.0 * math.pi/180,
            'RightElbowPitch':      0.0 * math.pi/180,
            'RightWristYaw':        0.0 * math.pi/180,
            'RightWristRoll':       0.0 * math.pi/180,
            'RightGripperYaw':      0.0 * math.pi/180,
            'Right_GRIPPER_X1':     0.0 * math.pi/180,
            'Right_GRIPPER_X2':     0.0 * math.pi/180
        },
    ),
    actuators={
        "SpineYaw": ImplicitActuatorCfg(
            joint_names_expr=["SpineYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpineRoll": ImplicitActuatorCfg(
            joint_names_expr=["SpineRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpinePitch": ImplicitActuatorCfg(
            joint_names_expr=["SpinePitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckYaw": ImplicitActuatorCfg(
            joint_names_expr=["NeckYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckPitch": ImplicitActuatorCfg(
            joint_names_expr=["NeckPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
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
        "Left_GRIPPER_X1": ImplicitActuatorCfg(
            joint_names_expr=["Left_GRIPPER_X1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_GRIPPER_X2": ImplicitActuatorCfg(
            joint_names_expr=["Left_GRIPPER_X2"],
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
        "Right_GRIPPER_X1": ImplicitActuatorCfg(
            joint_names_expr=["Right_GRIPPER_X1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_GRIPPER_X2": ImplicitActuatorCfg(
            joint_names_expr=["Right_GRIPPER_X2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

ALEX_ROBOT_PSYONIC_HANDS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/keyhan/Documents/alex_usd_files/Alex_Psyonic_Hands/Alex_PsyonicHands.usd",
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
            'SpineYaw':             0.0 * math.pi/180,
            'SpineRoll':            0.0 * math.pi/180,
            'SpinePitch':           0.0 * math.pi/180,
            'NeckYaw':              0.0 * math.pi/180,
            'NeckPitch':            0.0 * math.pi/180,
            'LeftShoulderPitch':    0.0 * math.pi/180,
            'LeftShoulderRoll':     0.0 * math.pi/180,
            'LeftShoulderYaw':      0.0 * math.pi/180,
            'LeftElbowPitch':       0.0 * math.pi/180,
            'LeftWristYaw':         0.0 * math.pi/180,
            'LeftWristRoll':        0.0 * math.pi/180,
            'LeftGripperYaw':       0.0 * math.pi/180,
            'Left_index_q1':        0.0 * math.pi/180,
            'Left_index_q2':        0.0 * math.pi/180,
            'Left_middle_q1':       0.0 * math.pi/180,
            'Left_middle_q2':       0.0 * math.pi/180,
            'Left_ring_q1':         0.0 * math.pi/180,
            'Left_ring_q2':         0.0 * math.pi/180,
            'Left_pinky_q1':        0.0 * math.pi/180,
            'Left_pinky_q2':        0.0 * math.pi/180,
            'Left_thumb_q1':        0.0 * math.pi/180,
            'Left_thumb_q2':        0.0 * math.pi/180,
            'RightShoulderPitch':   0.0 * math.pi/180,
            'RightShoulderRoll':    0.0 * math.pi/180,
            'RightShoulderYaw':     0.0 * math.pi/180,
            'RightElbowPitch':      0.0 * math.pi/180,
            'RightWristYaw':        0.0 * math.pi/180,
            'RightWristRoll':       0.0 * math.pi/180,
            'RightGripperYaw':      0.0 * math.pi/180,
            'Right_index_q1':       0.0 * math.pi/180,
            'Right_index_q2':       0.0 * math.pi/180,
            'Right_middle_q1':      0.0 * math.pi/180,
            'Right_middle_q2':      0.0 * math.pi/180,
            'Right_ring_q1':        0.0 * math.pi/180,
            'Right_ring_q2':        0.0 * math.pi/180,
            'Right_pinky_q1':       0.0 * math.pi/180,
            'Right_pinky_q2':       0.0 * math.pi/180,
            'Right_thumb_q1':       0.0 * math.pi/180,
            'Right_thumb_q2':       0.0 * math.pi/180
        },
    ),
    actuators={
        "SpineYaw": ImplicitActuatorCfg(
            joint_names_expr=["SpineYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpineRoll": ImplicitActuatorCfg(
            joint_names_expr=["SpineRoll"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "SpinePitch": ImplicitActuatorCfg(
            joint_names_expr=["SpinePitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckYaw": ImplicitActuatorCfg(
            joint_names_expr=["NeckYaw"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "NeckPitch": ImplicitActuatorCfg(
            joint_names_expr=["NeckPitch"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
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
        "Left_index_q1": ImplicitActuatorCfg(
            joint_names_expr=["Left_index_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_index_q2": ImplicitActuatorCfg(
            joint_names_expr=["Left_index_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_middle_q1": ImplicitActuatorCfg(
            joint_names_expr=["Left_middle_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_middle_q2": ImplicitActuatorCfg(
            joint_names_expr=["Left_middle_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_ring_q1": ImplicitActuatorCfg(
            joint_names_expr=["Left_ring_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_ring_q2": ImplicitActuatorCfg(
            joint_names_expr=["Left_ring_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_pinky_q1": ImplicitActuatorCfg(
            joint_names_expr=["Left_pinky_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_pinky_q2": ImplicitActuatorCfg(
            joint_names_expr=["Left_pinky_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_thumb_q1": ImplicitActuatorCfg(
            joint_names_expr=["Left_thumb_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Left_thumb_q2": ImplicitActuatorCfg(
            joint_names_expr=["Left_thumb_q2"],
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
        "Right_index_q1": ImplicitActuatorCfg(
            joint_names_expr=["Right_index_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_index_q2": ImplicitActuatorCfg(
            joint_names_expr=["Right_index_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_middle_q1": ImplicitActuatorCfg(
            joint_names_expr=["Right_middle_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_middle_q2": ImplicitActuatorCfg(
            joint_names_expr=["Right_middle_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_ring_q1": ImplicitActuatorCfg(
            joint_names_expr=["Right_ring_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_ring_q2": ImplicitActuatorCfg(
            joint_names_expr=["Right_ring_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_pinky_q1": ImplicitActuatorCfg(
            joint_names_expr=["Right_pinky_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_pinky_q2": ImplicitActuatorCfg(
            joint_names_expr=["Right_pinky_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_thumb_q1": ImplicitActuatorCfg(
            joint_names_expr=["Right_thumb_q1"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "Right_thumb_q2": ImplicitActuatorCfg(
            joint_names_expr=["Right_thumb_q2"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

def scene_design():

    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn dome light
    cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Origin 1 with alex robot
    prim_utils.create_prim("/World/Origin", "Xform", translation=(0.0, 0.0, 0.0))

    # -- Stand
    cfg = sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/ThorlabsTable/table_instanceable.usd", scale=(2.0, 2.0, 2.0)
    )
    cfg.func(f"/World/Origin/Stand", cfg, translation=(0.0, 0.0, 1.59))

    # spawn a usd file of a robot into the scene
    # alex_robot_cfg = ALEX_ROBOT_NUB_HANDS_CFG.replace(prim_path="/World/Origin/Robot")
    # alex_robot_cfg = ALEX_ROBOT_SAKE_HANDS_CFG.replace(prim_path="/World/Origin/Robot")
    alex_robot_cfg = ALEX_ROBOT_PSYONIC_HANDS_CFG.replace(prim_path="/World/Origin/Robot")
    alex_robot_cfg.init_state.pos = (0.0, 0.0, 1.59)
    alex = Articulation(cfg=alex_robot_cfg)

    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(0.25, 0.25, 0.25),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(),
    )
    cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.5, 0.0, 2.25))

def main():

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([3.5, 0.0, 2.5], [-180 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180])

    # Design scene by adding assets to it
    scene_design()

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
