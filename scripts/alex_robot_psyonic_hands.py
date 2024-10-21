import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

import math

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

"""Configuration of Alex Robot with Psyonic Hands"""

"""

num_of_joints = 62

(0, b'SpineYaw', 0, 7, 6, 1, 0.05, 0.0, -1.0472, 1.0472, 150.0, 9.0, b'SpineYawLink', (0.0, 0.0, 1.0), (0.010295, 0.001599, 0.048041999999999994), (0.0, 0.0, 0.0, 1.0), -1)
(1, b'SpineRoll', 0, 8, 7, 1, 0.05, 0.0, -0.436332, 0.436332, 150.0, 6.0, b'SpineRollLink', (1.0, 0.0, 0.0), (-0.02285, -0.001802, 0.022146), (0.0, 0.0, 0.0, 1.0), 0)
(2, b'SpinePitch', 0, 9, 8, 1, 0.05, 0.0, -0.523599, 0.785398, 250.0, 6.0, b'Torso', (0.0, 1.0, 0.0), (-0.000674, -0.000626, 0.016004000000000004), (0.0, 0.0, 0.0, 1.0), 1)
(3, b'TorsoLeftIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'TorsoLeftIMULink', (0.0, 0.0, 0.0), (0.07611195, 0.10926393999999999, 0.015948440000000008), (0.6408563820557885, 0.2988362387301197, 0.6408563820557885, -0.29883623873012016), 2)
(4, b'TorsoRightIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'TorsoRightIMULink', (0.0, 0.0, 0.0), (0.07611195, -0.09061580000000001, 0.0001974300000000262), (-0.2988362387301197, -0.6408563820557884, -0.2988362387301197, 0.6408563820557887), 2)
(5, b'NeckYaw', 0, 10, 9, 1, 0.05, 0.0, -1.5708, 1.5708, 20.0, 20.0, b'NeckYawLink', (0.0, 0.0, 1.0), (-0.003673, -5.9e-05, 0.147664), (0.0, 0.0, 0.0, 1.0), 2)
(6, b'NeckPitch', 0, 11, 10, 1, 0.05, 0.0, -1.0472, 1.0472, 20.0, 20.0, b'Head', (0.0, 1.0, 0.0), (0.017857, 0.003057, 0.09742899999999999), (0.0, 0.0, 0.0, 1.0), 5)
(7, b'TorsoIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'TorsoIMULink', (0.0, 0.0, 0.0), (0.0003242499999999999, -0.01230921, 0.08483139), (0.0, 0.0, 0.7071067811865475, 0.7071067811865476), 2)
(8, b'LeftShoulderPitch', 0, 12, 11, 1, 0.05, 0.0, -3.141592, 1.22173, 150.0, 9.0, b'LeftShoulderPitchLink', (0.0, 1.0, 0.0), (-0.003673, 0.127931, 0.075405), (-0.3420202839047467, 0.0, 0.0, 0.9396925696192966), 2)
(9, b'LeftShoulderRoll', 0, 13, 12, 1, 0.05, 0.0, -0.349066, 2.79253, 150.0, 9.0, b'LeftShoulderRollLink', (1.0, 0.0, 0.0), (0.00264, 0.03541000000000001, -0.006175999999999999), (0.3420202839047467, 0.0, 0.0, 0.9396925696192966), 8)
(10, b'LeftShoulderYaw', 0, 14, 13, 1, 0.05, 0.0, -1.91986, 1.22173, 80.0, 11.5, b'LeftShoulderYawLink', (0.0, 0.0, 1.0), (-0.01251711, 0.016169100000000002, 0.03849849), (0.0, 0.0, 0.0, 1.0), 9)
(11, b'LeftElbowPitch', 0, 15, 14, 1, 0.05, 0.0, -2.35619, 0.174532925, 80.0, 11.5, b'LeftElbowPitchLink', (0.0, 1.0, 0.0), (0.00989597, -0.00314332, -0.09574586999999998), (0.0, 0.0, 0.0, 1.0), 10)
(12, b'LeftWristYaw', 0, 16, 15, 1, 0.05, 0.0, -2.61799, 2.61799, 20.0, 25.0, b'LeftWristYawLink', (0.0, 0.0, 1.0), (-0.0031967799999999998, 0.0035763, 0.04946747), (0.0, 0.0, 0.0, 1.0), 11)
(13, b'LeftWristRoll', 0, 17, 16, 1, 0.05, 0.0, -1.8326, 0.610865, 20.0, 25.0, b'LeftWristRollLink', (1.0, 0.0, 0.0), (0.00263832, -0.01249466, -0.11981623999999999), (0.0, 0.0, 0.0, 1.0), 12)
(14, b'LeftGripperYaw', 0, 18, 17, 1, 0.05, 0.0, -2.61799, 2.61799, 20.0, 25.0, b'LeftGripperYawLink', (0.0, 0.0, 1.0), (-4.053e-05, 0.00261605, -0.03135532), (0.0, 0.0, 0.0, 1.0), 13)
(15, b'leftGrippertAttachmentFixedJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'LeftPsyonicAbilityBaseLink', (0.0, 0.0, 0.0), (0.000217, 0.000349, -0.035361000000000004), (0.9999999999999466, 0.0, 0.0, -3.2679489653813835e-07), 14)
(16, b'Left_wrist2thumb', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_thumb_base', (0.0, 0.0, 0.0), (0.0016285564999999995, 0.00393922946, 0.03234284789), (0.0006102695033793133, 0.9990212031492024, 2.701918460132315e-05, -0.04422965633392243), 15)
(17, b'Left_index_q1', 0, 19, 18, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Left_index_L1', (0.0, 0.0, 1.0), (-0.03190911, -0.01288201799, -0.06293680134), (0.052185779141515704, -0.7441290289055104, 0.12811340794472745, 0.6535561089151588), 16)
(18, b'Left_index_q2', 0, 20, 19, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Left_index_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 17)
(19, b'Left_idx_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_index_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 18)
(20, b'Left_middle_q1', 0, 21, 20, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Left_middle_L1', (0.0, 0.0, 1.0), (-0.012765918999999999, -0.01515228899, -0.06784075033999999), (0.07563666688532018, -0.7289207710104808, 0.10065238482505288, 0.6729210218450563), 16)
(21, b'Left_middle_q2', 0, 22, 21, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Left_middle_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 20)
(22, b'Left_mid_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_middle_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 21)
(23, b'Left_ring_q1', 0, 23, 22, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Left_ring_L1', (0.0, 0.0, 1.0), (0.007535150000000001, -0.01405450999, -0.06727290634), (0.10152710824637613, -0.7124733082016416, 0.0729479506121178, 0.6904727568071237), 16)
(24, b'Left_ring_q2', 0, 24, 23, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Left_ring_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 23)
(25, b'Left_rng_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_ring_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 24)
(26, b'Left_pinky_q1', 0, 25, 24, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Left_pinky_L1', (0.0, 0.0, 1.0), (0.027102183000000002, -0.01084660099, -0.06301586633999999), (0.12421444596850766, -0.7129681457596536, 0.049364739840190255, 0.6883388097479964), 16)
(27, b'Left_pinky_q2', 0, 26, 25, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Left_pinky_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 26)
(28, b'Left_pnky_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_pinky_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 27)
(29, b'Left_thumb_q1', 0, 27, 26, 1, 0.001, 0.001, -2.0943951, 0.0, 1.2, 40.338888721, b'Left_thumb_L1', (0.0, 0.0, 1.0), (-0.02241911, 0.00015798201, 1.319866e-05), (0.0, 0.0, 0.9955455375767813, 0.09428193151902145), 16)
(30, b'Left_thumb_q2', 0, 28, 27, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Left_thumb_L2', (0.0, 0.0, 1.0), (0.0132223501, 0.001890854, -0.0116351644), (0.7933533402912348, 0.0, 0.0, 0.608761429008721), 29)
(31, b'Left_thmb_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Left_thumb_anchor', (0.0, 0.0, 0.0), (0.021980703900000002, 0.0224139371, 0.0032655998220000003), (0.0, 0.0, 0.0, 1.0), 30)
(32, b'LeftWristYawIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'LeftWristYawIMULink', (0.0, 0.0, 0.0), (-0.04634768, 0.01216334, -0.01929223999999999), (0.49999998962930636, 0.5001018366018476, 0.49999998962930636, -0.4998981633981524), 12)
(33, b'LeftShoulderYawIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'LeftShoulderYawIMULink', (0.0, 0.0, 0.0), (-0.01144126, -0.009838369999999999, 0.01813945), (0.11061581516564918, 0.6984012491631778, -0.6984010268571269, 0.11061574158980478), 10)
(34, b'LeftShoulderPitchIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'LeftShoulderPitchIMULink', (0.0, 0.0, 0.0), (0.016966210000000002, -0.045842900000000006, 0.004366170000000001), (0.707106781188241, -0.7071067811847785, 2.3107888739870806e-07, 2.3107888739983953e-07), 8)
(35, b'RightShoulderPitch', 0, 29, 28, 1, 0.05, 0.0, -3.141592, 1.22173, 1000.0, 100.0, b'RightShoulderPitchLink', (0.0, 1.0, 0.0), (-0.003673, -0.128049, 0.075405), (0.3420202839047467, 0.0, 0.0, 0.9396925696192966), 2)
(36, b'RightShoulderRoll', 0, 30, 29, 1, 0.05, 0.0, -2.79253, 0.349066, 1000.0, 100.0, b'RightShoulderRollLink', (1.0, 0.0, 0.0), (0.00264, -0.03541000000000001, -0.006175999999999999), (-0.3420202839047467, 0.0, 0.0, 0.9396925696192966), 35)
(37, b'RightShoulderYaw', 0, 31, 30, 1, 0.05, 0.0, -1.22173, 1.91986, 1000.0, 100.0, b'RightShoulderYawLink', (0.0, 0.0, 1.0), (-0.012489, -0.016180000000000003, 0.0385), (0.0, 0.0, 0.0, 1.0), 36)
(38, b'RightElbowPitch', 0, 32, 31, 1, 0.05, 0.0, -2.35619, 0.174532925, 1000.0, 100.0, b'RightElbowPitchLink', (0.0, 1.0, 0.0), (0.00976, 0.00274, -0.09524999999999997), (0.0, 0.0, 0.0, 1.0), 37)
(39, b'RightWristYaw', 0, 33, 32, 1, 0.05, 0.0, -2.61799, 2.61799, 20.0, 25.0, b'RightWristYawLink', (0.0, 0.0, 1.0), (-0.0023, -0.006913, 0.048421), (0.0, 0.0, 0.0, 1.0), 38)
(40, b'RightWristRoll', 0, 34, 33, 1, 0.05, 0.0, -0.610865, 1.8326, 20.0, 25.0, b'RightWristRollLink', (1.0, 0.0, 0.0), (0.002616, 0.012537999999999999, -0.11981), (0.0, 0.0, 0.0, 1.0), 39)
(41, b'RightGripperYaw', 0, 35, 34, 1, 0.05, 0.0, -2.61799, 2.61799, 20.0, 25.0, b'RightGripperYawLink', (0.0, 0.0, 1.0), (4.81e-05, -0.0008570000000000001, -0.031196), (0.0, 0.0, 0.0, 1.0), 40)
(42, b'rightGrippertAttachmentFixedJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'RightPsyonicAbilityBaseLink', (0.0, 0.0, 0.0), (0.000217, 0.000349, -0.035361000000000004), (3.267948965381209e-07, 0.9999999999998932, -3.267948965381209e-07, 1.0679490440337254e-13), 41)
(43, b'Right_wrist2thumb', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_thumb_base', (0.0, 0.0, 0.0), (-0.0464667765, 0.00393922946, 0.03234284789), (-0.0006102695033793133, 0.9990212031492024, 2.701918460132315e-05, 0.04422965633392243), 42)
(44, b'Right_index_q1', 0, 36, 35, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Right_index_L1', (0.0, 0.0, 1.0), (-0.012929109999999999, -0.01288201799, -0.06293680134), (0.1281134079447275, -0.6535561089151587, 0.05218577914151566, 0.7441290289055105), 43)
(45, b'Right_index_q2', 0, 37, 36, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Right_index_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 44)
(46, b'Right_idx_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_index_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 45)
(47, b'Right_middle_q1', 0, 38, 37, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Right_middle_L1', (0.0, 0.0, 1.0), (-0.032072301, -0.01515228899, -0.06784075033999999), (0.10065238482505291, -0.6729210218450563, 0.07563666688532024, 0.7289207710104808), 43)
(48, b'Right_middle_q2', 0, 39, 38, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Right_middle_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 47)
(49, b'Right_mid_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_middle_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 48)
(50, b'Right_ring_q1', 0, 40, 39, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Right_ring_L1', (0.0, 0.0, 1.0), (-0.05237337, -0.01405450999, -0.06727290634), (0.07294795061211781, -0.6904727568071238, 0.10152710824637616, 0.7124733082016415), 43)
(51, b'Right_ring_q2', 0, 41, 40, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Right_ring_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 50)
(52, b'Right_rng_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_ring_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 51)
(53, b'Right_pinky_q1', 0, 42, 41, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Right_pinky_L1', (0.0, 0.0, 1.0), (-0.071940403, -0.01084660099, -0.06301586633999999), (0.049364739840190415, -0.6883388097479965, 0.12421444596850753, 0.7129681457596535), 43)
(54, b'Right_pinky_q2', 0, 43, 42, 1, 0.001, 0.001, 0.0, 2.6586, 6.0, 8.0677777442, b'Right_pinky_L2', (0.0, 0.0, 1.0), (0.016053613, 0.00341567701, 1.319866e-05), (0.0, 0.0, -0.04222444290435509, 0.9991081505129547), 53)
(55, b'Right_pnky_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_pinky_anchor', (0.0, 0.0, 0.0), (0.01818592, -0.0016860799999999995, 0.00016885999999999998), (0.0, 0.0, 0.0, 1.0), 54)
(56, b'Right_thumb_q1', 0, 44, 43, 1, 0.001, 0.001, -2.0943951, 0.0, 1.2, 40.338888721, b'Right_thumb_L1', (0.0, 0.0, 1.0), (-0.02241911, 0.00015798201, 1.319866e-05), (0.9955455375767813, -0.09428193151902146, -5.773103282609979e-18, -6.095958279994181e-17), 43)
(57, b'Right_thumb_q2', 0, 45, 44, 1, 0.001, 0.001, 0.0, 2.0943951, 6.0, 8.0677777442, b'Right_thumb_L2', (0.0, 0.0, 1.0), (0.0132223501, 0.0018908540000000002, 0.017866235600000002), (-0.7933533402912349, -2.128773524066493e-18, -1.633465376311301e-18, 0.6087614290087211), 56)
(58, b'Right_thmb_anchor', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'Right_thumb_anchor', (0.0, 0.0, 0.0), (0.021980705240000002, 0.02241393333, 0.01113527138), (0.0, 0.0, 0.0, 1.0), 57)
(59, b'RightWristYawIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'RightWristYawIMULink', (0.0, 0.0, 0.0), (-0.04637, 0.011196, -0.019285999999999998), (0.49999998962930636, 0.5001018366018476, 0.49999998962930636, -0.4998981633981524), 39)
(60, b'RightShoulderYawIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'RightShoulderYawIMULink', (0.0, 0.0, 0.0), (-0.004048319999999999, 0.011881339999999999, 0.01863532000000001), (0.6984010238931885, 0.11061576030339419, -0.11061579645206604, 0.6984012521271159), 37)
(61, b'RightShoulderPitchIMUJoint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'RightShoulderPitchIMULink', (0.0, 0.0, 0.0), (0.016966210000000002, 0.021337010000000003, 0.004366170000000001), (0.707106781188241, -0.7071067811847785, -2.3107888739870806e-07, -2.3107888739983953e-07), 35)
"""