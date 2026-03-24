#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, Bool, Int32
from geometry_msgs.msg import WrenchStamped
from ur_msgs.msg import ToolDataMsg
import csv
import time


LEFT_ARM_JOINTS = [
    'left_ur_arm_shoulder_pan_joint',
    'left_ur_arm_shoulder_lift_joint',
    'left_ur_arm_elbow_joint',
    'left_ur_arm_wrist_1_joint',
    'left_ur_arm_wrist_2_joint',
    'left_ur_arm_wrist_3_joint',
]

RIGHT_ARM_JOINTS = [
    'right_ur_arm_shoulder_pan_joint',
    'right_ur_arm_shoulder_lift_joint',
    'right_ur_arm_elbow_joint',
    'right_ur_arm_wrist_1_joint',
    'right_ur_arm_wrist_2_joint',
    'right_ur_arm_wrist_3_joint',
]


class ArmLogger:
    """Handles logging for a single arm."""

    def __init__(self, arm_name, joint_names, timestamp_str):
        self.arm_name = arm_name
        self.joint_names = joint_names
        self.n_joints = len(joint_names)

        filename = f"energy_log_{arm_name}_{timestamp_str}.csv"
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        # Velocity differentiation state
        self.prev_pos = None
        self.prev_time = None

        # Tool data (from ur_hardware_interface/tool_data)
        self.tool_current = 0.0
        self.tool_output_voltage = 0.0

        # Speed scaling (0.0 – 1.0)
        self.speed_scaling = 1.0

        # End-effector wrench
        self.wrench_fx = 0.0
        self.wrench_fy = 0.0
        self.wrench_fz = 0.0
        self.wrench_tx = 0.0
        self.wrench_ty = 0.0
        self.wrench_tz = 0.0

        header = ['timestamp']
        header += [f'pos_{i}' for i in range(self.n_joints)]
        header += [f'vel_{i}' for i in range(self.n_joints)]
        header += [f'eff_{i}' for i in range(self.n_joints)]
        header += [
            'arm_power',
            'tool_power',
            'speed_scaling',
            'wrench_fx', 'wrench_fy', 'wrench_fz',
            'wrench_tx', 'wrench_ty', 'wrench_tz',
            'in_cycle',
        ]
        self.writer.writerow(header)

        rospy.loginfo(f"[{arm_name}] Logging to {filename}")

    def update_tool_data(self, msg):
        self.tool_current = msg.tool_current
        self.tool_output_voltage = float(msg.tool_output_voltage)

    def update_speed_scaling(self, msg):
        self.speed_scaling = msg.data

    def update_wrench(self, msg):
        self.wrench_fx = msg.wrench.force.x
        self.wrench_fy = msg.wrench.force.y
        self.wrench_fz = msg.wrench.force.z
        self.wrench_tx = msg.wrench.torque.x
        self.wrench_ty = msg.wrench.torque.y
        self.wrench_tz = msg.wrench.torque.z

    def process(self, msg, timestamp, in_cycle):
        """Called from joint_callback when a matching full-arm message arrives."""

        indices = [msg.name.index(j) for j in self.joint_names if j in msg.name]
        if len(indices) != self.n_joints:
            return

        pos = [msg.position[i] for i in indices]
        eff = [msg.effort[i] for i in indices]

        if self.prev_pos is not None and self.prev_time is not None:
            dt = timestamp - self.prev_time
            if dt > 0:
                vel = [(pos[i] - self.prev_pos[i]) / dt for i in range(self.n_joints)]
            else:
                vel = [0.0] * self.n_joints
        else:
            vel = [0.0] * self.n_joints

        self.prev_pos = pos
        self.prev_time = timestamp

        arm_power = sum(eff[i] * vel[i] for i in range(self.n_joints))
        tool_power = self.tool_current * self.tool_output_voltage

        row = [timestamp]
        row += pos
        row += vel
        row += eff
        row += [
            arm_power,
            tool_power,
            self.speed_scaling,
            self.wrench_fx, self.wrench_fy, self.wrench_fz,
            self.wrench_tx, self.wrench_ty, self.wrench_tz,
            int(in_cycle),
        ]

        self.writer.writerow(row)

    def close(self):
        self.file.close()


class BaseLogger:
    """Logs mobile base power and thermal data to a separate file."""

    def __init__(self, timestamp_str):
        filename = f"energy_log_base_{timestamp_str}.csv"
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.left_current = 0.0
        self.right_current = 0.0
        self.left_voltage = 0.0
        self.right_voltage = 0.0
        self.left_motor_temp = 0
        self.right_motor_temp = 0
        self.left_heatsink_temp = 0
        self.right_heatsink_temp = 0

        header = [
            'timestamp',
            'left_voltage', 'left_current',
            'right_voltage', 'right_current',
            'base_power',
            'left_motor_temp', 'right_motor_temp',
            'left_heatsink_temp', 'right_heatsink_temp',
            'in_cycle',
        ]
        self.writer.writerow(header)
        rospy.loginfo(f"[base] Logging to {filename}")

    def log(self, timestamp, in_cycle):
        base_power = (
            self.left_voltage * self.left_current +
            self.right_voltage * self.right_current
        )
        row = [
            timestamp,
            self.left_voltage, self.left_current,
            self.right_voltage, self.right_current,
            base_power,
            self.left_motor_temp, self.right_motor_temp,
            self.left_heatsink_temp, self.right_heatsink_temp,
            int(in_cycle),
        ]
        self.writer.writerow(row)

    def close(self):
        self.file.close()


class EnergyLogger:

    def __init__(self):
        timestamp_str = str(int(time.time()))

        self.in_cycle = False

        self.left_arm = ArmLogger('left', LEFT_ARM_JOINTS, timestamp_str)
        self.right_arm = ArmLogger('right', RIGHT_ARM_JOINTS, timestamp_str)
        self.base = BaseLogger(timestamp_str)

        # --- Joint states (arms) ---
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        # --- Cycle marker ---
        rospy.Subscriber('/cycle_active', Bool,
                         lambda m: setattr(self, 'in_cycle', m.data))

        # --- Base power ---
        rospy.Subscriber('/left_drive/status/battery_current',
                         Float64, lambda m: setattr(self.base, 'left_current', m.data))
        rospy.Subscriber('/right_drive/status/battery_current',
                         Float64, lambda m: setattr(self.base, 'right_current', m.data))
        rospy.Subscriber('/left_drive/status/battery_voltage',
                         Float64, lambda m: setattr(self.base, 'left_voltage', m.data))
        rospy.Subscriber('/right_drive/status/battery_voltage',
                         Float64, lambda m: setattr(self.base, 'right_voltage', m.data))

        # --- Base temperatures ---
        rospy.Subscriber('/left_drive/status/motor_temperature',
                         Int32, lambda m: setattr(self.base, 'left_motor_temp', m.data))
        rospy.Subscriber('/right_drive/status/motor_temperature',
                         Int32, lambda m: setattr(self.base, 'right_motor_temp', m.data))
        rospy.Subscriber('/left_drive/status/heatsink_temperature',
                         Int32, lambda m: setattr(self.base, 'left_heatsink_temp', m.data))
        rospy.Subscriber('/right_drive/status/heatsink_temperature',
                         Int32, lambda m: setattr(self.base, 'right_heatsink_temp', m.data))

        # --- Tool data (gripper/flange power) ---
        rospy.Subscriber('/left_ur/ur_hardware_interface/tool_data',
                         ToolDataMsg, self.left_arm.update_tool_data)
        rospy.Subscriber('/right_ur/ur_hardware_interface/tool_data',
                         ToolDataMsg, self.right_arm.update_tool_data)

        # --- Speed scaling ---
        rospy.Subscriber('/left_ur/speed_scaling_factor',
                         Float64, self.left_arm.update_speed_scaling)
        rospy.Subscriber('/right_ur/speed_scaling_factor',
                         Float64, self.right_arm.update_speed_scaling)

        # --- End-effector wrench ---
        rospy.Subscriber('/left_ur/wrench',
                         WrenchStamped, self.left_arm.update_wrench)
        rospy.Subscriber('/right_ur/wrench',
                         WrenchStamped, self.right_arm.update_wrench)

    def joint_callback(self, msg):
        if self.base.left_current == 0.0 and self.base.right_current == 0.0:
            rospy.logwarn_throttle(
                5.0, "Base battery topics not received — base_power is 0")

        timestamp = rospy.Time.now().to_sec()

        names = set(msg.name)
        if LEFT_ARM_JOINTS[0] in names:
            self.left_arm.process(msg, timestamp, self.in_cycle)
            self.base.log(timestamp, self.in_cycle)
        elif RIGHT_ARM_JOINTS[0] in names:
            self.right_arm.process(msg, timestamp, self.in_cycle)

    def shutdown(self):
        self.left_arm.close()
        self.right_arm.close()
        self.base.close()
        rospy.loginfo("Energy logger shut down — all files closed.")


if __name__ == '__main__':
    rospy.init_node('full_energy_logger', anonymous=True)

    logger = EnergyLogger()

    rospy.on_shutdown(logger.shutdown)

    rospy.loginfo("Full energy logger started (left arm + right arm + base)...")
    rospy.spin()