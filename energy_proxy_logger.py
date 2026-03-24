#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import csv
import time

class EnergyLogger:

    def __init__(self):
        self.filename = f"energy_log_{int(time.time())}.csv"
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        # Base values
        self.left_current = 0.0
        self.right_current = 0.0
        self.left_voltage = 0.0
        self.right_voltage = 0.0

        # Header
        header = ['timestamp']
        header += [f'pos_{i}' for i in range(10)]
        header += [f'vel_{i}' for i in range(10)]
        header += [f'eff_{i}' for i in range(10)]
        header += ['arm_power', 'base_power', 'total_power']
        self.writer.writerow(header)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        rospy.Subscriber('/left_drive/status/battery_current', Float32, self.left_current_cb)
        rospy.Subscriber('/right_drive/status/battery_current', Float32, self.right_current_cb)
        rospy.Subscriber('/left_drive/status/battery_voltage', Float32, self.left_voltage_cb)
        rospy.Subscriber('/right_drive/status/battery_voltage', Float32, self.right_voltage_cb)

    # --- Base callbacks ---
    def left_current_cb(self, msg):
        self.left_current = msg.data

    def right_current_cb(self, msg):
        self.right_current = msg.data

    def left_voltage_cb(self, msg):
        self.left_voltage = msg.data

    def right_voltage_cb(self, msg):
        self.right_voltage = msg.data

    # --- Joint callback ---
    def joint_callback(self, msg):
        timestamp = msg.header.stamp.to_sec()

        pos = list(msg.position)
        vel = list(msg.velocity)
        eff = list(msg.effort)

        n = min(len(pos), len(vel), len(eff))

        # Arm power
        arm_power = sum(eff[i] * vel[i] for i in range(n))

        # Base power
        base_power = (
            self.left_voltage * self.left_current +
            self.right_voltage * self.right_current
        )

        total_power = arm_power + base_power

        row = [timestamp]

        row += pos + [0]*(10 - len(pos))
        row += vel + [0]*(10 - len(vel))
        row += eff + [0]*(10 - len(eff))

        row += [arm_power, base_power, total_power]

        self.writer.writerow(row)

    def shutdown(self):
        self.file.close()


if __name__ == '__main__':
    rospy.init_node('full_energy_logger', anonymous=True)

    logger = EnergyLogger()

    rospy.on_shutdown(logger.shutdown)

    rospy.loginfo("Full energy logger started...")
    rospy.spin()