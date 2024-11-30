#!/usr/bin/python3

import rospy
import sys
from sensor_msgs.msg import JointState

class JointStateSimulator:
    def __init__(self, simulate: bool):
        rospy.init_node('joint_state_simulator', anonymous=True)
        
        self.simulate = simulate

        self.prev_positions = None
        self.prev_time = None
        self.moment_of_inertia = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Momento de inercia para cada eje

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.pub = rospy.Publisher("/joint_state_v2", JointState, queue_size=10)

    def joint_state_callback(self, msg):
        if not self.simulate:

            joint_positions = list(msg.position) if msg.position else [0.0] * 6
            joint_velocities = list(msg.velocity) if msg.velocity else [0.0] * 6
            joint_efforts = list(msg.effort) if msg.effort else [0.0] * 6

            traceability_code = int(rospy.Time.now().to_sec() * 1000) 

            formatted_output = {
                "TraceabilityCode": traceability_code,
                "VelocityValues": [joint_velocities],  
                "VelocityUnits": "rad/s",
                "RotationValues": [joint_positions], 
                "RotationUnits": "rad",
                "TorqueValues": [joint_efforts], 
                "TorqueUnits": "N·m"
            }
            return

        current_positions = msg.position[:6]
        current_time = rospy.Time.now()

        if self.prev_positions is not None:
            delta_time = (current_time - self.prev_time).to_sec()

            if delta_time <= 0:
                rospy.logwarn("Delta time is zero or negative; skipping update.")
                return

            # Calcular velocidades angulares (derivada de posiciones)
            velocities = [
                (curr - prev) / delta_time for curr, prev in zip(current_positions, self.prev_positions)
            ]

            # Calcular aceleraciones y torques
            if hasattr(msg, "velocity") and len(msg.velocity) >= 6:
                accelerations = [
                    (curr - prev) / delta_time for curr, prev in zip(velocities, msg.velocity[:6])
                ]
            else:
                accelerations = [0] * len(velocities)

            torques = [
                J * a for J, a in zip(self.moment_of_inertia, accelerations)
            ]

            new_msg = JointState()
            new_msg.header.stamp = rospy.Time.now()
            new_msg.name = msg.name[:6]  
            new_msg.position = current_positions
            new_msg.velocity = velocities
            new_msg.effort = torques

            self.pub.publish(new_msg)

        else:
            rospy.loginfo("Esperando datos iniciales para simulación...")

        self.prev_positions = current_positions
        self.prev_time = current_time


if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            print("Uso: joint_state_simulator.py <simulate: true|false>")
            sys.exit(1)

        simulate_arg = sys.argv[1].lower()
        if simulate_arg not in ["true", "false"]:
            print("El argumento debe ser 'true' o 'false'.")
            sys.exit(1)

        simulate = simulate_arg == "true"

        if simulate:
            rospy.loginfo("Modo simulación activado.")
        else:
            rospy.loginfo("Modo simulación desactivado.")

        simulator = JointStateSimulator(simulate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
