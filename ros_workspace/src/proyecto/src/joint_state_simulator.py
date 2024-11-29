#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from math import pi

class JointStateSimulator:
    def __init__(self):
        # Inicializar nodo
        rospy.init_node('joint_state_simulator', anonymous=True)
        
        # Inicializar variables para el cálculo de velocidades y torques
        self.prev_positions = None
        self.prev_time = None
        self.moment_of_inertia = [0.1] * 6  # Momento de inercia para cada eje (ejemplo)

        # Suscripción al tópico original y publicación en el nuevo
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.pub = rospy.Publisher("/joint_state_v2", JointState, queue_size=10)

    def joint_state_callback(self, msg):
        # Leer posiciones actuales de los ejes
        current_positions = msg.position
        current_time = rospy.Time.now()

        if self.prev_positions is not None:
            # Calcular el intervalo de tiempo
            delta_time = (current_time - self.prev_time).to_sec()

            # Calcular velocidades angulares (derivada de posiciones)
            velocities = [
                (curr - prev) / delta_time for curr, prev in zip(current_positions, self.prev_positions)
            ]

            # Simular torques (proporcional a la aceleración angular)
            if hasattr(msg, "velocity") and len(msg.velocity) > 0:
                accelerations = [
                    (curr - prev) / delta_time for curr, prev in zip(velocities, msg.velocity)
                ]
            else:
                accelerations = [0] * len(velocities)

            torques = [
                J * a for J, a in zip(self.moment_of_inertia, accelerations)
            ]

            # Crear un nuevo mensaje JointState
            new_msg = JointState()
            new_msg.header.stamp = rospy.Time.now()
            new_msg.name = msg.name
            new_msg.position = current_positions
            new_msg.velocity = velocities
            new_msg.effort = torques

            # Publicar mensaje
            self.pub.publish(new_msg)

        # Actualizar variables previas
        self.prev_positions = current_positions
        self.prev_time = current_time


if __name__ == '__main__':
    try:
        simulator = JointStateSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
