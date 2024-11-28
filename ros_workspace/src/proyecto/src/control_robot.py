#!/usr/bin/python3

import sys
import rospy
from commands import Command
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from std_msgs.msg import Int32
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from math import pi
from influx import InfluxDBHandler
from datetime import datetime

import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient


class ControlRobot:
    influx_handler = InfluxDBHandler(
        url="https://deusto-influxdb-001-v2qrmk5znqme3f.eu-west-1.timestream-influxdb.amazonaws.com:8086",
        token="ySWNrRMtO_PmWPyxsWns5jRU8_EzSjBlK4IgfAsepArgbSv6UxGYXXkoMQb0ZdXUqrIRELV0TcD1x4udlW900Q==",
        org="Grupo_6",
        bucket="Grupo_6"
    )

    def __init__(self) -> None:
        # Inicialización de ROS y MoveIt
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        
        # Configuración del robot y escena de planificación
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        
        # Añadir el obstáculo suelo
        self.add_floor()
        rospy.Subscriber("/consignas", Int32, self.handle_command)

        rospy.loginfo("Ready to take commands for planning group %s.", self.group_name)

    def handle_command(self, msg: Int32) -> None:
        # Acción a realizar en función de la orden recibida
        command = msg.data
        if command == Command.COGER_FRUTA.value:
            rospy.loginfo("Coger fruta")
            self.move_to_specific_position("coger_fruta")
        elif command == Command.POSICION_INICIAL.value:
            rospy.loginfo("Posición inicial")
            self.move_to_specific_position("posicion_inicial")
        elif command == Command.CAJA_BUENA_ARRIBA.value:
            rospy.loginfo("Caja 1 arriba")
            self.move_to_specific_position("caja_1_arriba")
        elif command == Command.CAJA_BUENA_ABAJO.value:
            rospy.loginfo("Caja 1 abajo")
            self.move_to_specific_position("caja_1_abajo")
        elif command == Command.CAJA_MALA_ARRIBA.value:
            rospy.loginfo("Caja 2 arriba")
            self.move_to_specific_position("caja_2_arriba")
        elif command == Command.CAJA_MALA_ABAJO.value:
            rospy.loginfo("Caja 2 abajo")
            self.move_to_specific_position("caja_2_abajo")
        elif command == Command.ABRIR_PINZA.value:
            self.mover_pinza(100.0, 40.0)
        elif command == Command.CERRAR_PINZA.value:
            self.mover_pinza(0.0, 40.0)
        else:
            rospy.logwarn("Comando no reconocido")

    def add_floor(self) -> None:
        # Añade el suelo como obstáculo a la escena
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, 0.05))
        rospy.loginfo("Obstáculo 'suelo' añadido a la escena de planificación")

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (0.1, 0.1, 0.1)) -> None:
        # Añade una caja a la escena de planificación en una posición dada
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def move_to_pose(self) -> None:
        # Mueve el robot a una pose objetivo específica
        pose_goal = Pose()
        pose_goal.position.x = 0.5 
        pose_goal.position.y = 0.5
        pose_goal.position.z = 0.5
        pose_goal.orientation.w = 1.0
        self.move_group.set_pose_target(pose_goal)
        
        if self.move_group.go(wait=True):
            rospy.loginfo("Robot movido a la pose especificada")
        else:
            rospy.logwarn("No se pudo mover a la pose deseada")

        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        
        return result.reached_goal

    def move_to_configuration(self, joints) -> None:
        if self.move_group.go(joints, wait=True):
            rospy.loginfo("Robot movido a la configuración especificada")
        else:
            rospy.logwarn("No se pudo mover a la configuración deseada")

        self.move_group.stop()

    def follow_trajectory(self) -> None:
        # Define una trayectoria y hace que el robot la siga
        waypoints = []

        # Pose inicial (posición actual del extremo)
        start_pose = self.move_group.get_current_pose().pose
        waypoints.append(start_pose)

        # Pose intermedia
        wpose = Pose()
        wpose.position.x = start_pose.position.x + 0.1
        wpose.position.y = start_pose.position.y
        wpose.position.z = start_pose.position.z + 0.1
        waypoints.append(wpose)

        # Pose final
        wpose.position.z -= 0.2
        waypoints.append(wpose)

        # Planea y ejecuta la trayectoria
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        
        if fraction == 1.0:
            rospy.loginfo("Ejecutando trayectoria completa")
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logwarn("No se pudo ejecutar la trayectoria completa")

    def move_to_specific_position(self, action: str) -> None:
        joints = {
            "coger_fruta": [0.6133421063423157, -0.7791211170009156, 0.6027692000018519, -1.4883136761239548, -1.5353906790362757, 0.5786222219467163],
            "posicion_inicial": [1.0743645429611206, -0.9983374041369935, 0.4319809118853968, -1.0997422498515625, -1.5645702520953577, -0.11793691316713506],
            "caja_1_arriba": [1.7889834642410278, -1.310633049612381, 0.4467585722552698, -0.8000471752933045, -1.614591423665182, -1.3403261343585413],
            "caja_1_abajo": [1.788036584854126, -1.6146403751769007, 1.7082722822772425, -1.7573992214598597, -1.6137712637530726, -1.3365376631366175],
            "caja_2_arriba": [-0.2870891729937952, -1.487642579977848, 0.627301041279928, -0.7119994324496766, -1.5659797827350062, -1.353751007710592],
            "caja_2_abajo": [-0.28807336488832647, -1.7917567692198695, 1.7556470076190394, -1.536181890671589, -1.564662281666891, -1.3506696859942835]
        }

        if action in joints:
            self.move_to_configuration(joints[action])
            
            traceability_code = self.generate_traceability_code()
            metrics = self.collect_robot_metrics(traceability_code=traceability_code)
            self.influx_handler.write_data("robot_metrics", [
                {
                    "TraceabilityCode": metrics["TraceabilityCode"],
                    "VelocityValues": metrics["VelocityValues"],
                    "VelocityUnits": metrics["VelocityUnits"],
                    "RotationValues": metrics["RotationValues"],
                    "RotationUnits": metrics["RotationUnits"],
                    "TorqueValues": metrics["TorqueValues"],
                    "TorqueUnits": metrics["TorqueUnits"]
                }
            ])

    def collect_robot_metrics(self, traceability_code): 
        joint_positions = self.move_group.get_current_joint_values()
        velocity_values = [0.1] * len(joint_positions)  
        torque_values = [0.2] * len(joint_positions) 
        
        return {
            "TraceabilityCode": traceability_code,
            "VelocityValues": velocity_values,
            "VelocityUnits": "rad/s",
            "RotationValues": joint_positions,
            "RotationUnits": "rad",
            "TorqueValues": torque_values,
            "TorqueUnits": "N·m"
        }

    def generate_traceability_code(self) -> str:
        now = datetime.now()
        year = now.strftime("%y") 
        julian_day = now.strftime("%j") 
        time_part = now.strftime("%H%M%S") 
        return f"{year}{julian_day}{time_part}"
    
    def close(self) -> None:
        self.influx_handler.close()

if __name__ == '__main__':
    try:
        control = ControlRobot()
        rospy.spin()

        control.close()
    except rospy.ROSInterruptException:
        pass

# suscribir al topic de joint states para las medidas