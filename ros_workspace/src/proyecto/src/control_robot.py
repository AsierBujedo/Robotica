#!/usr/bin/python3

import sys
import rospy
from datetime import datetime
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from actionlib import SimpleActionClient
from influx import InfluxDBHandler
from commands import Command


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
        
        # Suscribirse a topics
        rospy.Subscriber("/consignas", Int32, self.handle_command)
        rospy.Subscriber("/joint_states", JointState, self.handle_joint_states)

        # Añadir el obstáculo suelo
        self.add_floor()
        rospy.loginfo("Ready to take commands for planning group %s.", self.group_name)

    def handle_command(self, msg: Int32) -> None:
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
            rospy.loginfo("Abrir pinza")
            self.mover_pinza(100.0, 10.0)
            rospy.sleep(2)
        elif command == Command.CERRAR_PINZA.value:
            self.mover_pinza(0.0, 10.0)
            rospy.loginfo("Cerrar pinza")
            rospy.sleep(2)
        elif command == Command.CERRAR_PINZA_MALA.value:
            rospy.loginfo("Cerrar pinza lentamente")
            self.mover_pinza(0.0, 10.0)
            rospy.sleep(2)
        else:
            rospy.logwarn("Comando no reconocido")

        rospy.sleep(1) # Change if necessary

    def handle_joint_states(self, msg: JointState) -> None:

        joint_positions = list(msg.position) if msg.position else []
        joint_velocities = list(msg.velocity) if msg.velocity else []
        joint_efforts = list(msg.effort) if msg.effort else []

        is_moving = any(abs(vel) > 0 for vel in joint_velocities)

        if not is_moving:
            return

        traceability_code = self.generate_traceability_code()

        metrics = {
            "TraceabilityCode": traceability_code,
            "VelocityValues": [list(joint_velocities)],
            "VelocityUnits": "rad/s",
            "RotationValues": [list(joint_positions)],
            "RotationUnits": "rad",
            "TorqueValues": [list(joint_efforts)],
            "TorqueUnits": "N·m"
        }

        print(metrics)

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

    def print_current_joint_values(self) -> None:
        joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo(f"Current joint values: {joint_values}")

    def generate_traceability_code(self) -> int:
        now = datetime.now()
        year = now.strftime("%y")
        julian_day = now.strftime("%j")
        time_part = now.strftime("%H%M%S")
        return int(f"{year}{julian_day}{time_part}")

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, 0.05))
        rospy.loginfo("Obstáculo 'suelo' añadido a la escena de planificación")

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (0.1, 0.1, 0.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def move_to_specific_position(self, action: str) -> None:
        joints = {
            "coger_fruta2": [0.6133421063423157, -0.7791211170009156, 0.6027692000018519, -1.4883136761239548, -1.5353906790362757, 0.5786222219467163],
            "coger_fruta": [1.3250775337219238, -1.690538068810934, 2.024757687245504, -1.9064632854857386, -1.5653255621539515, -0.15932780901064092],
            "posicion_inicial2": [1.0743645429611206, -0.9983374041369935, 0.4319809118853968, -1.0997422498515625, -1.5645702520953577, -0.11793691316713506],
            "posicion_inicial": [1.326235294342041, -1.5628456531888624, 0.9138191382037562, -0.9232736390880127, -1.5653212706195276, -0.1632168928729456],
            "caja_1_arriba2": [1.7889834642410278, -1.310633049612381, 0.4467585722552698, -0.8000471752933045, -1.614591423665182, -1.3403261343585413],
            "caja_1_arriba": [2.581453800201416, -1.5279813569835206, 0.8575785795794886, -0.9018660348704834, -1.5653975645648401, -0.19458467165102178],
            "caja_1_abajo2": [1.788036584854126, -1.6146403751769007, 1.7082722822772425, -1.7573992214598597, -1.6137712637530726, -1.3365376631366175],
            "caja_1_abajo": [2.580962896347046, -1.6945468387999476, 1.468241039906637, -1.3459847134402771, -1.5646947065936487, -0.19279414812196904],
            "caja_2_arriba": [-0.2870891729937952, -1.487642579977848, 0.627301041279928, -0.7119994324496766, -1.5659797827350062, -1.353751007710592],
            #"caja_2_abajo2": [-0.28807336488832647, -1.7917567692198695, 1.7556470076190394, -1.536181890671589, -1.564662281666891, -1.3506696859942835],
            "caja_2_abajo": [-0.2878049055682581, -1.7509356937804164, 1.4428313414203089, -1.264292524462082, -1.564610783253805, -1.351699177418844]
        }

        if action in joints:
            self.move_group.go(joints[action], wait=True)

    # Check the doc out http://docs.ros.org/en/hydro/api/ric_mc/html/GripperCommand_8h_source.html for GripperCommandGoal     
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        return result.reached_goal

    def close(self) -> None:
        self.influx_handler.close()


if __name__ == '__main__':
    try:
        control = ControlRobot()

        #while not rospy.is_shutdown():
        #    control.print_current_joint_values()

        rospy.spin()
        control.close()
    except rospy.ROSInterruptException:
        pass
