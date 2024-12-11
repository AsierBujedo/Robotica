import rospy
from std_msgs.msg import Int32
from enum import Enum

class Command(Enum):
    COGER_FRUTA = 0
    POSICION_INICIAL = 1
    CAJA_BUENA_ARRIBA = 2
    CAJA_BUENA_ABAJO = 3
    CAJA_MALA_ARRIBA = 4
    CAJA_MALA_ABAJO = 5
    ABRIR_PINZA = 6
    CERRAR_PINZA = 7
    CERRAR_PINZA_MALA = 8


    @staticmethod
    def is_command(value: int) -> bool:
        return any(value == item.value for item in Command)


class CommandQueue:
    queue = []
    pub = None
    rate = None

    def __init__(self) -> None:
        rospy.init_node('nodo_ordenes', anonymous=True)
        self.pub = rospy.Publisher('/consignas', Int32, queue_size=20)
        self.rate = rospy.Rate(1)

    def push(self, command: Command) -> None:
        if Command.is_command(command.value):
            self.queue.append(command)

    def send(self, command: Command) -> None:
        rospy.loginfo(f"Sending command {command.value} ({command.name})")
        self.pub.publish(command.value)
        self.rate.sleep()

    def flush(self) -> None:
        rospy.loginfo("Sending new command queue")
        # self.send(Command.POSICION_INICIAL)
        # self.send(Command.ABRIR_PINZA)

        for command in self.queue:
            self.send(command)
        
        # self.send(Command.POSICION_INICIAL)
        # self.send(Command.ABRIR_PINZA)

        self.queue.clear()