#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32
from controller import *

def nodo_ordenes():
    rospy.init_node('nodo_ordenes', anonymous=True) #inicio del nodo ROS
    pub = rospy.Publisher('/consignas', Int32, queue_size=20) # Crear publicador para enviar mensajes al topic '/consignas'-->tipo de mensaje Int32.
    rate = rospy.Rate(1)  # Frecuencia de ejecución del bucle->1 Hz

    while not rospy.is_shutdown():
        print("Seleccione una opción:")
        print("0 - Coger fruta")
        print("1 - Posición inicial")
        print("2 - Caja 1 arriba")
        print("3 - Caja 1 abajo")
        print("4 - Caja 2 arriba")
        print("5 - Caja 2 abajo")
        print("6 - Abrir pinza")
        print("7 - Cerrar pinza")
        print("8 - poner caja buena")
        print("9 - poner caja mala")
        comando = int(input("Inserte un número del menú: "))

        if comando in [0, 1, 2, 3, 4, 5, 6, 7]:
            rospy.loginfo(f"Publicando orden: {comando}")
            pub.publish(comando)
        elif comando == 8:
            poner_caja_buena()
        elif comando == 9:
            poner_caja_mala()
        else:
            print("Comando no válido, por favor ingrese un número entre 1 y 6")

        rate.sleep()

if __name__ == '__main__':
    try:
        nodo_ordenes()
    except rospy.ROSInterruptException:
        pass
