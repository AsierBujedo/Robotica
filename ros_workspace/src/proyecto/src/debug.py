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
        print("1 - Coger fruta")
        print("2 - Posición inicial")
        print("3 - Caja 1 arriba")
        print("4 - Caja 1 abajo")
        print("5 - Caja 2 arriba")
        print("6 - Caja 2 abajo")
        print("7 - Abrir pinza")
        print("8 - Cerrar pinza")
        print("9 - poner caja buena")
        print("10 - poner caja mala")
        comando = int(input("Inserte un número del menú: "))

        if comando in [1, 2, 3, 4, 5, 6, 7, 8]:
            rospy.loginfo(f"Publicando orden: {comando}")
            pub.publish(comando)
        elif comando == 9:
            poner_caja_buena()
        elif comando == 10:
            poner_caja_mala()
        else:
            print("Comando no válido, por favor ingrese un número entre 1 y 6")

        rate.sleep()

if __name__ == '__main__':
    try:
        nodo_ordenes()
    except rospy.ROSInterruptException:
        pass
