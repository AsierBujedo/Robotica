#!/usr/bin/python3

from commands import Command, CommandQueue

queue = CommandQueue()

def poner_caja_buena():
    global queue
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.ABRIR_PINZA)
    queue.push(Command.COGER_FRUTA)
    queue.push(Command.CERRAR_PINZA)
    queue.push(Command.CAJA_BUENA_ARRIBA)
    queue.push(Command.CAJA_BUENA_ABAJO)
    queue.push(Command.ABRIR_PINZA)
    queue.push(Command.CAJA_BUENA_ARRIBA)
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.ABRIR_PINZA)
    queue.flush()

def poner_caja_mala():
    global queue
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.ABRIR_PINZA)
    queue.push(Command.COGER_FRUTA)
    queue.push(Command.CERRAR_PINZA_MALA)
    queue.push(Command.CAJA_MALA_ARRIBA)
    queue.push(Command.CAJA_MALA_ABAJO)
    queue.push(Command.ABRIR_PINZA)
    queue.push(Command.CAJA_MALA_ARRIBA)
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.ABRIR_PINZA)
    queue.flush()
