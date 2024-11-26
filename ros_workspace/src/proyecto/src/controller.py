#!/usr/bin/python3

from commands import Command, CommandQueue

queue = CommandQueue()

def poner_caja_buena():
    global queue
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.COGER_FRUTA)
    # CERRAR PINZA
    queue.push(Command.CAJA_BUENA_ARRIBA)
    queue.push(Command.CAJA_BUENA_ABAJO)
    # ABRIR PINZA
    queue.push(Command.POSICION_INICIAL)
    queue.flush()

def poner_caja_mala():
    global queue
    queue.push(Command.POSICION_INICIAL)
    queue.push(Command.COGER_FRUTA)
    # CERRAR PINZA
    queue.push(Command.CAJA_MALA_ARRIBA)
    queue.push(Command.CAJA_MALA_ABAJO)
    # ABRIR PINZA
    queue.push(Command.POSICION_INICIAL)
    queue.flush()