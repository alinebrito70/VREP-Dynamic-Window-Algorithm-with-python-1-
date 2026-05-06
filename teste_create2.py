from lib import rapinterface as ri

def init():
    global motor_direito, motor_esquerdo

    motor_direito = ri.getobject("MOTOR_DIREITO")
    motor_esquerdo = ri.getobject("MOTOR_ESQUERDO")

    print("Motores encontrados.")

def loop():
    print("Andando para frente...")

    ri.setvelocity(motor_direito, 2)
    ri.setvelocity(motor_esquerdo, 2)

    ri.sleep(3)

    print("Parando...")

    ri.setvelocity(motor_direito, 0)
    ri.setvelocity(motor_esquerdo, 0)

    return True

def finnaly():
    print("Teste finalizado.")

ri.start(init, loop, finnaly, 10)