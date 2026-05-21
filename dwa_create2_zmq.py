import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

R = 0.036
L = 0.235

MAX_V = 0.35
MIN_V = -0.10
MAX_W = 1.8
MAX_ACCEL_V = 0.8
MAX_ACCEL_W = 3.0


def normalizar_angulo(a):
    return math.atan2(math.sin(a), math.cos(a))


def ler_sensores(sim, sensores):
    distancias = []
    obstaculos_detectados = 0

    for sensor in sensores:
        res, dist, point, obj, n = sim.readProximitySensor(sensor)

        if res > 0 and dist < 0.6:
            distancias.append(dist)
            obstaculos_detectados += 1
        else:
            distancias.append(0.6)

    return distancias, obstaculos_detectados


def aplicar_velocidade_motores(sim, motor_esquerdo, motor_direito, v, w):
    vel_esq = (v - (w * L / 2)) / R
    vel_dir = (v + (w * L / 2)) / R

    sim.setJointTargetVelocity(motor_esquerdo, vel_esq)
    sim.setJointTargetVelocity(motor_direito, vel_dir)


def parar_robo(sim, motor_esquerdo, motor_direito):
    sim.setJointTargetVelocity(motor_esquerdo, 0.0)
    sim.setJointTargetVelocity(motor_direito, 0.0)


def calibrar_frente_robo(sim, client, robot_base, motor_esquerdo, motor_direito):
    print("Calibrando frente real do robô...")

    pos1 = sim.getObjectPosition(robot_base, -1)
    ori1 = sim.getObjectOrientation(robot_base, -1)

    sim.setJointTargetVelocity(motor_esquerdo, 2.0)
    sim.setJointTargetVelocity(motor_direito, 2.0)

    for _ in range(20):
        client.step()

    parar_robo(sim, motor_esquerdo, motor_direito)

    pos2 = sim.getObjectPosition(robot_base, -1)

    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]

    angulo_movimento = math.atan2(dy, dx)
    offset = normalizar_angulo(angulo_movimento - ori1[2])

    print(f"Offset da frente calibrado: {offset:.2f} rad")

    return offset


def main():
    print("Conectando ao CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")

    print("Conectado! Buscando handles...")

    motor_esquerdo = sim.getObject("/Cuboid/MOTOR_ESQUERDO")
    motor_direito = sim.getObject("/Cuboid/MOTOR_DIREITO")

    sensores = [
        sim.getObject("/Cuboid/SENSOR_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_ESQUERDO"),
        sim.getObject("/Cuboid/SENSOR_MEIO"),
        sim.getObject("/Cuboid/SENSOR_DIAG_DIREITO"),
        sim.getObject("/Cuboid/SENSOR_DIREITO")
    ]

    goal_handle = sim.getObject("/Target")
    robot_base = sim.getObject("/Cuboid")

    v_atual = 0.0
    w_atual = 0.0
    modo_fuga = 0
    direcao_fuga = 1

    tempo_acumulado = 0.0
    intervalo_print = 1.0

    client.setStepping(True)
    sim.startSimulation()

    offset_frente = calibrar_frente_robo(
        sim, client, robot_base, motor_esquerdo, motor_direito
    )

    print("Simulação iniciada.")

    try:
        while True:
            dt = sim.getSimulationTimeStep()
            tempo_acumulado += dt

            pos = sim.getObjectPosition(robot_base, -1)
            goal_pos = sim.getObjectPosition(goal_handle, -1)

            dist_to_goal = math.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1])

            if dist_to_goal < 0.2:
                print("[EVENTO] Target alcançado! Encerrando simulação...")
                parar_robo(sim, motor_esquerdo, motor_direito)
                client.step()
                break

            distancias, obstaculos_detectados = ler_sensores(sim, sensores)

            ori = sim.getObjectOrientation(robot_base, -1)
            theta = normalizar_angulo(ori[2] + offset_frente)

            angulo_alvo = math.atan2(goal_pos[1] - pos[1], goal_pos[0] - pos[0])
            erro = normalizar_angulo(angulo_alvo - theta)

            frente_livre = distancias[2] > 0.32
            diagonal_livre = distancias[1] > 0.25 and distancias[3] > 0.25

            if obstaculos_detectados == 0 or (frente_livre and diagonal_livre):
                modo_fuga = 0

                v_atual = 0.30
                w_atual = 3.2 * erro

                if w_atual > MAX_W:
                    w_atual = MAX_W
                elif w_atual < -MAX_W:
                    w_atual = -MAX_W

                if abs(erro) > 0.7:
                    v_atual = 0.15

            else:
                esquerda = distancias[0] + distancias[1]
                direita = distancias[3] + distancias[4]

                if esquerda > direita:
                    direcao_fuga = 1
                else:
                    direcao_fuga = -1

                if modo_fuga == 0:
                    modo_fuga = 80

                # 1) dá ré forte
                if modo_fuga > 55:
                    v_atual = -0.18
                    w_atual = 0.0

                # 2) gira MUITO parado
                elif modo_fuga > 35:
                    v_atual = 0.0
                    w_atual = 2.8 * direcao_fuga

                # 3) avança fazendo curva forte
                elif modo_fuga > 15:
                    v_atual = 0.18
                    w_atual = 2.0 * direcao_fuga

                # 4) volta pro target suavemente
                else:
                    v_atual = 0.24
                    w_atual = 0.6 * direcao_fuga

                modo_fuga -= 1

            aplicar_velocidade_motores(
                sim, motor_esquerdo, motor_direito, v_atual, w_atual
            )

            if tempo_acumulado >= intervalo_print:
                print(
                    f"Vel: {v_atual:.2f} | Giro: {w_atual:.2f} | "
                    f"Erro: {erro:.2f} | Obstáculos: {obstaculos_detectados}/5 | "
                    f"Distância Target: {dist_to_goal:.2f}m | Fuga: {modo_fuga}"
                )
                tempo_acumulado = 0.0

            client.step()

    except KeyboardInterrupt:
        print("Simulação interrompida.")

    finally:
        parar_robo(sim, motor_esquerdo, motor_direito)
        sim.stopSimulation()
        print("Conexão encerrada.")


if __name__ == "__main__":
    main()