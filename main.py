import math

import numpy as np
import matplotlib.pyplot as plt

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import dynamic_window_approach as dw
import mapa_ocupacao as mo


SINAL_FRENTE = +1


client = RemoteAPIClient()
sim = client.require("sim")

param = {}
dwa = dw.DWAController()


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_required_object(path):
    try:
        return sim.getObject(path)
    except Exception as exc:
        raise RuntimeError(f"Objeto obrigatorio nao encontrado: {path}") from exc


def is_descendant_of(handle, parent):
    current = handle

    while current != -1:
        if current == parent:
            return True

        current = sim.getObjectParent(current)

    return False


def transform_point(matrix, point):
    return np.array(
        [
            matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
            matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
            matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
        ],
        dtype=float,
    )


def add_rectangle_points(points, min_x, max_x, min_y, max_y, step=0.06):
    x = min_x

    while x <= max_x:
        points.append([x, min_y])
        points.append([x, max_y])
        x += step

    y = min_y

    while y <= max_y:
        points.append([min_x, y])
        points.append([max_x, y])
        y += step


def fill_rectangle_points(points, min_x, max_x, min_y, max_y, step=0.05):
    x = min_x

    while x <= max_x:
        y = min_y

        while y <= max_y:
            points.append([x, y])
            y += step

        x += step


def create_static_obstacles():
    points = []

    floor = get_required_object("/Floor")
    floor_pos = sim.getObjectPosition(floor, -1)

    floor_min_x = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_x)
    floor_max_x = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_x)
    floor_min_y = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_y)
    floor_max_y = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_y)

    add_rectangle_points(
        points,
        floor_pos[0] + floor_min_x,
        floor_pos[0] + floor_max_x,
        floor_pos[1] + floor_min_y,
        floor_pos[1] + floor_max_y,
    )

    for obj in sim.getObjectsInTree(sim.handle_scene):
        if sim.getObjectType(obj) != sim.object_shape_type:
            continue

        alias = sim.getObjectAlias(obj, 0)

        if alias in {"Floor", "box", "Goal", "Target"}:
            continue

        if obj == param["robot"] or is_descendant_of(obj, param["robot"]):
            continue

        pos = sim.getObjectPosition(obj, -1)

        min_x = pos[0] + sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_x)
        max_x = pos[0] + sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_x)
        min_y = pos[1] + sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_min_y)
        max_y = pos[1] + sim.getObjectFloatParam(obj, sim.objfloatparam_objbbox_max_y)

        width = max_x - min_x
        height = max_y - min_y

        if width > 3.0 and height > 3.0:
            continue

        fill_rectangle_points(points, min_x, max_x, min_y, max_y)

    unique_points = {}

    for x, y in points:
        unique_points[(round(x, 2), round(y, 2))] = [x, y]

    return np.array(list(unique_points.values()), dtype=float)


def get_robot_state(v=0.0, w=0.0):
    position = sim.getObjectPosition(param["robot"], -1)
    orientation = sim.getObjectOrientation(param["robot"], -1)

    heading_offset = param.get("heading_offset", 0.0)
    flip = 0.0 if SINAL_FRENTE >= 0 else math.pi
    theta_front = normalize_angle(orientation[2] + heading_offset + flip)

    return np.array(
        [
            position[0],
            position[1],
            theta_front,
            v,
            w,
        ],
        dtype=float,
    )


def medir_heading_offset():
    sensor_frontal = param["sensors"][0]

    matrix = sim.getObjectMatrix(sensor_frontal, -1)

    front_x = matrix[2]
    front_y = matrix[6]
    front_yaw = math.atan2(front_y, front_x)

    yaw_modelo = sim.getObjectOrientation(param["robot"], -1)[2]

    param["heading_offset"] = normalize_angle(front_yaw - yaw_modelo)

    print(
        "Heading offset (frente - yaw):",
        round(math.degrees(param["heading_offset"]), 1),
        "graus",
    )


def get_sensor_obstacles():
    obstacles = []

    for sensor in param["sensors"]:
        result, distance, point, obj, normal = sim.readProximitySensor(sensor)

        if result > 0:
            detected_point = np.array(point, dtype=float)

            if np.linalg.norm(detected_point) <= 0.0 and distance > 0.0:
                detected_point = np.array([distance, 0.0, 0.0], dtype=float)

            matrix = sim.getObjectMatrix(sensor, -1)

            obstacle_world = transform_point(matrix, detected_point)

            obstacles.append([obstacle_world[0], obstacle_world[1]])

    return np.array(obstacles, dtype=float)


def get_obstacles(x):
    local_obstacles = []

    static_obstacles = param.get("static_obstacles")

    if static_obstacles is not None and len(static_obstacles) > 0:
        distances = np.hypot(
            static_obstacles[:, 0] - x[0],
            static_obstacles[:, 1] - x[1]
        )

        local_obstacles.extend(
            static_obstacles[distances <= 1.4].tolist()
        )

    sensor_obstacles = get_sensor_obstacles()

    if len(sensor_obstacles) > 0:
        local_obstacles.extend(sensor_obstacles.tolist())

    return np.array(local_obstacles, dtype=float)


def would_collide(x):
    static_obstacles = param.get("static_obstacles")

    if static_obstacles is None or len(static_obstacles) == 0:
        return False

    distances = np.hypot(
        static_obstacles[:, 0] - x[0],
        static_obstacles[:, 1] - x[1]
    )

    return float(np.min(distances)) <= dwa.collision_radius


def build_global_path():
    obstacles = param["static_obstacles"]

    sx, sy = float(param["x"][0]), float(param["x"][1])
    gx, gy = float(param["goal_coords"][0]), float(param["goal_coords"][1])

    rx = ry = kx = ky = None

    for rr in (0.24, 0.20, 0.16):
        planner = dw.AStarPlanner(
            obstacles[:, 0].tolist(),
            obstacles[:, 1].tolist(),
            resolution=0.1,
            rr=rr,
        )

        rx, ry, kx, ky = planner.planning(sx, sy, gx, gy)

        if not planner.last_plan_failed:
            print(f"A* encontrou rota com margem rr={rr:.2f} m")
            break

        print(f"A* sem rota com rr={rr:.2f} m; reduzindo margem...")

    else:
        print("A* não encontrou rota mesmo com margem mínima; seguindo em linha reta.")

        if len(obstacles) > 0:
            d_start = float(np.min(np.hypot(obstacles[:, 0] - sx, obstacles[:, 1] - sy)))
            d_goal = float(np.min(np.hypot(obstacles[:, 0] - gx, obstacles[:, 1] - gy)))

            print(
                f"   diag: obstáculo mais próximo do INÍCIO = {d_start*100:.0f} cm, "
                f"do GOAL = {d_goal*100:.0f} cm "
                f"(raio de colisão = {dwa.collision_radius*100:.0f} cm)"
            )

            if d_goal < dwa.collision_radius:
                print(
                    "   -> O GOAL está praticamente encostado/dentro de um obstáculo. "
                    "Afaste o Goal da parede ~20 cm ou reduza collision_radius."
                )

            if d_start < dwa.collision_radius:
                print(
                    "   -> O INÍCIO do robô está colado a um obstáculo. "
                    "Verifique a exclusão do robô / posição inicial."
                )

        rx = [sx, gx]
        ry = [sy, gy]
        kx = [sx, gx]
        ky = [sy, gy]

    param["global_path"] = list(zip(rx, ry))
    param["key_points"] = list(zip(kx, ky))

    param["path_index"] = 0

    print("Waypoints (caminho suavizado):", len(param["global_path"]))
    print("Pontos-chave (A* melhorado):", len(param["key_points"]))


def get_path_target(x):
    path = param.get("global_path", [param["goal_coords"]])

    while param["path_index"] < len(path) - 1:
        target = path[param["path_index"]]

        if math.hypot(target[0] - x[0], target[1] - x[1]) >= 0.45:
            break

        param["path_index"] += 1

    lookahead_index = min(param["path_index"] + 3, len(path) - 1)

    return path[lookahead_index]


def robot_motion(u):
    v = float(u[0])
    w = float(u[1])

    dt = dwa.dt

    wheel_radius = 0.0375
    wheel_base = 0.15

    x = param["x"].copy()

    x[2] = normalize_angle(x[2] + w * dt)

    x[0] += v * math.cos(x[2]) * dt
    x[1] += v * math.sin(x[2]) * dt

    x[3] = v
    x[4] = w

    if would_collide(x):
        x = param["x"].copy()

        x[2] = normalize_angle(x[2] + w * dt)

        x[3] = 0.0
        x[4] = w

        v = 0.0

    wr = (2.0 * v + w * wheel_base) / (2.0 * wheel_radius)
    wl = (2.0 * v - w * wheel_base) / (2.0 * wheel_radius)

    wr = max(min(wr, 20.0), -20.0)
    wl = max(min(wl, 20.0), -20.0)

    sim.setJointTargetVelocity(param["motorRight"], wr)
    sim.setJointTargetVelocity(param["motorLeft"], wl)

    sim.setObjectPosition(
        param["robot"],
        -1,
        [x[0], x[1], param["robot_z"]]
    )

    heading_offset = param.get("heading_offset", 0.0)
    flip = 0.0 if SINAL_FRENTE >= 0 else math.pi
    yaw_modelo = normalize_angle(x[2] - heading_offset - flip)

    sim.setObjectOrientation(
        param["robot"],
        -1,
        [param["robot_roll"], param["robot_pitch"], yaw_modelo]
    )

    try:
        sim.resetDynamicObject(param["robot"])
    except Exception:
        pass

    sim.step()

    return x


def obter_obstaculos_estaticos():
    excluir = [
        (float(param["x"][0]), float(param["x"][1]), dwa.robot_radius + 0.20),
        (float(param["goal_coords"][0]), float(param["goal_coords"][1]), dwa.robot_radius + 0.10),
    ]

    try:
        floor = get_required_object("/Floor")

        pontos, grade, info = mo.construir_obstaculos_por_visao(
            sim, floor, excluir=excluir
        )

        param["occ_grade"] = grade
        param["occ_info"] = info

        ocupadas = int(np.count_nonzero(grade))

        print(
            "Grade de ocupação:",
            f"{grade.shape[1]}x{grade.shape[0]} células,",
            f"{ocupadas} ocupadas,",
            f"célula={info['passo']*100:.1f} cm,",
            f"{len(pontos)} pontos de obstáculo",
        )

        if len(pontos) >= 4:
            return pontos

        print("Grade de ocupação vazia; usando scan por bounding box.")

    except Exception as exc:
        print("Falha na grade de ocupação (", exc, "); usando bounding box.")

    return create_static_obstacles()


def preparar_mapa_navegacao():
    """
    Abre uma janela do mapa sem travar o robô.
    Mostra obstáculos, robô, goal, rota planejada e trajetória real.
    """

    plt.ion()

    fig, ax = plt.subplots(figsize=(8, 8))

    param["mapa_fig"] = fig
    param["mapa_ax"] = ax

    ax.set_title("Mapa de Ocupação + Rota do Robô")
    ax.set_xlabel("X do mundo")
    ax.set_ylabel("Y do mundo")
    ax.grid(True)
    ax.axis("equal")

    info = param.get("occ_info")

    if info is not None:
        centro = info["centro"]
        tamanho = info["tamanho"]

        cx, cy = centro

        ax.set_xlim(cx - tamanho / 2.0, cx + tamanho / 2.0)
        ax.set_ylim(cy - tamanho / 2.0, cy + tamanho / 2.0)

    else:
        ax.set_xlim(-2.7, 2.7)
        ax.set_ylim(-2.7, 2.7)

    obstaculos = param.get("static_obstacles")

    if obstaculos is not None and len(obstaculos) > 0:
        obstaculos = np.array(obstaculos)

        ax.scatter(
            obstaculos[:, 0],
            obstaculos[:, 1],
            s=8,
            c="black",
            marker="s",
            label="Obstáculos"
        )

    caminho = param.get("global_path")

    if caminho is not None and len(caminho) > 0:
        caminho = np.array(caminho)

        ax.plot(
            caminho[:, 0],
            caminho[:, 1],
            "b-",
            linewidth=2.5,
            label="Rota planejada A*"
        )

    key_points = param.get("key_points")

    if key_points is not None and len(key_points) > 0:
        key_points = np.array(key_points)

        ax.plot(
            key_points[:, 0],
            key_points[:, 1],
            "yx",
            markersize=9,
            markeredgewidth=2,
            label="Pontos-chave"
        )

    robot_plot, = ax.plot(
        param["x"][0],
        param["x"][1],
        "go",
        markersize=11,
        label="Robô"
    )

    goal_plot, = ax.plot(
        param["goal_coords"][0],
        param["goal_coords"][1],
        "ro",
        markersize=11,
        label="Goal"
    )

    traj_plot, = ax.plot(
        [param["x"][0]],
        [param["x"][1]],
        "m-",
        linewidth=2.5,
        label="Trajetória real"
    )

    param["plot_robot"] = robot_plot
    param["plot_goal"] = goal_plot
    param["plot_traj"] = traj_plot
    param["trajetoria_real"] = [(float(param["x"][0]), float(param["x"][1]))]

    ax.legend(loc="best")

    plt.tight_layout()

    plt.savefig("captura_mapa_com_rota.png", dpi=180)
    print("[OK] Imagem inicial salva como: captura_mapa_com_rota.png")

    plt.show(block=False)
    plt.pause(0.1)

    print("[OK] Janela do mapa aberta sem travar o robô.")


def atualizar_mapa_navegacao():
    """
    Atualiza a posição do robô e desenha a trajetória real.
    """

    if "mapa_fig" not in param:
        return

    try:
        if not plt.fignum_exists(param["mapa_fig"].number):
            return
    except Exception:
        return

    if param.get("step_count", 0) % 2 != 0:
        return

    x = param["x"]

    param["trajetoria_real"].append((float(x[0]), float(x[1])))

    traj = np.array(param["trajetoria_real"])

    param["plot_robot"].set_data([x[0]], [x[1]])

    param["plot_goal"].set_data(
        [param["goal_coords"][0]],
        [param["goal_coords"][1]]
    )

    param["plot_traj"].set_data(traj[:, 0], traj[:, 1])

    param["mapa_fig"].canvas.draw_idle()
    plt.pause(0.001)


def init():
    param["motorRight"] = get_required_object("/MOTOR_DIREITO")
    param["motorLeft"] = get_required_object("/MOTOR_ESQUERDO")

    param["robot"] = sim.getObjectParent(param["motorRight"])

    param["goal"] = get_required_object("/Target")

    param["sensors"] = [
        get_required_object("/SENSOR_MEIO"),
        get_required_object("/SENSOR_DIAG_DIREITO"),
        get_required_object("/SENSOR_DIAG_ESQUERDO"),
        get_required_object("/SENSOR_DIREITO"),
        get_required_object("/SENSOR_ESQUERDO"),
    ]

    robot_pos = sim.getObjectPosition(param["robot"], -1)
    robot_ori = sim.getObjectOrientation(param["robot"], -1)

    param["robot_z"] = robot_pos[2]

    param["robot_roll"] = robot_ori[0]
    param["robot_pitch"] = robot_ori[1]

    medir_heading_offset()

    param["x"] = get_robot_state()

    goal_pos = sim.getObjectPosition(param["goal"], -1)

    param["goal_coords"] = [goal_pos[0], goal_pos[1]]

    param["static_obstacles"] = obter_obstaculos_estaticos()

    param["step_count"] = 0

    build_global_path()

    preparar_mapa_navegacao()

    print("Obstaculos estaticos:", len(param["static_obstacles"]))

    print(
        "Estado inicial:",
        [round(float(v), 3) for v in param["x"]]
    )

    print(
        "Goal:",
        [round(float(v), 3) for v in param["goal_coords"]]
    )


def replanejar():
    build_global_path()

    print(
        "Replanejado a partir de",
        [round(float(param["x"][0]), 2), round(float(param["x"][1]), 2)]
    )


def comando_recuperacao(x, target):
    ang = math.atan2(target[1] - x[1], target[0] - x[0])
    turn = normalize_angle(ang - x[2])
    w = max(min(1.2 * turn, dwa.max_yaw_rate), -dwa.max_yaw_rate)

    fase = param.get("_recovery", 0)

    if fase > 22:
        return [-0.08, 0.3 * w]

    if abs(turn) > 0.30:
        return [0.0, 0.9 if turn >= 0.0 else -0.9]

    return [0.12, 0.5 * w]


def loop():
    if param["goal"] is not None:
        goal_pos = sim.getObjectPosition(param["goal"], -1)
        param["goal_coords"] = [goal_pos[0], goal_pos[1]]

    x = param["x"]

    obstacles = get_obstacles(x)

    current_target = get_path_target(x)

    ref = param.get("_stuck_ref")

    if ref is None or math.hypot(x[0] - ref[0], x[1] - ref[1]) > 0.06:
        param["_stuck_ref"] = (float(x[0]), float(x[1]))
        param["_stuck_steps"] = 0
    else:
        param["_stuck_steps"] = param.get("_stuck_steps", 0) + 1

    if param.get("_recovery", 0) > 0:
        u = comando_recuperacao(x, current_target)
        param["_recovery"] -= 1
    else:
        u, predicted_trajectory = dwa.plan(
            x[0:3],
            x[3],
            x[4],
            current_target,
            obstacles,
        )

        if param.get("_stuck_steps", 0) >= 30:
            print("Robô preso -> replanejando e iniciando recuperação")
            replanejar()
            param["_recovery"] = 35
            param["_stuck_steps"] = 0

    x = robot_motion(u)

    param["x"] = x

    param["step_count"] += 1

    atualizar_mapa_navegacao()

    dist_goal = math.hypot(
        x[0] - param["goal_coords"][0],
        x[1] - param["goal_coords"][1],
    )

    if param["step_count"] % 10 == 0:
        print(
            "step",
            param["step_count"],
            "dist",
            round(dist_goal, 2),
            "pos",
            [round(float(x[0]), 2), round(float(x[1]), 2)],
            "u",
            [round(float(u[0]), 2), round(float(u[1]), 2)],
            "wp",
            param.get("path_index", 0),
            "obs",
            len(obstacles),
        )

    if dist_goal <= 0.20:
        print("GOAL ATINGIDO!")
        return True

    return False


if __name__ == "__main__":
    print("Iniciando simulacao...")

    sim.setStepping(True)

    sim.startSimulation()

    try:
        init()

        while True:
            if loop():
                break

    except KeyboardInterrupt:
        print("Parado pelo usuario.")

    finally:
        if "motorRight" in param and "motorLeft" in param:
            sim.setJointTargetVelocity(param["motorRight"], 0.0)
            sim.setJointTargetVelocity(param["motorLeft"], 0.0)

        sim.stopSimulation()

        print("Simulacao encerrada.")