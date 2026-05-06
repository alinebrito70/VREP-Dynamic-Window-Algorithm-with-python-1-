import sys
import math
import time
import numpy as np

COPPELIASIM_ZMQ_PATH = r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src"

if COPPELIASIM_ZMQ_PATH not in sys.path:
    sys.path.append(COPPELIASIM_ZMQ_PATH)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Config:
    def __init__(self):
        self.max_speed = 0.035
        self.min_speed = 0.015
        self.max_yaw_rate = 0.7

        self.v_resolution = 0.005
        self.yaw_rate_resolution = 0.15

        self.dt = 0.05
        self.predict_time = 1.0

        self.to_goal_cost_gain = 1.2
        self.speed_cost_gain = 0.3
        self.obstacle_cost_gain = 5.0

        self.robot_radius = 0.22
        self.safe_distance = 0.65
        self.emergency_distance = 0.42

        self.wheel_radius = 0.036
        self.axle_half = 0.12

        # Limites virtuais do chão
        self.x_min = -2.0
        self.x_max = 2.0
        self.y_min = -2.0
        self.y_max = 2.0
        self.edge_safe = 0.35


def near_edge(x, y, config):
    if x < config.x_min + config.edge_safe:
        return "left"
    if x > config.x_max - config.edge_safe:
        return "right"
    if y < config.y_min + config.edge_safe:
        return "bottom"
    if y > config.y_max - config.edge_safe:
        return "top"
    return None


def get_object(sim, names):
    for name in names:
        try:
            return sim.getObject(name)
        except Exception:
            pass
    raise Exception(f"Objeto não encontrado. Tentei estes nomes: {names}")


def try_get_object(sim, name):
    try:
        return sim.getObject(name)
    except Exception:
        return None


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_pose(sim, body_handle):
    pos = sim.getObjectPosition(body_handle, -1)
    ori = sim.getObjectOrientation(body_handle, -1)
    return pos[0], pos[1], ori[2]


def motion(x, u, dt):
    x = np.array(x, dtype=float)
    x[2] += u[1] * dt
    x[2] = normalize_angle(x[2])
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x


def predict_trajectory(x_init, v, w, config):
    x = np.array(x_init, dtype=float)
    trajectory = np.array([x])

    t = 0.0
    while t <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        trajectory = np.vstack((trajectory, x))
        t += config.dt

    return trajectory


def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    target_angle = math.atan2(dy, dx)
    error_angle = normalize_angle(target_angle - trajectory[-1, 2])
    return abs(error_angle)


def calc_obstacle_cost(trajectory, obstacles, config):
    if obstacles.size == 0:
        return 0.0

    ox = obstacles[:, 0]
    oy = obstacles[:, 1]

    dx = trajectory[:, 0][:, None] - ox[None, :]
    dy = trajectory[:, 1][:, None] - oy[None, :]

    distances = np.hypot(dx, dy)

    if np.any(distances <= config.robot_radius):
        return float("inf")

    min_distance = np.min(distances)

    if min_distance <= 0:
        return float("inf")

    return 1.0 / min_distance


def dwa_control(x, config, goal, obstacles):
    best_u = [0.02, 0.0]
    best_cost = float("inf")
    best_traj = np.array([x])

    v_values = np.arange(config.min_speed, config.max_speed + 1e-6, config.v_resolution)
    w_values = np.arange(-config.max_yaw_rate, config.max_yaw_rate + 1e-6, config.yaw_rate_resolution)

    for v in v_values:
        for w in w_values:
            traj = predict_trajectory(x, v, w, config)

            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - v)
            obstacle_cost = config.obstacle_cost_gain * calc_obstacle_cost(traj, obstacles, config)

            final_cost = to_goal_cost + speed_cost + obstacle_cost

            if final_cost < best_cost:
                best_cost = final_cost
                best_u = [v, w]
                best_traj = traj

    if not np.isfinite(best_cost):
        best_u = [0.00, 0.5]

    return best_u, best_traj


def vw_to_wheel_speeds(v, w, config):
    wr = (v + config.axle_half * w) / config.wheel_radius
    wl = (v - config.axle_half * w) / config.wheel_radius
    return wr, wl


def parse_sensor_data(data):
    if not isinstance(data, (list, tuple)):
        return None

    if len(data) == 0:
        return None

    detected = data[0]

    if detected <= 0:
        return None

    if len(data) >= 3 and isinstance(data[2], (list, tuple)):
        return data[2]

    if len(data) >= 2 and isinstance(data[1], (list, tuple)):
        return data[1]

    if len(data) >= 2 and isinstance(data[1], (int, float)):
        return [float(data[1]), 0.0, 0.0]

    return None


def local_point_to_world_2d(sim, sensor, local_point):
    s_pos = sim.getObjectPosition(sensor, -1)
    s_ori = sim.getObjectOrientation(sensor, -1)

    distance = math.sqrt(
        local_point[0] ** 2 +
        local_point[1] ** 2 +
        local_point[2] ** 2
    )

    ox = s_pos[0] + distance * math.cos(s_ori[2])
    oy = s_pos[1] + distance * math.sin(s_ori[2])

    return ox, oy


def read_sensor_obstacles(sim, sensor_handles):
    obstacles = []

    for sensor in sensor_handles:
        try:
            data = sim.readProximitySensor(sensor)
            local_point = parse_sensor_data(data)

            if local_point is not None:
                ox, oy = local_point_to_world_2d(sim, sensor, local_point)
                obstacles.append([ox, oy])

        except Exception:
            pass

    return obstacles


def get_named_obstacle_handles(sim):
    handles = []
    possible_names = []

    for i in range(1, 20):
        possible_names.append(f"/OBSTACULO{i}")
        possible_names.append(f"OBSTACULO{i}")
        possible_names.append(f"/OBSTACULO0{i}")
        possible_names.append(f"OBSTACULO0{i}")

    for i in range(20):
        possible_names.append(f"/Cuboid[{i}]")
        possible_names.append(f"Cuboid[{i}]")

    for name in possible_names:
        h = try_get_object(sim, name)

        if h is not None and h not in handles:
            handles.append(h)

    return handles


def read_named_obstacles(sim, obstacle_handles):
    obstacles = []

    for h in obstacle_handles:
        try:
            pos = sim.getObjectPosition(h, -1)
            obstacles.append([pos[0], pos[1]])
        except Exception:
            pass

    return obstacles


def read_all_obstacles(sim, sensor_handles, obstacle_handles):
    obstacles = []
    obstacles.extend(read_sensor_obstacles(sim, sensor_handles))
    obstacles.extend(read_named_obstacles(sim, obstacle_handles))

    if len(obstacles) == 0:
        return np.empty((0, 2))

    return np.array(obstacles, dtype=float)


def closest_front_obstacle(x, y, yaw, obstacles):
    if obstacles.size == 0:
        return None

    closest = None
    min_dist = float("inf")

    for ox, oy in obstacles:
        dx = ox - x
        dy = oy - y

        dist = math.hypot(dx, dy)
        angle_to_obstacle = math.atan2(dy, dx)
        relative_angle = normalize_angle(angle_to_obstacle - yaw)

        is_front = abs(relative_angle) < math.radians(90)

        if is_front and dist < min_dist:
            min_dist = dist
            closest = {
                "dist": dist,
                "relative_angle": relative_angle,
                "x": ox,
                "y": oy
            }

    return closest


def stop_if_running(sim):
    try:
        state = sim.getSimulationState()

        if state != sim.simulation_stopped:
            sim.stopSimulation()
            time.sleep(1.0)

    except Exception:
        pass


def main():
    print("Iniciando conexão com o CoppeliaSim...")

    client = RemoteAPIClient(host="127.0.0.1", port=23000)
    sim = client.getObject("sim")

    print("Conectado ao CoppeliaSim!")

    motor_right = get_object(sim, [
        "/MOTOR_DIREITO",
        "MOTOR_DIREITO",
        "/Pioneer_p3dx_rightMotor",
        "Pioneer_p3dx_rightMotor",
        "/Pioneer_p3dx/rightMotor",
        "/rightMotor",
        "rightMotor"
    ])

    motor_left = get_object(sim, [
        "/MOTOR_ESQUERDO",
        "MOTOR_ESQUERDO",
        "/Pioneer_p3dx_leftMotor",
        "Pioneer_p3dx_leftMotor",
        "/Pioneer_p3dx/leftMotor",
        "/leftMotor",
        "leftMotor"
    ])

    body = get_object(sim, [
        "/Cuboid[0]",
        "Cuboid[0]",
        "/Pioneer_p3dx",
        "Pioneer_p3dx",
        "/Cuboid",
        "Cuboid"
    ])

    target = try_get_object(sim, "/Target")
    if target is None:
        target = try_get_object(sim, "Target")

    sensor_handles = []

    sensor_names = [
        ["/SENSOR_MEIO", "SENSOR_MEIO"],
        ["/SENSOR_DIREITO", "SENSOR_DIREITO"],
        ["/SENSOR_DIAG_DIREITO", "SENSOR_DIAG_DIREITO"],
        ["/SENSOR_ESQUERDO", "SENSOR_ESQUERDO"],
        ["/SENSOR_DIAG_ESQUERDO", "SENSOR_DIAG_ESQUERDO"],
    ]

    for names in sensor_names:
        try:
            sensor_handles.append(get_object(sim, names))
        except Exception:
            pass

    obstacle_handles = get_named_obstacle_handles(sim)

    print(f"Sensores encontrados: {len(sensor_handles)}")
    print(f"Obstáculos encontrados: {len(obstacle_handles)}")

    config = Config()

    stop_if_running(sim)

    print("Iniciando simulação...")
    sim.startSimulation()
    time.sleep(1.0)

    try:
        config.dt = float(sim.getSimulationTimeStep())
    except Exception:
        config.dt = 0.05

    x0, y0, yaw0 = get_pose(sim, body)

    if target is not None:
        target_pos = sim.getObjectPosition(target, -1)
        goal = np.array([target_pos[0], target_pos[1]])
        print(f"Objetivo definido no Target: x={goal[0]:.3f}, y={goal[1]:.3f}")
    else:
        goal = np.array([
            x0 + 2.0 * math.cos(yaw0),
            y0 + 2.0 * math.sin(yaw0)
        ])
        print(f"Target não encontrado. Objetivo criado à frente: x={goal[0]:.3f}, y={goal[1]:.3f}")

    print("Rodando Janela Dinâmica pelo VSCode e visualizando no CoppeliaSim...")

    current_v = 0.0
    current_w = 0.0

    avoid_mode = False
    avoid_counter = 0
    avoid_direction = 1

    try:
        for step in range(800):
            x, y, yaw = get_pose(sim, body)

            edge = near_edge(x, y, config)

            if edge is not None:
                print(f"PERTO DA BORDA: {edge}, corrigindo rota")

                v = -0.01
                w = 0.6

                wr, wl = vw_to_wheel_speeds(v, w, config)

                wr = max(min(wr, 1.0), -1.0)
                wl = max(min(wl, 1.0), -1.0)

                sim.setJointTargetVelocity(motor_right, wr)
                sim.setJointTargetVelocity(motor_left, wl)

                time.sleep(0.3)

                current_v = v
                current_w = w

                continue

            state = np.array([x, y, yaw, current_v, current_w], dtype=float)

            obstacles = read_all_obstacles(sim, sensor_handles, obstacle_handles)
            front_obst = closest_front_obstacle(x, y, yaw, obstacles)

            if front_obst is not None and front_obst["dist"] < config.safe_distance and not avoid_mode:
                avoid_mode = True
                avoid_counter = 70

                if front_obst["relative_angle"] >= 0:
                    avoid_direction = -1
                else:
                    avoid_direction = 1

            if avoid_mode:
                if avoid_counter > 40:
                    v = 0.00
                    w = 0.55 * avoid_direction
                    mode_text = "DESVIANDO: girando"
                else:
                    v = 0.025
                    w = 0.22 * avoid_direction
                    mode_text = "DESVIANDO: contornando"

                avoid_counter -= 1

                if avoid_counter <= 0:
                    avoid_mode = False

                u = [v, w]

            else:
                u, predicted_trajectory = dwa_control(state, config, goal, obstacles)

                if u[0] < 0.015:
                    u[0] = 0.015

                if abs(u[1]) > 0.30:
                    u[1] = 0.30 if u[1] > 0 else -0.30

                mode_text = "DWA normal"

            wr, wl = vw_to_wheel_speeds(u[0], u[1], config)

            wr = max(min(wr, 1.0), -1.0)
            wl = max(min(wl, 1.0), -1.0)

            sim.setJointTargetVelocity(motor_right, wr)
            sim.setJointTargetVelocity(motor_left, wl)

            current_v = u[0]
            current_w = u[1]

            dist_goal = math.hypot(goal[0] - x, goal[1] - y)

            if front_obst is None:
                obst_text = "sem obstaculo frontal"
            else:
                obst_text = f"obst_frente={front_obst['dist']:.2f}"

            print(
                f"passo={step:03d} | "
                f"{mode_text} | "
                f"x={x:.3f} y={y:.3f} yaw={yaw:.2f} | "
                f"v={u[0]:.3f} w={u[1]:.2f} | "
                f"wr={wr:.2f} wl={wl:.2f} | "
                f"obst={len(obstacles)} | "
                f"{obst_text} | "
                f"dist_goal={dist_goal:.3f}"
            )

            if dist_goal < 0.18:
                print("Objetivo alcançado!")
                break

            time.sleep(config.dt)

    finally:
        print("Parando robô...")

        sim.setJointTargetVelocity(motor_right, 0.0)
        sim.setJointTargetVelocity(motor_left, 0.0)

        time.sleep(0.5)

        sim.stopSimulation()
        print("Simulação encerrada.")


if __name__ == "__main__":
    main()