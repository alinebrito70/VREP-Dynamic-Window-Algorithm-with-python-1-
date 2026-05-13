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
        self.max_speed = 0.25
        self.min_speed = 0.0

        self.max_yaw_rate = 90.0 * math.pi / 180.0
        self.max_accel = 0.25
        self.max_delta_yaw_rate = 120.0 * math.pi / 180.0

        self.v_resolution = 0.01
        self.yaw_rate_resolution = 3.0 * math.pi / 180.0

        self.dt = 0.10
        self.predict_time = 2.0

        self.to_goal_cost_gain = 3.0
        self.angle_cost_gain = 4.0
        self.speed_cost_gain = 0.2
        self.obstacle_cost_gain = 2.0

        self.robot_radius = 0.28
        self.safety_margin = 0.20
        self.stop_goal_distance = 0.50

        self.wheel_radius = 0.0975 / 2
        self.axle_half = 0.165

        self.brake_accel = 0.25

        self.x_min = -2.5
        self.x_max = 2.5
        self.y_min = -2.5
        self.y_max = 2.5

        self.left_sign = 1
        self.right_sign = 1
        self.rotation_sign = 1


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def get_obj(sim, name):
    try:
        h = sim.getObject(name)
        print("Encontrado:", name)
        return h
    except Exception:
        print("NÃO encontrei:", name)
        return None


def stop_robot(sim, left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def get_yaw(sim, sensor_front, robot):
    try:
        ori = sim.getObjectOrientation(sensor_front, -1)
        return ori[2]
    except Exception:
        ori = sim.getObjectOrientation(robot, -1)
        return ori[2]


def motion(x, u, dt):
    v = u[0]
    w = u[1]

    x = np.array(x, dtype=float)

    x[2] = normalize_angle(x[2] + w * dt)
    x[0] += v * math.cos(x[2]) * dt
    x[1] += v * math.sin(x[2]) * dt
    x[3] = v
    x[4] = w

    return x


def calc_dynamic_window(x, config):
    Vs = [
        config.min_speed,
        config.max_speed,
        -config.max_yaw_rate,
        config.max_yaw_rate
    ]

    Vd = [
        x[3] - config.max_accel * config.dt,
        x[3] + config.max_accel * config.dt,
        x[4] - config.max_delta_yaw_rate * config.dt,
        x[4] + config.max_delta_yaw_rate * config.dt
    ]

    return [
        max(Vs[0], Vd[0]),
        min(Vs[1], Vd[1]),
        max(Vs[2], Vd[2]),
        min(Vs[3], Vd[3])
    ]


def predict_trajectory(x_init, v, w, config):
    x = np.array(x_init, dtype=float)
    traj = [x.copy()]

    t = 0.0

    while t <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        traj.append(x.copy())
        t += config.dt

    return np.array(traj)


def stopping_distance(v, config):
    return (v * v) / (2.0 * config.brake_accel)


def calc_goal_cost(traj, goal):
    dx = goal[0] - traj[-1, 0]
    dy = goal[1] - traj[-1, 1]
    return math.hypot(dx, dy)


def calc_angle_cost(traj, goal):
    dx = goal[0] - traj[-1, 0]
    dy = goal[1] - traj[-1, 1]

    target_angle = math.atan2(dy, dx)
    robot_angle = traj[-1, 2]

    return abs(normalize_angle(target_angle - robot_angle))


def calc_obstacle_cost(traj, obstacles, v, config):
    if obstacles is None or len(obstacles) == 0:
        return 0.0

    obstacles = np.array(obstacles, dtype=float)

    dx = traj[:, 0][:, None] - obstacles[:, 0][None, :]
    dy = traj[:, 1][:, None] - obstacles[:, 1][None, :]

    dist = np.hypot(dx, dy)

    min_dist = np.min(dist)

    safe_distance = config.robot_radius + config.safety_margin + stopping_distance(abs(v), config)

    if min_dist <= safe_distance:
        return float("inf")

    return 1.0 / min_dist


def dwa_control(x, config, goal, obstacles):
    dw = calc_dynamic_window(x, config)

    best_u = [0.0, 0.0]
    best_cost = float("inf")

    for v in np.arange(dw[0], dw[1] + config.v_resolution, config.v_resolution):
        for w in np.arange(dw[2], dw[3] + config.yaw_rate_resolution, config.yaw_rate_resolution):

            traj = predict_trajectory(x, v, w, config)

            obstacle_cost = calc_obstacle_cost(traj, obstacles, v, config)

            if obstacle_cost == float("inf"):
                continue

            goal_cost = config.to_goal_cost_gain * calc_goal_cost(traj, goal)
            angle_cost = config.angle_cost_gain * calc_angle_cost(traj, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - v)
            obstacle_cost = config.obstacle_cost_gain * obstacle_cost

            final_cost = goal_cost + angle_cost + speed_cost + obstacle_cost

            if final_cost < best_cost:
                best_cost = final_cost
                best_u = [v, w]

    if best_cost == float("inf"):
        return [0.0, 0.8]

    return best_u


def transform_point(matrix, point):
    x = matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3]
    y = matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7]

    return [x, y]


def get_sensor_obstacles(sim, sensors):
    obs = []

    for s in sensors:
        try:
            result = sim.readProximitySensor(s)

            detected = result[0]
            point = result[2]

            if detected > 0:
                matrix = sim.getObjectMatrix(s, -1)
                obs.append(transform_point(matrix, point))

        except Exception:
            pass

    return obs


def get_cuboid_obstacles(sim):
    obs = []

    cuboids = [
        "/Cuboid[1]",
        "/Cuboid[2]",
        "/Cuboid[3]",
        "/Cuboid[4]",
        "/Cuboid[5]",
        "/Cuboid[6]",
        "/Cuboid[7]",
        "/Cuboid[8]"
    ]

    for name in cuboids:
        try:
            obj = sim.getObject(name)
            pos = sim.getObjectPosition(obj, -1)

            for dx in np.arange(-0.40, 0.41, 0.10):
                for dy in np.arange(-0.40, 0.41, 0.10):
                    obs.append([pos[0] + dx, pos[1] + dy])

        except Exception:
            pass

    return obs


def get_wall_obstacles(config):
    obs = []

    for x in np.arange(config.x_min, config.x_max + 0.1, 0.10):
        obs.append([x, config.y_min])
        obs.append([x, config.y_max])

    for y in np.arange(config.y_min, config.y_max + 0.1, 0.10):
        obs.append([config.x_min, y])
        obs.append([config.x_max, y])

    return obs


def send_velocity(sim, left_motor, right_motor, v, w, config):
    w = w * config.rotation_sign

    right = (v + config.axle_half * w) / config.wheel_radius
    left = (v - config.axle_half * w) / config.wheel_radius

    sim.setJointTargetVelocity(right_motor, config.right_sign * right)
    sim.setJointTargetVelocity(left_motor, config.left_sign * left)


def calibrate_forward(sim, robot, sensor_front, left_motor, right_motor, config):
    print("Calibrando sentido para frente...")

    p0 = sim.getObjectPosition(robot, -1)
    yaw = get_yaw(sim, sensor_front, robot)

    send_velocity(sim, left_motor, right_motor, 0.08, 0.0, config)

    time.sleep(0.45)

    stop_robot(sim, left_motor, right_motor)

    time.sleep(0.2)

    p1 = sim.getObjectPosition(robot, -1)

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]

    frente_x = math.cos(yaw)
    frente_y = math.sin(yaw)

    produto = dx * frente_x + dy * frente_y

    if produto < 0:
        config.left_sign *= -1
        config.right_sign *= -1
        print("Motores invertidos automaticamente.")
    else:
        print("Sentido dos motores OK.")


def main():
    print("Conectando ao CoppeliaSim...")

    client = RemoteAPIClient("127.0.0.1", 23000)
    sim = client.getObject("sim")

    config = Config()

    motor_right = get_obj(sim, "/MOTOR_DIREITO")
    motor_left = get_obj(sim, "/MOTOR_ESQUERDO")
    robot = get_obj(sim, "/Cylinder")
    target = get_obj(sim, "/Target")

    sensor_front = get_obj(sim, "/SENSOR_MEIO")

    sensor_names = [
        "/SENSOR_MEIO",
        "/SENSOR_DIREITO",
        "/SENSOR_DIAG_DIREITO",
        "/SENSOR_ESQUERDO",
        "/SENSOR_DIAG_ESQUERDO"
    ]

    sensors = []

    for name in sensor_names:
        s = get_obj(sim, name)

        if s is not None:
            sensors.append(s)

    if motor_right is None or motor_left is None or robot is None or target is None:
        print("ERRO: motor, robô ou target não encontrado.")
        return

    sim.startSimulation()

    time.sleep(0.5)

    calibrate_forward(sim, robot, sensor_front, motor_left, motor_right, config)

    u_prev = [0.0, 0.0]

    try:
        while True:
            robot_pos = sim.getObjectPosition(robot, -1)
            target_pos = sim.getObjectPosition(target, -1)

            yaw = get_yaw(sim, sensor_front, robot)

            goal = np.array([target_pos[0], target_pos[1]])

            dist_goal = math.hypot(
                goal[0] - robot_pos[0],
                goal[1] - robot_pos[1]
            )

            if dist_goal <= config.stop_goal_distance:
                print("Chegou perto do target e parou antes de encostar.")
                stop_robot(sim, motor_left, motor_right)
                break

            x_state = np.array([
                robot_pos[0],
                robot_pos[1],
                yaw,
                u_prev[0],
                u_prev[1]
            ])

            obstacles = []
            obstacles.extend(get_sensor_obstacles(sim, sensors))
            obstacles.extend(get_cuboid_obstacles(sim))
            obstacles.extend(get_wall_obstacles(config))

            u = dwa_control(x_state, config, goal, obstacles)

            u_prev = u

            send_velocity(sim, motor_left, motor_right, u[0], u[1], config)

            print(
                f"dist_target={dist_goal:.2f} | "
                f"v={u[0]:.2f} | "
                f"w={u[1]:.2f} | "
                f"obst={len(obstacles)}"
            )

            time.sleep(config.dt)

    except KeyboardInterrupt:
        print("Interrompido.")

    finally:
        stop_robot(sim, motor_left, motor_right)
        sim.stopSimulation()
        print("Fim.")


if __name__ == "__main__":
    main()