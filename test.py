# Path: /mnt/data/frenet_overtake_demo.py

import numpy as np
import matplotlib.pyplot as plt
import copy
import math

# Import all necessary components from the original module
from FrenetOptimalTrajectory.frenet_optimal_trajectory import (
    frenet_optimal_planning,
    generate_target_course,
    SIM_LOOP,
    show_animation,
    DT,
)

# Update the function for dynamic obstacles
def create_dynamic_obstacle(initial_position, speed, duration):
    """Generate the trajectory of a moving obstacle."""
    trajectory = []
    for t in range(int(duration / DT)):
        x = initial_position[0] + speed * DT * t
        y = initial_position[1]
        trajectory.append([x, y])
    return np.array(trajectory)

def main_overtake():
    print("Starting overtaking simulation!")

    # Initial and waypoints for the road
    wx = [0.0, 10.0, 30.0, 60.0, 90.0]
    wy = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Obstacle Vehicle Dynamics
    ob_speed = 15.0 / 3.6  # Obstacle speed [m/s]
    ob_initial_pos = [20.0, 0.0]  # Initial obstacle position
    ob = create_dynamic_obstacle(ob_initial_pos, ob_speed, 20.0)  # Dynamic trajectory

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # Ego vehicle's initial state
    c_speed = 30.0 / 3.6  # Ego speed [m/s]
    c_accel = 0.0  # Ego acceleration [m/s^2]
    c_d = 0.0  # Ego lateral position [m]
    c_d_d = 0.0  # Ego lateral speed [m/s]
    c_d_dd = 0.0  # Ego lateral acceleration [m/s^2]
    s0 = 0.0  # Initial course position

    area = 30.0  # Animation area size

    for _ in range(SIM_LOOP):
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)

        # Advance simulation state
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_accel = path.s_dd[1]

        # Exit if goal is reached
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal Reached!")
            break

        if show_animation:
            plt.cla()
            plt.plot(tx, ty, label="Target Path")
            plt.plot(ob[:, 0], ob[:, 1], "xk", label="Obstacle Trajectory")
            plt.plot(path.x[1:], path.y[1:], "-or", label="Ego Path")
            plt.plot(path.x[1], path.y[1], "vc", label="Ego Vehicle")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title(f"Ego Speed [km/h]: {c_speed * 3.6:.1f}")
            plt.grid(True)
            plt.legend()
            plt.pause(0.0001)

    print("Overtaking Complete!")
    if show_animation:
        plt.show()

if __name__ == '__main__':
    main_overtake()
