import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

import \
    cubic_spline_planner

road_width = 10  #meters
obs_length = 5   #meters
obs_width = 2    #meters
dt = 0.2
SIM_LOOP = 5

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt

def generate_target_course(x, y):
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp
    
def main():
    print(__file__ + " start!!")
    
    wx = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    wy = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
    
 # initial state of obs vehicle
    obs_position = 10.0 # current position
    obs_vstart = 15.0 / 3.6  # current speed [m/s]
    obs_astart = 0.0  # current acceleration [m/ss]
    obs_vend = 15.0 / 3.6 #end velocity
    obs_aend = 0.0 #end acc
    s0 = 0.0  # current course position
    
    for i in range(SIM_LOOP):
        
        # s = obs_vstart*dt
        # obs_position += s
        obs_path = []
        obs_path = QuarticPolynomial(obs_position, obs_vstart, obs_vend, obs_astart, obs_aend, dt)
        x_t = obs_path.calc_point(dt)
        obs_position = x_t
        print(obs_position)
        # path = frenet_optimal_planning(
        #     csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)

        # s0 = path.s[1]
        # c_d = path.d[1]
        # c_d_d = path.d_d[1]
        # c_d_dd = path.d_dd[1]
        # c_speed = path.s_d[1]
        # c_accel = path.s_dd[1]

        # if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
        #     print("Goal")
        #     break

        if show_animation:  # pragma: no cover
            plt.cla()
            # plt.plot(obs_position,0, "*")
            # for stopping simulation with the esc key.
            # plt.gcf().canvas.mpl_connect(
            #     'key_release_event',
            #     lambda event: [exit(0) if event.key == 'escape' else None])
            # plt.plot(tx, ty)
            # plt.plot(ob[:, 0], ob[:, 1], "xk")
            # plt.plot(path.x[1:], path.y[1:], "-or")
            # plt.plot(path.x[1], path.y[1], "vc")
            # plt.xlim(path.x[1] - area, path.x[1] + area)
            # plt.ylim(path.y[1] - area, path.y[1] + area)
            # plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            # plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()