
import os
import time
from collections import OrderedDict

import numpy as np
import matplotlib.pyplot as plt
import scipy.io
from scipy.spatial.transform import Rotation
from ament_index_python import get_package_share_directory

from pnc_ros.atlas_state_publisher import AtlasStatePublisher

class Interpolation():
    def __init__(self, t0, tf, ts):
        self.t0 = t0
        self.tf = tf
        self.ts = ts

    def interpolate(self, t, timeseries):
        t = np.min([np.max([t, self.t0]).item(), self.tf.item()]).item()
        index = int(np.min([np.floor(((t-self.t0) - np.remainder((t-self.t0), self.ts))/self.ts + 0.0001).item(), len(timeseries) - 1]).item())
        return timeseries[index].eval(t)

class Spline():
    def __init__(self, a=0, b=0, c=0, d=0):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def eval(self, t):
        return self.a*t**3 + self.b*t**2 + self.c*t + self.d

    def __str__(self):
        return str(self.a) + "*t^3 + " + str(self.b) + "*t^2 + " + str(self.c) + "*t + " + str(self.d)

class Line():
    def __init__(self, m=0, b=0):
        self.m = m
        self.b = b

    def eval(self, t):
        return self.m*t+self.b

    def __str__(self):
        return str(self.m) + "*t + " + str(self.b)

if __name__ == "__main__":

    JointNameLookup = [
        "back_bkz",
        "back_bky",
        "back_bkx",
        "l_arm_shz",
        "l_arm_shx",
        "l_arm_ely",
        "l_arm_elx",
        "l_arm_wry",
        "l_arm_wrx",
        "l_arm_wry2",
        "neck_ry",
        "r_arm_shz",
        "r_arm_shx",
        "r_arm_ely",
        "r_arm_elx",
        "r_arm_wry",
        "r_arm_wrx",
        "r_arm_wry2",
        "l_leg_hpz",
        "l_leg_hpx",
        "l_leg_hpy",
        "l_leg_kny",
        "l_leg_aky",
        "l_leg_akx",
        "r_leg_hpz",
        "r_leg_hpx",
        "r_leg_hpy",
        "r_leg_kny",
        "r_leg_aky",
        "r_leg_akx"]

    traject_file = os.path.join(get_package_share_directory("pnc_ros"), "resource", "output_interpolation.mat")
    output = scipy.io.loadmat(traject_file)

    num_joints = len(JointNameLookup)
    num_states = np.shape(output.get("spline_segments"))[0]
    num_splines = np.shape(output.get("spline_segments"))[1]

    q_offset = 0
    q_dot_offset = num_joints
    r_offset = q_dot_offset + num_joints
    r_dot_offset = r_offset + 3
    r_ddot_offset = r_dot_offset + 3
    h_offset = r_ddot_offset + 3
    h_dot_offset = h_offset + 3
    c1_offset = h_dot_offset + 3
    c2_offset = c1_offset + 3
    f1_offset = c2_offset + 3
    f2_offset = f1_offset + 3
    tht_offset = f2_offset + 3

    num_states = tht_offset + 3

    offset_lookup = {
        "q_offset": q_offset,
        "q_dot_offset": q_dot_offset,
        "r_offset": r_offset,
        "r_dot_offset": r_dot_offset,
        "r_ddot_offset": r_ddot_offset,
        "h_offset": h_offset,
        "h_dot_offset": h_dot_offset,
        "c1_offset": c1_offset,
        "c2_offset": c2_offset,
        "f1_offset": f1_offset,
        "f2_offset": f2_offset,
        "tht_offset" : tht_offset,
    }

    t_total = output.get("t_total")
    timestep = output.get("timestep")
    t_knot = np.arange(0, t_total + timestep, timestep)
    t_cont = np.arange(0, t_total, timestep/1000)

    state_trajectories_cubic = OrderedDict()
    state_trajectories_linear = OrderedDict()

    spline_segments = output.get("spline_segments")
    line_segments = output.get("line_segments")
    pelvis_segments = output.get("pelvis_segments")

    for offset in ["q_offset", "q_dot_offset"]:
        for i in range(num_joints):
            spline_arr = []
            line_arr = []
            for col in range(num_splines):
                offset_index = offset_lookup.get(offset)
                spline_arr.append(Spline(
                    spline_segments[offset_index + i, col][0].item(), 
                    spline_segments[offset_index + i, col][1].item(), 
                    spline_segments[offset_index + i, col][2].item(), 
                    spline_segments[offset_index + i, col][3].item()))

                # line_arr.append(Line(
                #     line_segments[offset_index + i, col][0].item(),
                #     line_segments[offset_index + i, col][1].item(),
                # ))

            key = JointNameLookup[i] if (offset == "q_offset") else JointNameLookup[i] +  "_vel"

            state_trajectories_cubic[key] = spline_arr
            # state_trajectories_linear[key] = line_arr
        

    dirs = ["x", "y", "z"]
    for offset in ["r_offset", "r_dot_offset", "r_ddot_offset", "h_offset", "h_dot_offset", "f1_offset", "f2_offset"]:
        for i in range(3):
            dir = dirs[i]
            spline_arr = []
            # line_arr = []
            for col in range(num_splines):
                offset_index = offset_lookup.get(offset)
                spline_arr.append(Spline(
                    spline_segments[offset_index + i, col][0].item(), 
                    spline_segments[offset_index + i, col][1].item(), 
                    spline_segments[offset_index + i, col][2].item(), 
                    spline_segments[offset_index + i, col][3].item()
                ))

                # line_arr.append(Line(
                #     line_segments[offset_index + i, col][0].item(),
                #     line_segments[offset_index + i, col][1].item(),
                # ))

            key = offset[0:-6] + dir

            state_trajectories_cubic[key] = spline_arr
            # state_trajectories_linear[key] = line_arr

    dirs = ["x", "y", "z"]
    for offset in ["tht_offset"]:
        for i in range(3):
            dir = dirs[i]
            spline_arr = []
            # line_arr = []
            for col in range(num_splines):
                offset_index = offset_lookup.get(offset)
                spline_arr.append(Spline(
                    spline_segments[offset_index + i, col][0].item(), 
                    spline_segments[offset_index + i, col][1].item(), 
                    spline_segments[offset_index + i, col][2].item(), 
                    spline_segments[offset_index + i, col][3].item()
                ))

                # line_arr.append(Line(
                #     line_segments[offset_index + i, col][0].item(),
                #     line_segments[offset_index + i, col][1].item(),
                # ))

            key = offset[0:-6] + dir

            state_trajectories_cubic[key] = spline_arr
            # state_trajectories_linear[key] = line_arr

    dirs = ["x", "y", "z"]
    for i in range(3):
        dir = dirs[i]
        spline_arr = []
        line_arr = []

        for col in range(num_splines):
            spline_arr.append(Spline(
                pelvis_segments[i, col][0].item(), 
                pelvis_segments[i, col][1].item(), 
                pelvis_segments[i, col][2].item(), 
                pelvis_segments[i, col][3].item()
            ))

        key = "pelvis_pos_" + dir

        state_trajectories_cubic[key] = spline_arr

    Interpolator = Interpolation(0, t_total, timestep)

    cubic_approx = np.zeros([1, len(t_cont)])
    linear_approx = np.zeros([1, len(t_cont)])
    knots = np.zeros([1, len(t_knot)])

    for i in range(len(t_cont)):
        cubic_approx[0, i] = Interpolator.interpolate(t_cont[i], state_trajectories_cubic["pelvis_pos_x"])
        # linear_approx[0, i] = Interpolator.interpolate(t_cont[i], state_trajectories_linear["pelvis_pos_x"])

    for i in range(len(t_knot)):
        knots[0, i] = Interpolator.interpolate(t_knot[i], state_trajectories_cubic["pelvis_pos_x"])

    # plt.figure()
    # plt.plot(t_cont, cubic_approx[0, :])
    # plt.plot(t_cont, linear_approx[0, :], linestyle='--')
    # plt.scatter(t_knot, knots, s=10, c="k", zorder=10)
    # plt.axvline(x=t_knot[output.get("windup_end").item()], color='k', linestyle='--')
    # plt.axvline(x=t_knot[output.get("takeoff_end").item()], color='k', linestyle='--')
    # plt.axvline(x=t_knot[output.get("landing_start").item()], color='k', linestyle='--')

    # plt.show()

    start_time = time.monotonic()
    dt = 1/40
    speed_scale = 3
    ros_state_pub = AtlasStatePublisher()
    t = 0
    offset = 2
    while(1):
        t = (time.monotonic() - start_time - offset)/speed_scale

        robot_state = OrderedDict()
        robot_state["joint_pos"] = OrderedDict()
        robot_state["joint_vel"] = OrderedDict()

        for q_name in JointNameLookup:
            robot_state["joint_pos"][q_name] = Interpolator.interpolate(t, state_trajectories_cubic[q_name])
            robot_state["joint_vel"][q_name] = Interpolator.interpolate(t, state_trajectories_cubic[q_name + "_vel"])

        robot_state["base_joint_pos"] = np.array([
            Interpolator.interpolate(t, state_trajectories_cubic["pelvis_pos_x"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["pelvis_pos_y"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["pelvis_pos_z"])
            ])

        robot_state["base_joint_lin_vel"] = np.array([
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_x"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_y"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_z"])
            ])

        # robot_state["base_joint_quat"] = np.array([0, 0, 0, 1])
        tht_x = Interpolator.interpolate(t, state_trajectories_cubic["tht_x"])
        tht_y = Interpolator.interpolate(t, state_trajectories_cubic["tht_y"])
        tht_z = Interpolator.interpolate(t, state_trajectories_cubic["tht_z"])
        # Create rotation object from Euler tht
        rot = Rotation.from_euler('xyz', [tht_x, tht_y, tht_z])
        #Convert to quaternions and print
        rot_quat = rot.as_quat()

        robot_state["base_joint_quat"] = np.array([
            rot_quat[0],
            rot_quat[1],
            rot_quat[2],
            rot_quat[3]
       ])

        robot_state["base_joint_ang_vel"] = np.array([
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_x"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_y"]), 
            Interpolator.interpolate(t, state_trajectories_cubic["r_dot_z"])
            ])

        robot_state["base_com_pos"] = np.array([0, 0, 0])
        robot_state["base_com_quat"] = np.array([0, 0, 0, 1])
        robot_state["base_com_lin_vel"] = np.array([0, 0, 0])
        robot_state["base_com_ang_vel"] = np.array([0, 0, 0])

        ros_state_pub.update_robot_model(robot_state)
        ros_state_pub.publish_robot_state()

        time.sleep(dt)