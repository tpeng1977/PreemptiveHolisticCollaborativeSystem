import os
import copy
import sys
import time
import traceback
import traci
import numpy as np
from scipy.interpolate import CubicSpline
from tools import scale_series
import threading


def analyze_array(arr):
    if len(arr) < 2:
        return False, None, None, arr[0] if len(arr) > 0 else None, arr[0] if len(arr) > 0 else None
    diffs = np.diff(arr)
    is_strictly_increasing = np.all(diffs > 0)
    max_diff = np.max(diffs)
    min_diff = np.min(diffs)
    if min_diff <= 0:
        print("error")
        t_idx = np.where(diffs < 0)[0][0]
        print(t_idx)
    return is_strictly_increasing, max_diff, min_diff, np.min(arr), np.max(arr)


class TrajectoryMerge:
    def __init__(self, add_len=40):
        # Scheduled merging trajectories for the vehicles in the system.
        # Trajectories of the vehicles in the system.
        self.scheduled_trajectories = {}
        self.check_new_vehicle_lock = threading.Lock()
        #  Before the offset, the vehilces are in mainline or ramp line. After it, the vehicle are in merged lanes.
        self.m_offset = 1022.56 - add_len
        self.r_offset = 1001.47 - add_len
        # {"vehicle": vehicle_id, "enter_time": time} The vehicle_id and its corresponding time entering the list.
        # mainline list, ramp list and merged list.
        self.m_list, self.r_list, self.merge_list = [], [], []
        self.m_lanes_info = None
        self.r_lanes_info = None
        self.run = True
        # self.check_trajectories()

    def close(self):
        self.run = False
        time.sleep(2)

    def check_trajectories(self):
        def analyze_array(arr):
            if len(arr) < 2:
                return False, None, None, arr[0] if len(arr) > 0 else None, arr[0] if len(arr) > 0 else None
            diffs = np.diff(arr)
            is_strictly_increasing = np.all(diffs > 0)
            max_diff = np.max(diffs)
            min_diff = np.min(diffs)
            if min_diff <= 0:
                print("error")
                t_idx = np.where(diffs < 0)[0][0]
                print(t_idx)
            return is_strictly_increasing, max_diff, min_diff, np.min(arr), np.max(arr)

        def res_to_str(in_res):
            return f"strictly increasing: {str(in_res[0])}, max_diff: {in_res[1]:.3f}, min_diff: {in_res[2]:.3f}, min: {in_res[3]:.3f} max: {in_res[4]:.3f}"

        def checking():
            i = 0
            while True:
                i += 1
                try:
                    if i % 100 == 0:
                        print("\n ==>")
                        print(
                            f"Current Time: {traci.simulation.getTime()} len(m_list)={len(self.m_list)}, len(r_list)={len(self.r_list)}, len(merge_list)={len(self.merge_list)})")
                        for t_traj in self.scheduled_trajectories.values():
                            t_vehicle = t_traj['vehicle']
                            time_res = analyze_array(t_traj['trajectory']['time'])
                            x_res = analyze_array(t_traj['trajectory']['x'])
                            print(f"Vehicle: {t_vehicle}, time: {res_to_str(time_res)}, x: {res_to_str(x_res)}")
                except Exception as e:
                    print(f"error: {repr(e)}")
                if not self.run:
                    return
                time.sleep(0.1)

        thread = threading.Thread(target=checking, args=())
        thread.start()
        print('Started trajectory checking ... ')

    def obtain_lanes_info(self, t_vehicle):
        current_lane = traci.vehicle.getLaneID(t_vehicle)
        route_id = traci.vehicle.getRouteID(t_vehicle)
        route_edges = traci.route.getEdges(route_id)
        lanes_to_end = [current_lane]
        current_edge = traci.lane.getEdgeID(current_lane)
        lane = traci.vehicle.getLaneID(t_vehicle)
        allowed_speed = traci.lane.getMaxSpeed(lane)
        lanes_info = [{'lane_id': lane, 'length': traci.lane.getLength(lane), 'maxSpeed': allowed_speed}]
        for next_edge in route_edges[route_edges.index(current_edge) + 1:]:
            finished = False  # Whether finding lanes to this next edge is finished.
            while not finished:
                links = traci.lane.getLinks(current_lane)
                for link in links:
                    if traci.lane.getEdgeID(link[0]) == next_edge:
                        if link[4] not in lanes_to_end:
                            lanes_to_end.append(link[4])
                            lanes_info.append({'lane_id': link[4], 'length': traci.lane.getLength(link[4]),
                                               'maxSpeed': traci.lane.getMaxSpeed(link[4])})
                            current_lane = link[4]
                        if link[0] not in lanes_to_end:
                            lanes_to_end.append(link[0])
                            lanes_info.append({'lane_id': link[0], 'length': traci.lane.getLength(link[0]),
                                               'maxSpeed': traci.lane.getMaxSpeed(link[0])})
                            current_lane = link[0]
                        finished = True
                        break
        return lanes_info

    def compse_mono_trajectory(self, t_vehicle, from_time=None, additional_space=3.0, init_speed=None, start_x=None,
                               end=None):
        # (t_vehicle, from_time=time, init_speed=speed, start=x, end=self.m_offset)
        if init_speed is None:
            speed = traci.vehicle.getSpeed(t_vehicle)
        else:
            speed = init_speed
        if start_x is None:
            x = traci.vehicle.getDistance(t_vehicle)
        else:
            x = start_x
        if end is None:
            return None
        if from_time is None:
            current_time = traci.simulation.getTime()
        else:
            current_time = from_time
        if start_x < 0.25:
            start_x = 0.35
        time_step_length = traci.simulation.getDeltaT()
        deceleration = traci.vehicle.getDecel(t_vehicle)
        max_acceleration = traci.vehicle.getAccel(t_vehicle)
        max_vehicle_speed = traci.vehicle.getMaxSpeed(t_vehicle)
        vehicle_length = traci.vehicle.getLength(t_vehicle)
        speed = traci.vehicle.getSpeed(t_vehicle)
        x = traci.vehicle.getDistance(t_vehicle)
        t_speed = speed
        tail_x = x
        if t_vehicle[0] == 'm':
            lanes_info = self.m_lanes_info
        else:
            lanes_info = self.r_lanes_info


        def total_length_of_lanes():
            sum = 0.0
            for t_lane_info in lanes_info:
                sum += t_lane_info['length']
            return sum

        def get_lane(t_x):
            if t_x < 0:
                return None
            cumulated_x = 0
            my_t = None
            for t_lane in lanes_info:
                my_t = t_lane
                if cumulated_x <= t_x < cumulated_x + t_lane['length']:
                    return t_lane, t_x - cumulated_x
                cumulated_x += t_lane['length']
            return my_t, t_x - cumulated_x

        t_time = current_time
        t_acceleration = min(max_acceleration, deceleration)
        # trajectory = [{'time': t_time, 'tail_x': tail_x, 'head_x': head_x}]
        time_list = [t_time]
        x_list = [tail_x]
        speed_list = [t_speed]
        lane_list = []
        lane_position = []
        total_len = min(total_length_of_lanes(), end)
        while tail_x < total_len:
            if t_speed < min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 0.995:
                t_speed += time_step_length * t_acceleration * 0.4
            if t_speed > min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 1.005:
                t_speed -= time_step_length * t_acceleration * 0.4
            t_len = t_speed * time_step_length
            t_time += time_step_length
            tail_x += t_len
            time_list.append(t_time)
            x_list.append(tail_x)
            speed_list.append(t_speed)
        x_dis = scale_series(np.array(time_list), np.array(x_list), x_list[0], x_list[-1], end)
        x_dis[x_dis < 0.0] = 0.0
        np.round(x_dis, decimals=3)

        for t_x in list(x_dis):
            cal_position = t_x + vehicle_length + 0.1
            lane_list.append(get_lane(cal_position)[0]['lane_id'])
            lane_position.append(get_lane(cal_position)[1])
        return {'vehicle': t_vehicle,
                'trajectory': {'time': np.array(time_list), 'x': x_dis, 'speed': np.array(speed_list),
                               'lane_list': lane_list, 'lane_position': lane_position},
                'space_len': vehicle_length + additional_space,
                'lanes_info': lanes_info}

    def complete_mono_trajectory(self, in_trajectory, additional_space=3.0):
        t_vehicle = in_trajectory['vehicle']
        time_step_length = traci.simulation.getDeltaT()
        deceleration = traci.vehicle.getDecel(t_vehicle)
        max_acceleration = traci.vehicle.getAccel(t_vehicle)
        max_vehicle_speed = traci.vehicle.getMaxSpeed(t_vehicle)
        vehicle_length = traci.vehicle.getLength(t_vehicle)
        if t_vehicle[0] == 'm':
            lanes_info = self.m_lanes_info
        else:
            lanes_info = self.r_lanes_info

        def total_length_of_lanes():
            sum = 0.0
            for t_lane_info in lanes_info:
                sum += t_lane_info['length']
            return sum

        def get_lane(t_x):
            if t_x < 0:
                return None
            cumulated_x = 0
            my_t = None
            for t_lane in lanes_info:
                my_t = t_lane
                if cumulated_x <= t_x < cumulated_x + t_lane['length']:
                    return t_lane, t_x - cumulated_x
                cumulated_x += t_lane['length']
            return my_t, t_x - cumulated_x

        t_time = in_trajectory['trajectory']['time'][-1]
        t_speed = in_trajectory['trajectory']['speed'][-1]
        tail_x = in_trajectory['trajectory']['x'][-1]
        t_acceleration = min(max_acceleration, deceleration)
        # trajectory = [{'time': t_time, 'tail_x': tail_x, 'head_x': head_x}]
        time_list = []
        x_list = []
        speed_list = []
        lane_list = []
        lane_position = []
        total_len = total_length_of_lanes()
        while tail_x < total_len:
            if t_speed < min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 0.995:
                t_speed += time_step_length * t_acceleration * 0.4
            if t_speed > min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 1.005:
                t_speed -= time_step_length * t_acceleration * 0.4
            t_len = t_speed * time_step_length
            t_time += time_step_length
            tail_x += t_len
            time_list.append(t_time)
            x_list.append(tail_x)
            speed_list.append(t_speed)
        for t_x in x_list:
            cal_position = t_x + vehicle_length + 0.1
            lane_list.append(get_lane(cal_position)[0]['lane_id'])
            lane_position.append(get_lane(cal_position)[1])

        t_diff = in_trajectory['trajectory']['time'][-1] - [time_list[0]]
        x_diff = in_trajectory['trajectory']['x'][-1] - [x_list[0]]
        if t_diff > 0:
            print("error")
        print(f"t_diff: {t_diff}, x_diff: {x_diff}")
        time_vals = np.concatenate((in_trajectory['trajectory']['time'], np.array(time_list)))
        x_vals = np.concatenate((in_trajectory['trajectory']['x'], np.array(x_list)))
        speed_vals = np.concatenate((in_trajectory['trajectory']['speed'], np.array(speed_list)))
        lane_vals = [*in_trajectory['trajectory']['lane_list'], *lane_list]
        lane_position_vals = [*in_trajectory['trajectory']['lane_position'], *lane_position]
        return {'vehicle': t_vehicle,
                'trajectory': {'time': time_vals, 'x': x_vals, 'speed': speed_vals,
                               'lane_list': lane_vals, 'lane_position': lane_position_vals},
                'space_len': vehicle_length + additional_space,
                'lanes_info': lanes_info}

    # compose_follow_trajectory(t_vehicle, from_time=time, init_speed=speed, start_x=x,
    #                                                              end=self.m_offset,leader=t_leader)
    def compose_follow_trajectory(self, in_vehicle, from_time=None, init_speed=None, start_x=None, end=None,
                                  leader=None, additional_space=3.0):
        t_vehicle = in_vehicle
        time_step_length = traci.simulation.getDeltaT()
        deceleration = traci.vehicle.getDecel(t_vehicle)
        max_acceleration = traci.vehicle.getAccel(t_vehicle)
        max_vehicle_speed = traci.vehicle.getMaxSpeed(t_vehicle)
        vehicle_length = traci.vehicle.getLength(t_vehicle)
        if t_vehicle[0] == 'm':
            lanes_info = self.m_lanes_info
        else:
            lanes_info = self.r_lanes_info

        def total_length_of_lanes():
            sum = 0.0
            for t_lane_info in lanes_info:
                sum += t_lane_info['length']
            return sum

        def get_lane(t_x):
            if t_x < 0:
                return None
            cumulated_x = 0
            my_t = None
            for t_lane in lanes_info:
                my_t = t_lane
                if cumulated_x <= t_x < cumulated_x + t_lane['length']:
                    return t_lane, t_x - cumulated_x
                cumulated_x += t_lane['length']
            return my_t, t_x - cumulated_x

        t_time = from_time
        t_speed = init_speed
        if start_x < 0.25:
            start_x = 0.35
        tail_x = start_x
        leader_trajectory = copy.deepcopy(
            self.scheduled_trajectories[leader['vehicle']]['trajectory'])  # 'time' and 'x'
        # fix the difference of two types of offsets.
        if leader['vehicle'][0] == 'm':
            leader_trajectory['x'] -= self.m_offset
        else:
            leader_trajectory['x'] -= self.r_offset
        if in_vehicle[0] == 'm':
            leader_trajectory['x'] += self.m_offset
        else:
            leader_trajectory['x'] += self.r_offset

        def get_leader_x(in_time):
            try:
                idx = np.where(leader_trajectory['time'] >= in_time)[0][0]
                if not idx.any():
                    return None
                else:
                    t_res_x = leader_trajectory['x'][idx]
                    t_res_speed = leader_trajectory['speed'][idx]
                    return t_res_x, t_res_speed
            except Exception as e:
                return None

        space_len = vehicle_length + additional_space
        t_acceleration = min(max_acceleration, deceleration)
        # trajectory = [{'time': t_time, 'tail_x': tail_x, 'head_x': head_x}]
        time_list = []
        x_list = []
        speed_list = []
        lane_list = []
        lane_position = []
        try:
            if end is None:
                total_len = total_length_of_lanes()
            else:
                total_len = min(total_length_of_lanes(), end)
        except Exception as e:
            print(f"error {repr(e)}")
        strict_follow = False
        strict_gap = space_len
        while tail_x < total_len:
            try:
                try:
                    t_lead_x_sample, t_lead_speed_sample = get_leader_x(t_time)
                    leader_res = True
                except Exception as e:
                    leader_res = False
                if t_speed < min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 0.995:
                    if not leader_res:
                        t_speed += time_step_length * t_acceleration * 0.4
                    if leader_res and tail_x + space_len + 5 < t_lead_x_sample and t_speed < t_lead_speed_sample:
                        t_speed += time_step_length * t_acceleration * 0.4

                if t_speed > min(max_vehicle_speed, get_lane(tail_x)[0]['maxSpeed']) * 1.005:
                    t_speed -= time_step_length * t_acceleration * 0.4
                if leader_res and tail_x + space_len > t_lead_x_sample:
                    t_speed -= time_step_length * t_acceleration * 1.9
                if leader_res and (t_speed > t_lead_speed_sample) and tail_x + space_len > t_lead_x_sample:
                    t_speed -= time_step_length * t_acceleration * 3.2

                if leader_res and tail_x + space_len > t_lead_x_sample - 2.0 and tail_x < space_len + 8:
                    t_speed = t_lead_speed_sample
                if leader_res and (t_speed > t_lead_speed_sample) and tail_x < 8:
                    t_speed = t_lead_speed_sample
                if leader_res and (t_speed > t_lead_speed_sample + 0.2) and tail_x + space_len > t_lead_x_sample - 5.0:
                    t_speed = t_lead_speed_sample
                if leader_res and (
                        t_speed > t_lead_speed_sample) and tail_x + space_len > t_lead_x_sample - 8.0 and tail_x > 5.2 and start_x < 1.0:
                    t_speed -= time_step_length * t_acceleration * 3.2
                if leader_res and tail_x + space_len > t_lead_x_sample and tail_x < space_len + 8:
                    t_speed = 0.0
                if leader_res and tail_x + space_len > t_lead_x_sample - 4.0 and tail_x > 5.2 and start_x < 1.0:
                    if t_vehicle == 'm.0':
                        print(f"speed: {t_speed}, {t_lead_speed_sample}, distance between: {t_lead_x_sample - tail_x}")
                    t_speed -= time_step_length * t_acceleration * 5.2

                if t_speed < 0.0:
                    t_speed = 0.0

                if leader_res and -2.0 < tail_x + space_len - t_lead_x_sample < 0 and abs(
                        t_speed - t_lead_speed_sample) < 0.5:
                    strict_follow = True
                    strict_gap = t_lead_x_sample - tail_x
                if leader_res and strict_follow:
                    n_tail_x = t_lead_x_sample - strict_gap
                    t_speed = t_lead_speed_sample
                    tail_x = n_tail_x
                else:
                    t_len = t_speed * time_step_length
                    tail_x += t_len
            except Exception as e:
                print(f"follow error: {repr(e)}.")

            try:
                if leader_res and tail_x + space_len > t_lead_x_sample + 0.3 and tail_x > 5.2 and start_x < 1.0:
                    print(" Too near! Failed to follow")
                    return None
                if leader_res and tail_x + space_len > t_lead_x_sample + 0.3 and tail_x > 5.2:
                    print(" Too near! Failed to follow")
                    return None
                t_time += time_step_length
                time_list.append(t_time)
                x_list.append(tail_x)
                speed_list.append(t_speed)
            except Exception as e:
                print(f"follow error: {repr(e)}.")
        if end is None:
            try:
                end = x_list[-1]
            except IndexError as e:
                print(f"Index error: {repr(e)}.")
        x_dis = scale_series(np.array(time_list), np.array(x_list), x_list[0], x_list[-1], end)
        x_dis[x_dis < 0.0] = 0.0
        np.round(x_dis, decimals=3)
        for t_x in list(x_dis):
            cal_position = t_x + vehicle_length + 0.1
            lane_list.append(get_lane(cal_position)[0]['lane_id'])
            lane_position.append(get_lane(cal_position)[1])
        return {'vehicle': t_vehicle,
                'trajectory': {'time': np.array(time_list), 'x': x_dis, 'speed': np.array(speed_list),
                               'lane_list': lane_list, 'lane_position': lane_position},
                'space_len': vehicle_length + additional_space,
                'lanes_info': lanes_info}

    def modify_trajectory_end_time(self, origin_trajectory, end_time):
        time_step_length = traci.simulation.getDeltaT()
        t_vehicle = origin_trajectory['vehicle']
        vehicle_length = traci.vehicle.getLength(t_vehicle)
        lanes_info = origin_trajectory['lanes_info']
        space_len = origin_trajectory['space_len']
        o_x = origin_trajectory['trajectory']['x']
        o_time = origin_trajectory['trajectory']['time']
        t_time = o_time[0]
        from_time = t_time
        time_list = []
        while t_time <= end_time:
            time_list.append(t_time)
            t_time += time_step_length

        n_time = np.array(time_list)
        cal_time = from_time + (n_time - from_time) * (np.max(o_time) - np.min(o_time)) / (
                np.max(n_time) - np.min(n_time))

        cs = CubicSpline(o_time, o_x)
        cal_x = cs(cal_time)

        # print(f"end error: {cal_x[-1] - o_x[-1]}, start error: {cal_x[0]-o_x[0]}")

        def get_lane(t_x):
            if t_x < 0:
                return None
            cumulated_x = 0
            my_t = None
            for t_lane in lanes_info:
                my_t = t_lane
                if cumulated_x <= t_x < cumulated_x + t_lane['length']:
                    return t_lane, t_x - cumulated_x
                cumulated_x += t_lane['length']
            return my_t, t_x - cumulated_x

        lane_list = []
        lane_position = []
        speed_list = []
        t_i = 0
        for t_x in list(cal_x):
            if t_i == 0:
                speed_list.append(origin_trajectory['trajectory']['speed'][0])
            else:
                t_speed = (cal_x[t_i] - cal_x[t_i - 1]) / time_step_length
                speed_list.append(t_speed)
            t_i += 1
            cal_position = t_x + vehicle_length + 0.1
            lane_list.append(get_lane(cal_position)[0]['lane_id'])
            lane_position.append(get_lane(cal_position)[1])

        # if n_time[-1] != end_time:
        #     real_end = n_time[-1]
        #     print(f"real end: {real_end}, required end: {end_time}")
        return {'vehicle': t_vehicle,
                'trajectory': {'time': n_time, 'x': cal_x, 'speed': np.array(speed_list),
                               'lane_list': lane_list, 'lane_position': lane_position},
                'space_len': space_len,
                'lanes_info': lanes_info}

    def concatenate_trajectories(self, trajectory_a, trajectory_b):
        if trajectory_b['vehicle'] != trajectory_b['vehicle']:
            print('Wrong!')
        if trajectory_a['trajectory']['time'][-1] > trajectory_b['trajectory']['time'][0]:
            print(
                f"Concatenate trajectory error=> time_a: {trajectory_a['trajectory']['time'][-1]}  time_b: {trajectory_b['trajectory']['time'][0]}")
        t_time_res = np.concatenate((trajectory_a['trajectory']['time'], trajectory_b['trajectory']['time']))
        t_x_res = np.concatenate((trajectory_a['trajectory']['x'], trajectory_b['trajectory']['x']))
        t_speed_res = np.concatenate((trajectory_a['trajectory']['speed'], trajectory_b['trajectory']['speed']))
        t_lane_list = [*trajectory_a['trajectory']['lane_list'], *trajectory_b['trajectory']['lane_list']]
        t_lane_position = [*trajectory_a['trajectory']['lane_position'], *trajectory_b['trajectory']['lane_position']]
        t_res = copy.deepcopy(trajectory_a)
        t_res['trajectory'] = {'time': t_time_res, 'x': t_x_res, 'speed': t_speed_res,
                               'lane_list': t_lane_list, 'lane_position': t_lane_position}
        for t_lane in t_res['trajectory']['lane_list']:
            if t_lane not in [item['lane_id'] for item in t_res['lanes_info']]:
                print("Wrong")
        return t_res

    def merge_into(self, in_vehicle, in_trajectory, additional_space=3.0):
        """
        (t_vehicle, m_trajectory)
        """
        need_recompose_trajectory = []
        try_enter_time = in_trajectory['trajectory']['time'][-1]
        space_len = in_trajectory['space_len']
        for i in range(len(self.merge_list)):
            t_item = self.merge_list[i]
            if t_item['time'] > try_enter_time:
                need_recompose_trajectory = self.merge_list[i:]
                self.merge_list = self.merge_list[:i]
                break
            i += 1
        if len(self.merge_list) == 0:
            self.merge_list.append({'vehicle': in_vehicle, 'time': in_trajectory['trajectory']['time'][-1],
                                    'speed': in_trajectory['trajectory']['speed'][-1],
                                    'x': in_trajectory['trajectory']['x'][-1]})
            t_complete_trajectory = self.complete_mono_trajectory(in_trajectory)
            self.scheduled_trajectories[in_vehicle] = t_complete_trajectory
        else:
            leader = self.merge_list[-1]
            leader_id = leader['vehicle']
            if leader_id[0] == 'm':
                t_offset = self.m_offset
            else:
                t_offset = self.r_offset
            idx = np.where(self.scheduled_trajectories[leader_id]['trajectory']['x'] - t_offset - space_len > 0)[0][0]
            may_enter_time = self.scheduled_trajectories[leader_id]['trajectory']['time'][idx]
            # in_vehicle, from_time=None, init_speed=None, start_x=None, end=None,
            #                                   leader=None, additional_space=3.0
            start_x = in_trajectory['trajectory']['x'][-1]
            f_trajectory = None
            t_idx = 0
            while f_trajectory is None:
                in_trajectory = self.modify_trajectory_end_time(in_trajectory,
                                                                max(may_enter_time, try_enter_time) + t_idx * 1.0)
                continue_speed = in_trajectory['trajectory']['speed'][-1]
                f_trajectory = self.compose_follow_trajectory(in_vehicle, from_time=max(may_enter_time,
                                                                                        try_enter_time) + t_idx * 1.0,
                                                              init_speed=continue_speed, start_x=start_x, leader=leader,
                                                              additional_space=additional_space)
                t_idx += 1
            t_trajectory = self.concatenate_trajectories(in_trajectory, f_trajectory)
            self.merge_list.append({'vehicle': in_vehicle, 'time': in_trajectory['trajectory']['time'][-1],
                                    'speed': in_trajectory['trajectory']['speed'][-1],
                                    'x': in_trajectory['trajectory']['x'][-1]})
            self.scheduled_trajectories[in_vehicle] = t_trajectory

        # Handle all the trajectories on need_recompose_trajectory
        i = 0
        for t_recompose in need_recompose_trajectory:
            t_vehicle = t_recompose['vehicle']
            if t_vehicle[0] == 'm':
                my_offset = self.m_offset
            else:
                my_offset = self.r_offset
            leader = self.merge_list[-1]
            leader_id = leader['vehicle']
            if leader_id[0] == 'm':
                t_offset = self.m_offset
            else:
                t_offset = self.r_offset
            idx = np.where(self.scheduled_trajectories[leader_id]['trajectory']['x'] - t_offset - space_len > 0)[0][0]
            may_enter_time = self.scheduled_trajectories[leader_id]['trajectory']['time'][idx]

            def cut_trajectory(t_trajectory, in_offset):
                # cut off the end part of in_trajectory from offset.
                t_len = np.where(t_trajectory['trajectory']['x'] > in_offset)[0][0]
                """
                'trajectory': {'time': cal_time, 'x': cal_x, 'speed': np.array(speed_list),
                               'lane_list': lane_list, 'lane_position': lane_position},
                """
                t_trajectory['trajectory']['time'] = t_trajectory['trajectory']['time'][:t_len]
                t_trajectory['trajectory']['x'] = t_trajectory['trajectory']['x'][:t_len]
                t_trajectory['trajectory']['speed'] = t_trajectory['trajectory']['speed'][:t_len]
                return t_trajectory

            origin_trajectory = self.scheduled_trajectories[t_vehicle]
            try:
                n_trajectory = cut_trajectory(origin_trajectory, my_offset)
            except Exception as e:
                # print(f"error: {repr(e)}")
                n_trajectory = origin_trajectory

            if i == 0:
                t_trajectory = self.modify_trajectory_end_time(n_trajectory, may_enter_time)
                t_end_ts = t_trajectory['trajectory']['time'][-1]
                print(f"{t_end_ts}-{may_enter_time}")
            else:
                """
                compose_follow_trajectory(self, in_vehicle, from_time=None, init_speed=None, start_x=None, end=None,
                                  leader=None, additional_space=3.0):
                """
                f_time = origin_trajectory['trajectory']['time'][0]
                i_speed = origin_trajectory['trajectory']['speed'][0]
                s_x = origin_trajectory['trajectory']['x'][0]
                t_end = my_offset
                my_leader = need_recompose_trajectory[i - 1]

                t_trajectory = self.compose_follow_trajectory(t_vehicle, from_time=f_time, init_speed=i_speed,
                                                              start_x=s_x, end=t_end, leader=my_leader)

            f_i_x = t_trajectory['trajectory']['x'][-1]
            f_i_time = t_trajectory['trajectory']['time'][-1]

            f_trajectory = None
            t_idx = 0
            while f_trajectory is None:
                t_trajectory = self.modify_trajectory_end_time(t_trajectory,
                                                               f_i_time + t_idx * 1.0)
                continue_speed = t_trajectory['trajectory']['speed'][-1]
                f_trajectory = self.compose_follow_trajectory(t_vehicle, from_time=f_i_time + t_idx * 1.0,
                                                              init_speed=continue_speed, start_x=f_i_x, leader=leader,
                                                              additional_space=additional_space)
                t_idx += 1

            self.merge_list.append({'vehicle': t_vehicle, 'time': t_trajectory['trajectory']['time'][-1],
                                    'speed': t_trajectory['trajectory']['speed'][-1],
                                    'x': t_trajectory['trajectory']['x'][-1]})
            con_trajectory = self.concatenate_trajectories(t_trajectory, f_trajectory)
            self.scheduled_trajectories[t_vehicle] = con_trajectory
            i += 1

    def check_new_vehicle(self, t_vehicle, additional_space=3.0):
        print(f"New vehilce: {t_vehicle} entered:")
        self.check_new_vehicle_lock.acquire()
        try:
            current_lane = traci.vehicle.getLaneID(t_vehicle)
            if current_lane == '':
                try:
                    traci.vehicle.remove(t_vehicle)
                except Exception as e:
                    print(f"vehicle: {t_vehicle} remove failed. message: {repr(e)}")
                    self.check_new_vehicle_lock.release()
                    return
        except Exception as e:
            print(f"vehicle: {t_vehicle} obtain information failed. message: {repr(e)}")
            self.check_new_vehicle_lock.release()
            return

        print(f"Check new: {t_vehicle} Computing Trajectory<<<<<<")
        vehicle_length = traci.vehicle.getLength(t_vehicle)
        space_len = vehicle_length + additional_space
        time = traci.simulation.getTime()
        speed = traci.vehicle.getSpeed(t_vehicle)
        x = traci.vehicle.getDistance(t_vehicle)
        if t_vehicle[0] == 'm':
            if self.m_lanes_info is None:
                self.m_lanes_info = self.obtain_lanes_info(t_vehicle)
            if len(self.m_list) == 0:
                m_trajectory = self.compse_mono_trajectory(t_vehicle, from_time=time, init_speed=speed, start_x=x,
                                                           end=self.m_offset, additional_space=additional_space)
            else:
                t_leader = self.m_list[-1]
                m_trajectory = self.compose_follow_trajectory(t_vehicle, from_time=time, init_speed=speed, start_x=x,
                                                              end=self.m_offset, leader=t_leader,
                                                              additional_space=additional_space)
                if m_trajectory is None:
                    print('failed.')
            self.m_list.append({'vehicle': t_vehicle, 'time': time, 'speed': speed, 'x': x})
            # try entering the merged lanes with already composed trajectory.
            if len(self.merge_list) == 0:
                self.merge_list.append({'vehicle': t_vehicle, 'time': m_trajectory['trajectory']['time'][-1],
                                        'speed': m_trajectory['trajectory']['speed'][-1],
                                        'x': m_trajectory['trajectory']['x'][-1]})
                t_complete_trajectory = self.complete_mono_trajectory(m_trajectory, additional_space=additional_space)
                self.scheduled_trajectories[t_vehicle] = t_complete_trajectory
            else:
                try:
                    self.merge_into(t_vehicle, m_trajectory, additional_space=additional_space)
                except Exception as e:
                    print(f"m vehicle merge failed: {repr(e)}")
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_tb(exc_traceback)

        if t_vehicle[0] == 'r':
            if self.r_lanes_info is None:
                self.r_lanes_info = self.obtain_lanes_info(t_vehicle)
            if len(self.r_list) == 0:
                r_trajectory = self.compse_mono_trajectory(t_vehicle, from_time=time, init_speed=speed, start_x=x,
                                                           end=self.r_offset, additional_space=additional_space)
            else:
                t_leader = self.r_list[-1]
                try:
                    r_trajectory = self.compose_follow_trajectory(t_vehicle, from_time=time, init_speed=speed,
                                                                  start_x=x,
                                                                  end=self.r_offset, leader=t_leader,
                                                                  additional_space=additional_space)
                    if r_trajectory is None:
                        print('Failed')
                except Exception as e:
                    print(f"compose follow trajectory for {t_vehicle} failed with error {repr(e)}.")
            self.r_list.append({'vehicle': t_vehicle, 'time': time, 'speed': speed, 'x': x})
            if len(self.merge_list) == 0:
                self.merge_list.append({'vehicle': t_vehicle, 'time': m_trajectory['trajectory']['time'][-1],
                                        'speed': m_trajectory['trajectory']['speed'][-1],
                                        'x': m_trajectory['trajectory']['x'][-1]})
                t_complete_trajectory = self.complete_mono_trajectory(r_trajectory, additional_space=additional_space)
                self.scheduled_trajectories[t_vehicle] = t_complete_trajectory
            else:
                try:
                    self.merge_into(t_vehicle, r_trajectory, additional_space=additional_space)
                except Exception as e:
                    print(f"r vehicle merge failed: {repr(e)}")
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_tb(exc_traceback)

        self.check_new_vehicle_lock.release()
        print(f"!!!! <====================scheduled trajectory for vehicle:{t_vehicle}.================>!!!")
        return None
