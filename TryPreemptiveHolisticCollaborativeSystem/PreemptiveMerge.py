import platform
import copy
import os
import sys
import traci
from xml.etree import ElementTree as ET
import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import csv
from preemptive_follow import TrajectoryMerge
from tools import scale_series



np.random.seed(1024)

# Prefix directory for output files.
prefix = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
# Check if the directory exists
if not os.path.exists(prefix):
    # Create the directory if it doesn't exist
    os.makedirs(prefix)
    print(f"Directory '{prefix}' created.")
else:
    print(f"Directory '{prefix}' already exists.")

system = platform.system()
if system == "Linux":
    tools = r'/usr/share/sumo/tools'
    sumo_gui = r'/usr/bin/sumo-gui'
else:
    if 'SUMO_HOME' in os.environ or True:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sumo_gui = os.path.join(os.environ['SUMO_HOME'], 'bin\\sumo-gui')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")


# 根据车流量生成rou文件
def update_rou(e_flow, r_flow):
    # 创建根标签
    root = ET.Element("routes")

    # 创建节点vType
    vType0 = ET.Element('vType',
                        {'id': "cav_m", 'vClass': "passenger", 'color': "blue", 'carFollowModel': "Krauss",
                         'minGap': "0.1", 'accel': "2.5", 'decel': "4.5", 'tau': '0.1', 'maxSpeed': "20",
                         'speedFactor': "1.1", 'speedDev': "0.1"})
    vType1 = ET.Element('vType',
                        {'id': "cav_r", 'vClass': "passenger", 'color': "yellow", 'carFollowModel': "Krauss",
                         'minGap': "0.1", 'accel': "2.5", 'decel': "4.5", 'tau': '0.1', 'maxSpeed': "20",
                         'speedFactor': "1.1", 'speedDev': "0.1"})
    # 增加到routes根目录
    root.append(vType0)
    root.append(vType1)

    # 增加车流:主路车
    number = e_flow
    period = number / 3600
    if number > 0:
        flow = ET.Element('flow',
                          {'id': 'm', 'type': "cav_m", 'from': 'E0', 'to': 'E3', 'departLane': 'random',
                           'departSpeed': '14', 'begin': "0", 'period': f'exp({period})', 'number': str(number)})
        root.append(flow)
    # 增加车流:匝道车
    number = r_flow
    period = number / 3600
    if number > 0:
        flow = ET.Element('flow',
                          {'id': 'r', 'type': "cav_r", 'from': 'E1', 'to': 'E3', 'departLane': 'random',
                           'departSpeed': '7', 'begin': "0", 'period': f'exp({period})', 'number': str(number)})
        root.append(flow)
    # 保存文件
    tree = ET.ElementTree(root)
    tree.write("1.test.rou.xml", encoding='utf-8', xml_declaration=True)






# 主函数
def run_sumo(t, e_flow, r_flow):
    step = 0
    # 用来储存数据
    veh_info = []
    veh_td_3d = []
    veh_timeloss = {}
    veh_fuel = []
    previous_vehicle_list = []
    tp = TrajectoryMerge()
    # 调节仿真时间
    while traci.simulation.getTime() < t:
        # 仿真运行一步
        traci.simulationStep()
        # print(f"simulation time of Current Step: {traci.simulation.getTime()}")
        # 获取车辆列表
        vehicle_list = traci.vehicle.getIDList()
        for t_vehicle in vehicle_list:
            if t_vehicle not in previous_vehicle_list:
                try:
                    tp.check_new_vehicle(t_vehicle)
                except Exception as e:
                    pass
                previous_vehicle_list.append(t_vehicle)

        # for t_vehicle in previous_vehicle_list:
        #     if t_vehicle not in vehicle_list:
        #         try:
        #             previous_vehicle_list.remove(t_vehicle)
        #             del tp.scheduled_trajectories[t_vehicle]
        #         except Exception as e:
        #             pass

        # 获取燃油消耗数据
        if step % 100 == 0:
            if step / 100 == 0:
                veh_fuel.append([0, 0])
            else:
                fuel_sum = 0
                for i in vehicle_list:
                    fuel = traci.vehicle.getFuelConsumption(i)
                    fuel_sum += fuel
                veh_fuel.append([step / 100, round((veh_fuel[-1][-1] + fuel_sum / 1000000), 2)])
        current_time = traci.simulation.getTime()
        for i in vehicle_list:
            try:
                try:
                    t_index = np.where(tp.scheduled_trajectories[i]['trajectory']['time'] >= current_time)[0][0]
                except IndexError:
                    try:
                        traci.vehicle.remove(i)
                    except Exception as e:
                        pass
                    continue
                if step % 1000 == 0:
                    try:
                        t_time = tp.scheduled_trajectories[i]['trajectory']['time'][t_index]
                        t_x = tp.scheduled_trajectories[i]['trajectory']['x'][t_index]
                        x = traci.vehicle.getDistance(i)
                        t_lane = traci.vehicle.getLaneID(i)
                        t_lane_pos = traci.vehicle.getLanePosition(i)
                        lane_id = tp.scheduled_trajectories[i]['trajectory']['lane_list'][t_index]
                        lane_pos = tp.scheduled_trajectories[i]['trajectory']['lane_position'][t_index]
                        if abs(t_x-x) > 0.4:
                            print(f" vehicle: {i}, difference: {t_x - x}, real_lane: {t_lane}, target_lane: {lane_id}, real_lane_pos: {t_lane_pos}, target_lane_pos: {lane_pos}", )
                            print(f"vehicle: {i}, current time: {current_time}, t_time: {t_time}, real_x: {x}, target_x: {t_x}")
                    except Exception as e:
                        print(f"while evaluate difference error: {repr(e)}")
                lane_id = tp.scheduled_trajectories[i]['trajectory']['lane_list'][t_index]
                lane_pos = tp.scheduled_trajectories[i]['trajectory']['lane_position'][t_index]
                x = tp.scheduled_trajectories[i]['trajectory']['x'][t_index]
                try:
                    traci.vehicle.moveTo(i, lane_id, lane_pos)
                except Exception as e:
                    print(f"while move vehicle {i} error: {repr(e)}")
            except Exception as e:
                print(f"Get error: {repr(e)} while control vehicle {i}")
            # 获取车辆信息
            try:
                distance = traci.vehicle.getDistance(i)
                position = traci.vehicle.getLanePosition(i)
                lane = traci.vehicle.getLaneID(i)
                # x = traci.vehicle.getDistance(i)
                speed = traci.vehicle.getSpeed(i)
                timeloss = traci.vehicle.getTimeLoss(i)
            except Exception as e:
                print(f"Obtain information error: {repr(e)} from vehicle {i}")
                continue

            # 储存热力时空图数据
            if step % 5 == 0:
                if i[0] == 'm':
                    veh_info.append([i, 'm', step / 100, x, speed, lane, position, distance])
                elif i[0] == 'r':
                    veh_info.append([i, 'r', step / 100, x, speed, lane, position, distance])
            if step % 100 == 0 and 'J' not in lane:
                # 储存3d时空图数据
                veh_td_3d.append([i, step / 100, lane, distance])
                # 储存延误数据
                if i not in veh_timeloss:
                    veh_timeloss[i] = [i[0], timeloss]
                else:
                    veh_timeloss[i][-1] = timeloss
        # 仿真运行一步
        step += 1
    tp.close()
    traci.close()
    # 列名
    header = ['id', 'flow', 'time', 'x', 'speed', 'lane', 'position', 'distance']
    # 按照 id 和 time 进行排序
    sorted_data = sorted(veh_info, key=lambda x: (x[0], x[1]))
    # 保存为 CSV 文件
    filename = os.path.join(prefix, f'data_step{e_flow, r_flow}.csv')
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        # 写入列名行
        writer.writerow(header)
        # 写入数据行
        writer.writerows(sorted_data)
    data = pd.read_csv(filename)
    # 添加一列记录时间区间
    data['time_interval'] = (data['time'] // 5) * 5
    # 根据来源和时间区间分组计算平均速度
    grouped = data.groupby(['flow', 'time_interval'])['speed'].mean().reset_index()
    # 保存结果为新的CSV文件
    t_filename = os.path.join(prefix, f"data_avg_speed{e_flow, r_flow}.csv")
    grouped.to_csv(t_filename, index=False)

    # 列名
    header = ['id', 'time', 'lane', 'distance']
    file_path = os.path.join(prefix, f"data_td_3d{e_flow, r_flow}.csv")
    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        # 写入列名行
        writer.writerow(header)
        for item in veh_td_3d:
            writer.writerow(item)

    # 指定路径

    csv_file = os.path.join(prefix, 'data_timeloss.csv')
    # 进行写入
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # 写入表头
        writer.writerow(['id', 'Type', 'TimeLoss'])
        # 逐行写入
        for key, value in veh_timeloss.items():
            writer.writerow([key] + value)

    # 列名
    header = ['time', 'fuel_sum']
    file_path = os.path.join(prefix, f"data_fuel{e_flow, r_flow}.csv")

    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        # 写入列名行
        writer.writerow(header)
        for item in veh_fuel:
            writer.writerow(item)


def draw_td(flow_value, e_flow, r_flow):
    # 设置中文显示
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    # 读取数据
    t_filename = os.path.join(prefix, f'data_step{e_flow, r_flow}.csv')
    data = pd.read_csv(t_filename)

    # 根据 flow 列筛选数据
    lane_data = data[data['flow'] == flow_value]

    # 提取车辆编号
    x_vehID = lane_data['id'].drop_duplicates().reset_index(drop=True)

    # 绘制轨迹图
    plt.figure(figsize=(16,9), dpi=300)
    # plt.ylim(0, 1000)
    # plt.xlim(0, 600)
    plt.xticks(fontproperties="Times New Roman", size=12)
    plt.yticks(fontproperties="Times New Roman", size=12)
    plt.ylabel('位置 (m)')
    plt.xlabel('时间 (s)')

    for i in range(len(x_vehID)):
        # 获取当前车辆的数据
        car_data = lane_data[lane_data['id'] == x_vehID[i]]

        # 提取时间和相对移动距离
        x = car_data['time']
        y = np.sqrt(np.square(car_data['x']))

        # 提取速度，并作为颜色映射
        v = car_data['speed']

        # 绘制散点图
        plt.scatter(x, y, marker='.', s=1, c=v, cmap='jet_r', vmin=0, vmax=25)

    plt.colorbar(label='速度 (m/s)')
    # 保存图形为图片文件
    plt.savefig(os.path.join(prefix, f'{flow_value, e_flow, r_flow}_td.png'), format='png', dpi=300)

    # 显示图形
    plt.show()


def process_timeloss():
    # 读取CSV文件
    df = pd.read_csv(os.path.join(prefix, 'data_timeloss.csv'))

    # 将Type列中的m和r分别替换为主线车和匝道车
    df['Type'] = df['Type'].replace({'m': '主线车', 'r': '匝道车'})

    # 计算主线车流量
    mainline_cars = df[df['Type'] == '主线车']
    mainline_flow = len(mainline_cars)

    # 计算匝道车流量
    ramp_cars = df[df['Type'] == '匝道车']
    ramp_flow = len(ramp_cars)

    # 计算主线车平均延误
    mainline_avg_delay = mainline_cars['TimeLoss'].mean()

    # 计算匝道车平均延误
    ramp_avg_delay = ramp_cars['TimeLoss'].mean()

    # 计算所有车辆平均延误
    avg_delay = df['TimeLoss'].mean()

    # 检查根目录下是否存在'3.timeloss.txt'文件
    if os.path.isfile(os.path.join(prefix, 'timeloss.txt')):
        # 将数据追加到文件中
        with open(os.path.join('timeloss.txt'), 'a') as f:
            f.write(f"主线车流量, {mainline_flow * 6},")
            f.write(f"匝道车流量, {ramp_flow * 6}\n")
            f.write(f"主线车平均延误, {mainline_avg_delay},")
            f.write(f"匝道车平均延误, {ramp_avg_delay},")
            f.write(f"所有车辆平均延误, {avg_delay}\n")
    else:
        # 创建新文件，并将数据写入其中
        with open(os.path.join(prefix, 'timeloss.txt'), 'w', encoding='utf-8') as f:
            f.write(f"主线车流量, {mainline_flow * 6},")
            f.write(f"匝道车流量, {ramp_flow * 6}\n")
            f.write(f"主线车平均延误, {mainline_avg_delay},")
            f.write(f"匝道车平均延误, {ramp_avg_delay},")
            f.write(f"所有车辆平均延误, {avg_delay}\n")


def main():
    for i in [1800]:
        for j in [1600]:
            # 仿真时间,主路和匝道车流（veh/h）
            t = 600
            e_flow, r_flow = i, j
            update_rou(e_flow, r_flow)

            # traci启动仿真
            traci.start([sumo_gui, "-c", "1.test.sumocfg", "--seed", "1024"])

            # call the trajectory control function
            run_sumo(t, e_flow, r_flow)
            draw_td('m', e_flow, r_flow)
            draw_td('r', e_flow, r_flow)
            process_timeloss()


if __name__ == "__main__":
    main()
