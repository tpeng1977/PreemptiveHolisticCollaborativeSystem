import os
import sys
# 运行方式
import traci
from xml.etree import ElementTree as ET
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import csv

# Prefix directory for output files.
prefix = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
# Check if the directory exists
if not os.path.exists(prefix):
    # Create the directory if it doesn't exist
    os.makedirs(prefix)
    print(f"Directory '{prefix}' created.")
else:
    print(f"Directory '{prefix}' already exists.")


np.random.seed(1024)

if 'SUMO_HOME' in os.environ:
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
                         'minGap': "2", 'accel': "2.5", 'decel': "4.5", 'tau': '1', 'maxSpeed': "20",
                         'speedFactor': "1.1", 'speedDev': "0.1", 'laneChangeModel': "LC2013"})
    vType1 = ET.Element('vType',
                        {'id': "cav_r", 'vClass': "passenger", 'color': "yellow", 'carFollowModel': "Krauss",
                         'minGap': "2", 'accel': "2.5", 'decel': "4.5", 'tau': '1', 'maxSpeed': "20",
                         'speedFactor': "1.1", 'speedDev': "0.1", 'laneChangeModel': "LC2013"})
    # 增加到routes根目录
    root.append(vType0)
    root.append(vType1)

    # 增加车流:主路车
    number = e_flow
    period = number / 3600
    if number > 0:
        flow = ET.Element('flow',
                          {'id': 'm', 'type': "cav_m", 'from': 'E0', 'to': 'E3', 'departLane': 'random',
                           'departSpeed': '20', 'begin': "0", 'period': f'exp({period})', 'number': str(number)})
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
    # 调节仿真时间
    while traci.simulation.getTime() < t:
        # 仿真运行一步
        traci.simulationStep()
        # 获取车辆列表
        vehicle_list = traci.vehicle.getIDList()
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
        for i in vehicle_list:
            # 获取车辆信息
            distance = traci.vehicle.getDistance(i)
            lane = traci.vehicle.getLaneID(i)
            x = traci.vehicle.getDistance(i)
            speed = traci.vehicle.getSpeed(i)
            timeloss = traci.vehicle.getTimeLoss(i)
            # 储存热力时空图数据
            if step % 5 == 0:
                if i[0] == 'm':
                    veh_info.append([i, 'm', step / 100, x, speed])
                elif i[0] == 'r':
                    veh_info.append([i, 'r', step / 100, x, speed])
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
    traci.close()
    # 列名
    header = ['id', 'flow', 'time', 'x', 'speed']
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
        with open(os.path.join(prefix, 'timeloss.txt'), 'a') as f:
            f.write(f"主线车流量, {mainline_flow * 6},")
            f.write(f"匝道车流量, {ramp_flow * 6}\n")
            f.write(f"主线车平均延误, {mainline_avg_delay},")
            f.write(f"匝道车平均延误, {ramp_avg_delay},")
            f.write(f"所有车辆平均延误, {avg_delay}\n")
    else:
        # 创建新文件，并将数据写入其中
        with open(os.path.join(prefix, 'timeloss.txt'), 'w') as f:
            f.write(f"主线车流量, {mainline_flow * 6},")
            f.write(f"匝道车流量, {ramp_flow * 6}\n")
            f.write(f"主线车平均延误, {mainline_avg_delay},")
            f.write(f"匝道车平均延误, {ramp_avg_delay},")
            f.write(f"所有车辆平均延误, {avg_delay}\n")


if __name__ == "__main__":
    for i in [1800]:
        for j in [1600]:
            # 仿真时间,主路和匝道车流（veh/h）
            t = 600
            e_flow, r_flow = i, j
            update_rou(e_flow, r_flow)

            # traci启动仿真
            traci.start([sumo_gui, "-c", "1.test.sumocfg", "--seed", "1024"])
            # 运行自己编写的主函数
            run_sumo(t, e_flow, r_flow)
            draw_td('m', e_flow, r_flow)
            draw_td('r', e_flow, r_flow)
            process_timeloss()
