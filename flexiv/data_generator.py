import subprocess
import matplotlib.pyplot as plt
import numpy as np
import time
import json
import os

# 这里使用相对路径，是本目录下 CMake 编译 test_every_joint 生成的可执行文件
cpp_program_path = "cmake-build-debug/base/main"

p = subprocess.Popen(cpp_program_path, stdin=subprocess.PIPE)

# 这个计数是用在时间测量以及运动图绘制的情景中，如果不用到 matplotlib 功能就不需要这个变量
time_cnt = 0
total_time = 250  # 这个就是对应的绘图总时长, 单位是 s

# 同理，这里采用相对路径, 如果要对任意路径运行该脚本都可用，可以换成绝对路径
# 这个是示例测试文件，实际情况下我们使用的是 order + number + .txt
file_path = "simulate_datasets/order.txt"
# 这个时间差是手动生成时间差，用于对应仿真过程的时间差，由于一开始启动步往往会扭矩很大，我们这里采用先慢后快的方式，确保一开始稳定启动不会超过转速、扭矩的上限
time_span = 110  
# 这是一个初始值，用于计算差值，确保第一帧的数据是有效的
last_data = [0, -40.0 * np.pi / 180.0, 0, 90 * np.pi / 180, 0, 40 / 180 * np.pi, 0]
# 这两个变量是用来记录某一个关节的轴角度随着时间变化的信息，用于绘制图像
data_a4_last = []
data_a4_angle = []
# 记录夹爪是否闭合
is_gripper_closed = False

finish_data = []

# 判断指令文件是否存在
def file_exists(directory, filename):
    file_path = os.path.join(directory, filename)
    return os.path.exists(file_path) and os.path.isfile(file_path)

file_cnt = 0

# 以下指令，用于等待仿真指令生成的过程，如果采取串行措施，在order生成完成之后再执行，这四行可以注释了
while not file_exists('simulate_datasets','order0.txt'):
    print('waiting...')

time.sleep(4)

while 1:
    file_name = 'order' + str(file_cnt) + '.txt'
    if (file_exists('simulate_datasets','order1.txt') and file_cnt == 0) or (file_exists('simulate_datasets',file_name) and file_cnt > 0):
        try:
            print("Processing file " + str(file_cnt))
            with open('simulate_datasets/' + file_name, "r") as file:
                for line in file:
                    line = line.replace("None", "null")
                    time_cnt = time_cnt + 1
                    # 到达一定时间，经过加速过程，减少指令时间间隔，提高运行速度
                    if time_cnt > 10:
                        time_span = 15
                        # print("Move faster")
                    try:
                        data = json.loads(line)  # 解析JSON
                        # 增加绘图内容部分
                        if data["joint_positions"][3]:
                            data_a4_angle.append(data["joint_positions"][2])

                        is_gripper_closed = bool(data["joint_positions"][0])

                        diff_data = []
                        # 计算两个变量的差值，用于预测下一帧的走向
                        if data["joint_positions"][0]:
                            for j in range(7):
                                diff_data.append(
                                    (data["joint_positions"][j] - last_data[j]) / time_span
                                )
                            last_data = data["joint_positions"]

                        for i in range(time_span):
                            local_data = [0, 0, 0, 0, 0, 0, 0]
                            if data["joint_positions"][0]:
                                for j in range(7):
                                    local_data[j] = data["joint_positions"][
                                        j
                                    ] 
                                    local_data[j] = local_data[j] + diff_data[j] * i
                            # 确保每一行都是有效的JSON格式
                            # 为了保持和官网上示例一致，这里的控制我们暂时不加入速度控制
                            if data["joint_positions"][0]:
                                local_datas = {
                                    "joint_positions": local_data,
                                    "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                    "joint_efforts": is_gripper_closed,
                                }
                                finish_data = local_datas
                            else:
                                local_datas = {
                                    "joint_positions": last_data,
                                    "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                    "joint_efforts": is_gripper_closed,
                                }
                                finish_data = local_datas
                            json_string = json.dumps(
                                local_datas
                            )  # 将解析后的JSON重新编码为字符串
                            data_a4_last.append(local_data[2] * 180 / np.pi)
                            p.stdin.write(
                                json_string.encode() + b"\n"
                            )  # 发送JSON字符串给C++程序
                            p.stdin.flush()  # 确保数据被发送
                            time.sleep(0.001)  # 以1000Hz的频率发送数据
                            # if not next(file):
                            #     break

                    except json.JSONDecodeError as e:
                        print(f"JSON解析错误: {e}")

                while 1:
                    json_finish_string = json.dumps(finish_data)
                    p.stdin.write(json_finish_string.encode() + b"\n")
                    p.stdin.flush()
                    time.sleep(0.001)  # 机械臂的控制频率就是 1kHz, 如果过低就会导致机械臂认为丢失控制数据而停止
                    if file_exists('simulate_datasets','order' + str(file_cnt + 1)+'.txt'):
                        break

        except KeyboardInterrupt:
            p.terminate()  # 用户中断脚本时终止C++程序

        file_cnt = file_cnt + 1
        print("Finish file " + str(file_cnt))
    else:
        break
