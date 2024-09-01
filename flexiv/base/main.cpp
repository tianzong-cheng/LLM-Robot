#include <flexiv/Exception.hpp>
#include <flexiv/Gripper.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Robot.hpp>
#include <flexiv/Utility.hpp>

#include "../lib/nlohmann/json.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <iomanip>

#define _USE_MATH_DEFINES

const int MAX_INSTRUCTIONS = 30;

// 定义两个向量相加
std::vector<double> vector_add(const std::vector<double> &v1,
                               const std::vector<double> &v2) {
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("Vectors must be of the same size");
    }

    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = v1[i] + v2[i];
    }

    return result;
}

void vector_deg2rad(std::vector<double> &v1) {
    for (double &i: v1) {
        i = i / 180 * M_PI;
    }
}

int main() {
    // 使用nlohmann/json库的命名空间
    using json = nlohmann::json;

    flexiv::Log log;
    std::string robotIP = "192.168.2.100";
    std::string localIP = "192.168.2.105"; // todo: 记得修改这里的 ip 地址
    flexiv::Robot robot(robotIP, localIP);
    flexiv::Gripper gripper(robot);
    flexiv::RobotStates robotStates;
    std::string instructions[MAX_INSTRUCTIONS];
    std::fstream iFile;

    bool is_gripper_closed = false;
    bool last_flag = is_gripper_closed;
    std::vector<double> default_pos = {0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0};
    vector_deg2rad(default_pos);

    std::vector<double> default_acc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> real_pos = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> real_vel = {0, 0, 0, 0, 0, 0, 0};

    log.info("You Yuchen's Control Demo:");
    // 清除错误指令（如果可行）
    if (robot.isFault()) {
        log.warn("Fault occurred on robot server, trying to clear ...");
        robot.clearFault();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (robot.isFault()) {
            log.error("Fault cannot be cleared, exiting ...");
            return 1;
        }
        log.info("Fault on robot server is cleared");
    }

    log.info("Enabling robot ...");
    robot.enable();
    robot.setMode(
            flexiv::Mode::NRT_PRIMITIVE_EXECUTION); // 初始化需要执行 home 函数
    
    // 初始条件下爪子是张开的，机械臂回到 home 位置
    gripper.move(0.09, 0.1, 20);
    robot.executePrimitive("Home()");
    while (flexiv::utility::parsePtStates(robot.getPrimitiveStates(),
                                          "reachedTarget") != "1") {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } // 完成初始化，这里达到 home 位置

    robot.setMode(
            flexiv::Mode::RT_JOINT_POSITION); // 根据输入使用实时 joint 速度位置控制

    // 读取数据直到EOF
    for (std::string line; std::getline(std::cin, line);) {
        try {
            // 解析JSON数据
            auto j = json::parse(line);

            // 检查键是否存在以及是否为预期的类型
            if (j.contains("joint_positions") && j["joint_positions"].is_array() &&
                j.contains("joint_velocities") && j["joint_velocities"].is_array()) {

                // 从JSON对象中提取向量
                std::vector<double> joint_positions =
                        j["joint_positions"].get<std::vector<double>>();
                std::vector<double> joint_velocities = {0, 0, 0, 0, 0, 0, 0};  // 不用速度来看效果还挺不错的
                bool effort = j["joint_efforts"].get<bool>();  
                // 读取上升沿 作为判断夹爪开闭的 flag
                if (last_flag && !effort) {
                    is_gripper_closed = !is_gripper_closed;
                }
                last_flag = effort;

                robot.streamJointPosition(joint_positions, joint_velocities,
                                          default_acc);

                robot.getRobotStates(robotStates);
                auto initPose = robotStates.tcpPose;  // 获取实时位姿信息
                std::cout<<std::setprecision(3) << "current TCP pose: [x: " << initPose[0] << " y: " << initPose[1] << " z:" << initPose[2]
                          << "], unit: m, gripper state: " << (is_gripper_closed ? "close" : "open") << "\r";
        
                if (!is_gripper_closed) {
                    gripper.move(0.09, 0.1, 10);
                } else {
                    gripper.move(0.02, 0.1, 10);
                }
            }
        }
        catch (json::parse_error &e) {
            std::cerr << "Parse error: " << e.what() << std::endl;
            // 处理错误或跳过无效的数据包
        }
    }

    robot.stop();

    return 0;
}
