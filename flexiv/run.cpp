/**
 * @example basics3_primitive_execution.cpp
 * This tutorial executes several basic robot primitives (unit skills). For
 * detailed documentation on all available primitives, please see [Flexiv
 * Primitives](https://www.flexiv.com/primitives/).
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Robot.hpp>
#include <flexiv/Utility.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

/** @brief Print tutorial description */
void printDescription() {
  std::cout << "This tutorial executes several basic robot primitives (unit "
               "skills). For "
               "detailed documentation on all available primitives, please see "
               "[Flexiv "
               "Primitives](https://www.flexiv.com/primitives/)."
            << std::endl
            << std::endl;
}

/** @brief Print program usage help */
void printHelp() {
  // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
  // clang-format on
}

int main(int argc, char* argv[]) {
  // Program Setup
  // =============================================================================================
  // Logger for printing message with timestamp and coloring
  flexiv::Log log;

  // Parse parameters
  if (argc < 3 ||
      flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
    printHelp();
    return 1;
  }
  // IP of the robot server
  std::string robotIP = argv[1];
  // IP of the workstation PC running this program
  std::string localIP = argv[2];

  // Print description
  log.info("Tutorial description:");
  printDescription();

  try {
    // RDK Initialization
    // =========================================================================================
    // Instantiate robot interface
    flexiv::Robot robot(robotIP, localIP);

    // Clear fault on robot server if any
    if (robot.isFault()) {
      log.warn("Fault occurred on robot server, trying to clear ...");
      // Try to clear the fault
      robot.clearFault();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      // Check again
      if (robot.isFault()) {
        log.error("Fault cannot be cleared, exiting ...");
        return 1;
      }
      log.info("Fault on robot server is cleared");
    }

    // Enable the robot, make sure the E-stop is released before enabling
    log.info("Enabling robot ...");
    robot.enable();

    // Wait for the robot to become operational
    while (!robot.isOperational()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    log.info("Robot is now operational");

    // Execute Primitives
    // =========================================================================================
    // Switch to primitive execution mode
    robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);

    // (3) Move robot TCP to a target position in world (base) frame
    // -----------------------------------------------------------------------------------------
    // Required parameter:
    //   target: final target position
    //       [pos_x pos_y pos_z rot_x rot_y rot_z ref_frame ref_point]
    //       Unit: m, deg
    // Optional parameter:
    //   waypoints: waypoints to pass before reaching final target
    //       (same format as above, but can repeat for number of waypoints)
    //   maxVel: maximum TCP linear velocity
    //       Unit: m/s
    // NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle
    // around X axis
    for (int i = 1; i <= 30; i++) {
      log.info("Executing primitive " + std::to_string(i));

      std::string file_path =
          "/home/tianzong/Workspace/LLM-Robot/temp/order/joint_positions_" +
          std::to_string(i) + ".txt";
      std::ifstream file(file_path);
      std::stringstream buffer;
      buffer << file.rdbuf();
      std::string primitive = buffer.str();
      file.close();

      // Send command to robot
      robot.executePrimitive(primitive);

      // The [Move] series primitive won't terminate itself, so we determine if
      // the robot has reached target location by checking the primitive state
      // "reachedTarget = 1" in the list of current primitive states, and
      // terminate the current primitive manually by sending a new primitive
      // command.
      while (flexiv::utility::parsePtStates(robot.getPrimitiveStates(),
                                            "reachedTarget") != "1") {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // All done, stop robot and put into IDLE mode
    robot.stop();

  } catch (const flexiv::Exception& e) {
    log.error(e.what());
    return 1;
  }

  return 0;
}
