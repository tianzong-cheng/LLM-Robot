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
#include <iostream>
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

    // (1) Go to home pose
    // -----------------------------------------------------------------------------------------
    // All parameters of the "Home" primitive are optional, thus we can skip the
    // parameters and the default values will be used
    log.info("Executing primitive: Home");

    // Send command to robot
    robot.executePrimitive(
        "MoveL(target=0.6 -0.2 0.6 180 0 180 WORLD WORLD_ORIGIN, maxVel=0.2)");

    // Wait for the primitive to finish
    while (flexiv::utility::parsePtStates(robot.getPrimitiveStates(),
                                          "reachedTarget") != "1") {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // All done, stop robot and put into IDLE mode
    robot.stop();

  } catch (const flexiv::Exception& e) {
    log.error(e.what());
    return 1;
  }

  return 0;
}
