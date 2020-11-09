
#include <thread>
#include <chrono>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try {
        franka::Robot robot(argv[1]);

        while(true) {

            robot.read([](const franka::RobotState& robot_state) {
              // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
              // should not be done in a control loop.
              // std::cout << robot_state.q << std::endl;
              std::array<double, 7> current_joints = robot_state.q;
              std::cout << "q  =  [ " << current_joints[0] << "   "  << current_joints[1] << "   "  << current_joints[2]  << "   "
                                      << current_joints[3] << "   "  << current_joints[4] << "   "  << current_joints[5]  << "   "
                                      << current_joints[6] << "  ]" << std::endl;
              return false;
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    std::cout << "Done." << std::endl;

  return 0;
}
