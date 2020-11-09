

#include <yarp/os/all.h>
#include <event-driven/all.h>

#include "panda-control.h"


using namespace ev;
using namespace yarp::os;


void setRobotCollisionBehavior(franka::Robot* robot) {
    robot->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot->setJointImpedance({{2000, 2000, 2000, 1500, 1500, 1000, 1000}});
    robot->setCartesianImpedance({{2000, 2000, 2000, 200, 200, 200}});
}


void executeSinMotion(franka::Robot* robot, double sin_amplitude, double sin_period) {

    double time = 0.0;
    std::array<double, 16> initial_pose;

    setDefaultBehavior(robot);

    try {
        robot->control([&sin_amplitude, &sin_period, &time, &initial_pose](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();

            if (time == 0.0) {
                initial_pose = robot_state.O_T_EE_c;
                std::cout << sin_amplitude << sin_period << std::endl;
            }

//            double delta = sin_amplitude*std::sin(2.0*M_PI*time/sin_period);

            double delta = sin_amplitude*(std::sin(M_PI_4*(1-std::cos(M_PI/(sin_period/2)*time))));

            std::array<double, 16> new_pose = initial_pose;
            new_pose[13] += delta;



//                      constexpr double kRadius = 0.2;
//                      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
//                      double delta_x = kRadius * std::sin(angle);
//                      double delta_z = kRadius * (std::cos(angle) - 1);

//                      std::array<double, 16> new_pose = initial_pose;
//                      new_pose[12] += delta_x;
//                      new_pose[14] += delta_z;

//                      std::cout << delta << "    " << delta_x << std::endl;



            std::cout << delta << "   " << new_pose[14]+delta << std::endl;

            if (time >= 11.0) {
                std::cout << std::endl << "Finished motion" << std::endl;
                return franka::MotionFinished(new_pose);
            }
            return new_pose;
        });
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        // return -1;
    }
}


class PandaCartesianController : public RFModule, public Thread {

    private:

        vReadPort< vector<AE> > input_port;
        vWritePort output_port;
        RpcServer handler_port;

        std::string robot_ip;
        franka::Robot *robot;

        float sin_amplitude, sin_period;
        float t0, t;

        bool execute_sin;

    public:

    PandaCartesianController() { robot = nullptr; }

    virtual bool configure(yarp::os::ResourceFinder& rf) {
        //set the module name used to name ports
        setName((rf.check("name", Value("/panda_cart_controller")).asString()).c_str());

        //open io ports
        if(!input_port.open(getName() + "/AE:i")) {
            yError() << "Could not open input port";
            return false;
        }

        output_port.setWriteType(AE::tag);
        if(!output_port.open(getName() + "/AE:o")) {
            yError() << "Could not open input port";
            return false;
        }

        //open rpc port
        handler_port.open(getName()+"/rpc:i");
        attach(handler_port);

        //read flags and parameters
        robot_ip = rf.check("robot_ip", Value("172.16.0.2")).asString();

        //do any other set-up required here
        try {
            robot = new franka::Robot(robot_ip);
//            move2HomePosition(robot, 0.1);
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            setRobotCollisionBehavior(robot);
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Running error recovery..." << std::endl;
            robot->automaticErrorRecovery();
            return -1;
        }

        sin_amplitude = 0;
        sin_period = 0;

        execute_sin = false;

        return Thread::start();
    }

    virtual double getPeriod() {
        return 0.1;
    }

    bool interruptModule() {
        return Thread::stop();
    }

    void onStop() {
        if (robot != nullptr)
            delete robot;

        input_port.close();
        output_port.close();
        handler_port.close();
    }

    virtual bool updateModule() {
        if (execute_sin == true) {
//            yInfo() << 0.25+sin_amplitude*sin(2.0*M_PI*(Time::now()-t0)/sin_period);
            go2Position(robot, 0.5, 0.0, 0.35+sin_amplitude*sin(2.0*M_PI*(Time::now()-t0)/sin_period), getPeriod());
        }
        return Thread::isRunning();
    }

    void run() {
        Stamp yarpstamp;
        deque<AE> out_queue;

        while(true) {
            const vector<AE> * q = input_port.read(yarpstamp);
            if(!q || Thread::isStopping()) return;

            for(auto &qi : *q) {
                out_queue.push_back(qi);
            }

            if(out_queue.size()) {
                output_port.write(out_queue, yarpstamp);
                out_queue.clear();
            }
        }
    }

    //rpc respond function
    bool respond(const Bottle &command, Bottle &reply) {
        string cmd=command.get(0).asString();
        if (cmd=="help") {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- printJointsPosition (print 7D vector containing joint values)");
            reply.addString("- printCartesianPose (print T_o_ee matrix)");
            reply.addString("- moveHome (bring robot back to home position)");
            reply.addString("- move (move to specified x y z coordinates (no orientation))");
            reply.addString("- moveWithOrient (move to specified x y z coordinates with orientation ex ey ez and angle[rad])");
            reply.addString("- sin (generates a sinusoidal trajectory for the EE with given amplitude and period");
            reply.addString("- sinOff (turn off the sinusoidal motion");
            reply.addString("- quit");
        } else if (cmd=="printJointsPosition") {
            printJointsPosition(robot);
            reply.addString("printed q vector on main terminal!");
        } else if (cmd=="printCartesianPose") {
            printCartesianPose(robot);
            reply.addString("printed p vector on main terminal!");
        } else if (cmd=="moveHome") {
            try {
                move2HomePosition(robot, 0.1);
                setRobotCollisionBehavior(robot);
            } catch (const franka::Exception& e) {
                std::cout << e.what() << std::endl;
                std::cout << "Running error recovery..." << std::endl;
                robot->automaticErrorRecovery();
                reply.addString("an error occurred during motion and automaticErrorRecovery has been launched!");
                return -1;
            }
            reply.addString("ack");
            reply.addString("Back home!");
        } else if (cmd=="move") {
            float xd = command.get(1).asDouble();
            float yd = command.get(2).asDouble();
            float zd = command.get(3).asDouble();
            float motion_duration = command.get(4).asDouble();

           go2Position(robot, xd, yd, zd, motion_duration);

           reply.addString("ack");
           reply.addString("Moving the robot!");
        } else if (cmd=="moveWithOrient") {
            float xd = command.get(1).asDouble();
            float yd = command.get(2).asDouble();
            float zd = command.get(3).asDouble();
            double ex = command.get(4).asDouble();
            double ey = command.get(5).asDouble();
            double ez = command.get(6).asDouble();
            double theta = command.get(7).asDouble();
            float motion_duration = command.get(8).asDouble();

            go2Pose(robot, xd, yd, zd, ex, ey, ez, theta, motion_duration);

            reply.addString("ack");
            reply.addString("Moving the robot!");

        } else if (cmd=="sin") {
            sin_amplitude = command.get(1).asDouble();
            sin_period = command.get(2).asDouble();

//            go2Position(robot, 0.5, 0.0, 0.25, 3.0);
//            Time::delay(2.0);
//            executeSinMotion(robot, sin_amplitude, sin_period);
            t0 = Time::now();
            execute_sin = true;

            reply.addString("ack");
            reply.addString("Moving the robot!");
        } else if (cmd=="sinOff") {
            execute_sin = false;
            move2HomePosition(robot, 0.1);
            reply.addString("ack");
            reply.addString("Sinusoid off! Back to home position.");
        } else {
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        }
        return true;
    }
};


int main(int argc, char * argv[]) {
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "pandaCartController" );
    rf.setDefaultConfigFile( "panda_cart_controller.ini" );
    rf.configure( argc, argv );

    /* create the module */
    PandaCartesianController cart_controller;
    return cart_controller.runModule(rf);
}
