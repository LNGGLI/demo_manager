
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <demo_manager/demo_manager_node.h>
#include <ros/ros.h>

using namespace demo_manager;


int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_manager_node");
    ros::NodeHandle nh;

    std::string start_controller = "joint_reconfiguration_demo";
    std::string stop_controller = "";
    bool ok = switch_controller(start_controller,stop_controller);

    if(ok)
        std::cout << "Lo switch dei controller è stato effettuato!" << std::endl;

    









    
    return 0;

}


