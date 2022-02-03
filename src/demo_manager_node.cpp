
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <realtime_tools/realtime_publisher.h>
#include <demo_manager/demo_manager_node.h>

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>



using namespace demo_manager;

std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
std::array<double, 7> q_init{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
// TODO: rosservice list -n per vedere che nodo offre lo switch
int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_manager_node");
    ros::NodeHandle nh;

    std::string start_controller = "joint_reconfiguration_demo";
    std::string stop_controller = "";
    bool ok = switch_controller(start_controller,stop_controller);

    if(ok)
        std::cout << "Lo switch dei controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch dei controller non è andato a buon fine " << std::endl;

    ros::Publisher command_pb = nh.advertise<std_msgs::Float64MultiArray>("joint_commands", 1);
    //ros::Subscriber state_sub = nh.subscribe<sensor_msgs::JointState>();


    ros::Rate loop_rate(1000);
    double begin = ros::Time::now().toSec();
    double t;
    

    while (ros::ok())
    {
        
        t = ros::Time::now().toSec() - begin; // tempo trascorso
        double tau = t/demo_manager::Tf; // asse dei tempi normalizzato

        std_msgs::Float64MultiArray command_msg;

        if(tau<1){ 
        // Costruzione del messaggio che invia il comando q(t) = q_init + (q_goal - q_init)*q(t/tf)
            for(int i = 0; i< 7; i++){
                command_msg.data.push_back(q_init[i] + (q_goal[i] - q_init[i])*(6*pow(tau,5)-15*pow(tau,4)+10*pow(tau,3)) );
                }
        }
        else{
            for(int i = 0; i< 7; i++){
                command_msg.data.push_back(q_goal[i]);
                }
        }


        command_pb.publish(command_msg);

        ros::spinOnce();
        loop_rate.sleep();
        
    }









    
    return 0;

}


