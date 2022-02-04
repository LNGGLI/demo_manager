
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <realtime_tools/realtime_publisher.h>
#include <demo_manager/demo_manager_node.h>

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>



using namespace demo_manager;


int main(int argc, char** argv) {

    ros::init(argc, argv, "demo_manager_node");
    ros::NodeHandle nh;
    ros::Publisher command_pb = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("joint_commands", 1);
    ros::Subscriber state_sub = nh.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states",1,stateCB);
    
    ros::Rate loop_rate(1000); // 1kHz

    while(!initial_read){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        
    }
    
    std::cout << "Configurazione iniziale acquisita: \n";
    for(int i = 0 ; i < 7 ; i++)
        std::cout << "q" << i+1 << "= " << q_init[i]<< "\n";
    
    
    std::string start_controller = "joint_reconfiguration_pilotato";
    std::string stop_controller = "";
    bool ok = switch_controller(start_controller,stop_controller);

    if(ok)
        std::cout << "Lo switch dei controller è stato effettuato!" << std::endl;
    else
        std::cout << "Lo switch dei controller non è andato a buon fine " << std::endl;

    
    
    double begin = ros::Time::now().toSec();
    double t;
    

    while (ros::ok() && t < Tf)
    {
        
        t = ros::Time::now().toSec() - begin; // tempo trascorso
        double tau = t/Tf; // asse dei tempi normalizzato

        trajectory_msgs::JointTrajectoryPoint command_msg;

        if(tau<1){ 
        // Costruzione del messaggio che invia il comando q(t) = q_init + (q_goal - q_init)*q(t/tf)
            for(int i = 0; i< 7; i++){
                command_msg.positions.push_back(q_init[i] + (q_goal[i] - q_init[i])*(6*pow(tau,5)-15*pow(tau,4)+10*pow(tau,3)) );
                }
        }
        else{
            for(int i = 0; i< 7; i++){
                command_msg.positions.push_back(q_goal[i]);
                }
        }

        command_pb.publish(command_msg);

        loop_rate.sleep();
        
    }

    start_controller="";
    stop_controller="joint_reconfiguration_pilotato";
    switch_controller(start_controller,stop_controller);
    
        








    
    return 0;

}


