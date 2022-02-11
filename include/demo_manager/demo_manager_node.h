
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <sensor_msgs/JointState.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>


using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;




namespace demo_manager{


    std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    
    std::array<double, 7> q_init{{-1,-1,-1,-1,-1,-1,-1}};
    // q_goal: [6.946831287570189e-05, -0.7849671420412789, 0.00016540849472852385, -2.356101957380501, -0.00021844535540237149, 1.5710111441744694, 0.7849975134233633]

    bool initial_read = false;
    double Tf = 10;

    bool switch_controller(const std::string& start_controller, const std::string& stop_controller){

        SwitchControllerRequest switch_req;
        SwitchControllerResponse switch_res;
        
        switch_req.start_controllers.push_back(start_controller);
        switch_req.stop_controllers.push_back(stop_controller);
        switch_req.strictness = 1; // BEST_EFFORT 
        switch_req.start_asap = false;
        switch_req.timeout = 0.0;
        ros::service::call("/controller_manager/switch_controller",switch_req,switch_res);     
        if(switch_res.ok==true)
        
            if(start_controller != "" && stop_controller !="")
                ROS_INFO_STREAM("Attivato " << start_controller << " e fermato "<< stop_controller);

            else if (start_controller != "" && stop_controller =="")
                ROS_INFO_STREAM("Attivato " << start_controller);

            else if (start_controller == "" && stop_controller !="")
                ROS_INFO_STREAM("Fermato " << stop_controller);   
        else
            ROS_INFO("Operazione switch non riuscita");     

        return switch_res.ok;
    }
    

    void stateCB(const sensor_msgs::JointState::ConstPtr& msg){
        for(int i = 0 ; i<7 ;i++)
            q_init[i] = msg->position[i];

        initial_read = true;
    }


}