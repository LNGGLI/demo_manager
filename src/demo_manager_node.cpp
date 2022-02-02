
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <demo_controllers/joint_reconfiguration_demo.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;

int main(int argc, char** argv) {

  ros::init(argc, argv, "demo_manager_node");
  ros::NodeHandle nh;

  SwitchControllerRequest switch_req;
  SwitchControllerResponse switch_res;
  
  switch_req.start_controllers.push_back("joint_reconfiguration_demo");
  switch_req.stop_controllers = {};
  switch_req.strictness = 1; // BEST_EFFORT 
  switch_req.start_asap = false;
  switch_req.timeout = 0.0;
  ros::service::call("/controller_manager/switch_controller",switch_req,switch_res);     
  if(switch_res.ok==true)
      ROS_INFO("Il controllere e' partito");
  else
      ROS_INFO("Il controller non e' riuscito a partire");       

    

//   ros::Rate r(10); // 10 hz
//    while (!ros::service::call("controller_manager/load_controller", loader_srv)){

//     ROS_INFO("Joint reconfiguration non e' stato caricato correttamente");
//     ros::spinOnce();
//     r.sleep();
    
//     }

    

  
//exit
  return 0;
  
}
