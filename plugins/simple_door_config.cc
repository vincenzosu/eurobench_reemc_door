#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdlib.h>
#include "LUT.h"

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/service_client.h"

#include "eurobench_bms_msgs_and_srvs/MadrobBenchmarkParams.h"


namespace gazebo {

  
  class SimpleDoorConfig : public ModelPlugin {
    
    // node uses for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // ROS subscriber
    private: ros::Subscriber rosSub;
    // ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    // thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    // LUT vector
    private: int currentLUT[181];  
    
    // Pointer to the model
    private: physics::ModelPtr model;
    
    private: physics::LinkPtr link;
    
    private: physics::JointPtr joint;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  
    public: SimpleDoorConfig() : ModelPlugin() {
    
    }
  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      // Store the pointer to the model, link and joint

      this->model = _parent;
      this->joint=  this->model->GetJoint("door_simple::joint_frame_door");

      if (const char* direction = std::getenv("GAZEBO_DOOR_MODEL_DIRECTION")){
        
        if(std::strcmp(direction,"push") ==0){
          joint->SetUpperLimit(0,1.57);
          joint->SetLowerLimit(0,-0.03);
        }
        else if(std::strcmp(direction,"pull") ==0){
          joint->SetUpperLimit(0,0.03);
          joint->SetLowerLimit(0,-1.57);
        }
        else if(std::strcmp(direction,"pushpull") ==0){
          joint->SetUpperLimit(0,1.57);
          joint->SetLowerLimit(0,-1.57); 
        }
        else {
          std::cerr << "Bad door direction: "<< direction << " , [push][pull][pushpull]" << std::endl;
        }

      }

      if (const char* selfClose = std::getenv("GAZEBO_DOOR_MODEL_SELFCLOSE")){
        
        if(std::strcmp(selfClose,"y") ==0){
          //index,spring_stiffnes,damping,spring_zero_load_position
          joint->SetStiffnessDamping(0,1.5,0.1,0.0);  
       
        }
        else if(std::strcmp(selfClose,"n") ==0){

        }

        else {
          std::cerr << "Bad door self_closing argument: "<< selfClose << " , [n][y]" << std::endl;
        }

      }
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleDoorConfig::OnUpdate, this));
    }



    // Called by the world update start event
    public: void OnUpdate() {
            
      eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response response = getBenchParams();	
      //srv.response.benchmark_type << ", "<< srv.response.door_opening_side << ", "<< srv.response.robot_approach_side
      setLUTVector(response.benchmark_type, response.door_opening_side);
      
      double angle = this->joint->GetAngle(0).Degree();
      float force = getForceFromLutValues(angle, response.door_opening_side);
      this->joint->SetForce(0, force);
      
      std::cerr << "********* I am changing he LUT values"<<
      ", with angle: "<<angle<<" and force "<< force <<", door dir: "
      << std::getenv("GAZEBO_DOOR_MODEL_DIRECTION") << std::endl;
    }
    
    private: eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response getBenchParams() {
  
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams>
                                                    ("madrob/gui/benchmark_params");
        eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams srv;
        if (client.call(srv)) {
            //std::cerr<<"****RESPONSE: " <<  srv.response.benchmark_type << ", "
            //<< srv.response.door_opening_side << ", "<< srv.response.robot_approach_side << std::endl; 
        } else {
            ROS_ERROR("Failed to call service madrob/gui/benchmark_params");
            //return new eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response();
        }
        return srv.response;
    }
    
    private: float getForceFromLutValues(double angle, std::string door_opening_side) {
    // interpolate the LUT

        float p = static_cast<float>(angle); 

        //float p = (position + 1.0f) * 90.0f;

        float p_ = floorf(p);
        float r_ = p - p_;
        int32_t idx = (size_t)(p_);

        if(idx < 0) {
            idx = 0;
            r_ = 0.0f;
        }

        if(idx > 179) {
            idx = 179;
            r_ = 1.0f;
        }

        float tmp_braking_force = 0.0f;
        std::cerr<<"**** IDX: " <<    idx << std::endl;
        if(door_opening_side.compare("CW")) {
            tmp_braking_force = currentLUT[idx] * (1.0f - r_) + currentLUT[idx+1] * r_;
        } else if (door_opening_side.compare("CCW")){
            tmp_braking_force = currentLUT[idx] * (1.0f - r_) + currentLUT[idx+1] * r_;
        } else {
            // This should not happen
            tmp_braking_force = 0.0f;
        }
        return tmp_braking_force;
    }
    
    
    private: void setLUTVector(std::string benchmark_type, std::string door_opening_side){
        if (benchmark_type.compare("No Force") == 0){
            memcpy(currentLUT, no_force, sizeof(currentLUT));
        } else if (benchmark_type.compare("Constant Force") == 0){
            memcpy(currentLUT, constant_force, sizeof(currentLUT));
        } else if (benchmark_type.compare("Sudden Force") == 0){
             if (door_opening_side.compare("CCW") == 0){
                memcpy(currentLUT, sudden_force_ccw, sizeof(currentLUT));
                std::cerr<<"*************************SUDDEN FORCE CCW " << std::endl;
             } else {
                memcpy(currentLUT, sudden_force_cw, sizeof(currentLUT));
                std::cerr<<"*************************SUDDEN FORCE CW " << std::endl;
             }
        } else if (benchmark_type.compare("Sudden Ramp") == 0){
              if (door_opening_side.compare("CCW") == 0){
                memcpy(currentLUT, sudden_ramp_ccw, sizeof(currentLUT));
                std::cerr<<"*************************SUDDEN RAMP CCW " << std::endl;
             } else {
                memcpy(currentLUT, sudden_ramp_cw, sizeof(currentLUT));
                std::cerr<<"*************************SUDDEN RAMP CW " << std::endl;
             }
        } else if (benchmark_type.compare("Wind Ramp") == 0){
              if (door_opening_side.compare("CCW") == 0){
                std::cerr<<"*************************WIND RAMP CCW " << std::endl;
                memcpy(currentLUT, wind_ramp_ccw, sizeof(currentLUT));
             } else {
                std::cerr<<"*************************WIND RAMP CW " << std::endl;
                memcpy(currentLUT, wind_ramp_cw, sizeof(currentLUT));
             }
        }
    }
   
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SimpleDoorConfig);
}

