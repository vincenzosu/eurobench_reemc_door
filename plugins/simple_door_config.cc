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
  
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    private: int currentLUT[];  
  
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
          joint->SetStiffnessDamping(0,1.5,0.1,0.0);  //index,spring_stiffnes,damping,spring_zero_load_position
       
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
      
      // velocity = (position - last_position) / (((float)delta_t) / 1000.0f);
      
      eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response response = getBenchParams();	
      //srv.response.benchmark_type << ", "<< srv.response.door_opening_side << ", "<< srv.response.robot_approach_side
      setLUTVector(response.benchmark_type, response.door_opening_side);
      
      double angle = this->joint->GetAngle(0).Degree();
      float force = getForceFromLutValues(angle);
      this->joint->SetForce(0, force);
      
      
      std::cerr << "********* I am changing he LUT values"<<
      ", with angle: "<<angle<<" and force "<< force <<", door dir: "<< std::getenv("GAZEBO_DOOR_MODEL_DIRECTION") << std::endl;
    }
    
    private: eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response getBenchParams() {
  
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams>
                                                    ("madrob/gui/benchmark_params");
        eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams srv;
        if (client.call(srv)) {
            //std::cerr<<"****RESPONSE: " <<  srv.response.benchmark_type << ", "<< srv.response.door_opening_side << ", "<< srv.response.robot_approach_side << std::endl; 
        } else {
            ROS_ERROR("Failed to call service ");
            return new eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams::Response();
        }
        return srv.response;
    }
    
    private: float getForceFromLutValues(double angle) {
    // interpolate the LUT

        float position = static_cast<float>(angle); 

        float p = (position + 1.0f) * 90.0f;

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
        
        // ************* TOREMOVE ******************
        int MADROB_DOOR_STATE_LUT_CW = 0;
        int MADROB_DOOR_STATE_LUT_CCW = 1;
        int state = 0;
        // ************* END TOREMOVE ******************
                
        /*if(state == MADROB_DOOR_STATE_LUT_CW) {
            tmp_braking_force = lut_cw_angle[idx] * (1.0f - r_) + lut_cw_angle[idx+1] * r_;
        } else if (state == MADROB_DOOR_STATE_LUT_CCW){
            tmp_braking_force = lut_ccw_angle[idx] * (1.0f - r_) + lut_ccw_angle[idx+1] * r_;
        } else {
            // This should not happen
            tmp_braking_force = 0.0f;
        }*/
        return tmp_braking_force;
    }
    
    
    private: void setLUTVector(std::string benchmark_type, std::string door_opening_side){
        if (benchmark_type.compare("No Force") == 0){
            currentLUT = no_force;    
        } else if (benchmark_type.compare("Constant Force") == 0){
            currentLUT = constant_force;            
        } else if (benchmark_type.compare("Sudden Force") == 0){
             if (door_opening_side.compare("CCW") == 0){
                currentLUT = sudden_force_ccw;
                std::cerr<<"*************************SUDDEN FORCE CCW " << std::endl;
             } else {
                currentLUT = sudden_force_cw;
                std::cerr<<"*************************SUDDEN FORCE CW " << std::endl;
             }
        } else if (benchmark_type.compare("Sudden Ramp") == 0){
              if (door_opening_side.compare("CCW") == 0){
                currentLUT = sudden_ramp_ccw;
                std::cerr<<"*************************SUDDEN RAMP CCW " << std::endl;
             } else {
                currentLUT = sudden_ramp_cw;
                std::cerr<<"*************************SUDDEN RAMP CW " << std::endl;
             }
        } else if (benchmark_type.compare("Wind Ramp") == 0){
              if (door_opening_side.compare("CCW") == 0){
                currentLUT = wind_ramp_ccw;
                std::cerr<<"*************************WIND RAMP CCW " << std::endl;
             } else {
                std::cerr<<"*************************WIND RAMP CW " << std::endl;
                currentLUT = wind_ramp_cw;
             }
        }
    }
    

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: physics::JointPtr joint;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SimpleDoorConfig);
}

