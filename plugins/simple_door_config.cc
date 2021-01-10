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
      
      getBenchParams();	

      
      double angle = this->joint->GetAngle(0).Degree();
      float force = getForceFromLutValues(angle);
      this->joint->SetForce(0, force);
      
      
      std::cerr << "********* I am changing he LUT values"<<
      ", with angle: "<<angle<<" and force "<< force <<", door dir: "<< std::getenv("GAZEBO_DOOR_MODEL_DIRECTION") << std::endl;
    }
    
    private: void getBenchParams() {
    
        // *********************** dal modulo python
  /*      get_benchmark_params = rospy.ServiceProxy('madrob/gui/benchmark_params', MadrobBenchmarkParams)
        except rospy.ServiceException, e:
            print "ServiceProxy failed: %s"%e
            exit(0)
        response = get_benchmark_params()
        ebws.current_benchmark_name = response.benchmark_type
        ebws.current_door_opening_side = response.door_opening_side
        ebws.current_robot_approach_side = response.robot_approach_side */
          //fine del modulo python
          // ****************** dal tutorial
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams>("madrob/gui/benchmark_params");
          //ros::ServiceClient client = rosNode->serviceClient<eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams>("madrob_benchmark_params");
        
        eurobench_bms_msgs_and_srvs::MadrobBenchmarkParams srv;
      //  srv.request.a = atoll(argv[1]);
       // srv.request.b = atoll(argv[2]);
        if (client.call(srv)) {
            //ROS_INFO("Sum: %ld", (long int)srv.response.benchmark_type);
            std::cerr<<"****RESPONSE: " <<  srv.response.benchmark_type << ", "<< srv.response.door_opening_side << ", "<< srv.response.robot_approach_side << std::endl; 
        } else {
            ROS_ERROR("Failed to call service ");
            return;
        }
          // ****************** fine dal tutorial
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

