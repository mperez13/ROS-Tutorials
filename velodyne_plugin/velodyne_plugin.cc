#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/*ADDED FOR ROS TRANSPORT*/
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo{
  // \brief A plugin to control a Velodyne sensor
	class VelodynePlugin : public ModelPlugin{
		public : VelodynePlugin() { }

    // \brief Load function is called by Gazebo when the plugin is inserted into simulation
    // \param[in] _model : A pointer to the model that this plugin is attached to
    // \param[in _sdf : A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      // Safety check
			if (_model->GetJointCount() == 0){
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}

      // Store the model pointer for convenience.
			this->model = _model;

      // Get the first joint (assumption about the model having one joint that is the rotational joint)
			this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
			this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
			this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

      //default to zero velocity
			double velocity = 0;

      //check velocity element exists, then read value
			if(_sdf->HasElement("velocity")){
				velocity = _sdf->Get<double>("velocity");
			}

			this->SetVelocity(velocity);

      // Create the node
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init(this->model->GetWorld()->GetName());

      // Create a topic name
			std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the toic, and register a callback
			this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);

      /*****************ADDED FOR ROS TRANSPORT***************/
      // Initialize ros, if it has not already been initialized.
			if (!ros::isInitialized()){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
			}

// Create our ROS node. This acts in a similar manner to the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so = 
			ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/vel_cmd",
				1, boost::bind(&VelodynePlugin::OnRosMsg, 
					this, _1), 
				ros::VoidPtr(), &this->rosQueue);

			this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
			this->rosQueueThread = boost::thread(std::bind(&VelodynePlugin::QueueThread, this));
    /********************************************************/
		}

    // \brief Handle incoming message
    // \param[in] _msg Repurpose a vector3 message. This function will only use the x component.
		private: void OnMsg(ConstVector3dPtr &_msg){
			this->SetVelocity(_msg->x());
		}

    // \brief : Set the velocity of the Velodyne
// \param[in] : _vel New target velocity
		public: void SetVelocity(const double &_vel){
  // Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(
				this->joint->GetScopedName(), _vel);
		}

    /***************ADDED FOR ROS TRANSPORT**************/
    // Handle an incoming message from ROS
// _msg: A float value that is used to set the velocity of the Velodyne
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg){
			this->SetVelocity(_msg->data);
		}

// \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->rosNode->ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
/********************************************************/
    // \brief A node used for transport
		private: transport::NodePtr node;

    // \brief A subscriber to a named topic.
		private: transport::SubscriberPtr sub;

    // \brief Pointer to the model
		private: physics::ModelPtr model; 

    // \brief Pointer to the joint
		private: physics::JointPtr joint;

    // \brief A PID controller for the joint
		private: common::PID pid;

    /**************ADDED FOR ROS TRANSPORT***************/
    // A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS subscriber
		private: ros::Subscriber rosSub;

    // A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
		private: boost::thread rosQueueThread;

	};

  //Tell Gazebo about this plugin, so that Gazebo can tell Load on this plugin
	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif