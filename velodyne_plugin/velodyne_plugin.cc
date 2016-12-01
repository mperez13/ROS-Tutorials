#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//Added for Create an API
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//header files to add ROS transport
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo{
    //A plugin to control a Velodyne sensor.
	class VelodynePlugin : public ModelPlugin{
    	//Constructor
		public: VelodynePlugin() {}

		/*The Load function is called by Gazebo when the plugin is inserted into simulation.*/
		/*_model: a pointer to the plugin's SDF element*/
		/*_sdf: a pointer to the plugin's SDF element*/
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
			//just output a message for now
			//std::cerr << "\nThe velodyne plugin is attach to model[" << _model->GetName() << "]\n";

			/*Modify Load function to control Velodyne's joint*/
			//safety check
			if(_model->GetJointCount() == 0){
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}

			/*Store the model pointer for convenience*/
			this->model = _model;

			/*Get the 1st joint (making assumption that model has one rotational joint)*/
			this->joint = _model->GetJoints()[0];

			/*Setup a P-Controller, w/ a gain of 0.1*/
			this->pid = common::PID(0.1, 0, 0);

			/*Apply P-Controller to the joint*/
			this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

			/*Added for Plugin Configuration*/
			//Set default velocity to 0
			double velocity = 0;

			//Check velocity element exists, then read the value
			if(_sdf->HasElement("velocity")){
				velocity = _sdf->Get<double>("velocity");
			}

			this->SetVelocity(velocity);

			/*Create the node*/
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init(this->model->GetWorld()->GetName());

			/*Create a topic name*/
			std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

			/*subscribe to the topic & register a callback*/
			this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);

			/*ADDED TO ADD ROS TRANSPORT*/
			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "gazebo_client",
			      ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::Float32>(
			      "/" + this->model->GetName() + "/vel_cmd",
			      1,
			      boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
			      ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  boost::thread(std::bind(&VelodynePlugin::QueueThread, this));

			/*Set joint's target velocity (just for demonstration purposes)*/
			//this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);
			//modified for Plugin Configuration & then moved to function SetVelocity
			//this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);
		} 

		/*Added for Create an API*/
		// Set velocity of the Velodyne
		// _vel: SetVelocity(cons double &_vel)
		public: void SetVelocity(const double &_vel){
			// set joint's target velocity
			this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
		}

		/* Handle incoming message*/
		/* _msg: Repurpose a vector3 message; this function will only use the x component */
		private: void OnMsg(ConstVector3dPtr &_msg){
			this->SetVelocity(_msg->x());
		}


		/*Functions to ADD ROS TRANSPORT*/
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

		//A node used for transport
		private: transport::NodePtr node;

		// A subscriber to a named topic.
		private: transport::SubscriberPtr sub;

	    // Pointer to the model.
		private: physics::ModelPtr model;

	    // Pointer to the joint.
		private: physics::JointPtr joint;

	    // A PID controller for the joint.
		private: common::PID pid;

	    // A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		private: ros::Subscriber rosSub;

		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// A thread the keeps running the rosQueue
		private: boost::thread rosQueueThread;

	};

	/*Tell Gazebo about this plugin, so Gazebo can call Load on this plugin*/
	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif