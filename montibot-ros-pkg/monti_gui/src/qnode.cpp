/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/monti_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monti_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		init();
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	printf("Hello!\n");
	ros::init(init_argc,init_argv,"monti_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// Add your ros communications here.
	monti_connection_pub = n.advertise<std_msgs::String>("monti/connection_status", 1000);
	monti_config_pub = n.advertise<monti_msgs::Monti_Config>("monti/config", 1000);
	monti_control_pub = n.advertise<monti_msgs::Monti_Control>("monti/control_command", 1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"monti_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
	monti_connection_pub = n.advertise<std_msgs::String>("monti/connection_status", 1000);
	// monti_config_pub = n.advertise<std_msgs::String>("monti/config", 1000);
	// monti_connection_pub = n.advertise<std_msgs::String>("monti/control_command", 1000);
	start();

	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		// msg.data = ss.str();
		// chatter_publisher.publish(msg);
		//log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log_hello(){
	ROS_INFO_STREAM("Hello I'm Working!");
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// ros::init(init_argc,init_argv,"monti_gui");
// 	if ( ! ros::master::check() ) {
// 		return false;
// 	}
// 	ros::start(); // explicitly needed since our nodehandle is going out of scope.
// 	ros::NodeHandle n;

// Monti Control Functionality
void QNode::update_monti_connection(bool connection_status){  //Connect to the monti rov, 1=connect, 0=disconnect
	std_msgs::String msg;
	std::stringstream ss;
	if (connection_status){ss << "connect";}
	else{ss << "disconnect";}
	msg.data = ss.str();
	monti_connection_pub.publish(msg);
	//log(Info,std::string("I sent: ")+msg.data);
}

void QNode::move_monti(uint8_t direction, uint8_t throttle){
	monti_control_msg.direction = direction;
	monti_control_msg.throttle = throttle;
	monti_control_msg.actuation_time = 5;

	monti_control_pub.publish(monti_control_msg);
}

void QNode::set_monti_config(uint8_t drive_type, uint8_t num_pods, uint8_t pod_ids[]){}

}  // namespace monti_gui
