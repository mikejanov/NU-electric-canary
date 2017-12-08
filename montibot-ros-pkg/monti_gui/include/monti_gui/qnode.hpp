/**
 * @file /include/monti_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef monti_gui_QNODE_HPP_
#define monti_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <monti_msgs/Monti_Control.h>
#include <monti_msgs/Monti_Config.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monti_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void log_hello();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	//Monti ROV Control Functions
	void update_monti_connection(bool connection_status);  //Connect to the monti rov, 1=connect, 0=disconnect
	void move_monti(uint8_t direction, uint8_t throttle);
	void set_monti_config(uint8_t drive_type, uint8_t num_pods, uint8_t pod_ids[]);
	void set_monti_drive_config(uint8_t drive_type);
	const char* monti_pod_ids[5];
	int num_pods;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher monti_connection_pub;
	ros::Publisher monti_config_pub;
	ros::Publisher monti_control_pub;
	ros::Subscriber monti_state_sub;
    QStringListModel logging_model;
    monti_msgs::Monti_Control monti_control_msg;
    monti_msgs::Monti_Config monti_config_msg;
};

}  // namespace monti_gui

#endif /* monti_gui_QNODE_HPP_ */
