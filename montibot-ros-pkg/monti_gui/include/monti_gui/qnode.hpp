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

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher monti_connection_pub;
	ros::Publisher monti_config_pub;
	ros::Publisher monti_move_cmd_pub;
	ros::Subscriber monti_state_sub;
    QStringListModel logging_model;
};

}  // namespace monti_gui

#endif /* monti_gui_QNODE_HPP_ */
