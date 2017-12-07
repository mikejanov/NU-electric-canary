/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/ros.h>
#include "../include/monti_gui/main_window.hpp"
#include <monti_msgs/Monti_Control.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monti_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/monti_head_transparent.png"));
	ui.tab_manager->setCurrentIndex(1); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_publishpushButton_clicked(bool check){
	qnode.log_hello();
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>MoNTI App</h2><p>Created By NU Electric Canary</p><p>Capstone Summer 2/Fall 2017</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "monti_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "monti_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

/*****************************************************************************
** Implementation [Monti Control]
*****************************************************************************/

//Monti Connection Status
void MainWindow::on_pushButton_connectMonti_clicked(bool check){
	qnode.update_monti_connection(true);

	ui.pushButton_connectMonti->setEnabled(false);
	ui.pushButton_disconnectMonti->setEnabled(true);
	ui.spinBox_setDrivetrain->setEnabled(true);
	ui.pushButton_setDrivetrain->setEnabled(true);
}

void MainWindow::on_pushButton_disconnectMonti_clicked(bool check){
	qnode.update_monti_connection(false);
	
	ui.pushButton_connectMonti->setEnabled(true);
	ui.pushButton_disconnectMonti->setEnabled(false);

	ui.spinBox_setDrivetrain->setEnabled(false);
    ui.pushButton_setDrivetrain->setEnabled(false);

    ui.spinBox_setNumPods->setEnabled(false);
    ui.pushButton_setNumPods->setEnabled(false);

    ui.spinBox_pod1->setEnabled(false);
    ui.spinBox_pod2->setEnabled(false);
    ui.spinBox_pod3->setEnabled(false);
    ui.spinBox_pod4->setEnabled(false);
    ui.spinBox_pod5->setEnabled(false);
    ui.pushButton_setIds->setEnabled(false);
}

//Set Monti configuration
void MainWindow::on_pushButton_setDrivetrain_clicked(bool check){
	ui.spinBox_setNumPods->setEnabled(true);
    ui.pushButton_setNumPods->setEnabled(true);
}
void MainWindow::on_pushButton_setNumPods_clicked(bool check){
	int num_pods = ui.spinBox_setNumPods->value();
	if (num_pods == 1)
	{
		ui.spinBox_pod1->setEnabled(true);
		ui.spinBox_pod2->setEnabled(false);
    	ui.spinBox_pod3->setEnabled(false);
    	ui.spinBox_pod4->setEnabled(false);
    	ui.spinBox_pod5->setEnabled(false);
		ui.pushButton_setIds->setEnabled(true);
	}
	else if (num_pods == 2)
	{
		ui.spinBox_pod1->setEnabled(true);
		ui.spinBox_pod2->setEnabled(true);
		ui.spinBox_pod3->setEnabled(false);
    	ui.spinBox_pod4->setEnabled(false);
    	ui.spinBox_pod5->setEnabled(false);
		ui.pushButton_setIds->setEnabled(true);
	}
	else if (num_pods == 3)
	{
		ui.spinBox_pod1->setEnabled(true);
		ui.spinBox_pod2->setEnabled(true);
		ui.spinBox_pod3->setEnabled(true);
    	ui.spinBox_pod4->setEnabled(false);
    	ui.spinBox_pod5->setEnabled(false);
		ui.pushButton_setIds->setEnabled(true);
	}
	else if (num_pods == 4)
	{
		ui.spinBox_pod1->setEnabled(true);
		ui.spinBox_pod2->setEnabled(true);
		ui.spinBox_pod3->setEnabled(true);
		ui.spinBox_pod4->setEnabled(true);
		ui.spinBox_pod5->setEnabled(false);
		ui.pushButton_setIds->setEnabled(true);
	}
	else if (num_pods == 5)
	{
		ui.spinBox_pod1->setEnabled(true);
		ui.spinBox_pod2->setEnabled(true);
		ui.spinBox_pod3->setEnabled(true);
		ui.spinBox_pod4->setEnabled(true);
		ui.spinBox_pod5->setEnabled(true);
		ui.pushButton_setIds->setEnabled(true);
	}
	else
	{
		ui.spinBox_pod1->setEnabled(false);
	    ui.spinBox_pod2->setEnabled(false);
	    ui.spinBox_pod3->setEnabled(false);
	    ui.spinBox_pod4->setEnabled(false);
	    ui.spinBox_pod5->setEnabled(false);
	    ui.pushButton_setIds->setEnabled(false);
	}

	ui.tableWidget_sensors->setColumnCount(2);
	ui.tableWidget_sensors->setRowCount(5);
	ui.tableWidget_sensors->verticalHeader()->setVisible(false);
	QStringList tableWidget_sensors_header;
	tableWidget_sensors_header<<"Pod ID"<<"Data";
	ui.tableWidget_sensors->setHorizontalHeaderLabels(tableWidget_sensors_header);
	ui.tableWidget_sensors->horizontalHeader()->setStretchLastSection(true);
	ui.tableWidget_sensors->setEditTriggers(QAbstractItemView::NoEditTriggers);

}

void MainWindow::on_pushButton_setIds_clicked(bool check){}

//Motion Control
uint8_t MainWindow::get_throttle(){
	//Convert the throttle scaled level to a usable value by the ROV
	int throttle_level = ui.verticalSlider_throttle->value();

	//TODO: insert switch case here for conversion

	return throttle_level;
}

void MainWindow::on_pushButton_idle_clicked(bool check){
	qnode.move_monti(0, 0); //Don't move!
}

//Standard Directions
void MainWindow::on_pushButton_forward_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(0, throttle); //DEG_0 in firmware
}

void MainWindow::on_pushButton_backward_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(4, throttle); //DEG_180 in firmware
}
void MainWindow::on_pushButton_left_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(6, throttle); //DEG_270 in firmware
}

void MainWindow::on_pushButton_right_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(2, throttle); //DEG_90 in firmware
}

//Angle Directions

void MainWindow::on_pushButton_45_tr_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(1, throttle); //DEG_45 in firmware
}

void MainWindow::on_pushButton_45_br_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(3, throttle); //DEG_135 in firmware
}

void MainWindow::on_pushButton_45_bl_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(5, throttle); //DEG_225 in firmware
}

void MainWindow::on_pushButton_45_tl_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(7, throttle); //DEG_315 in firmware
}

//Spinning Commands

void MainWindow::on_pushButton_180_r_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(9, throttle); //DEG_315 in firmware
}

void MainWindow::on_pushButton_180_l_clicked(bool check){
	int throttle = get_throttle();
	qnode.move_monti(10, throttle); //DEG_315 in firmware
}

}  // namespace monti_gui
