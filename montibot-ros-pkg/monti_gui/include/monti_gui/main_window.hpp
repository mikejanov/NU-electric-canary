/**
 * @file /include/monti_gui/main_window.hpp
 *
 * @brief Qt based gui for monti_gui.
 *
 * @date November 2010
 **/
#ifndef monti_gui_MAIN_WINDOW_H
#define monti_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace monti_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void on_publishpushButton_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    /******************************************
    ** Monti connections
    *******************************************/

    void on_pushButton_connectMonti_clicked(bool check);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace monti_gui

#endif // monti_gui_MAIN_WINDOW_H
