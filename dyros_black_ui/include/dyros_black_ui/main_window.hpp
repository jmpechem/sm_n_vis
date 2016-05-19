/**
 * @file /include/dyros_black_ui/main_window.hpp
 *
 * @brief Qt based gui for dyros_black_ui.
 *
 * @date November 2010
 **/
#ifndef dyros_black_ui_MAIN_WINDOW_H
#define dyros_black_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <vector>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPushButton>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dyros_black_ui {

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
    void on_manuButton_clicked(bool check );
    void on_button_joint_control_clicked(bool check);
    void on_button_power_on_clicked(bool check);
    void on_button_minus_clicked(bool check);
    void on_button_plus_clicked(bool check);
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateJointView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    std::vector<int> jointID;

};
/*
class QJointControlButtonFrame : public QFrame {
Q_OBJECT

public:
    QJointControlButtonFrame(QWidget *parent=0)
        : QFrame(parent),
    button_minus1deg(0), button_minus5deg(0), button_plus1deg(0), button_plus5deg(0)
    {   }


    virtual ~QJointControlButtonFrame()
    {
        if(button_minus1deg)
            delete button_minus1deg;
        if(button_minus5deg)
            delete button_minus5deg;
        if(button_plus1deg)
            delete button_plus1deg;
        if(button_plus5deg)
            delete button_plus5deg;
    }

protected:
    QPushButton *button_minus1deg;
    QPushButton *button_minus5deg;
    QPushButton *button_plus1deg;
    QPushButton *button_plus5deg;
};
*/
}  // namespace dyros_black_ui

#endif // dyros_black_ui_MAIN_WINDOW_H
