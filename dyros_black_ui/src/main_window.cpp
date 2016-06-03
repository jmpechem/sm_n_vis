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
#include <QString>
#include "../include/dyros_black_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dyros_black_ui {

using namespace Qt;

// std::string motorID = {"R-SP", "L-SP", "R-SR", ""};
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
    , isConnected(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(jointStateUpdated()), this, SLOT(updateJointView()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/


    // | Creating UI
    // -- Table
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    QStringList horizonHeaderLabel;
    horizonHeaderLabel.append("Name");
    horizonHeaderLabel.append("Pos");
    horizonHeaderLabel.append("Torque");
    horizonHeaderLabel.append("Error");
    ui.motor_table->setHorizontalHeaderLabels(horizonHeaderLabel);

    for(int i=0; i<32; i++)
    {
        jointID.push_back(i+1);
    }

    // -- State

    autoMissionSelectVisible(0);
    QObject::connect(ui.button_power_on, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

    QObject::connect(ui.button_auto, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

    QObject::connect(ui.button_auto_door_init, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_door_open, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_door_push, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_door_reach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_door_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_door_start, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

    QObject::connect(ui.button_auto_valve_approach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_valve_close, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_valve_init, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_valve_reach, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_valve_ready, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_auto_valve_start, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

    QObject::connect(ui.button_manual, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));

    QObject::connect(ui.button_manual_joint_ctrl, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_manual_task_ctrl, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));
    QObject::connect(ui.button_manual_recog_ctrl,SIGNAL(clicked()),this,SLOT(stateButtonClicked()));

    QObject::connect(ui.button_mode_change, SIGNAL(clicked()), this, SLOT(stateButtonClicked()));


    QObject::connect(ui.button_scan,SIGNAL(clicked()),this,SLOT(on_button_scan_clicked()));

    // -- Joint Control Set
    for (int i = 0; i < 32; i++)
    {
        button_joint_ctrl[i][0] = new QPushButton("-1");
        button_joint_ctrl[i][1] = new QPushButton("+1");
        button_joint_ctrl[i][2] = new QPushButton("SET");
        doubleSpin_joint_ctrl[i] = new QDoubleSpinBox;

        QString text = tr("-- [%1] --").arg(i+1);
        label_joint_ctrl[i] = new QLabel(text);
        //labels_joint_ctrl_ids[i].setText(text);

        button_joint_ctrl[i][0]->setMaximumWidth(40);
        button_joint_ctrl[i][1]->setMaximumWidth(40);
        button_joint_ctrl[i][2]->setMaximumWidth(60);

        button_joint_ctrl[i][0]->setObjectName(tr("%1").arg(i+1));
        button_joint_ctrl[i][1]->setObjectName(tr("%1").arg(i+1));
        button_joint_ctrl[i][2]->setObjectName(tr("%1").arg(i+1));

        doubleSpin_joint_ctrl[i]->setMaximumWidth(80);
        doubleSpin_joint_ctrl[i]->setValue(0.0);
        doubleSpin_joint_ctrl[i]->setMinimum(-20.0);
        doubleSpin_joint_ctrl[i]->setMaximum(20.0);

        label_joint_ctrl[i]->setMaximumWidth(60);
        label_joint_ctrl[i]->setAlignment(Qt::AlignCenter);


        ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][0], i, 0, Qt::AlignTop);
        ui.gridLayout_joint_ctrl->addWidget(label_joint_ctrl[i]    , i, 1, Qt::AlignTop);
        ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][1], i, 2, Qt::AlignTop);
        ui.gridLayout_joint_ctrl->addWidget(doubleSpin_joint_ctrl[i] , i, 3, Qt::AlignTop);
        ui.gridLayout_joint_ctrl->addWidget(button_joint_ctrl[i][2], i, 4, Qt::AlignTop);

        QObject::connect(button_joint_ctrl[i][0], SIGNAL(clicked()), this, SLOT(jointCtrlMinusClicked()));
        QObject::connect(button_joint_ctrl[i][1], SIGNAL(clicked()), this, SLOT(jointCtrlPlusClicked()));
        QObject::connect(button_joint_ctrl[i][2], SIGNAL(clicked()), this, SLOT(jointCtrlSetClicked()));
    }
    // -- Task Control Set
    for (int i = 0; i < 12; i++)
    {
        button_task_ctrl[i][0] = new QPushButton("<");
        button_task_ctrl[i][1] = new QPushButton(">");

        doubleSpin_task_ctrl[i] = new QDoubleSpinBox;

        QString text = "";
        switch(i)
        {
            case 0:
                text = "left arm x";
                break;
            case 1:
                text = "left arm y";
                break;
            case 2:
                text = "left arm z";
                break;
            case 3:
                text = "left arm roll";
                break;
            case 4:
                text = "left arm pitch";
            break;
            case 5:
                text = "left arm yaw";
            break;
            case 6:
                text = "right arm x";
            break;
            case 7:
                text = "right arm y";
            break;
            case 8:
                text = "right arm z";
            break;
            case 9:
                text = "right arm roll";
            break;
            case 10:
                text = "right arm pitch";
            break;
            case 11:
                text = "right arm yaw";
            break;
        }

        label_task_ctrl[i] = new QLabel(text);
        //labels_joint_ctrl_ids[i].setText(text);

        button_task_ctrl[i][0]->setMaximumWidth(40);
        button_task_ctrl[i][1]->setMaximumWidth(40);

        button_task_ctrl[i][0]->setObjectName(tr("%1").arg(i+1));
        button_task_ctrl[i][1]->setObjectName(tr("%1").arg(i+1));

        doubleSpin_task_ctrl[i]->setMinimumWidth(80);
        doubleSpin_task_ctrl[i]->setValue(0.0);
        doubleSpin_task_ctrl[i]->setMinimum(0.0);
        doubleSpin_task_ctrl[i]->setMaximum(30.0);

        label_task_ctrl[i]->setMaximumWidth(100);
        //label_task_ctrl[i]->setAlignment(Qt::AlignCenter);

        if(i<6)
        {
            ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[i]     , i, 0, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(doubleSpin_task_ctrl[i], i, 1, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][0] , i, 2, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][1] , i, 3, Qt::AlignTop);
        }
        else
        {
            ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[i]     , i+1, 0, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(doubleSpin_task_ctrl[i], i+1, 1, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][0] , i+1, 2, Qt::AlignTop);
            ui.gridLayout_task_ctrl->addWidget(button_task_ctrl[i][1] , i+1, 3, Qt::AlignTop);
        }


        QObject::connect(button_task_ctrl[i][0], SIGNAL(clicked()), this, SLOT(taskCtrlMinusClicked()));
        QObject::connect(button_task_ctrl[i][1], SIGNAL(clicked()), this, SLOT(taskCtrlPlusClicked()));
    }
    label_task_ctrl[12] = new QLabel("");
    ui.gridLayout_task_ctrl->addWidget(label_task_ctrl[12] , 6, 0, Qt::AlignTop);

    ui.gridLayout_task_ctrl->setColumnStretch(1,1);
    updateUI();

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

void MainWindow::autoMissionSelectVisible(int mission)
{
    ui.button_auto_valve_approach->setVisible(mission==1);
    ui.button_auto_valve_close->setVisible(mission==1);
    ui.button_auto_valve_init->setVisible(mission==1);
    ui.button_auto_valve_reach->setVisible(mission==1);
    ui.button_auto_valve_ready->setVisible(mission==1);
    ui.button_auto_door_init->setVisible(mission==2);
    ui.button_auto_door_open->setVisible(mission==2);
    ui.button_auto_door_push->setVisible(mission==2);
    ui.button_auto_door_reach->setVisible(mission==2);
    ui.button_auto_door_ready->setVisible(mission==2);
}
void MainWindow::updateUI()
{
    if(isConnected == true)
    {
        ui.groupBox_state->setEnabled(true);
        ui.groupBox_joint_ctrl->setEnabled(true);
        ui.groupBox_task_ctrl->setEnabled(true);
        ui.groupBox_recog_ctrl->setEnabled(true);
        ui.groupBox_walk_ctrl->setEnabled(true);
    }
    else
    {
        ui.groupBox_state->setEnabled(false);
        ui.groupBox_joint_ctrl->setEnabled(false);
        ui.groupBox_task_ctrl->setEnabled(false);
        ui.groupBox_recog_ctrl->setEnabled(false);
        ui.groupBox_walk_ctrl->setEnabled(false);
    }
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
            isConnected = true;
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
            isConnected = true;
		}
	}
    updateUI();
}

/*

void MainWindow::on_manuButton_clicked(bool check )
{
    qnode.send_transition("manu_on");
}
void MainWindow::on_autoButton_clicked(bool check)
{
    qnode.send_transition("auto_on");
}

void MainWindow::on_button_joint_control_clicked(bool check)
{
    qnode.send_transition("activate_jctrl");
}

void MainWindow::on_button_power_on_clicked(bool check)
{
    qnode.send_transition("power_on");
}
*/
/*
void MainWindow::on_button_plus_clicked(bool check)
{
    qnode.send_joint_ctrl(1,1.0); // degree
}

void MainWindow::on_button_minus_clicked(bool check)
{
    qnode.send_joint_ctrl(1,-1.0);
}
*/
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


void MainWindow::on_button_walk_start_clicked()
{
    thormang_ctrl_msgs::WalkingCmd msg;
    msg.command = "start";
    msg.planner = ui.comboBox_walk_planner->currentText().toStdString();
    msg.walking_alg = ui.comboBox_walk_algorithm->currentText().toStdString();
    msg.x = ui.doubleSpinBox_walk_x->value();
    msg.y = ui.doubleSpinBox_walk_y->value();
    msg.height = ui.doubleSpinBox_walk_height->value();
    msg.theta = ui.doubleSpinBox_walk_theta->value();
    msg.imp_time = ui.doubleSpinBox_walk_imp_time->value();
    msg.recov_time= ui.doubleSpinBox_walk_recov_time->value();

    qnode.send_walking_cmd(msg);
}

void MainWindow::on_button_walk_init_clicked()
{
    thormang_ctrl_msgs::WalkingCmd msg;
    msg.command = "init";
    msg.planner = ui.comboBox_walk_planner->currentText().toStdString();
    msg.walking_alg = ui.comboBox_walk_algorithm->currentText().toStdString();
    msg.x = ui.doubleSpinBox_walk_x->value();
    msg.y = ui.doubleSpinBox_walk_y->value();
    msg.height = ui.doubleSpinBox_walk_height->value();
    msg.theta = ui.doubleSpinBox_walk_theta->value();
    msg.imp_time = ui.doubleSpinBox_walk_imp_time->value();
    msg.recov_time= ui.doubleSpinBox_walk_recov_time->value();

    qnode.send_walking_cmd(msg);

}

void MainWindow::on_button_walk_stop_clicked()
{
    thormang_ctrl_msgs::WalkingCmd msg;
    msg.command = "stop";
    msg.planner = ui.comboBox_walk_planner->currentText().toStdString();
    msg.walking_alg = ui.comboBox_walk_algorithm->currentText().toStdString();
    msg.x = ui.doubleSpinBox_walk_x->value();
    msg.y = ui.doubleSpinBox_walk_y->value();
    msg.height = ui.doubleSpinBox_walk_height->value();
    msg.theta = ui.doubleSpinBox_walk_theta->value();
    msg.imp_time = ui.doubleSpinBox_walk_imp_time->value();
    msg.recov_time= ui.doubleSpinBox_walk_recov_time->value();

    qnode.send_walking_cmd(msg);

}
void MainWindow::on_button_scan_clicked()
{
   thormang_ctrl_msgs::RecogCmd msg;
   msg.id = 2; // id 2 means rotate lidar
   msg.x = 0;
   msg.y = 0;
   msg.z = 0;
   msg.roll = 0;
   msg.pitch = 0;
   msg.yaw = 0;
   qnode.send_recog_cmd(msg);
}
void MainWindow::stateButtonClicked()
{
    QString objName = sender()->objectName();
    std::string state;
    if(objName.compare("button_power_on") == 0)  {
        state = "power_on";
    } else if (objName.compare("button_auto") == 0) {
        state = "auto_on";
    } else if (objName.compare("button_auto_valve_start") == 0) {
        state = "mission1";
        autoMissionSelectVisible(1);
    } else if (objName.compare("button_auto_valve_approach") == 0) {
        state = "v_approach";
    } else if (objName.compare("button_auto_valve_close") == 0) {
        state = "v_close";
    } else if (objName.compare("button_auto_valve_init") == 0) {
        state = "v_init";
    } else if (objName.compare("button_auto_valve_reach") == 0) {
        state = "v_reach";
    } else if (objName.compare("button_auto_valve_ready") == 0) {
        state = "v_ready";
    } else if (objName.compare("button_auto_door_start") == 0) {
        state = "mission2";
        autoMissionSelectVisible(2);
    } else if (objName.compare("button_auto_door_init") == 0) {
        state = "d_init";
    } else if (objName.compare("button_auto_door_open") == 0) {
        state = "d_open";
    } else if (objName.compare("button_auto_door_push") == 0) {
        state = "d_push";
    } else if (objName.compare("button_auto_door_reach") == 0) {
        state = "d_reach";
    } else if (objName.compare("button_auto_door_ready") == 0) {
        state = "d_ready";
    } else if (objName.compare("button_manual") == 0) {
        state = "manu_on";
    } else if (objName.compare("button_manual_joint_ctrl") == 0) {
        state = "activate_jctrl";
    } else if (objName.compare("button_manual_task_ctrl") == 0) {
        state = "activate_tctrl";
    } else if (objName.compare("button_mode_change") == 0) {
        state = "cmd_modechg";
    } else if (objName.compare("button_manual_recog_ctrl") == 0) {
        state = "activate_recog";
    }
    qnode.send_transition(state);
}

void MainWindow::jointCtrlMinusClicked()
{
    int id = sender()->objectName().toInt();

    qnode.send_joint_ctrl(id,-1.0); // degree
}

void MainWindow::jointCtrlPlusClicked()
{
    int id = sender()->objectName().toInt();

    qnode.send_joint_ctrl(id,1.0); // degree
}

void MainWindow::jointCtrlSetClicked()
{
    int id = sender()->objectName().toInt();
    double deg = doubleSpin_joint_ctrl[id-1]->value();

    qnode.send_joint_ctrl(id, deg); // degree
}

void MainWindow::taskCtrlMinusClicked()
{

}
void MainWindow::taskCtrlPlusClicked()
{

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

void MainWindow::updateJointView() {
    for(int i=0;i<qnode.joint_msg.id.size(); i++)
    {
        QTableWidgetItem *newItem = new QTableWidgetItem(tr("%1").arg(
                                                            qnode.joint_msg.angle[i]));

        ui.motor_table->setItem(qnode.joint_msg.id[i]-1, 1, newItem);

    }
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "dyros_black_ui");
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
    QSettings settings("Qt-Ros Package", "dyros_black_ui");
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

}  // namespace dyros_black_ui

