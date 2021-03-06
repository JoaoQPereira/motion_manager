#include "../include/motion_manager/set_waypointsdialog.hpp"
//#include "ui_set_waypointsdialog.h"



namespace motion_manager {

using namespace Qt;



set_waypointsdialog::set_waypointsdialog(QNode *q,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::set_waypointsdialog)
{
    ui->setupUi(this);
    qnode = q;
}

set_waypointsdialog::~set_waypointsdialog()
{
    delete ui;
}



void set_waypointsdialog::on_pushButtonOK_clicked()
{
    int iPlatform;
    //bool bCheck = false;

    if(ui->radioButton_set_Polyscope->isChecked())
        iPlatform = 1;
    else if(ui->radioButton_set_Coppelia->isChecked())
        iPlatform = 0;

    //if(ui->checkBox_rememberChoice->isChecked())
    //    bCheck = true;

    Q_EMIT addPlat_setWps(iPlatform);
}


void set_waypointsdialog::on_pushButtonOK_pressed()
{
    if(ui->radioButton_set_Polyscope->isChecked())
        qnode->log(QNode::Info, std::string("defining waypoints in Polyscope . . ."));
    else if(ui->radioButton_set_Coppelia->isChecked())
        qnode->log(QNode::Info, std::string("defining waypoints in CoppeliaSim . . ."));
}


void set_waypointsdialog::on_pushButton_Load_clicked()
{

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load waypoints"),
                                                    "/home/joao/ros_ws/devel/lib/motion_manager/Models",
                                                    "All Files (*.*);; Tol Files (*.txt)");
    wp->LoadWaypointsFile(filename,this->mov_wps, this->mov_name,this->mov_gripper_vacuum);
    this->wps_mode=1; // load file
    Q_EMIT SaveLoadwps(this->mov_wps, this->mov_name,this->mov_gripper_vacuum);

    this->close();// close window


}

void set_waypointsdialog::get_waypoints(vector<vector<vector<double>>> &mov_wps , vector <QString> &mov_name,  vector <int> &mov_gripper_vacuum)
{
    mov_wps = this->mov_wps;
    mov_name = this->mov_name;
    mov_gripper_vacuum = this->mov_gripper_vacuum;
}

void set_waypointsdialog::get_waypoints_mode(bool &load_mode)
{
    load_mode=this->wps_mode;
}


}//namespace motion_manager
