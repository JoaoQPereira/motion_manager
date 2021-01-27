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


void set_waypointsdialog::on_radioButton_set_Polyscope_clicked()
{

}


}//namespace motion_manager
