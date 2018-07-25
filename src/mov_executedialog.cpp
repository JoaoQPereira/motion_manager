#include "../include/motion_manager/mov_executedialog.hpp"


namespace motion_manager {

using namespace Qt;

Mov_ExecuteDialog::Mov_ExecuteDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Mov_ExecuteDialogDesign)
{
    ui->setupUi(this);
    qnode = q;

#if ROBOT==0
    ui->radioButton_execMov_Robot->setEnabled(false);
#endif
#if MOVEIT==0
    ui->radioButton_execMov_MoveIt->setEnabled(false);
#endif
}


Mov_ExecuteDialog::~Mov_ExecuteDialog()
{
    delete ui;
}


void Mov_ExecuteDialog::on_pushButtonOK_clicked()
{
    int platform;
    bool check = false;

    if(ui->radioButton_execMov_VRep->isChecked())
        platform = 0;
    else if(ui->radioButton_execMov_Robot->isChecked())
        platform = 1;
    else if(ui->radioButton_execMov_MoveIt->isChecked())
        platform = 2;

    if(ui->checkBox_rememberChoice->isChecked())
        check = true;

    Q_EMIT addPlat_execMove(platform, check);
}


void Mov_ExecuteDialog::on_pushButtonOK_pressed()
{
    if(ui->radioButton_execMov_VRep->isChecked())
        qnode->log(QNode::Info,std::string("Executing the movement in V-REP . . ."));
    else if(ui->radioButton_execMov_Robot->isChecked())
        qnode->log(QNode::Info,std::string("Executing the movement in Robot . . ."));
    else if(ui->radioButton_execMov_MoveIt->isChecked())
        qnode->log(QNode::Info,std::string("Executing the movement in RViz. . ."));
}

} // namespace motion_manager



