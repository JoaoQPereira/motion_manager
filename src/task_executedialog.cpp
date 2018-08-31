#include "../include/motion_manager/task_executedialog.hpp"


namespace motion_manager {

using namespace Qt;

Task_ExecuteDialog::Task_ExecuteDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Task_ExecuteDialogDesign)
{
    ui->setupUi(this);
    qnode = q;

#if ROBOT==0
    ui->radioButton_execTask_Robot->setEnabled(false);
#endif
}


Task_ExecuteDialog::~Task_ExecuteDialog()
{
    delete ui;
}


void Task_ExecuteDialog::on_pushButtonOK_clicked()
{
    int platform;
    bool check = false;

    if(ui->radioButton_execTask_VRep->isChecked())
        platform = 0;
    else if(ui->radioButton_execTask_Robot->isChecked())
        platform = 1;

    if(ui->checkBox_rememberChoice->isChecked())
        check = true;

    Q_EMIT addPlat_execTask(platform, check);
}


void Task_ExecuteDialog::on_pushButtonOK_pressed()
{
    if(ui->radioButton_execTask_VRep->isChecked())
        qnode->log(QNode::Info,std::string("executing the task in V-REP . . ."));
    else if(ui->radioButton_execTask_Robot->isChecked())
        qnode->log(QNode::Info,std::string("executing the task in Robot . . ."));
}

} // namespace motion_manager
