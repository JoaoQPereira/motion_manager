#include "../include/motion_manager/task_executedialog.hpp"


namespace motion_manager {

using namespace Qt;

Task_ExecuteDialog::Task_ExecuteDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Task_ExecuteDialogDesign)
{
    ui->setupUi(this);
    qnode = q;

#if ROBOT == 0
    ui->radioButton_execTask_Robot->setEnabled(false);
#endif
    // By default, the V-Rep simulator option is checked
    ui->groupBox_gripperControl_plan->setEnabled(false);
    ui->groupBox_gripperControl_app->setEnabled(false);
    ui->groupBox_gripperControl_ret->setEnabled(false);
}


Task_ExecuteDialog::~Task_ExecuteDialog()
{
    delete ui;
}


void Task_ExecuteDialog::on_pushButtonOK_clicked()
{
    int iPlatform;
    bool bCheck = false;

    if(ui->radioButton_execTask_VRep->isChecked())
        iPlatform = 0;
    else if(ui->radioButton_execTask_Robot->isChecked())
        iPlatform = 1;

    if(ui->checkBox_rememberChoice_task->isChecked())
        bCheck = true;

    Q_EMIT addPlat_execTask(iPlatform, bCheck);
}


void Task_ExecuteDialog::on_pushButtonOK_pressed()
{
    if(ui->radioButton_execTask_VRep->isChecked())
        qnode->log(QNode::Info, std::string("executing the task in V-REP . . ."));
    else if(ui->radioButton_execTask_Robot->isChecked())
        qnode->log(QNode::Info, std::string("executing the task in Robot . . ."));
}


void Task_ExecuteDialog::on_radioButton_execTask_Robot_clicked()
{
    ui->groupBox_gripperControl_plan->setEnabled(true);
    ui->groupBox_gripperControl_app->setEnabled(true);
    ui->groupBox_gripperControl_ret->setEnabled(true);
}


void Task_ExecuteDialog::on_radioButton_execTask_VRep_clicked()
{
    ui->groupBox_gripperControl_plan->setEnabled(false);
    ui->groupBox_gripperControl_app->setEnabled(false);
    ui->groupBox_gripperControl_ret->setEnabled(false);
}


void Task_ExecuteDialog::on_pushButtonLoad_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Exec/TimeMapping/Sawyer",
                                                    "All Files (*.*);; tmap Files (*.tmap)");
    QFile f(filename);

    if(f.open(QIODevice::ReadOnly))
    {
        QTextStream stream(&f);
        QString line;

        while(!stream.atEnd())
        {
            line = f.readLine();

            if(line.at(0) != QChar('#'))
            {
                QStringList fields = line.split("=");

                if(QString::compare(fields.at(0), QString("diff_weight_tau_plan"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_tau_plan->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("dec_rate_a_plan"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_a_plan->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("time_const_w_plan"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_w_plan->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("diff_weight_tau_app"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_tau_app->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("dec_rate_a_app"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_a_app->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("time_const_w_app"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_w_app->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("diff_weight_tau_ret"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_tau_ret->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("dec_rate_a_ret"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_a_ret->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("time_const_w_ret"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_w_ret->setText(fields.at(1));
            }
        }
        f.close();
    }
}


void Task_ExecuteDialog::on_pushButtonSave_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Exec",
                                                    "All Files (*.*);; tmap Files (*.tmap)");

    QFile f(filename);
    if(f.open(QIODevice::WriteOnly))
    {
        QTextStream stream(&f);
        stream << "### Parameters of the Time mapping ###" << endl;
        stream << "### Plan Stage ###" << endl;
        stream << "diff_weight_tau_plan="<< ui->lineEdit_tau_plan->text().toStdString().c_str() << endl;
        stream << "dec_rate_a_plan="<< ui->lineEdit_a_plan->text().toStdString().c_str() << endl;
        stream << "time_const_w_plan="<< ui->lineEdit_w_plan->text().toStdString().c_str() << endl;
        stream << "### Approach Stage ###" << endl;
        stream << "diff_weight_tau_app="<< ui->lineEdit_tau_app->text().toStdString().c_str() << endl;
        stream << "dec_rate_a_app="<< ui->lineEdit_a_app->text().toStdString().c_str() << endl;
        stream << "time_const_w_app="<< ui->lineEdit_w_app->text().toStdString().c_str() << endl;
        stream << "### Retreat Stage ###" << endl;
        stream << "diff_weight_tau_ret="<< ui->lineEdit_tau_ret->text().toStdString().c_str() << endl;
        stream << "dec_rate_a_ret="<< ui->lineEdit_a_ret->text().toStdString().c_str() << endl;
        stream << "time_const_w_ret="<< ui->lineEdit_w_ret->text().toStdString().c_str() << endl;
    }

    f.close();
}


void Task_ExecuteDialog::getTimeMappingParams(vector<vector<double>> &params)
{
    std::vector<double> pplan;
    pplan.push_back(ui->lineEdit_a_plan->text().toDouble());
    pplan.push_back(ui->lineEdit_tau_plan->text().toDouble());
    pplan.push_back(ui->lineEdit_w_plan->text().toDouble());

    std::vector<double> papp;
    papp.push_back(ui->lineEdit_a_app->text().toDouble());
    papp.push_back(ui->lineEdit_tau_app->text().toDouble());
    papp.push_back(ui->lineEdit_w_app->text().toDouble());

    std::vector<double> pret;
    pret.push_back(ui->lineEdit_a_ret->text().toDouble());
    pret.push_back(ui->lineEdit_tau_ret->text().toDouble());
    pret.push_back(ui->lineEdit_w_ret->text().toDouble());

    params.clear();
    params.push_back(pplan);
    params.push_back(papp);
    params.push_back(pret);
}
} // namespace motion_manager
