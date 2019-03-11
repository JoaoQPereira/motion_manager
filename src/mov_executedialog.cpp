#include "../include/motion_manager/mov_executedialog.hpp"


namespace motion_manager {

using namespace Qt;


Mov_ExecuteDialog::Mov_ExecuteDialog(QNode *q, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Mov_ExecuteDialogDesign)
{
    ui->setupUi(this);
    qnode = q;

#if ROBOT == 0
    ui->radioButton_execMov_Robot->setEnabled(false);
#endif
    // By default, the V-Rep simulator option is checked
    ui->groupBox_gripperControl->setEnabled(false);
}


Mov_ExecuteDialog::~Mov_ExecuteDialog()
{
    delete ui;
}


void Mov_ExecuteDialog::on_pushButtonOK_clicked()
{
    int iPlatform;
    bool bCheck = false;

    if(ui->radioButton_execMov_VRep->isChecked())
        iPlatform = 0;
    else if(ui->radioButton_execMov_Robot->isChecked())
        iPlatform = 1;

    if(ui->checkBox_rememberChoice->isChecked())
        bCheck = true;

    Q_EMIT addPlat_execMove(iPlatform, bCheck);
}


void Mov_ExecuteDialog::on_pushButtonOK_pressed()
{
    if(ui->radioButton_execMov_VRep->isChecked())
        qnode->log(QNode::Info,std::string("executing the movement in V-REP . . ."));
    else if(ui->radioButton_execMov_Robot->isChecked())
        qnode->log(QNode::Info,std::string("executing the movement in Robot . . ."));
}


void Mov_ExecuteDialog::on_radioButton_execMov_Robot_clicked(bool check)
{
    ui->groupBox_gripperControl->setEnabled(true);
}


void Mov_ExecuteDialog::on_radioButton_execMov_VRep_clicked()
{
    ui->groupBox_gripperControl->setEnabled(false);
}


void Mov_ExecuteDialog::on_pushButtonLoad_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Exec",
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

                if(QString::compare(fields.at(0), QString("diff_weight_tau"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_tau->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("dec_rate_a"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_a->setText(fields.at(1));
                else if(QString::compare(fields.at(0), QString("time_const_w"), Qt::CaseInsensitive) == 0)
                    this->ui->lineEdit_w->setText(fields.at(1));
            }
        }
        f.close();
    }
}


void Mov_ExecuteDialog::on_pushButtonSave_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the file of tolerances"),
                                                    QString(MAIN_PATH)+"/Exec",
                                                    "All Files (*.*);; tmap Files (*.tmap)");

    QFile f(filename);
    if(f.open(QIODevice::WriteOnly))
    {
        QTextStream stream(&f);
        stream << "### Parameters of the Time mapping  ###" << endl;
        stream << "diff_weight_tau="<< ui->lineEdit_tau->text().toStdString().c_str() << endl;
        stream << "dec_rate_a="<< ui->lineEdit_a->text().toStdString().c_str() << endl;
        stream << "time_const_w="<< ui->lineEdit_w->text().toStdString().c_str() << endl;
    }
    f.close();
}


void Mov_ExecuteDialog::getTimeMappingParams(vector<double> &params)
{
    params.clear();
    params.push_back(ui->lineEdit_a->text().toDouble());
    params.push_back(ui->lineEdit_tau->text().toDouble());
    params.push_back(ui->lineEdit_w->text().toDouble());
}
} // namespace motion_manager
