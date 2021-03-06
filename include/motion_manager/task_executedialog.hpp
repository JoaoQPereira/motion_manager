#ifndef TASK_EXECUTEDIALOG_H
#define TASK_EXECUTEDIALOG_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QDialog>
#include <ui_task_executedialog.h>
#include "qnode.hpp"


namespace motion_manager
{

class Task_ExecuteDialog : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief Task_ExecuteDialog, a constructor
     * @param q
     * @param parent
     */
    explicit Task_ExecuteDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~Task_ExecuteDialog, a destructor
     */
    ~Task_ExecuteDialog();

    /**
    * @brief getTimeMappingParams
    *
    * @param params ()
    */
    void getTimeMappingParams(vector<vector<double>> &params);


public Q_SLOTS:
    /**
     * @brief on_pushButtonOK_clicked
     * This method checks which platform to use to execute the task
     */
    void on_pushButtonOK_clicked();

    /**
     * @brief on_pushButtonOK_pressed
     * This method checks if the task is being executed
     */
    void on_pushButtonOK_pressed();

    /**
     * @brief on_radioButton_execTask_Robot_clicked
     */
    void on_radioButton_execTask_Robot_clicked();

    /**
     * @brief on_radioButton_execTask_VRep_clicked
     */
    void on_radioButton_execTask_VRep_clicked();

    /**
     * @brief on_pushButtonLoad_clicked
     */
    void on_pushButtonLoad_clicked();

    /**
     * @brief on_pushButtonSave_clicked
     */
    void on_pushButtonSave_clicked();


private:
    Ui::Task_ExecuteDialogDesign *ui; /**< handle of the user interface */
    QNode *qnode; /**< pointer of the ROS node */


Q_SIGNALS:
    /**
     * @brief addPlat_execTask
     * This method signals which platform is used to execute the task and if this dialog will be displayed again
     * @param c (c = 0 => "V-rep simulator", c = 1 => "Robot", c = 2 => "RViz MoveIt")
     * @param a (a = true => "Don't ask again", a = false => "ask again")
     */
    void addPlat_execTask(int c, bool a);    
};

}// namespace motion_manager

#endif // TASK_EXECUTEDIALOG_H
