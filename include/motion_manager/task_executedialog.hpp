#ifndef TASK_EXECUTEDIALOG_H
#define TASK_EXECUTEDIALOG_H

#include <QDialog>
#include <ui_task_executedialog.h>
#include "qnode.hpp"


namespace motion_manager{

class Task_ExecuteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Task_ExecuteDialog(QNode* q, QWidget *parent = 0);

    ~Task_ExecuteDialog();

public Q_SLOTS:
    void on_pushButtonOK_clicked();
    void on_pushButtonOK_pressed();

private:
    Ui::Task_ExecuteDialogDesign *ui;
    QNode *qnode; /**< pointer of the ROS node */

Q_SIGNALS:
    void addPlat_execTask(int c, bool a);
};

}// namespace motion_manager

#endif // TASK_EXECUTEDIALOG_H
