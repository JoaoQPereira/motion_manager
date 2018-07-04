#ifndef MOV_EXECUTEDIALOG_H
#define MOV_EXECUTEDIALOG_H

#include <QDialog>
#include <ui_mov_executedialog.h>
#include "config.hpp"
#include "qnode.hpp"


namespace motion_manager{

class Mov_ExecuteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Mov_ExecuteDialog(QNode* q, QWidget *parent = 0);

    ~Mov_ExecuteDialog();

public Q_SLOTS:
    void on_pushButtonOK_clicked();
    void on_pushButtonOK_pressed();

private:
    Ui::Mov_ExecuteDialogDesign *ui;
    QNode *qnode; /**< pointer of the ROS node */

Q_SIGNALS:
    void addPlat_execMove(int c, bool a);
};

}// namespace motion_manager

#endif // MOV_EXECUTEDIALOG_H
