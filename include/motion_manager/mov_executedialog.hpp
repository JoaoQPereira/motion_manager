#ifndef MOV_EXECUTEDIALOG_H
#define MOV_EXECUTEDIALOG_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QDialog>
#include <ui_mov_executedialog.h>
#include "qnode.hpp"


namespace motion_manager
{

class Mov_ExecuteDialog : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief Mov_ExecuteDialog, a contructor
     * @param q
     * @param parent
     */
    explicit Mov_ExecuteDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~Mov_ExecuteDialog, a destructor
     */
    ~Mov_ExecuteDialog();

    /**
    * @brief getTimeMappingParams
    *
    * @param params ()
    */
    void getTimeMappingParams(vector<double> &params);


public Q_SLOTS:
    /**
     * @brief on_pushButtonOK_clicked
     * This method checks which platform to use to execute the planned movement
     */
    void on_pushButtonOK_clicked();

    /**
     * @brief on_pushButtonOK_pressed
     * This method checks if the movement is being executed
     */
    void on_pushButtonOK_pressed();

    /**
     * @brief on_radioButton_execMov_Robot_clicked
     *
     */
    void on_radioButton_execMov_Robot_clicked();

    /**
     * @brief on_radioButton_execMov_VRep_clicked
     *
     */
    void on_radioButton_execMov_VRep_clicked();

    /**
     * @brief on_pushButtonLoad_clicked
     *
     */
    void on_pushButtonLoad_clicked();

    /**
     * @brief on_pushButtonSave_clicked
     *
     */
    void on_pushButtonSave_clicked();


private:
    Ui::Mov_ExecuteDialogDesign *ui; /**< handle of the user interface */
    QNode *qnode; /**< pointer of the ROS node */


Q_SIGNALS:
    /**
     * @brief addPlat_execMove
     * This method signals which platform is used to execute the planned movement and if this dialog will be displayed again
     * @param c (c = 0 => "V-rep simulator", c = 1 => "Robot", c = 2 => "RViz MoveIt")
     * @param a (a = true => "Don't ask again", a = false => "ask again")
     */
    void addPlat_execMove(int c, bool a);
};

}// namespace motion_manager

#endif // MOV_EXECUTEDIALOG_H
