#ifndef SET_WAYPOINTSDIALOG_H
#define SET_WAYPOINTSDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <ui_set_waypointsdialog.h>
#include "qnode.hpp"


namespace motion_manager
{

class set_waypointsdialog : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief set_waypointsdialog
     * @param q
     * @param parent
     */
    explicit set_waypointsdialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~set_waypointsdialog, a destructor
     */
    ~set_waypointsdialog();

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
    void on_radioButton_set_Polyscope_clicked();



private:
    Ui::set_waypointsdialog *ui;
    QNode *qnode; /**< pointer of the ROS node */

Q_SIGNALS:
    /**
     * @brief addPlat_execMove
     * This method signals which platform is used to execute the planned movement and if this dialog will be displayed again
     * @param c (c = 0 => "CoppeliaSim simulator", c = 1 => "Polyscope")
     */
    void addPlat_setWps(int c);
};

}// namespace motion_manager

#endif // SET_WAYPOINTSDIALOG_H
