#ifndef VREPCOMMDIALOG_HPP
#define VREPCOMMDIALOG_HPP

#include <QDialog>
#include <ui_vrepcommdialog.h>
#include "qnode.hpp"


namespace motion_manager
{

class VrepCommDialog : public QDialog
{
    Q_OBJECT

public:
    /**
     * @brief VrepCommDialog, a constructor
     * @param q
     * @param parent
     */
    VrepCommDialog(QNode* q, QWidget *parent = 0);

    /**
     * @brief ~VrepCommDialog, a destructor
     */
    ~VrepCommDialog();

public Q_SLOTS:
    /**
     * @brief This method checks if V-REP has started
     * @param check
     */
    void on_button_check_clicked(bool check);

private:
    Ui::VrepCommDialogDesign *ui;/**< handle of the user interface */
    QNode *qnode; /**< pointer of the ROS node */

Q_SIGNALS:
    /**
     * @brief This method signals if V-REP is on-line (c=true)
     * @param c
     */
    void vrepConnected(bool c);
};

} // namespace motion_manager

#endif // VREPCOMMDIALOG_HPP
