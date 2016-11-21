#ifndef RRTCONNECTDIALOG_HPP
#define RRTCONNECTDIALOG_HPP

#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <cstring>
#include <ui_rrtconnectdialog.h>
#include "config.hpp"

namespace motion_manager {

class RRTConnectDialog : public QDialog
{
    Q_OBJECT

public Q_SLOTS:

    /**
     * @brief This method saves the tuning parameters to a file
     */
    void on_pushButton_save_clicked();

    /**
     * @brief This method loads the tuning parameters from a file
     */
    void on_pushButton_load_clicked();

public:
    /**
     * @brief RRTConnectDialog, a constructor
     * @param parent
     */
    explicit RRTConnectDialog(QWidget *parent = 0);

    /**
     * @brief ~RRTConnectDialog, a destructor
     */
    ~RRTConnectDialog();

    /**
     * @brief getPreGraspApproach
     * @param pre_grasp
     */
    void getPreGraspApproach(std::vector<double>& pre_grasp);

    /**
     * @brief getPostGraspRetreat
     * @param post_grasp
     */
    void getPostGraspRetreat(std::vector<double>& post_grasp);

    /**
     * @brief getPrePlaceApproach
     * @param pre_place
     */
    void getPrePlaceApproach(std::vector<double>& pre_place);

    /**
     * @brief getPostPlaceRetreat
     * @param post_place
     */
    void getPostPlaceRetreat(std::vector<double>& post_place);

    /**
     * @brief getConfig
     * @return
     */
    std::string getConfig();

    /**
     * @brief This method sets the information about the tuning of the planner
     * @param info
     */
    void setInfo(std::string info);



private:
    Ui::RRTConnectDialog *ui; /**< handle of the user interface */
    std::string config; /**< current configuration */
    std::string infoLine; /**< information about the tuning of the planner */
};

} // namespace motion_manager
#endif // RRTCONNECTDIALOG_HPP
