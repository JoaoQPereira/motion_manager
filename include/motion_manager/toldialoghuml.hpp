#ifndef TOLDIALOGHUML_H
#define TOLDIALOGHUML_H

#include<QFileDialog>
#include <QFile>
#include <QTextStream>
#include<QMessageBox>
#include <ui_toldialoghuml.h>
#include <eigen3/Eigen/Dense>
#include "config.hpp"

namespace motion_manager{

using namespace std;
using namespace Eigen;

//! The TolDialogHUML class
/**
 * @brief This class defines the tuning process of the human-like motion planner
 */
class TolDialogHUML : public QDialog
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

    /**
     * @brief checkApproach
     * @param state
     */
    void checkApproach(int state);

    /**
     * @brief checkRetreat
     * @param state
     */
    void checkRetreat(int state);


public:

    /**
     * @brief TolDialogHUML, a constructor
     * @param parent
     */
    explicit TolDialogHUML(QWidget *parent = 0);

    /**
     * @brief TolDialogHUML, a destructor
     */
    ~TolDialogHUML();

    /**
     * @brief This method gets the tolerances of the arm
     * @param tols
     */
    void getTolsArm(vector<double>& tols);

    /**
     * @brief This method gets the tolerances of the hand
     * @param tols
     */
    void getTolsHand(MatrixXd& tols);


    /**
     * @brief This method gets the weights of the objective function
     * @param lambda
     */
    void getLambda(std::vector<double>& lambda);

    /**
     * @brief This method gets the tolerances of the obstacles
     * @param tols
     */
    void getTolsObstacles(MatrixXd& tols);

    /**
     * @brief This method gets the tolerances of the target
     * @param tols
     */
    void getTolsTarget(MatrixXd& tols);

    /**
     * @brief This method gets the number of steps in a movement
     * @return
     */
    int getSteps();

    /**
     * @brief This method gets the maximum angula velocity allowed for each joint
     * @return
     */
    double getWMax();


    /**
     * @brief This method gets the tolerances in positioning the end-effector
     * @return
     */
    double getTolTarPos();

    /**
     * @brief This method gets the tolerances in orienting the end-effector
     * @return
     */
    double getTolTarOr();

    /**
     * @brief This method gets the target avoidance flag
     * @return
     */
    bool getTargetAvoidance();

    /**
     * @brief This method gets the obstacle avoidance flag
     * @return
     */
    bool getObstacleAvoidance();

    /**
     * @brief getApproach
     * @return
     */
    bool getApproach();

    /**
     * @brief getRetreat
     * @return
     */
    bool getRetreat();

    /**
     * @brief getInitVel
     * @param init_vel
     */
    void getInitVel(std::vector<double>& init_vel);

    /**
     * @brief getFinalVel
     * @param final_vel
     */
    void getFinalVel(std::vector<double>& final_vel);

    /**
     * @brief getInitAcc
     * @param init_acc
     */
    void getInitAcc(std::vector<double>& init_acc);

    /**
     * @brief getFinalAcc
     * @param final_acc
     */
    void getFinalAcc(std::vector<double>& final_acc);

    /**
     * @brief getVelApproach
     * @param vel_approach
     */
    void getVelApproach(std::vector<double>& vel_approach);

    /**
     * @brief getAccApproach
     * @param acc_approach
     */
    void getAccApproach(std::vector<double>& acc_approach);

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
     * @brief This method sets the information about the tuning of the planner
     * @param info
     */
    void setInfo(string info);

    /**
     * @brief getRandInit
     * @return
     */
    bool getRandInit();





private:
    Ui::TolDialogHUML *ui; /**< handle of the user interface */
    string infoLine; /**< information about the tuning of the planner */
    bool rand_init;
};

} // namespace motion_manager

#endif // TOLDIALOGHUML_H
