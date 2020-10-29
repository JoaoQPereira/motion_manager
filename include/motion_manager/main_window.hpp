#ifndef motion_manager_MAIN_WINDOW_H
#define motion_manager_MAIN_WINDOW_H

#include <qcustomplot.h>
#include <qcpdocumentobject.h>
#include <qwt3d_io.h>
#include <qwt3d_io_gl2ps.h>
#include <persistence1d.hpp>
#include <QtGui/QMainWindow>
#include <QtGui/QListWidgetItem>
#include <boost/smart_ptr.hpp>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "roscommdialog.hpp"
#include "vrepcommdialog.hpp"
#include "toldialoghump.hpp"
#include "mov_executedialog.hpp"
#include "task_executedialog.hpp"
#include "config.hpp"
#include "results_plan_joints_dialog.hpp"
#include "comp_velocity_dialog.hpp"
#include "handposplot.hpp"


using namespace std;
using namespace p1d;

namespace motion_manager
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief MainWindow, a constructor
     * @param argc
     * @param argv
     * @param parent
     */
    MainWindow(int argc, char** argv, QWidget *parent = 0);

    /**
     * @brief ~MainWindow, a destructor
     */
    ~MainWindow();

    /**
     * @brief This method loads up qt program settings at startup
     */
    void ReadSettings();

    /**
     * @brief This method saves qt program settings when closing the main_window
     */
    void WriteSettings();

    /**
     * @brief This method menages the close event
     * @param event
     */
    void closeEvent(QCloseEvent *event);

    /**
     * @brief getDerivative
     * @param function
     * @param step_values
     * @param derFunction
     */
    void getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction);

    /**
     * @brief getNumberMovementUnits
     * @param function
     * @param time
     * @return
     */
    int getNumberMovementUnits(vector<double> &function, QVector<double> &time);

    /**
     * @brief getMedian
     * @param v
     * @return
     */
    double getMedian(vector<double> v);

    /**
     * @brief getFirstQuartile
     * @param v
     * @return
     */
    double getFirstQuartile(vector<double> v);

    /**
     * @brief getThirdQuartile
     * @param v
     * @return
     */
    double getThirdQuartile(vector<double> v);

    /**
     * @brief getMedian
     * @param v
     * @return
     */
    double getMedian(vector<int> v);

    /**
     * @brief getFirstQuartile
     * @param v
     * @return
     */
    double getFirstQuartile(vector<int> v);

    /**
     * @brief getThirdQuartile
     * @param v
     * @return
     */
    double getThirdQuartile(vector<int> v);

public Q_SLOTS:
    /**
     * @brief This method shows the about dialog
     */
    void on_actionAbout_triggered();

    /**
     * @brief This method shows the ROS communication dialog
     */
    void on_actionRos_Communication_triggered();

    /**
     * @brief This method shows the V-REP communication dialog
     */
    void on_actionVrep_Communication_triggered();

    /**
     * @brief This method loads the selected scenario
     */
    void on_pushButton_loadScenario_clicked();

    /**
     * @brief This method retrievies the information about
     * the elements in the scenario
     */
    void on_pushButton_getElements_clicked();

    /**
     * @brief This method retrievies the information about
     * the elements in the scenario
     */
    void on_pushButton_getElements_pressed();

    /**
     * @brief This method adds a movement to the task
     */
    void on_pushButton_addMov_clicked();

    /**
     * @brief This method selects the type of task
     * @param i
     * index of the task. Single-arm task: i=0. Dual-arm task: i=1;
     */
    void on_comboBox_Task_currentIndexChanged(int i);

    /**
     * @brief This method selects the type of task
     * @param i
     * <table>
     * <caption id="multi_row">Types of the movement</caption>
     * <tr><th>Type      <th>index
     * <tr><td>Reach-to-grasp <td>0
     * <tr><td>Reaching <td>1
     * <tr><td>Transport <td>2
     * <tr><td>Engage <td>3
     * <tr><td>Disengage <td>4
     * <tr><td>Go home <td>5
     * </table>
     */
    void on_comboBox_mov_currentIndexChanged(int i);

    /**
     * @brief This method plans the selected movement
     */
    void on_pushButton_plan_clicked();

    /**
     * @brief on_pushButton_plan_trials_clicked
     */
    void on_pushButton_plan_trials_clicked();

    /**
     * @brief This method plans the selected movement
     */
    void on_pushButton_plan_pressed();

    /**
     * @brief This method executes the selected movement
     */
    void on_pushButton_execMov_clicked();

    /**
     * @brief This method reload the scenario and resets the related variables
     */
    void on_pushButton_scene_reset_clicked();

    /**
     * @brief This method shows the tuning dialog
     */
    void on_pushButton_tuning_clicked();

    /**
     * @brief This method appends the movement to the current task
     */
    void on_pushButton_append_mov_clicked();

    /**
     * @brief This method deletes the current task
     */
    void on_pushButton_clear_task_clicked();

    /**
     * @brief This method saves the current task to file
     */
    void on_pushButton_save_task_clicked();

    /**
     * @brief This method executes the task
     */
    void on_pushButton_execTask_clicked();

    /**
     * @brief This method loads the current task from file
     */
    void on_pushButton_load_task_clicked();

    /**
     * @brief This method stops the execution of the movement
     */
    void on_pushButton_stop_mov_clicked();

    /**
     * @brief This method stops the execution of the task
     */
    void on_pushButton_stop_task_clicked();

    /**
     * @brief This method saves the end posture data
     */
    void on_pushButton_save_end_posture_clicked();

    /**
     * This method is signalled by the underlying model. When the model changes,
     * this will drop the cursor down to the last line in the QListview to ensure
     * the user can always see the latest log message.
     */
    void updateLoggingView();

    /**
     * @brief This method updates the ROS status
     * @param c
     * c=true => "connected", c=false => "disconnected"
     */
    void updateRosStatus(bool c);

    /**
     * @brief This method executes the planned movement in the selected plataform
     * @param c , c = 0 => "V-Rep simulator", c = 1 => "robot"
     * @param a , a = true => "Don't ask again", a = false => "ask again"
     */
    void execMove(int c, bool a);

    /**
     * @brief This method executes the task in the selected plataform
     * @param c , c = 0 => "V-Rep simulator", c = 1 => "robot"
     * @param a , a = true => "Don't ask again", a = false => "ask again"
     */
    void execTask(int c, bool a);

    /**
     * @brief This method updates the V-REP status
     * @param c
     * c=true => "connected", c=false => "disconnected"
     */
    void updateVrepStatus(bool c);

    /**
     * @brief This method updates the RViz status
     * @param c
     * c=true => "launched", c=false => "not launched"
     */
    void updateRVizStatus(bool c);

    /**
     * @brief This method adds a new element to the widget
     * @param value
     */
    void addElement(string value);

    /**
     * @brief This method updates the info of the element with index id
     * @param id
     * @param value
     */
    void updateElement(int id,string value);
    /**
     * @brief This method adds a new waypoint to the widget
     * @param value
     */
    void addWaypoint(string value);

    /**
     * @brief This method add an object to the lists of
     * available objects for manipulation
     * @param value
     */
    void addObject(string value);

    /**
     * @brief addPose
     * @param value
     */
    void addPose(string value);

    /**
     * @brief This methods updates the home posture of the humanoid
     * @param value
     */
    void updateHomePosture(string value);

    /**
     * @brief This method lists the available scenarios
     * @param item
     */
    void onListScenarioItemClicked(QListWidgetItem* item);

    /**
     * @brief on_pushButton_plot_mov_clicked
     */
    void on_pushButton_plot_mov_clicked();

    /**
     * @brief on_pushButton_plot_task_clicked
     */
    void on_pushButton_plot_task_clicked();

    /**
     * @brief on_pushButton_joints_results_mov_clicked
     */
    void on_pushButton_joints_results_mov_clicked();

    /**
     * @brief on_pushButton_joints_results_task_clicked
     */
    void on_pushButton_joints_results_task_clicked();

    /**
     * @brief on_pushButton_comp_vel_mov_clicked
     */
    void on_pushButton_comp_vel_mov_clicked();

    /**
     * @brief on_pushButton_save_res_mov_clicked
     */
    void on_pushButton_save_res_mov_clicked();

    /**
     * @brief on_pushButton_save_res_task_clicked
     */
    void on_pushButton_save_res_task_clicked();

    void on_pushButton_setWaypoint_clicked();

    void on_pushButton_deleteWaypoint_clicked();


private:
    Ui::MainWindowDesign ui; /**< handles of the main user interface */
    QNode qnode; /**< ROS node handle */
    RosCommDialog *mrosCommdlg; /**< handle of the ROS communication dialog */
    VrepCommDialog *mvrepCommdlg; /**< handle of the V-REP communication dialog */
    TolDialogHUMP *mTolHumpdlg; /**< handle of the HUMP tuning dialog */
    Mov_ExecuteDialog *mMovExecutedlg; /**< handle of the Execute Settings dialog in movements*/
    Task_ExecuteDialog *mTaskExecutedlg; /**< handle of the Execute Settings dialog in tasks */
    ResultsJointsDialog *mResultsJointsdlg;/**< handle of the results joints dlg*/
    CompVelocityDialog *mCompVeldlg; /**< handle of the velocity components dlg */
    int scenario_id; /**< id of the current scenario */
    QVector<QString> scenarios;  /**< list of scenarios */
    int usedPlat_task; /**< platform used to execute the task */
    bool execSettings_task = false; /** < ask again you options: V-Rep or robot*/
    int usedPlat_move; /**< platform used to execute the planned movement */
    bool execSettings_move = false; /** < ask again you options: V-Rep or robot*/

    vector<vector<double>> robot_waypoints;
    vector< vector < double > > timesteps_mov; /**< current time steps of the movement */
    QVector<double> qtime_mov; /**< time of the current movement for plotting */
    vector<double> tols_stop_mov; /**< vector of the tolerances to stop each stage in the movement */
    vector< MatrixXd > jointsAcceleration_mov; /**< trajectory of the joint acceleration of the movement */
    vector< MatrixXd > jointsVelocity_mov; /**< trajectory of the joint velocity of the movement */
    vector< MatrixXd > jointsPosition_mov; /**< trajectory of the joint position of the movement */
    vector< MatrixXd > jointsPosition_mov_plan; /**< trajectory of the joint position (without offset) of the movement */
    vector< string > traj_descr_mov; /**< description of the trajectories */
    vector<double> jointsEndPosition_mov; /**< end joint position of the movement */
    vector<double> jointsEndVelocity_mov; /**< end joint velocity of the movement */
    vector<double> jointsEndAcceleration_mov; /**< end joint acceleration of the movement */
    vector< vector< MatrixXd > > jointsAcceleration_task; /**< trajectory of the joint acceleration of the task */
    vector< vector< MatrixXd > > jointsVelocity_task; /**< trajectory of the joint velocity of the task */
    vector< vector< MatrixXd > > jointsPosition_task; /**< trajectory of the joint position of the task */
    vector<vector< string >> traj_descr_task; /**< description of the trajectories of the task*/
    vector< vector< vector < double > > > timesteps_task; /**< vector of time steps of each movement in the task */
    QVector<double> qtime_task;/**< time of the current task for plotting */
    vector<vector<double>> tols_stop_task; /**< vector of the tolerances to stop each movement in the task */
    vector<string> vel_steps; /**< steps of the trajectory for saving/loading file */

    vector<vector<double>> handPosition_mov; /**< hand position during the movement. 0=x,1=y,2=z */
    vector<vector<double>> handOrientation_mov; /**< hand orientation during the movement. */
    vector<vector<double>> handLinearVelocity_mov; /**< hand linear velocity during the movement */
    vector<vector<double>> handAngularVelocity_mov;/**< hand angular velocity during the movement */
    vector<vector<double>> wristLinearVelocity_mov; /**< wrist linear velocity during the movement */
    vector<vector<double>> wristAngularVelocity_mov;/**< wrist angular velocity during the movement */
#if UR == 1
    vector<vector<double>> wrist1LinearVelocity_mov; /**< wrist linear velocity during the movement */
    vector<vector<double>> wrist1AngularVelocity_mov;/**< wrist angular velocity during the movement */
    vector<vector<double>> wrist2LinearVelocity_mov; /**< wrist linear velocity during the movement */
    vector<vector<double>> wrist2AngularVelocity_mov;/**< wrist angular velocity during the movement */
    vector<vector<double>> wrist3LinearVelocity_mov; /**< wrist linear velocity during the movement */
    vector<vector<double>> wrist3AngularVelocity_mov;/**< wrist angular velocity during the movement */
#endif
    vector<vector<double>> elbowLinearVelocity_mov; /**< elbow linear velocity during the movement */
    vector<vector<double>> elbowAngularVelocity_mov;/**< elbow angular velocity during the movement */
    vector<vector<double>> shoulderLinearVelocity_mov; /**< shoulder linear velocity during the movement */
    vector<vector<double>> shoulderAngularVelocity_mov;/**< shoulder angular velocity during the movement */
    vector<double> handVelocityNorm_mov; /**< hand velocity norm during the movement */
    vector<double> wristVelocityNorm_mov; /**< wrist velocity norm during the movement */
#if UR==1
    vector<double> wrist1VelocityNorm_mov; /**< wrist1 velocity norm during the movement */
    vector<double> wrist2VelocityNorm_mov; /**< wrist2 velocity norm during the movement */
    vector<double> wrist3VelocityNorm_mov; /**< wrist3  velocity norm during the movement */

#endif
    vector<double> elbowVelocityNorm_mov; /**< elbow velocity norm during the movement */
    vector<double> shoulderVelocityNorm_mov; /**< shoulder velocity norm during the movement */
    double prob_time_mov;/**< time taken to solve the problem */
    double njs_mov;/**< normalized jerk score of the movement */
    int nmu_mov;/**< number of the movement units */

    vector<vector<double>> handPosition_task; /**< hand position during the task. 0=x,1=y,2=z */
    vector<vector<double>> handOrientation_task; /**< hand orientation during the task. */
    vector<vector<double>> handLinearVelocity_task; /**< hand linear velocity during the task */
    vector<vector<double>> handAngularVelocity_task;/**< hand angular velocity during the task */
    vector<double> handVelocityNorm_task; /**< hand velocity norm during the task */
    vector<double> prob_time_task;/**< time taken to solve the problems in the task */
    vector<double> njs_task;/**< normalized jerk scores of the movements in the task */
    vector<int> nmu_task;/**< number of the movement units in the task */

    HUMotion::planning_result_ptr h_results; /**< results of the HUMP planner */
    movementPtr curr_mov; /**< current movement */
    taskPtr curr_task;/**< current task */
    scenarioPtr init_scene; /**< initial scenario */
    scenarioPtr curr_scene; /**< current scenario */

    //boost::shared_ptr<HandPosPlot> handPosPlot_mov_ptr; /**< pointer to the hand position plot of the movement */
    boost::shared_ptr<HandPosPlot> handPosPlot_mov_ptr; /**< pointer to the hand position plot of the movement */

    boost::shared_ptr<HandPosPlot> handPosPlot_task_ptr;/**< pointer to the hand position plot of the task */

    //waypointPtr waypoints; /** Pointer to the waypoints **/
    waypointPtr waypoints;
};

}  //  motion_manager

#endif // motion_manager_MAIN_WINDOW_H
