#ifndef motion_manager_QNODE_HPP_
#define motion_manager_QNODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <ros/callback_queue.h>
#include <QThread>
#include <QStringListModel>
#include <vrep_common/VrepInfo.h>
#include <vrep_common/ProximitySensorData.h>
#include <vrep_common/simRosSetStringSignal.h>
#include <algorithm>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include "config.hpp"
#include "task.hpp"
#include "scenario.hpp"


namespace motion_manager
{

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std;
using namespace logging::trivial;

typedef boost::shared_ptr<Scenario> scenarioPtr;/**< shared pointer to the current scenario */
typedef boost::shared_ptr<Task> taskPtr; /**< shared pointer to the current task */
typedef boost::shared_ptr<Object> objectPtr;/**< shared pointer to an object in the scenario */

const double MIN_EXEC_TIMESTEP_VALUE = 0.3; /**< minimum value of the timestep during the execution of the movement [sec]. It is used to join the stages of the movements when timestep is zero*/

class QNode : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief QNode, a constructor
     * @param argc
     * @param argv
     */
    QNode(int argc, char** argv );

    /**
     * @brief ~QNode, a destructor
     */
    virtual ~QNode();

    /**
     * @brief This method initializates the node
     * @return
     */
    bool on_init();

    /**
     * @brief This method initializates the node
     * @param master_url
     * @param host_url
     * @return
     */
    bool on_init_url(const string &master_url, const string &host_url);

    /**
     * @brief This method runs ending operations
     */
    void on_end();

    /**
     * @brief This method checks if V-REP is online
     * @return
     */
    bool checkVrep();

    /**
     * @brief This method checks if RViz is online
     * @return
     */
    bool checkRViz();

    /**
     * @brief This method loads the scenario with index id
     * @param path
     * @param id
     * @return
     */
    bool loadScenario(const string &path,int id);

#if MOVEIT==1
    /**
     * @brief loadRVizScenario
     * @param objs
     */
    void loadRVizScenario(std::vector<objectPtr>& objs);
#endif

    /**
     * @brief This method gets the elements of the scenario
     * @param scene
     * @return
     */
    bool getElements(scenarioPtr scene);

    /**
     * @brief getArmsHandles
     * @param robot
     * @return
     */
    bool getArmsHandles(int robot);

    /**
     * @brief execMovement
     * @param traj_mov
     * @param vel_mov
     * @param timesteps
     * @param tols_stop
     * @param traj_descr
     * @param mov
     * @param scene
     * @return
     */
    bool execMovement(std::vector<MatrixXd>& traj_mov, std::vector<MatrixXd>& vel_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string>& traj_descr, movementPtr mov, scenarioPtr scene);

    /**
     * @brief execMovement_Sawyer
     * @return
     */
#if ROBOT==1
    bool execMovement_Sawyer(std::vector<MatrixXd>& traj_mov);
#endif

    /**
     * @brief execTask
     * @param traj_task
     * @param vel_task
     * @param timesteps_task
     * @param tols_stop_task
     * @param traj_descr_task
     * @param task
     * @param scene
     * @return
     */
    bool execTask(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double> > > &timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task,taskPtr task, scenarioPtr scene);

    /**
     * @brief execTask_Sawyer
     * @param traj_task
     * @param vel_task
     * @param timesteps_task
     * @param tols_stop_task
     * @param traj_descr_task
     * @param task
     * @param scene
     * @return
     */
#if ROBOT==1
    bool execTask_Sawyer(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double> > > &timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task,taskPtr task, scenarioPtr scene);
#endif

    /**
     * @brief execTask_complete
     * @param traj_task
     * @param vel_task
     * @param timesteps_task
     * @param tols_stop_task
     * @param traj_descr_task
     * @param task
     * @param scene
     * @return
     */
    bool execTask_complete(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double> > > &timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task, taskPtr task, scenarioPtr scene);

    /**
     * @brief This method sets to zero the time of simulation
     */
    void resetSimTime();

    /**
     * @brief This method resets some global variables
     */
    void resetGlobals();

    /**
     * @brief This method stops the simulation in V-REP
     */
    void stopSim();

    /**
     * @brief This is the run() method of the thread
     */
    void run();

    /**
     * @brief This method return the list of loggings
     * @return
     */
    QStringListModel* loggingModel() { return &logging_model; }

    /**
     * @brief This enumerator is used for logging functionalities
     */
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    /**
     * @brief This method runs logging of the passed message
     * @param level
     * @param msg
     */
    void log(const LogLevel &level, const string &msg);

    /**
     * @brief This method subtracts the offset of the joints to their position
     * @param traj_mov
     * @return
     */
    vector<MatrixXd> realJointsPosition(std::vector<MatrixXd>& traj_mov);

private:
    /**
     * @brief This method gets the current date and time already formatted
     * @return
     */
    const string currentDateTime();

    /**
     * @brief This is the callback to retrieve information about the simulation in V-REP
     * @param info
     */
    void infoCallback(const vrep_common::VrepInfoConstPtr& info);

    /**
     * @brief This is the callback to retrieve the state of the proximity sensor on the right end-effector
     * @param data
     */
    void rightProxCallback(const vrep_common::ProximitySensorData& data);

    /**
     * @brief This is the callback to retrieve the state of the proximity sensor on the left end-effector
     * @param data
     */
    void leftProxCallback(const vrep_common::ProximitySensorData& data);

    /**
     * @brief This is the callback to retrieve the state of the blue column (toy vehicle scenario)
     * @param data
     */
    void BlueColumnCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the green column (toy vehicle scenario)
     * @param data
     */
    void GreenColumnCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the red column (toy vehicle scenario)
     * @param data
     */
    void RedColumnCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the magenta column (toy vehicle scenario)
     * @param data
     */
    void MagentaColumnCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the nut1 (toy vehicle scenario)
     * @param data
     */
    void Nut1Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the nut2 (toy vehicle scenario)
     * @param data
     */
    void Nut2Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the wheel1 (toy vehicle scenario)
     * @param data
     */
    void Wheel1Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the wheel2 (toy vehicle scenario)
     * @param data
     */
    void Wheel2Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the base (toy vehicle scenario)
     * @param data
     */
    void BaseCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the bottle tea (human assistance scenario)
     * @param data
     */
    void BottleTeaCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the bottle coffee (human assistance scenario)
     * @param data
     */
    void BottleCoffeeCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the bottle juice (human assistance scenario)
     * @param data
     */
    void BottleJuiceCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the cup (human assistance scenario)
     * @param data
     */
    void CupCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the cup 1 (human assistance scenario)
     * @param data
     */
    void Cup1Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the cup (challenging scenario)
     * @param data
     */
    void Cup_shelfCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf (challenging scenario)
     * @param data
     */
    void ShelfCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_1_b (challenging scenario)
     * @param data
     */
    void Shelf_1_bCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_2_a (challenging scenario)
     * @param data
     */
    void Shelf_2_aCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_2_b (challenging scenario)
     * @param data
     */
    void Shelf_2_bCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_3 (challenging scenario)
     * @param data
     */
    void Shelf_3Callback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_4_a (challenging scenario)
     * @param data
     */
    void Shelf_4_aCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_4_b (challenging scenario)
     * @param data
     */
    void Shelf_4_bCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_4_c (challenging scenario)
     * @param data
     */
    void Shelf_4_cCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the shelf_4_d (challenging scenario)
     * @param data
     */
    void Shelf_4_dCallback(const geometry_msgs::PoseStamped& data);

    /**
     * @brief This is the callback to retrieve the state of the joints
     * @param state
     * @param robot
     */
    void JointsCallback(const sensor_msgs::JointState& state);

 #if ROBOT == 1
    /**
     * @brief This is the callback to retrieve the state of the joints (Robot Sawyer)
     * @param state
     */
    void SawyerJointsCallback(const sensor_msgs::JointState &state);
#endif

    /**
     * @brief This method returns the linear interpolation
     * @param ya
     * @param yb
     * @param m
     * @return
     */
    double interpolate(double ya, double yb, double m);

    /**
     * @brief This method initializate the logging
     */
    void init();

#if HAND == 1
    /**
     * @brief This method closes the Barrett hand
     * @param hand
     * @return
     */
    bool closeBarrettHand(int hand);

    /**
     * @brief This method opens the Barrett hand
     * @param hand
     * @return
     */
    bool openBarrettHand(int hand);

    /**
     * @brief openBarrettHand_to_pos
     * @param hand
     * @param hand_posture
     * @return
     */
    bool openBarrettHand_to_pos(int hand, std::vector<double>& hand_posture);

    /**
     * @brief closeBarrettHand_to_pos
     * @param hand
     * @param hand_posture
     * @return
     */
    bool closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture);
#endif
    /**
     * @brief This method return the RPY values starting from the transformation matrix
     * @param Trans
     * @param rpy
     * @return
     */
    bool getRPY(Matrix4d Trans, std::vector<double>& rpy);

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     */
    void RPY_matrix(std::vector<double>rpy, Matrix3d &Rot);

    /**
     * @brief This method update the information of a generic object in V-REP
     * @param obj_id
     * @param name
     * @param data
     */
    void updateObjectInfo(int obj_id,std::string name, const geometry_msgs::PoseStamped &data);

    /**
     * @brief getSteps
     * @param maxLimits
     * @param minLimits
     * @param initPosture
     * @param finalPosture
     * @return
     */
    int getSteps(std::vector<double> &maxLimits, std::vector<double> &minLimits, std::vector<double> &initPosture, std::vector<double> &finalPosture);

    int init_argc; /**< initial argc */
    char** init_argv; /**< initial argv */
    //*********************************** VRep simulator
    ros::ServiceClient add_client;/**<  ROS client */
    ros::Subscriber subInfo; /**< ROS subscriber for information about the simulation */
    ros::Subscriber subJoints_state; /**< ROS subscriber to the topic /vrep/joint_state */
    ros::Subscriber subRightProxSensor;/**< ROS subscriber to the topic /vrep/right_prox_sensor */
    ros::Subscriber subLeftProxSensor; /**< ROS subscriber to the topic /vrep/left_prox_sensor */
    //*********************************** Toy vehicle scenario
    ros::Subscriber subBlueColumn; /**< ROS sunscriber to the topic /vrep/BlueColumn_pose (obj_id=0 in the toy vehicle scenario) */
    ros::Subscriber subGreenColumn; /**< ROS sunscriber to the topic /vrep/GreenColumn_pose (obj_id=1 in the toy vehicle scenario) */
    ros::Subscriber subRedColumn; /**< ROS sunscriber to the topic /vrep/RedColumn_pose (obj_id=2 in the toy vehicle scenario) */
    ros::Subscriber subMagentaColumn; /**< ROS sunscriber to the topic /vrep/MagentaColumn_pose (obj_id=3 in the toy vehicle scenario) */
    ros::Subscriber subNut1; /**< ROS sunscriber to the topic /vrep/Nut1_pose (obj_id=4 in the toy vehicle scenario) */
    ros::Subscriber subNut2; /**< ROS sunscriber to the topic /vrep/Nut2_pose (obj_id=5 in the toy vehicle scenario) */
    ros::Subscriber subWheel1; /**< ROS sunscriber to the topic /vrep/Wheel1_pose (obj_id=6 in the toy vehicle scenario) */
    ros::Subscriber subWheel2; /**< ROS sunscriber to the topic /vrep/Wheel2_pose (obj_id=7 in the toy vehicle scenario) */
    ros::Subscriber subBase; /**< ROS sunscriber to the topic /vrep/Base_pose (obj_id=8 in the toy vehicle scenario) */
    //*********************************** Human Assistance scenario
    ros::Subscriber subBottleTea; /**< ROS sunscriber to the topic /vrep/BottleTea_pose (obj_id=0 in the Human Assistance scenario) */
    ros::Subscriber subBottleCoffee; /**< ROS sunscriber to the topic /vrep/BottleCoffee_pose (obj_id=1 in the Human Assistance scenario) */
    ros::Subscriber subBottleJuice; /**< ROS sunscriber to the topic /vrep/BottleJuice_pose (obj_id=2 in the Human Assistance scenario) */
    ros::Subscriber subCup; /**< ROS sunscriber to the topic /vrep/Cup_pose (obj_id=3 in the Human Assistance scenario) */
    ros::Subscriber subCup1; /**< ROS sunscriber to the topic /vrep/Cup1_pose (obj_id=4 in the Human Assistance scenario) */
    //*********************************** Challenging scenario
    ros::Subscriber subCup_shelf; /**< ROS sunscriber to the topic /vrep/Cup_pose (obj_id=0 in the Human Challenging scenario) */
    ros::Subscriber subShelf; /**< ROS sunscriber to the topic /vrep/Shelf_pose (obj_id=1 in the Challenging scenario) */
    ros::Subscriber subShelf_1_b; /**< ROS sunscriber to the topic /vrep/Shelf_1_b_pose (obj_id=2 in the Challenging scenario) */
    ros::Subscriber subShelf_2_a; /**< ROS sunscriber to the topic /vrep/Shelf_2_a_pose (obj_id=3 in the Challenging scenario) */
    ros::Subscriber subShelf_2_b; /**< ROS sunscriber to the topic /vrep/Shelf_2_b_pose (obj_id=4 in the Challenging scenario) */
    ros::Subscriber subShelf_3; /**< ROS sunscriber to the topic /vrep/Shelf_3_pose (obj_id=5 in the Challenging scenario) */
    ros::Subscriber subShelf_4_a; /**< ROS sunscriber to the topic /vrep/Shelf_4_a_pose (obj_id=6 in the Challenging scenario) */
    ros::Subscriber subShelf_4_b; /**< ROS sunscriber to the topic /vrep/Shelf_4_b_pose (obj_id=7 in the Challenging scenario) */
    ros::Subscriber subShelf_4_c; /**< ROS sunscriber to the topic /vrep/Shelf_4_c_pose (obj_id=8 in the Challenging scenario) */
    ros::Subscriber subShelf_4_d; /**< ROS sunscriber to the topic /vrep/Shelf_4_d_pose (obj_id=9 in the Challenging scenario) */
#if ROBOT==1
    //*********************************** Real Robot
    ros::Subscriber subJoints_state_robot; /**< ROS subscriber to the topic /robot/joint_states*/
    ros::Publisher pubJointCommand_robot; /**< ROS publisher to the ropic /robot/limb/right/joint_command */
    ros::Publisher pubEnable_robot; /** < ROS publisher to the topic /robot/set_super_enable */
    ros::Publisher pubReset_robot; /** < ROS publisher to the topic /robot/set_super_reset */
    ros::Publisher pubStop_robot; /** < ROS publisher to the topic /robot/set_super_stop */
    ros::Publisher pubJointCommand_timeout_robot; /**< ROS publisher to the ropic /robot/limb/right/joint_command_timeout */
    ros::Publisher pubRate_robot;
    ros::Publisher pubSpeedRatio_robot;
#endif
#if MOVEIT==1
    boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_ptr;/**< scene interface */
#endif
    QStringListModel logging_model; /**< list of loggings */
    bool simulationRunning; /**< true if the simulation in V-REP is running */
    double simulationTime;/**< current time of the simulation */
    double simulationTimeStep;/**< current time step of the simulation */
    string nodeName; /**< name of the ROS node */
    double TotalTime; /**< total time of the movements */
    scenarioPtr curr_scene; /**< current scenario */
    movementPtr curr_mov; /**< current movement that is being executed */
    src::severity_logger< severity_level > lg; /**< logger */
    int right_sensor; /**< handle of the right hand proximity sensor */
    int left_sensor; /**< handle of the left hand proximity sensor */
    int h_detobj; /**< handle of the object that is currently detected by the proximity sensor of the end effector */
    int right_attach; /**< right hand attach point */
    int left_attach; /**< left hand attach point */
    bool got_scene; /**< true if we got all the elements of the scenario */
    bool obj_in_hand; /**< true if the object is in the hand */
    std::vector<double> theta_offset; /**< offset angle around the z axis between consecutive x axes in [rad]*/
    //*********************************** Hand
    std::vector<int> right_handles; /**< right arm and right hand joints handles */
    std::vector<int> left_handles; /**< left arm and left hand joints handles */
    MatrixXi right_hand_handles; /**< matrix of the handles of the right hand joints */
    MatrixXi left_hand_handles; /**< matrix of the handles of the left hand joints */
    std::vector<double> right_2hand_pos; /**< position of the right hand 2 phalanx */
    std::vector<double> right_2hand_vel; /**< velocity of the right hand 2 phalanx */
    std::vector<double> right_2hand_force; /**< forces of the right hand 2 phalanx */
    std::vector<double> left_2hand_pos; /**< position of the left hand 2 phalanx */
    std::vector<double> left_2hand_vel; /**< velocity of the left hand 2 phalanx */
    std::vector<double> left_2hand_force; /**< forces of the left hand 2 phalanx */
    //*********************************** Real Robot
    std::vector<double> robotPosture; /**< position of the right arm*/
#if HAND ==1
    std::vector<bool> firstPartLocked;
    std::vector<int> needFullOpening;
    std::vector<bool> closed;
#endif

Q_SIGNALS:
    /**
     * @brief This signal is used to adjust the scrollbar of the logging list
     */
    void loggingUpdated();

    /**
     * @brief This signal is used to close the main window
     */
    void rosShutdown();

    /**
     * @brief This method signals that a new element is part of the scenario
     * @param value
     */
    void newElement(string value);

    /**
     * @brief updateElement
     * @param id
     * @param value
     */
    void updateElement(int id,string value);

    /**
     * @brief This method signals a new object in the scenario
     * @param value
     */
    void newObject(string value);

    /**
     * @brief This method signals a new pose in the scenario
     * @param value
     */
    void newPose(string value);

    /**
     * @brief This method signals that a new joint is part of the robot
     * @param value
     */
    void newJoint(string value);
};

}  // namespace motion_manager

#endif /* motion_manager_QNODE_HPP_ */
