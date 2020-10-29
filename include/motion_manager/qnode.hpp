#ifndef motion_manager_QNODE_HPP_
#define motion_manager_QNODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <numeric>
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

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <intera_motion_msgs/MotionCommandAction.h>
#include <intera_core_msgs/IODeviceStatus.h>
#include <intera_core_msgs/IOComponentCommand.h>
#include <intera_core_msgs/IODataStatus.h>
#include <intera_core_msgs/RobotAssemblyState.h>
#include <intera_core_msgs/HeadState.h>
#include <intera_core_msgs/HeadPanCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>

#include <boost/log/core.hpp>
#include <boost/lexical_cast.hpp>
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

#include "waypoint.hpp"

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

typedef actionlib::SimpleActionClient<intera_motion_msgs::MotionCommandAction> motionCommClient; /**< */
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> followJointTrajectoryClient; /**< */

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


    //***************************** Functions related to the V-Rep simulator *****************************//
    /**
     * @brief This method checks if V-REP is online
     * @return
     */
    bool checkVrep();

    /**
     * @brief This is the run() method of the thread
     */
    void run();

    /**
     * @brief This method stops the simulation in V-REP
     */
    void stopSim();

    /**
     * @brief This method sets to zero the time of simulation
     */
    void resetSimTime();

    /**
     * @brief This method resets some global variables
     */
    void resetGlobals();

    /**
     * @brief getArmsHandles
     *
     * @param robot
     * @return
     */
    bool getArmsHandles(int robot);

    /**
     * @brief This method loads the scenario with index id
     * @param path
     * @param id
     * @return
     */
    bool loadScenario(const string &path,int id);

    /**
     * @brief This method gets the elements of the scenario
     * @param scene
     * @return
     */
    bool getElements(scenarioPtr scene);

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

    bool setWaypoint(vector<double> &robot_waypoints);

    void updateWaypoints(vector <vector<double>> robot_wps);

    bool execMovement(std::vector<MatrixXd> &traj_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string> &traj_descr, movementPtr mov, scenarioPtr scene);

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
    bool execTask(vector<vector<MatrixXd>> &traj_task, vector<vector<vector<double>>> &timesteps_task, vector<vector<double>> &tols_stop_task, vector<vector<string>> &traj_descr_task, taskPtr task, scenarioPtr scene);

    /**
     * @brief This method subtracts the offset of the joints to their position
     * @param traj_mov
     * @return
     */
    vector<MatrixXd> robotJointPositions(std::vector<MatrixXd>& traj_mov);


    //************************ Functions related to the collaborative robot Sawyer ***********************//
#if ROBOT == 1
    /**
     * @brief execMovementSawyer
     * @param traj_mov
     * @param timesteps
     * @param traj_descr
     * @param mov
     * @param paramsTimeMapping
     * @return
     */
    bool execMovementSawyer(std::vector<MatrixXd>& traj_mov, std::vector<std::vector<double>> timesteps_mov, std::vector<string>& traj_descr, movementPtr mov, vector<double> paramsTimeMapping);

    /**
     * @brief execTask_Sawyer
     * @param traj_task
     * @param vel_task
     * @param acc_task
     * @param timesteps
     * @return
     */
    bool execTaskSawyer(vector<vector<MatrixXd>> &traj_task, vector<vector<vector<double>>> &timesteps_task, vector<vector<double>> &tols_stop_task, vector<vector<string>> &traj_descr_task, taskPtr task, vector<vector<double>> &paramsTimeMapping);

#endif


    //************************* Functions related to the logging functionalities *************************//
    /** This enumerator is used for logging functionalities */
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
     * @brief This method return the list of loggings
     * @return
     */
    QStringListModel* loggingModel();

    /**
     * @brief This method gets the waypoints of the movement
     * @param scene
     * @return
     */
    bool getWaypoints(scenarioPtr scene);

private:
    /**
     * @brief This method gets the current date and time already formatted
     * @return
     */
    const string currentDateTime();

    /**
     * @brief This method initializate the logging
     */
    void init();

    /**
     * @brief This method update the information of a generic object in V-REP
     * @param obj_id
     * @param name
     * @param data
     */
    void updateObjectInfo(int obj_id, std::string name, const geometry_msgs::PoseStamped &data);

    /**
     * @brief This method returns the linear interpolation
     * @param ya
     * @param yb
     * @param m
     * @return
     */
    double interpolate(double ya, double yb, double m);

    /**
     * @brief This method return the RPY values starting from the transformation matrix
     * @param Trans
     * @param rpy
     * @return
     */
    bool getRPY(Matrix4d Trans, std::vector<double>& rpy);

    /**
     * @brief This method return the RPY values starting from the rotation matrix
     * @param rpy
     * @param rot
     * @param rpy
     * @return
     */
    bool RotgetRPY(Matrix3d rot, std::vector<double>& rpy);

    /**
     * @brief RPY_matrix
     * @param rpy
     * @param Rot
     */
    void RPY_matrix(std::vector<double>rpy, Matrix3d &Rot);

#if HAND == 0
    /**
     * @brief preMovementOperation
     * @param node
     * @param movType
     * @param retreat
     * @param armCode
     * @param attach
     * @param hand_pos
     * @param objName
     */
    void preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int arm, int attach, MatrixXd traj, string objName);
#elif HAND == 1
    /**
     * @brief preMovementOperation
     * @param node
     * @param movType
     * @param retreat
     * @param attach
     * @param objName
     */
    void preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int attach, string objName);
#elif HAND == 2
    /**
     * @brief preMovementOperation
     * @param node
     * @param movType
     * @param retreat
     * @param attach
     * @param objName
     */
    void preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int attach, string objName);
#endif

    /**
     * @brief posMovementOperation
     * @param node
     * @param movType
     * @param plan
     * @param attach
     */
    void posMovementOperation(ros::NodeHandle node, int movType, bool plan, int attach);

#if HAND == 0
    /**
     * @brief publishData
     * @param node
     * @param traj
     * @param timesteps
     * @param handles
     * @param hand_handles
     * @param sceneID
     * @param timeTot
     * @param tol
     */
    void publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, MatrixXi hand_handles, int sceneID, double timeTot, double tol);

#elif HAND == 1
    /**
     * @brief publishData
     * @param node
     * @param traj
     * @param timesteps
     * @param handles
     * @param sceneID
     * @param timeTot
     * @param tol
     */
    void publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, int sceneID, double timeTot, double tol);
#elif HAND == 2
    /**
     * @brief publishData
     * @param node
     * @param traj
     * @param timesteps
     * @param handles
     * @param sceneID
     * @param timeTot
     * @param tol
     */
    void publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, int sceneID, double timeTot, double tol);
#endif

    /**
     * @brief joinStages
     * @param traj_mov
     * @param timesteps_mov
     */
    void joinStages(std::vector<MatrixXd> &traj, std::vector<std::vector<double>> &timesteps, std::vector<string> &traj_descr);

    /**
     * @brief joinMovements
     * @param traj
     * @param timesteps
     * @param traj_descr
     */
    void joinMovements(vector<vector<MatrixXd>> &traj, vector<vector<vector<double>>> &timesteps, vector<string> &task_descr);


    //******************************* Functions related to the Barrett Hand ******************************//
#if HAND == 0
    /**
     * @brief closeBarrettHand_to_pos
     * @param hand
     * @param hand_posture
     * @return
     */
    bool closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture);
#endif


    //************************ Functions related to the Collaborative Robot Sawyer ***********************//
#if ROBOT == 1
    /**
     * @brief moveRobotToStartPos
     * @param simStartPos
     * @param tols
     */
    bool moveRobotToStartPos(VectorXd &goal, double tol);

    /**
     * @brief moveHeadToStartPos
     * @param tol
     */
    void moveHeadToStartPos(double tol);

#if HAND == 1
    /**
     * @brief setGripperPosition
     * @param newPos
     */
    void setGripperPosition(double newPos);

    /**
     * @brief moveGripperToStartPos
     * @param simStartPos
     * @param tols
     */
    void moveGripperToStartPos(double simStartPos, double tol);

    /**
     * @brief feedbackCb
     * @param feedback
     * @param tarArmPos
     * @param posGripper
     * @param params
     * @param initStepStage
     * @param movType
     * @param movStages
     */
    void feedbackCb(const control_msgs::FollowJointTrajectoryFeedback::ConstPtr &feedback, std::vector<vector<double>> &trajToExecute, vector<double> &params, bool isClosed);
#endif

    /**
     * @brief divideMovement
     * @param nMicroSteps
     * @param traj
     * @param timesteps
     */
    void jointInterpolation(int nMicroSteps, MatrixXd &traj_stage, std::vector<double> &timesteps_stage, std::vector<vector<double>> &jointsPos, std::vector<double> &timeFromStart);

    /**
     * @brief QNode::jointTrajectoryAction
     * @param trajToExecute
     * @param timeFromStart
     * @return
     */
    bool jointTrajectoryAction(std::vector<vector<double>> &trajToExecute, std::vector<double> &timeFromStart, std::vector<double> params, bool closeGripper, bool isGripperClosed);
#endif


    //********************************* Callbacks of the V-REP scenarios *********************************//
    /**
     * @brief This is the callback to retrieve information about the simulation in V-REP
     * @param info
     */
    void infoCallback(const vrep_common::VrepInfoConstPtr& info);

    /**
     * @brief This is the callback to retrieve the state of the joints
     * @param state
     * @param robot
     */
    void JointsCallback(const sensor_msgs::JointState& state);


    //***************************************** Sensors callbacks ****************************************//
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


    //******************************* Callbacks of the Toy Vehicle scenario ******************************//
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


    //**************************** Callbacks of the Human Assistance scenario ****************************//
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


    //**************************** Callacks of the collaborative robot Sawyer ****************************//
#if ROBOT == 1
    /**
     * @brief This is the callback to retrieve the state of the joints (Robot Sawyer)
     * @param state
     */
    void SawyerJointsCallback(const sensor_msgs::JointState &state);

    /**
     * @brief SawyerHeadCallback
     * @param state
     */
    void SawyerHeadCallback(const intera_core_msgs::HeadState &state);

#if HAND == 1
    /**
     * @brief
     * @param state
     */
    void SawyerGripperCallback(const intera_core_msgs::IODeviceStatus &state);
#endif
#endif
    //**************************** Callacks of the collaborative robot UR10 ****************************//

    void URJointsCallback(const sensor_msgs::JointState &state);


    int init_argc; /**< initial argc */
    char** init_argv; /**< initial argv */
    QStringListModel logging_model; /**< list of loggings */
    string nodeName; /**< name of the ROS node */
    double TotalTime; /**< total time of the movements */
    scenarioPtr curr_scene; /**< current scenario */
    movementPtr curr_mov; /**< current movement that is being executed */
    src::severity_logger< severity_level > lg; /**< logger */


    //***************************************** V-Rep simulator ******************************************//
    bool simulationRunning; /**< true if the simulation in V-REP is running */
    double simulationTime;/**< current time of the simulation */
    double simulationTimeStep;/**< current time step of the simulation */
    bool got_scene; /**< true if we got all the elements of the scenario */
    bool obj_in_hand; /**< true if the object is in the hand */
    // **** Handles **** //
    int right_sensor; /**< handle of the right hand proximity sensor */
    int left_sensor; /**< handle of the left hand proximity sensor */
    int h_detobj; /**< handle of the object that is currently detected by the proximity sensor of the end effector */
    std::vector<int> right_handles; /**< right arm and right hand joints handles */
    std::vector<int> left_handles; /**< left arm and left hand joints handles */
    int right_attach; /**< right hand attach point */
    int left_attach; /**< left hand attach point */
#if HAND == 0
    // **** BarrettHand ****//
    std::vector<bool> firstPartLocked;
    std::vector<int> needFullOpening;
    std::vector<bool> closed;
    // Handles
    MatrixXi right_hand_handles; /**< matrix of the handles of the right hand joints */
    MatrixXi left_hand_handles; /**< matrix of the handles of the left hand joints */
    // Fingers Position, velocity and forces
    std::vector<double> right_2hand_pos; /**< position of the right hand 2 phalanx */
    std::vector<double> right_2hand_vel; /**< velocity of the right hand 2 phalanx */
    std::vector<double> right_2hand_force; /**< forces of the right hand 2 phalanx */
    std::vector<double> left_2hand_pos; /**< position of the left hand 2 phalanx */
    std::vector<double> left_2hand_vel; /**< velocity of the left hand 2 phalanx */
    std::vector<double> left_2hand_force; /**< forces of the left hand 2 phalanx */
#elif HAND == 1
    // **** Electric Gripper **** //
    bool closed;
#elif HAND == 2
    bool closed;
#endif
    // **** ROS communication **** //
    std::vector<double> theta_offset; /**< offset angle around the z axis between consecutive x axes in [rad]*/
    ros::ServiceClient add_client;/**<  ROS client */
    // Subscribers of the information, joint states and sensors
    ros::Subscriber subInfo; /**< ROS subscriber for information about the simulation */
    ros::Subscriber subJoints_state; /**< ROS subscriber to the topic /vrep/joint_state */
    ros::Subscriber subRightProxSensor;/**< ROS subscriber to the topic /vrep/right_prox_sensor */
    ros::Subscriber subLeftProxSensor; /**< ROS subscriber to the topic /vrep/left_prox_sensor */
    // Subscribers of the Toy vehicle scenario
    ros::Subscriber subBlueColumn; /**< ROS sunscriber to the topic /vrep/BlueColumn_pose (obj_id=0 in the toy vehicle scenario) */
    ros::Subscriber subGreenColumn; /**< ROS sunscriber to the topic /vrep/GreenColumn_pose (obj_id=1 in the toy vehicle scenario) */
    ros::Subscriber subRedColumn; /**< ROS sunscriber to the topic /vrep/RedColumn_pose (obj_id=2 in the toy vehicle scenario) */
    ros::Subscriber subMagentaColumn; /**< ROS sunscriber to the topic /vrep/MagentaColumn_pose (obj_id=3 in the toy vehicle scenario) */
    ros::Subscriber subNut1; /**< ROS sunscriber to the topic /vrep/Nut1_pose (obj_id=4 in the toy vehicle scenario) */
    ros::Subscriber subNut2; /**< ROS sunscriber to the topic /vrep/Nut2_pose (obj_id=5 in the toy vehicle scenario) */
    ros::Subscriber subWheel1; /**< ROS sunscriber to the topic /vrep/Wheel1_pose (obj_id=6 in the toy vehicle scenario) */
    ros::Subscriber subWheel2; /**< ROS sunscriber to the topic /vrep/Wheel2_pose (obj_id=7 in the toy vehicle scenario) */
    ros::Subscriber subBase; /**< ROS sunscriber to the topic /vrep/Base_pose (obj_id=8 in the toy vehicle scenario) */
    // Subscribers of the Human Assistance scenario
    ros::Subscriber subBottleTea; /**< ROS sunscriber to the topic /vrep/BottleTea_pose (obj_id=0 in the Human Assistance scenario) */
    ros::Subscriber subBottleCoffee; /**< ROS sunscriber to the topic /vrep/BottleCoffee_pose (obj_id=1 in the Human Assistance scenario) */
    ros::Subscriber subBottleJuice; /**< ROS sunscriber to the topic /vrep/BottleJuice_pose (obj_id=2 in the Human Assistance scenario) */
    ros::Subscriber subCup; /**< ROS sunscriber to the topic /vrep/Cup_pose (obj_id=3 in the Human Assistance scenario) */
    ros::Subscriber subCup1; /**< ROS sunscriber to the topic /vrep/Cup1_pose (obj_id=4 in the Human Assistance scenario) */
#if ROBOT == 1
    //************************************ Collaborative robot Sawyer ************************************//
    std::vector<double> robotPosture; /**< position of the right arm*/
    std::vector<double> robotVel; /**< velocity of the right arm*/
    // Head
    float pan;
    // **** ROS communication **** //
    // create the action client
    // Actionlibs clients
    motionCommClient* motionComm; /**< */
    followJointTrajectoryClient* folJointTraj; /**< */
    // Subscribers of the joints and robot states
    ros::Subscriber subJointsStateRobot; /**< ROS subscriber to the topic /robot/joint_states*/
    ros::Subscriber subHeadState;
    ros::Publisher pubHeadState;
#if HAND == 1
    // **** Electric Gripper **** //
    ros::Subscriber subGripperStateRobot; /**< ROS subscriber to the topic */
    ros::Publisher pubCommGripper; /**< */
    bool gripperCalibrated;
#endif
#endif
#if UR ==1
    //************************************ Collaborative robot Sawyer ************************************//
    std::vector<double> robotPosture_wp; /**< position of the right arm*/
    std::vector<double> robotVel_wp; /**< velocity of the right arm*/
    // Subscribers of the joints and robot states
    ros::Subscriber subJointsStateRobotUR; /**< ROS subscriber to the topic /joint_states*/
    vector<vector<double>> robot_waypoints;
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
     * @brief This method signals a new waypoint
     * @param value
     */
    void newWaypoint(string value);

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
