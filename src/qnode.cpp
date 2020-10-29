#include <ros/ros.h>

#include <ros/network.h>
#include <string>
#include <iterator>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <intera_core_msgs/JointCommand.h>
#include <sstream>
#include <time.h>
#include "../include/motion_manager/qnode.hpp"
#include <vrep_common/simRosLoadScene.h>
#include <vrep_common/simRosCloseScene.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>
#include <vrep_common/simRosGetFloatSignal.h>
#include <vrep_common/simRosGetIntegerSignal.h>
#include <vrep_common/simRosGetStringSignal.h>
#include <vrep_common/simRosSynchronous.h>
#include <vrep_common/simRosSynchronousTrigger.h>
#include <vrep_common/simRosSetJointTargetVelocity.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosEnablePublisher.h>
#include <vrep_common/simRosEnableSubscriber.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosReadProximitySensor.h>
#include <vrep_common/simRosSetObjectParent.h>
#include <vrep_common/JointSetStateData.h>
#include <vrep_common/simRosSetObjectIntParameter.h>
#include <vrep_common/simRosSetJointForce.h>
#include <vrep_common/simRosSetJointPosition.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include "../include/motion_manager/v_repConst.hpp"

namespace motion_manager{

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
    nodeName = "motion_manager";
    TotalTime = 0.0;

#if HAND == 0
    // Barrett Hand
    right_hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1);
    left_hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1);
    right_2hand_pos.assign(3, 0.0f);
    right_2hand_vel.assign(3, 0.0f);
    right_2hand_force.assign(3, 0.0f);
    left_2hand_pos.assign(3, 0.0f);
    left_2hand_vel.assign(3, 0.0f);
    left_2hand_force.assign(3, 0.0f);
    firstPartLocked.assign(3, false);
    needFullOpening.assign(3, 0);
    closed.assign(3, false);
#elif HAND == 1
    // Electric Gripper
    closed = false;
#endif

#if ROBOT == 1
#if HAND == 0
    int njoints = JOINTS_ARM;
#elif HAND == 1
    int njoints = JOINTS_ARM + JOINTS_HAND;
#endif
    // Collaborative Robot Sawyer
    robotPosture.assign(njoints, 0.0f);
    robotVel.assign(njoints, 0.0f);
#endif
#if UR == 1
    int njoints = JOINTS_ARM;
    // Collaborative Robot UR10
    robotPosture_wp.assign(njoints, 0.0f);
    robotVel_wp.assign(njoints, 0.0f);
#endif
    //Objects information
    got_scene = false;
    obj_in_hand = false;

    // logging
    init();
    logging::add_common_attributes();
}


QNode::~QNode()
{
    if(ros::isStarted())
        ros::shutdown();
    wait();
}


bool QNode::on_init()
{
    ros::init(init_argc, init_argv, "motion_manager");

    if(!ros::master::check())
        return false;

    ros::start();
    start();
    return true;
}


bool QNode::on_init_url(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;

    ros::init(remappings,"motion_manager");

    if(!ros::master::check())
        return false;

    ros::start();
    start();
    return true;
}


//*****************************************************************************************************************************//
//                                                       V-REP Simulator                                                       //
//*****************************************************************************************************************************//
bool QNode::checkVrep()
{
    bool online = false;
    std::string vrep_node = "/vrep";

    ros::V_string nodes;
    ros::master::getNodes(nodes);

    for(ros::V_string::iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        std::string node_name = *it;

        if(node_name == vrep_node)
            online = true;
    }

    return online;
}


void QNode::run()
{
    while (ros::ok())
    {
        // infinite loop while ros is running
    }

    ros::spinOnce();
}


void QNode::stopSim()
{
    ros::NodeHandle node;

    add_client = node.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);
}


void QNode::resetSimTime()
{
    this->TotalTime = 0.0;
}


void QNode::resetGlobals()
{
    obj_in_hand = false;

#if HAND == 0
    for(int i = 0; i < 3; ++i)
    {
        closed.at(i) = false;
        needFullOpening.at(i) = 0;
        firstPartLocked.at(i) = false;
    }
#elif HAND == 1
    closed = false;
#endif
}


bool QNode::getArmsHandles(int robot)
{
    bool succ = true;
    ros::NodeHandle node;
    ros::ServiceClient add_client;

    // get the joint arm + hand handles
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    vrep_common::simRosGetObjectHandle srvgetHandle;

    // **** Get the object handle of the arm joints
    for(int k = 0; k < JOINTS_ARM; ++k)
    {
        srvgetHandle.request.objectName = string("right_joint") + QString::number(k).toStdString();
        add_client.call(srvgetHandle);
        if(srvgetHandle.response.handle != -1)
            right_handles.push_back(srvgetHandle.response.handle);
        else
        {
            throw string("Error: Couldn't get the information about of right arm joints");
            succ = false;
        }

        // ARoS and Jade
        if(robot != 2 && robot != 3 )
        {
            srvgetHandle.request.objectName = string("left_joint") + QString::number(k).toStdString();
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                left_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the left arm joints");
                succ = false;
            }
        }
    }

    // **** Get the object handle of the hand joints
    for(int k = 0; k < JOINTS_HAND; ++k)
    {
        if(k == 0)
        {
#if HAND == 0
            srvgetHandle.request.objectName = string("right_BarrettHand_jointA_0");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointA_0");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointA_0");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_handles.push_back(srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointA_0");
                    succ = false;
                }
            }
#elif HAND == 1
            srvgetHandle.request.objectName = string("right_gripper_jointClose");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_gripper_jointClose");
                succ = false;
            }
#endif
        }
        else if(k == 1)
        {
#if HAND == 0
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_0");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_handles.push_back(srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_0");
                    succ = false;
                }
            }
#endif
        }
        else if(k == 2)
        {
#if HAND == 0
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_2");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_handles.push_back(srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_2");
                    succ = false;
                }
            }
#endif
        }
        else if(k == 3)
        {
#if HAND == 0
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_1");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_handles.push_back(srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_1");
                    succ = false;
                }
            }
#endif
        }
    }

    // **** Get the object handle of the fingers joints
#if HAND == 0
    for(int k = 0; k < HAND_FINGERS; ++k)
    {
        if(k != 1)
        {
            srvgetHandle.request.objectName = string("right_BarrettHand_jointA_")+QString::number(k).toStdString();
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 0) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointA_");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointA_")+QString::number(k).toStdString();
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 0) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointA_");
                    succ = false;
                }
            }
        }

        if(k == 0)
        {
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_0");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_0");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_0");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 2) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointC_0");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_0");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_0");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 2) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointC_0");
                    succ = false;
                }
            }
        }
        else if(k == 1)
        {
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_2");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_2");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_2");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 2) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointC_2");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_2");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_2");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_2");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle !=-1)
                    left_hand_handles(k, 2) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointC_2");
                    succ = false;
                }
            }
        }
        else if(k == 2)
        {
            srvgetHandle.request.objectName = string("right_BarrettHand_jointB_1");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_1");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_1");
            add_client.call(srvgetHandle);
            if(srvgetHandle.response.handle != -1)
                right_hand_handles(k, 2) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointC_1");
                succ = false;
            }

            if(robot != 2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_1");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_1");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_1");
                add_client.call(srvgetHandle);
                if(srvgetHandle.response.handle != -1)
                    left_hand_handles(k, 2) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointC_1");
                    succ = false;
                }
            }
        }
    }
#endif

    // **** Get the object handle of the sensors
#if HAND == 0
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_BarrettHand_attachProxSensor");
    add_client.call(srvgetHandle);
    if(srvgetHandle.response.handle != -1)
        right_sensor = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_BarrettHand_attachProxSensor");
        succ = false;
    }

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_BarrettHand_attachPoint");
    add_client.call(srvgetHandle);
    if(srvgetHandle.response.handle != -1)
        right_attach = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_BarrettHand_attachPoint");
        succ = false;
    }

    if(robot != 2)
    {
        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_BarrettHand_attachProxSensor");
        add_client.call(srvgetHandle);
        if(srvgetHandle.response.handle != -1)
            left_sensor = srvgetHandle.response.handle;
        else
        {
            throw string("Error: Couldn't get the information of the left_BarrettHand_attachProxSensor");
            succ = false;
        }

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_BarrettHand_attachPoint");
        add_client.call(srvgetHandle);
        if(srvgetHandle.response.handle != -1)
            left_attach = srvgetHandle.response.handle;
        else
        {
            throw string("Error: Couldn't get the information of the left_BarrettHand_attachPoint");
            succ = false;
        }
    }
#elif HAND == 1
    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_gripper_attachProxSensor");
    add_client.call(srvgetHandle);
    if(srvgetHandle.response.handle != -1)
        right_sensor = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_gripper_attachProxSensor");
        succ = false;
    }

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_gripper_attachPoint");
    add_client.call(srvgetHandle);
    if(srvgetHandle.response.handle != -1)
        right_attach = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_gripper_attachPoint");
        succ = false;
    }
#endif


    return succ;
}



bool QNode::loadScenario(const std::string& path,int id)
{
    ros::NodeHandle n;

#if ROBOT == 1
    // **** Robot subscribers **** //
    if(id >= 2)
    {
        subJointsStateRobot = n.subscribe("/robot/joint_states", 1, &QNode::SawyerJointsCallback, this);
        subHeadState = n.subscribe("/robot/head/head_state", 1, &QNode::SawyerHeadCallback, this);
#if HAND == 1
        subGripperStateRobot = n.subscribe("/io/end_effector/right_gripper/state", 1, &QNode::SawyerGripperCallback, this);
#endif

        // Create the action client specifying the server name to connect: "/motion/motion_command"
        motionComm = new motionCommClient("/motion/motion_command", true);
        motionComm->waitForServer(); // wait for the action server to start

        // Create the action client specifying the server name to connect: "/robot/limb/right/follow_joint_trajectory"
        folJointTraj = new followJointTrajectoryClient("/robot/limb/right/follow_joint_trajectory", true);
        folJointTraj->waitForServer();
    }
#endif

    // **** V-Rep subscribers **** //
    // pause simulations
    add_client = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);

    // close the old scene
    add_client = n.serviceClient<vrep_common::simRosCloseScene>("/vrep/simRosCloseScene");
    vrep_common::simRosCloseScene srvc;
    add_client.call(srvc);

    // load the new scene
    add_client = n.serviceClient<vrep_common::simRosLoadScene>("/vrep/simRosLoadScene");
    vrep_common::simRosLoadScene srv;
    srv.request.fileName = path;
    add_client.call(srv);
    int res = srv.response.result;

    if(res == 1)
    {
        //subscribe to V-REP's info stream
        subInfo = n.subscribe("/vrep/info", 1, &QNode::infoCallback, this);
        subJoints_state = n.subscribe("/vrep/joints_state", 1, &QNode::JointsCallback, this);
        subRightProxSensor = n.subscribe("/vrep/right_prox_sensor", 1, &QNode::rightProxCallback, this);

        if(id == 0 || id == 1)
            subLeftProxSensor = n.subscribe("/vrep/left_prox_sensor", 1, &QNode::leftProxCallback, this);

        switch(id)
        {
        case 0: case 2: case 4: case 5:
            // **** Assembly scenario: the Toy vehicle with ARoS **** //
            // **** Assembly scenario: the Toy vehicle with Jarde **** //
            // **** Assembly scenario: the Toy vehicle with Sawyer **** //
            // Blue Column (obj_id = 0)
            subBlueColumn = n.subscribe("/vrep/BlueColumn_pose", 1, &QNode::BlueColumnCallback, this);
            // Green Column (obj_id = 1)
            subGreenColumn = n.subscribe("/vrep/GreenColumn_pose", 1, &QNode::GreenColumnCallback, this);
            // RedColumn (obj_id = 2)
            subRedColumn = n.subscribe("/vrep/RedColumn_pose", 1, &QNode::RedColumnCallback, this);
            // MagentaColumn (obj_id = 3)
            subMagentaColumn = n.subscribe("/vrep/MagentaColumn_pose", 1, &QNode::MagentaColumnCallback, this);
            // Nut 1 (obj_id = 4)
            subNut1 = n.subscribe("/vrep/Nut1_pose", 1, &QNode::Nut1Callback, this);
            // Nut 2 (obj_id = 5)
            subNut2 = n.subscribe("/vrep/Nut2_pose", 1, &QNode::Nut2Callback, this);
            // Wheel 1 (obj_id = 6)
            subWheel1 = n.subscribe("/vrep/Wheel1_pose", 1, &QNode::Wheel1Callback, this);
            // Wheel 2 (obj_id = 7)
            subWheel2 = n.subscribe("/vrep/Wheel2_pose", 1, &QNode::Wheel2Callback, this);
            // Base (obj_id = 8)
            subBase = n.subscribe("/vrep/Base_pose", 1, &QNode::BaseCallback, this);
            break;
        case 1: case 3:
            // **** Human assistance scenario: Serving a drink with ARoS **** //
            // **** Human assistance scenario: Serving a drink with Sawyer **** //
            // Bottle Tea (obj_id = 0)
            subBottleTea = n.subscribe("/vrep/BottleTea_pose", 1, &QNode::BottleTeaCallback, this);
            // Bottle Coffee (obj_id = 1)
            subBottleCoffee = n.subscribe("/vrep/BottleCoffee_pose", 1, &QNode::BottleCoffeeCallback, this);
            // Bottle Juice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose", 1, &QNode::BottleJuiceCallback, this);
            // Cup (obj_id = 3)
            subCup = n.subscribe("/vrep/Cup_pose", 1, &QNode::CupCallback, this);
            // Cup 1 (obj_id = 4)
            subCup1 = n.subscribe("/vrep/Cup1_pose", 1, &QNode::Cup1Callback, this);
            break;

         case 6:
            //subscribe to the topics of the UR10 Pick and Place scene
            break;
        }

        ros::spinOnce();

        return true;
    }
    else
        return false;
}
// get the elements of the vrep simulation
bool QNode::getElements(scenarioPtr scene)
{
    ros::NodeHandle n;

    // start the simulation
    add_client = n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvc;
    add_client.call(srvc);
    sleep(1);

    int n_objs; // total number of objects in the scenario
    int n_poses; // total number of poses in the scenario
    int cnt_obj = 0; // index of the object being loaded
    int cnt_pose = 0; // index of the pose being loaded
    std::string signPrefix = ""; // prefix of each object
    std::string infoLine; // info line in the list of elements
    std::string signTarRight = "_targetRight";
    std::string signTarLeft = "_targetLeft";
    std::string signEngage ="_engage";
    bool succ = true;
    // **** object info **** //
    pos obj_pos;// position of the object
    orient obj_or;// orientation of the object
    dim obj_size;// size of the object
    std::string obj_info_str;
    std::vector<double> obj_info_vec;
    std::vector<std::string> objs_prefix;
    // **** target info **** //
    pos tarRight_pos;// target right position
    orient tarRight_or;// target right orientation
    pos tarLeft_pos;// target left position
    orient tarLeft_or;// target left orientation
    // **** engage info **** //
    pos engage_pos; // engage point position
    orient engage_or;// engage point orientation
    // **** robot info **** //
    robot_part robot_torso_specs;
    robot_part_q robot_UR_torso_specs;
    arm robot_arm_specs; // specs of the arms
    robot_part robot_head_specs; // specs of the head
    std::string Hname; // name of the robot
    // **** pose info **** //
    std::string pose_info_str;
    std::vector<double> pose_info_vec;
    pos pose_pos;// position of the pose
    orient pose_or;// orientation of the pose
    std::vector<std::string> poses_prefix; // names of the poses
    std::vector<bool> poses_rel; // relations of the poses
    std::vector<int> poses_obj_id; // id of the related object

    vrep_common::simRosGetIntegerSignal srvi;
    vrep_common::simRosGetFloatSignal srvf;
    vrep_common::simRosGetStringSignal srvs;
    vrep_common::simRosGetObjectHandle srv_get_handle;
    ros::ServiceClient client_getHandle;

    // **** DH parameters **** //
    int floatCount;
    std::vector<double> DH_params_vec; // DH parameters
    string DH_params_str;
    // **** Transformation matrices **** //
    // arm: matrices from world to right and left references
    std::string mat_arms_str;
    std::string mat_right_arm_str;
    std::string mat_left_arm_str;
    std::vector<double> mat_arms_vec;
    std::vector<double> mat_right_arm_vec;
    std::vector<double> mat_left_arm_vec;
    Matrix4d mat_right;
    Matrix4d mat_left;
    // hand: matrices from the last joint of the arm to the palm of the hand
    std::string r_mat_hand_str;
    std::string l_mat_hand_str;
    std::vector<double> r_mat_hand_vec;
    std::vector<double> l_mat_hand_vec;
    Matrix4d mat_r_hand;
    Matrix4d mat_l_hand;


    // ** Waypoints ** //
    waypoint wp_specs;
    wp_specs.JointSpace.PosJoints =  vector <double> (JOINTS_ARM); // size of the number of dof
    int wp_ws; // OP space - 1  Joint space - 0
    std::vector <waypoint> waypoints_vec;
    std::string wp_name; // waypoint name
    std::string wp_name_str;
    std::string wp_pos_str;
    string wp_op_or_str;
    std::vector<double> wp_op_or_vec;
    std::vector<double> wp_op_pos_vec;
    //std::vector<std::string> wp_str;
    int wp_nr; // number of waypoints in each trajectorie
    int traj_nr; // number of different trajectories with waypoints
    std::string traj_name; // name of a trajectorie with waypoints
#if HAND == 0
    // **** Barrett Hand parameters **** //
    barrett_hand robot_hand_specs; // specs of the barret hand
    double maxAp; //[mm]
    double Aw; // [mm]
    double A1; // [mm]
    double A2; // [mm]
    double A3; // [mm]
    double D3; // [mm]
    double phi2; // [rad]
    double phi3; // [rad]
#elif HAND == 1
    // **** Electric Parallel Gripper parameters **** //
    electric_gripper robot_gripper_specs; // specs of the electric parallel gripper
    double maxAp; //[mm]
    double minAp;//[mm]
    double A1; //[mm]
    double D3; //[mm]
#elif HAND == 2
    // **** Vacuum Gripper parameters **** //
    vacuum_gripper robot_vacuum_gripper;
    double D7;
#endif

    // **** torso parameters **** //
    robot_part torso;  // parameters of the torso (ARoS, Jarde and Sawyer)
    robot_part_q torso_UR;  // parameters of the UR torso ( orientation in quaternions)
    std::string torso_str;
    std::vector<double> torso_vec;
    // **** head parameters **** //
    robot_part head; // parameters of the head(ARoS and Sawyer)
    std::string head_str;
    std::vector<double> head_vec;

    // **** home postures **** //
    std::vector<double> rposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // right
    std::vector<double> lposture = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // left
    // **** joint limits **** //
    std::vector<double> min_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum right limits
    std::vector<double> max_rlimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum right limits
    std::vector<double> min_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // minimum left limits
    std::vector<double> max_llimits = std::vector<double>(JOINTS_ARM+JOINTS_HAND); // maximum left limits



    // **** others **** //
    int rows;
    const string NOBJECTS = string("n_objects");
    const string NPOSES = string("n_poses");

    // get scenario identification
    int scenarioID  = scene->getID();

    if(scenarioID == 0)
        throw string("No scenario");



    // ******************************* //
    //     Objects in the scenario     //
    // ******************************* //


    //service -> simRosGetIntegerSignal
    //This creates a client for simRosGetIntegerSingal service.
    //The ros::ServiceClient object (add_client)is used to call the service
    add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");
    //vrep_common::simRosGetIntegerSignal srvi-
    // generate a service class. A service class contains two members, request and response.
    //It also contains two class definitions, Request and Response.
    //srvi - is the service  and request a signalName that is the number of objects (n_objects)
    srvi.request.signalName = NOBJECTS;
    //calls the service
    add_client.call(srvi);

    //server srvi response
    if(srvi.response.result == 1)
        //return of objects in simulation
        n_objs = srvi.response.signalValue;
    else
    {
        succ = false;
        throw string("Communication error");
    }

    if(scenarioID == 2 || scenarioID == 4 || scenarioID == 6 ||scenarioID == 7)
    {
        //service class request a signalName NPOSES
        srvi.request.signalName = NPOSES;
        //call the service
        add_client.call(srvi);
        if(srvi.response.result == 1)
            //get the number of the poses
            n_poses = srvi.response.signalValue;
        else
        {
            succ = false;
            throw string("Communication error");
        }
    }
    //creates a client client_getHandle for simRosGetObjectHandle service
    client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    if(scenarioID == 1 || scenarioID ==  3 || scenarioID == 5 || scenarioID == 6) // Toy Vehicle scenarios (ARoS, Sawyer)
    {
        objs_prefix.push_back("BlueColumn");     // obj_id = 0
        objs_prefix.push_back("GreenColumn");    // obj_id = 1
        objs_prefix.push_back("RedColumn");      // obj_id = 2
        objs_prefix.push_back("MagentaColumn");  // obj_id = 3
        objs_prefix.push_back("Nut1");           // obj_id = 4
        objs_prefix.push_back("Nut2");           // obj_id = 5
        objs_prefix.push_back("Wheel1");         // obj_id = 6
        objs_prefix.push_back("Wheel2");         // obj_id = 7
        objs_prefix.push_back("Base");           // obj_id = 8
        objs_prefix.push_back("Table");          // obj_id = 9
    }
    else if(scenarioID == 2 || scenarioID == 4) // Drinking Service scenarios (ARoS, Sawyer)
    {
        objs_prefix.push_back("BottleTea");      // obj_id = 0
        objs_prefix.push_back("BottleCoffee");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");    // obj_id = 2
        objs_prefix.push_back("Cup");            // obj_id = 3
        objs_prefix.push_back("Cup1");           // obj_id = 4
        objs_prefix.push_back("Table");          // obj_id = 5
    }
    else if(scenarioID == 7){ //  (UR10 Universal Robot)
        objs_prefix.push_back("FaultyBox");      // obj_id = 0
        objs_prefix.push_back("GoodBox");        // obj_id = 1
        objs_prefix.push_back("Conveyor");        // obj_id = 1

    }

    while(cnt_obj < n_objs)
    {
        signPrefix = objs_prefix[cnt_obj];
        ///gets the information of the object
        //service simRosGetStringSignal
        // add_client -> client  for the service
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        // srvs -> object created to represent the service vrep_common::simRosGetStringSignal
        // service request info of the object signPrefix
        srvs.request.signalName = signPrefix + string("Info");
        add_client.call(srvs);
        if(srvs.response.result == 1)
            // obj_info_str gets the information of the object by vrep
            obj_info_str = srvs.response.signalValue;
        else
        {
            throw string("Error: Couldn't get the information of the object");
            succ = false;
        }
        if(succ)
        {
            floatCount = obj_info_str.size() / sizeof(float);

            if(!obj_info_vec.empty())
                obj_info_vec.clear();
            for(int k = 0; k < floatCount; ++k)
                //c_str() converts a C++ string into a C-style string which is essentially a null terminated array of bytes.
                //You use it when you want to pass a C++ string into a function that expects a C-style string
                obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

            // position of the object
            obj_pos.Xpos = obj_info_vec.at(0) * 1000; //[mm]
            obj_pos.Ypos = obj_info_vec.at(1) * 1000; //[mm]
            obj_pos.Zpos = obj_info_vec.at(2) * 1000; //[mm]
            // orientation of the object
            obj_or.roll = obj_info_vec.at(3) * static_cast<double>(M_PI)/180; //[rad]
            obj_or.pitch = obj_info_vec.at(4) * static_cast<double>(M_PI)/180; //[rad]
            obj_or.yaw = obj_info_vec.at(5) * static_cast<double>(M_PI)/180;//[rad]
            // size of the object
            obj_size.Xsize = obj_info_vec.at(6) * 1000; //[mm]
            obj_size.Ysize = obj_info_vec.at(7) * 1000; //[mm]
            obj_size.Zsize = obj_info_vec.at(8) * 1000; //[mm]
            if(obj_info_vec.size()>9)
            {
                // position of the target right
                tarRight_pos.Xpos = obj_info_vec.at(9) * 1000;//[mm]
                tarRight_pos.Ypos = obj_info_vec.at(10) * 1000;//[mm]
                tarRight_pos.Zpos = obj_info_vec.at(11) * 1000;//[mm]
                // orientation of the target right
                tarRight_or.roll = obj_info_vec.at(12) * static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.pitch = obj_info_vec.at(13) * static_cast<double>(M_PI)/180;//[rad]
                tarRight_or.yaw = obj_info_vec.at(14) * static_cast<double>(M_PI)/180;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = obj_info_vec.at(15) * 1000;//[mm]
                tarLeft_pos.Ypos = obj_info_vec.at(16) * 1000;//[mm]
                tarLeft_pos.Zpos = obj_info_vec.at(17) * 1000;//[mm]
                // orientation of the target left
                tarLeft_or.roll = obj_info_vec.at(18) * static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.pitch = obj_info_vec.at(19) * static_cast<double>(M_PI)/180;//[rad]
                tarLeft_or.yaw = obj_info_vec.at(20) * static_cast<double>(M_PI)/180;//[rad]
                // position of the engage point
                engage_pos.Xpos = obj_info_vec.at(21) * 1000;//[mm]
                engage_pos.Ypos = obj_info_vec.at(22) * 1000;//[mm]
                engage_pos.Zpos = obj_info_vec.at(23) * 1000;//[mm]
                // orientation of the engage point
                engage_or.roll = obj_info_vec.at(24) * static_cast<double>(M_PI)/180;//[rad]
                engage_or.pitch = obj_info_vec.at(25) * static_cast<double>(M_PI)/180;//[rad]
                engage_or.yaw = obj_info_vec.at(26) * static_cast<double>(M_PI)/180;//[rad]
            }
            else{ // Waypoint scenario only needs the position/orientation and size of the objects

                // position of the target right
                tarRight_pos.Xpos = NULL;//[mm]
                tarRight_pos.Ypos = NULL;//[mm]
                tarRight_pos.Zpos = NULL;//[mm]
                // orientation of the target right
                tarRight_or.roll = NULL;//[rad]
                tarRight_or.pitch = NULL;//[rad]
                tarRight_or.yaw = NULL;//[rad]
                // position of the target left
                tarLeft_pos.Xpos = NULL;//[mm]
                tarLeft_pos.Ypos = NULL;//[mm]
                tarLeft_pos.Zpos = NULL;//[mm]
                // orientation of the target left
                tarLeft_or.roll =NULL;//[rad]
                tarLeft_or.pitch = NULL;//[rad]
                tarLeft_or.yaw = NULL;//[rad]
                // position of the engage point
                engage_pos.Xpos = NULL;//[mm]
                engage_pos.Ypos = NULL;//[mm]
                engage_pos.Zpos = NULL;//[mm]
                // orientation of the engage point
                engage_or.roll = NULL;//[rad]
                engage_or.pitch = NULL;//[rad]
                engage_or.yaw = NULL;//[rad]
            }

            // object class

            Object* ob = new Object(signPrefix, obj_pos, obj_or, obj_size,
                                        new Target(signPrefix + signTarRight, tarRight_pos, tarRight_or),
                                        new Target(signPrefix + signTarLeft, tarLeft_pos, tarLeft_or),
                                        new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

            //get information about the object
            //Organize the information in the constructor above to print
            infoLine = ob->getInfoLine();
            //This method signals that a new element is part of the scenario
            Q_EMIT newElement(infoLine);
            //This method signals a new object in the scenario
            Q_EMIT newObject(ob->getName());

            //handle of the object
            //vrep_common::simRosGetObjectHandle srv_get_handle;
            //service srv_get_handle request a objectName
            //client calls the service
            srv_get_handle.request.objectName = signPrefix;
            client_getHandle.call(srv_get_handle);
            //handle of the visible part of the object (the body)
            ob->setHandle(srv_get_handle.response.handle);

            // handle of the visible object
            srv_get_handle.request.objectName = signPrefix + string("_body");
            client_getHandle.call(srv_get_handle);
            ob->setHandleBody(srv_get_handle.response.handle);
            // add the object to the scenario
            // objectPtr- shared pointer to an object in the scenario
            scene->addObject(objectPtr(ob));
            // add the pose to the scenario
            if(scenarioID == 2 || scenarioID == 4) // Drinking Service scenarios (ARoS, Sawyer)
            {
                Pose* ps = new Pose(signPrefix + string("_home"), tarRight_pos, tarRight_or, true, cnt_obj);
                Q_EMIT newPose(ps->getName());
                scene->addPose(posePtr(ps));
            }

            cnt_obj++;
        }
        else
            throw string("Error while retrieving the objects of the scenario");
    }


    // ******************************* //
    //      Poses in the scenario      //
    // ******************************* //
    if(scenarioID == 2 || scenarioID == 4 || scenarioID == 6) // Drinking Service scenarios (ARoS, Sawyer) and Toy Vehicle scenario (Sawyer with electric gripper)
    {
        if(scenarioID == 2 || scenarioID == 4)
        {
            // pose_id = 0
            poses_prefix.push_back("BottleJuice_pose1");
            poses_rel.push_back(true);
            poses_obj_id.push_back(2);
            // pose_id = 1
            poses_prefix.push_back("BottleJuice_pose2");
            poses_rel.push_back(true);
            poses_obj_id.push_back(2);
            // pose_id = 2
            poses_prefix.push_back("BottleJuice_pose3");
            poses_rel.push_back(true);
            poses_obj_id.push_back(2);
        }
        else if(scenarioID == 6)
        {
            // pose_id = 0
            poses_prefix.push_back("BlueColumn_EngPose");
            poses_rel.push_back(true);
            poses_obj_id.push_back(0);
            // pose_id = 1
            poses_prefix.push_back("GreenColumn_EngPose");
            poses_rel.push_back(true);
            poses_obj_id.push_back(1);
            // pose_id = 2
            poses_prefix.push_back("RedColumn_EngPose");
            poses_rel.push_back(true);
            poses_obj_id.push_back(2);
            // pose_id = 3
            poses_prefix.push_back("MagentaColumn_EngPose");
            poses_rel.push_back(true);
            poses_obj_id.push_back(3);
            // pose_id = 4
            poses_prefix.push_back("GreenColumn_Pose1");
            poses_rel.push_back(true);
            poses_obj_id.push_back(1);
        }

        while(cnt_pose < n_poses)
        {
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if(srvs.response.result == 1)
                pose_info_str = srvs.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the pose");
                succ = false;
            }

            if(succ)
            {
                floatCount = pose_info_str.size() / sizeof(float);

                if(!pose_info_vec.empty())
                    pose_info_vec.clear();
                for(int k = 0; k < floatCount; ++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0) * 1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1) * 1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2) * 1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3) * static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4) * static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5) * static_cast<double>(M_PI)/180;//[rad]

                //set the new pose
                Pose* ps = new Pose(signPrefix, pose_pos, pose_or, poses_rel[cnt_pose], poses_obj_id[cnt_pose]);
                //emit a signal newpose in the secnario
                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }
            else
                throw string("Error while retrieving the poses of the scenario");
        }
    }
    
    // ******************************* //
    //              Robot              //
    // ******************************* //
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    if(scenarioID == 1 || scenarioID == 2)
        srvs.request.signalName = string("HumanoidName");
    else if(scenarioID >= 3)
        srvs.request.signalName = string("RobotName");
    add_client.call(srvs);
    if(srvs.response.result == 1)
        Hname = srvs.response.signalValue;
    else
    {
        throw string("Error: Couldn't get the name of the robot");
        succ = false;
    }


    // ******************************* //
    //            Robot Arm            //
    // ******************************* //
    // get the handles of both arms
    if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
        succ = getArmsHandles(0);
    else if(scenarioID >= 3) // Sawyer scenarios (Toy Vehicle, Drinking Service)
        //succ = getArmsHandles(2);
        succ = getArmsHandles(3); // robot=3 just to test the UR scenario because does not have gripper for now

    // transformation matrix for the arm
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

    // ******************************* //
    //            Right Arm            //
    // ******************************* //
    //get the transformation matrix of the arm
    srvs.request.signalName = string("mat_right_arm");
    add_client.call(srvs);
    if(srvs.response.result == 1)
        mat_right_arm_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the transformation matrix of the arms");
    }

    if(!mat_right_arm_vec.empty())
        mat_right_arm_vec.clear();
    floatCount = mat_right_arm_str.size()/sizeof(float);
    for(int k = 0; k < floatCount; ++k)
        mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));

    // ******************************* //
    //             Left Arm            //
    // ******************************* //
    if(scenarioID == 1 || scenarioID == 2) // ARoS
    {
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if(srvs.response.result == 1)
            mat_left_arm_str = srvs.response.signalValue;
        else
        {
            succ = false;
            throw string("Error: Couldn't get the transformation matrix of the arms");
        }

        if(!mat_left_arm_vec.empty())
            mat_left_arm_vec.clear();
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for(int k = 0; k < floatCount; ++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
    }

    rows = 0;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 4; ++j)
        {
            if(i == 3 && j < 3)
            {
                mat_right(i,j) = 0;
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = 0;
            }
            else if(i == 3 && j == 3)
            {
                mat_right(i,j) = 1;
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = 1;
            }
            else if(i < 3 && j == 3)
            {
                mat_right(i,j) = mat_right_arm_vec.at(j + rows * 4) * 1000; //[mm]
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = mat_left_arm_vec.at(j + rows * 4) * 1000; //[mm]
            }
            else
            {
                mat_right(i,j) = mat_right_arm_vec.at(j + rows * 4);
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = mat_left_arm_vec.at(j + rows * 4);
            }
        }

        ++rows;
    }


    // ******************************* //
    //          DH Parameters          //
    // ******************************* //
    //get Denavit Hartenberg parameters of the arm
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("DH_params_arm");
    add_client.call(srvs);
    if(srvs.response.result == 1)
        DH_params_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the DH parameters of the arm");
    }

    floatCount = DH_params_str.size()/sizeof(float);
    if(!DH_params_vec.empty())
    {
        DH_params_vec.clear();
        theta_offset.clear();
    }
    //convert the received DH parameters - string-  to double/flout
    for(int k = 0; k < floatCount; ++k)
        DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

    //define the size of the vectors of DH parameters
    //robot_arm_specs.arm_specs.alpha = std::vector<double>(JOINTS_ARM);
    //robot_arm_specs.arm_specs.a = std::vector<double>(JOINTS_ARM);
    //robot_arm_specs.arm_specs.d = std::vector<double>(JOINTS_ARM);
    //robot_arm_specs.arm_specs.theta = std::vector<double>(JOINTS_ARM);
    //UR has more DH frames than joints - has 8
    int DH_frames=floatCount/4;
    robot_arm_specs.arm_specs.alpha = std::vector<double>(DH_frames);
    robot_arm_specs.arm_specs.a = std::vector<double>(DH_frames);
    robot_arm_specs.arm_specs.d = std::vector<double>(DH_frames);
    robot_arm_specs.arm_specs.theta = std::vector<double>(DH_frames);

   // for(int i = 0; i < JOINTS_ARM; ++i)
    for(int i = 0; i < DH_frames; ++i)
    {
        robot_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i) * static_cast<double>(M_PI) / 180; // [rad]
        robot_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i + DH_frames) * 1000; // [mm]
        robot_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i + DH_frames*2) * 1000; // [mm]
        theta_offset.push_back(DH_params_vec.at(i + DH_frames*3) * static_cast<double>(M_PI) / 180); // [rad]
    }


#if HAND == 0
    // ******************************* //
    //           Barrett Hand          //
    // ******************************* //
    // **** Max aperture **** //
    if(scenarioID!=7){ // HAND =0 and UR scene
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("maxAperture_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            maxAp = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the maximum aperture");
            succ = false;
        }

        // **** Aw **** //
        srvf.request.signalName = string("Aw_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            Aw = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the Aw");
            succ = false;
        }

        // **** A1 **** //
        srvf.request.signalName = string("A1_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            A1 = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the A1");
            succ = false;
        }

        // **** A2 **** //
        srvf.request.signalName = string("A2_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            A2 = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the A2");
            succ = false;
        }

        // **** A3 **** //
        srvf.request.signalName = string("A3_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            A3 = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the A3");
            succ = false;
        }

        // **** D3 **** //
        srvf.request.signalName = string("D3_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            D3 = srvf.response.signalValue * 1000;
        else
        {
            throw string("Error: Couldn't get the information of the D3");
            succ = false;
        }

        // **** Phi2 **** //
        srvf.request.signalName = string("phi2_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            phi2 = srvf.response.signalValue;
        else
        {
            throw string("Error: Couldn't get the information of the phi2");
            succ = false;
        }

        // **** Phi3 **** //
        srvf.request.signalName = string("phi3_info");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            phi3 = srvf.response.signalValue;
        else
        {
            throw string("Error: Couldn't get the information of the phi3");
            succ = false;
        }
    }

#elif HAND == 1

    // ******************************* //
    //         Electric Gripper        //
    // ******************************* //
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    // **** A1 **** //
    srvf.request.signalName = string("A1_info");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        A1 = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the A1");
        succ = false;
    }

    // **** D3 **** //
    srvf.request.signalName = string("D3_info");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        D3 = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the D3");
        succ = false;
    }

    // **** Max Aperture **** //
    srvf.request.signalName = string("maxAperture_info");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        maxAp = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the maximum aperture");
        succ = false;
    }

    // **** Min Aperture **** //
    srvf.request.signalName = string("minAperture_info");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        minAp = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the minimum aperture");
        succ = false;
    }
#elif HAND == 2
    // **** Vacuum gripper dimension **** //
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    srvf.request.signalName = string("vacuum_length");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        D7 = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the vacuum gripper dimension");
        succ = false;
    }

#endif // endif of Hand 1

    // ******************************* //
    //               Head              //
    // ******************************* //
// UR DOES NOT HAVE HEAD
#if UR == 0
#if HEAD == 1
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("HeadInfo");
    add_client.call(srvs);
    if(srvs.response.result == 1)
        head_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the information of the head");
    }

    floatCount = head_str.size()/sizeof(float);
    if(!head_vec.empty())
        head_vec.clear();
    for(int k = 0; k < floatCount; ++k)
        head_vec.push_back(static_cast<double>(((float*)head_str.c_str())[k]));

    //position of the head
    head.Xpos = head_vec.at(0) * 1000;//[mm]
    head.Ypos = head_vec.at(1) * 1000;//[mm]
    head.Zpos = head_vec.at(2) * 1000;//[mm]
    //orientation of the head
    head.Roll = head_vec.at(3) * static_cast<double>(M_PI) / 180; //[rad]
    head.Pitch = head_vec.at(4) * static_cast<double>(M_PI) / 180; //[rad]
    head.Yaw = head_vec.at(5) * static_cast<double>(M_PI) / 180; //[rad]
    //size of the head
    head.Xsize = head_vec.at(6) * 1000;//[mm]
    head.Ysize = head_vec.at(7) * 1000;//[mm]
    head.Zsize = head_vec.at(8) * 1000;//[mm]
#endif // endif HEAD
#endif


    // ******************************* //
    //               Torso             //
    // ******************************* //
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("TorsoInfo");
    add_client.call(srvs);
    if(srvs.response.result == 1)
        torso_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the information of the torso");
    }

    floatCount = torso_str.size()/sizeof(float);
    if(!torso_vec.empty())
        torso_vec.clear();
    for(int k = 0; k < floatCount; ++k)
        torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

#if UR == 1   // UR 6 DOFs scenario
    //position of the torso
    torso_UR.Xpos = torso_vec.at(0) * 1000;//[mm]
    torso_UR.Ypos = torso_vec.at(1) * 1000;//[mm]
    torso_UR.Zpos = torso_vec.at(2) * 1000;//[mm]
    //orientation of the torso
    torso_UR.q_X = torso_vec.at(3); //quaternion X scalar
    torso_UR.q_Y = torso_vec.at(4); //quaternion Y scalar
    torso_UR.q_Z = torso_vec.at(5); //quaternion Z scalar
    torso_UR.q_Z = torso_vec.at(6); // quaternion W real
    //size of the torso
    torso_UR.Xsize = torso_vec.at(7) * 1000;//[mm]
    torso_UR.Ysize = torso_vec.at(8) * 1000;//[mm]
    torso_UR.Zsize = torso_vec.at(9) * 1000;//[mm]

#elif UR == 0 // SUBSTITUIR POR WP DEPOIS
    //position of the torso
    torso.Xpos = torso_vec.at(0) * 1000;//[mm]
    torso.Ypos = torso_vec.at(1) * 1000;//[mm]
    torso.Zpos = torso_vec.at(2) * 1000;//[mm]
    //orientation of the torso
    torso.Roll = torso_vec.at(3) * static_cast<double>(M_PI) / 180; //[rad]
    torso.Pitch = torso_vec.at(4) * static_cast<double>(M_PI) / 180; //[rad]
    torso.Yaw = torso_vec.at(5) * static_cast<double>(M_PI) / 180; //[rad]
    //size of the torso
    torso.Xsize = torso_vec.at(6) * 1000;//[mm]
    torso.Ysize = torso_vec.at(7) * 1000;//[mm]
    torso.Zsize = torso_vec.at(8) * 1000;//[mm]
#endif


    // ******************************* //
    //        Joints information       //
    // ******************************* //
    // **** Right arm **** //
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    // **** [Right arm] Home posture **** //
    for(size_t i = 0; i < rposture.size(); ++i)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
        add_client.call(srvf);
        if(srvf.response.result == 1)
        {
            rposture.at(i) = srvf.response.signalValue;
        }
        else
        {
            throw string("Error: Couldn't get the information of the right home posture");
            succ = false;
        }
    }

    // **** [Right arm] Minimum limits **** //
    for(size_t i = 0; i < min_rlimits.size(); ++i)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
        add_client.call(srvf);
        if(srvf.response.result == 1)
        {
            min_rlimits.at(i) = srvf.response.signalValue;
        }
        else
        {
            throw string("Error: Couldn't get the information of the minimum right limits");
            succ = false;
        }
    }

    // **** [Right arm] Maximum limits **** //
    for(size_t i = 0; i < max_rlimits.size(); ++i)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
        add_client.call(srvf);
        if(srvf.response.result == 1)
        {
            max_rlimits.at(i) = srvf.response.signalValue;
        }
        else
        {
            throw string("Error: Couldn't get the information of the maximum right limits");
            succ = false;
        }
    }


    if(scenarioID == 1 || scenarioID == 2) // ARoS and Avatar scenarios (Toy Vehicle and Drinking Service)
    {
        // **** Left arm **** //
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        // **** [Right arm] Home posture **** //
        for(size_t i = 0; i < lposture.size(); ++i)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if(srvf.response.result == 1)
                lposture.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the left home posture");
                succ = false;
            }
        }

        // **** [Right arm] Minimum limits **** //
        for(size_t i = 0; i < min_llimits.size(); ++i)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if(srvf.response.result == 1)
                min_llimits.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the minimum left limits");
                succ = false;
            }
        }

        // **** [Right arm] Maximum limits **** //
        for(size_t i = 0; i < max_llimits.size(); ++i)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if(srvf.response.result == 1)
                max_llimits.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the maximum left limits");
                succ = false;
            }
        }
    }

#if WP==1

    // ******************************* //
    //            waypoints            //
    // ******************************* //

//     in this scene I have 5 possible trajectories:
//         pick
//         show
//         faulty box
//         good box
//     so I need to get the waypoints for these 4 trajectories
#if UR==0

    //get the number of different trajectories with waypoints
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    srvf.request.signalName = string("number_traj");
    add_client.call(srvf);
    if(srvf.response.result == 1)
        traj_nr = srvf.response.signalValue;
    else
    {
           succ = false;
           throw string("Error: Couldn't get the information of the waypoints trajectories quantity");
    }

    for (int traj_count=0; traj_count<traj_nr; traj_count++)
    {
        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("traj_name"+QString::number(traj_count+1).toStdString());
        add_client.call(srvs);
        if(srvs.response.result == 1)
            traj_name = srvs.response.signalValue;
        else
        {
            succ = false;
            throw string("Error: Couldn't get the information of the waypoints trajectorie name");
        }

        //get the number of waypoints to receive
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("number_wps"+QString::number(traj_count+1).toStdString());
        add_client.call(srvf);
        if(srvf.response.result == 1)
            wp_nr = srvf.response.signalValue;
        else
        {
               succ = false;
               throw string("Error: Couldn't get the information of the waypoints quantity");
        }


        //get the waypoints workspace
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        srvf.request.signalName = string("wp_workspace");
        add_client.call(srvf);
        if(srvf.response.result == 1)
            wp_ws = srvf.response.signalValue;
        else
        {
            succ = false;
            throw string("Error: Couldn't get the information of the waypoints workspace");
        }

        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = string("waypoints_val_"+QString::number(traj_count+1).toStdString());
        add_client.call(srvs);
        if(srvs.response.result == 1)
            wp_pos_str = srvs.response.signalValue;
        else
        {
            succ = false;
            throw string("Error: Couldn't get the information of the waypoints position and orientation");
        }


       //unpack waypoints string received from vrep
       floatCount = wp_pos_str.size()/sizeof(float);
       vector<vector<double>> wp_str(wp_nr, vector<double>(floatCount/wp_nr,0));
       int wp_index=0;
       int count=0;
       for (int i=0;i<floatCount;i++){
           wp_str[wp_index][count]=static_cast<double>(((float*)wp_pos_str.c_str())[i])* static_cast<double>(M_PI)/180;//[rad];
           count=count+1;
           if(count==floatCount/wp_nr){
               wp_index=wp_index+1;
               count=0;
           }
       }

       if(!waypoints_vec.empty())
           waypoints_vec.clear();

       for (int i=0;i<wp_nr;i++)
       {
           if(wp_ws==1){//operational space

               // waypoint position of the end_effector
               wp_specs.OperatSpace.position.Xpos = wp_str[i][0] * 1000;//[mm] pos X
               wp_specs.OperatSpace.position.Ypos = wp_str[i][1] * 1000;//[mm] pos Y
               wp_specs.OperatSpace.position.Zpos = wp_str[i][2] * 1000;//[mm] pos Z
               //waypoint orientation in quaternions of the end_effector
               wp_specs.OperatSpace.or_quat.X = wp_str[i][3];//rot in X
               wp_specs.OperatSpace.or_quat.Y = wp_str[i][4];//rot in Y
               wp_specs.OperatSpace.or_quat.Z = wp_str[i][5];//rot in Z
               wp_specs.OperatSpace.or_quat.W = wp_str[i][6];//rot in W

               //get RPY angles from quaternions
               Eigen::Quaterniond q;
               q.x() = wp_specs.OperatSpace.or_quat.X;
               q.y() = wp_specs.OperatSpace.or_quat.Y;
               q.z() = wp_specs.OperatSpace.or_quat.Z;
               q.w() = wp_specs.OperatSpace.or_quat.W;

               Eigen::Matrix3d Rot = q.normalized().toRotationMatrix();
               // get rpy from rotation matrix
               std::vector<double> rpy;
               if(this->RotgetRPY(Rot,rpy))
               {
                   wp_specs.OperatSpace.or_rpy.roll =  rpy.at(0);//rot in X
                   wp_specs.OperatSpace.or_rpy.pitch = rpy.at(1) ;//rot in Y
                   wp_specs.OperatSpace.or_rpy.yaw = rpy.at(2);//rot in Z

               }


           }else if(wp_ws==0){
              //joint space
              wp_specs.JointSpace.PosJoints = wp_str[i];

           }

           //waypoint name
           wp_specs.name = string(traj_name+QString::number(i+1).toStdString());

           //waypoints in an vector of waypoint structure
           waypoints_vec.push_back(wp_specs);
        }

       //set the waypoints
       //I received the initial and final points as waypoints. However, henceforth they will not count as waypoint
       // therefore, wp_nr - 2
       Waypoint *wp = new Waypoint(wp_nr-2,waypoints_vec,wp_ws,traj_name);
       // display info of the waypoints
       if(wp_ws==1){
           for (int i=0; i<wp_nr;i++){
               infoLine = wp->getInfoLine_OP(wp->get_waypoint(i));
               Q_EMIT newElement(infoLine);
           }
       }else{
           for (int i=0; i<wp_nr;i++){
               infoLine = wp->getInfoLine_Joint(wp->get_waypoint(i));
               Q_EMIT newElement(infoLine);
           }
       }
       //add the waypoints to the scene
       scene->addWaypoint(waypointPtr(wp));
       //add the waypoint to the combobox
       Q_EMIT newWaypoint(traj_name);
    }
 }
#elif UR==1
    wp_nr = this->robot_waypoints.size();
    wp_ws = false; //wp in joint space

    for(int i=0; i<robot_waypoints.size();i++)
    {
        wp_specs.JointSpace.PosJoints = this->robot_waypoints.at(i);
        wp_specs.name = string("wp"+QString::number(i+1).toStdString());
        //waypoints in an vector of waypoint structure
        waypoints_vec.push_back(wp_specs);
    }
        traj_name = "wp_traj";
        //set the waypoints
        //I received the initial and final points as waypoints. However, henceforth they will not count as waypoint
        // therefore, wp_nr - 2
        Waypoint *wp = new Waypoint(wp_nr-2,waypoints_vec,wp_ws,traj_name);
        // display info of the waypoints
        if(wp_ws==1){
            for (int i=0; i<wp_nr;i++){
                infoLine = wp->getInfoLine_OP(wp->get_waypoint(i));
                Q_EMIT newElement(infoLine);
            }
        }else{
            for (int i=0; i<wp_nr;i++){
                infoLine = wp->getInfoLine_Joint(wp->get_waypoint(i));
                Q_EMIT newElement(infoLine);
            }
        }
        //add the waypoints to the scene
        scene->addWaypoint(waypointPtr(wp));
        //add the waypoint to the combobox
        Q_EMIT newWaypoint(traj_name);

#endif
#endif

    // ******************************* //
    //           Create robot          //
    // ******************************* //
    if(succ)
    {
/*
#if UR == 1 // if UR -> 6 DOFs robot

        // **** Torso info **** //
        robot_UR_torso_specs.Xpos = torso_UR.Xpos;
        robot_UR_torso_specs.Ypos = torso_UR.Ypos;
        robot_UR_torso_specs.Zpos = torso_UR.Zpos;
        robot_UR_torso_specs.q_X = torso_UR.q_X;
        robot_UR_torso_specs.q_Y = torso_UR.q_Y;
        robot_UR_torso_specs.q_Z = torso_UR.q_Z;
        robot_UR_torso_specs.q_W = torso_UR.q_W;
        robot_UR_torso_specs.Xsize = torso_UR.Xsize;
        robot_UR_torso_specs.Ysize = torso_UR.Ysize;
        robot_UR_torso_specs.Zsize = torso_UR.Zsize;
#elif UR == 0 // 7 DOFs robots
*/
        // **** Torso info **** //
        robot_torso_specs.Xpos = torso.Xpos;
        robot_torso_specs.Ypos = torso.Ypos;
        robot_torso_specs.Zpos = torso.Zpos;
        robot_torso_specs.Roll = torso.Roll;
        robot_torso_specs.Pitch = torso.Pitch;
        robot_torso_specs.Yaw = torso.Yaw;
        robot_torso_specs.Xsize = torso.Xsize;
        robot_torso_specs.Ysize = torso.Ysize;
        robot_torso_specs.Zsize = torso.Zsize;
//#endif

#if HEAD == 1  // UR robot does not have head
        // **** Head info **** //
        robot_head_specs.Xpos = head.Xpos;
        robot_head_specs.Ypos = head.Ypos;
        robot_head_specs.Zpos = head.Zpos;
        robot_head_specs.Roll =  head.Roll;
        robot_head_specs.Pitch = head.Pitch;
        robot_head_specs.Yaw = head.Yaw;
        robot_head_specs.Xsize = head.Xsize;
        robot_head_specs.Ysize = head.Ysize;
        robot_head_specs.Zsize = head.Zsize;
#endif

#if HAND == 0
        // **** Barrett Hand info **** //
        robot_hand_specs.maxAperture = maxAp;
        robot_hand_specs.Aw = Aw;
        robot_hand_specs.A1 = A1;
        robot_hand_specs.A2 = A2;
        robot_hand_specs.A3 = A3;
        robot_hand_specs.D3 = D3 ;
        robot_hand_specs.phi2 = phi2;
        robot_hand_specs.phi3 = phi3;

        // add the joints offset
        std::transform(rposture.begin(), rposture.end(), theta_offset.begin(), rposture.begin(), std::plus<double>());
        if(scenarioID == 1 || scenarioID == 2) // ARoS
            std::transform(lposture.begin(), lposture.end(), theta_offset.begin(), lposture.begin(), std::plus<double>());
        else if(scenarioID >= 3) // Sawyer scenarios
        {
            lposture = rposture;
            min_llimits = min_rlimits;
            max_llimits = max_rlimits;
        }

#if HEAD == 1
        Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_hand_specs,
                                robot_head_specs, rposture, lposture,
                                min_rlimits, max_rlimits,
                                min_llimits, max_llimits);

#else
        Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_hand_specs,
                                rposture, lposture,
                                min_rlimits, max_rlimits,
                                min_llimits, max_llimits);

#endif

        // **** Transformation matrices **** //
        rptr->setMatRight(mat_right);
        if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios
            rptr->setMatLeft(mat_left);
        else if(scenarioID >= 3) // Sawyer scenarios
            rptr->setMatLeft(mat_right);

        // **** Right joints **** //
        std::vector<double> rightp;
        rptr->getRightPosture(rightp);
        std::transform(rightp.begin(), rightp.end(),theta_offset.begin(), rightp.begin(), std::minus<double>());

        std::vector<string> rj = std::vector<string>(rightp.size());
        for(size_t i = 0; i < rightp.size(); ++i)
        {
            rj.at(i) = string("right_joint "+ QString::number(i + 1).toStdString()+ ": "+
                              QString::number(rightp.at(i) * 180 / static_cast<double>(M_PI)).toStdString() + " [deg]");
            Q_EMIT newJoint(rj.at(i));
        }

        // **** Left joints **** //
        if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios
        {
            std::vector<double> leftp;
            rptr->getLeftPosture(leftp);
            std::transform(leftp.begin(), leftp.end(), theta_offset.begin(), leftp.begin(), std::minus<double>());

            std::vector<string> lj = std::vector<string>(leftp.size());
            for(size_t i = 0; i < leftp.size(); ++i)
            {
                lj.at(i) = string("left_joint "+ QString::number(i + 1).toStdString()+ ": "+
                                  QString::number(leftp.at(i) * 180 / static_cast<double>(M_PI)).toStdString() + " [deg]");
                Q_EMIT newJoint(lj.at(i));
            }
        }
        else if(scenarioID >= 3) // Sawyer scenarios
            rptr->getLeftPosture(rightp);

        // **** Create robot **** //
        // display info of the robot
        infoLine = rptr->getInfoLine();
        Q_EMIT newElement(infoLine);
        scene->addRobot(robotPtr(rptr));


#elif HAND ==1


        // **** Electric gripper info **** //
        robot_gripper_specs.maxAperture = maxAp;
        robot_gripper_specs.minAperture = minAp;
        robot_gripper_specs.A1 = A1;
        robot_gripper_specs.D3 = D3;


        // add the joints offset
        std::transform(rposture.begin(), rposture.end(), theta_offset.begin(), rposture.begin(), std::plus<double>());
        lposture = rposture;
        min_llimits = min_rlimits;
        max_llimits = max_rlimits;

#if HEAD == 1
        Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_gripper_specs,
                                robot_head_specs, rposture, lposture,
                                min_rlimits, max_rlimits,
                                min_llimits, max_llimits);
#elif HEAD == 0
        Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_gripper_specs,
                                rposture, lposture,
                                min_rlimits, max_rlimits,
                                min_llimits, max_llimits);
#endif // endif HEAD=1

        // **** Transformation matrices **** //
        rptr->setMatRight(mat_right);
        rptr->setMatLeft(mat_right);

        // **** Right joints **** //
        std::vector<double> rightp;
        rptr->getRightPosture(rightp);
        // subtract theta_offset from the range rightp.begin to rightp.end and save it from rightp.begin
        std::transform(rightp.begin(), rightp.end(),theta_offset.begin(), rightp.begin(), std::minus<double>());

        std::vector<string> rj = std::vector<string>(rightp.size());
        for(size_t i = 0; i < rightp.size(); ++i)
        {
            if(i < rightp.size()) // size of rightp is (JOINTS_ARM+JOINTS_HAND)

                rj.at(i) = string("right_joint "+ QString::number(i + 1).toStdString()+ ": "+
                                  QString::number(rightp.at(i) * 180 / static_cast<double>(M_PI)).toStdString() + " [deg]");
            else
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                  QString::number(rightp.at(i)).toStdString() + " [mm]");

            Q_EMIT newJoint(rj.at(i));
        }

        rptr->getLeftPosture(rightp);

        // **** Create robot **** //
        // display info of the robot
        infoLine = rptr->getInfoLine();
        Q_EMIT newElement(infoLine);
        scene->addRobot(robotPtr(rptr));

#elif HAND == 2 // vacuum gripper

        robot_vacuum_gripper.D7 = D7;
        // add the joints offset
        std::transform(rposture.begin(), rposture.end(), theta_offset.begin(), rposture.begin(), std::plus<double>());
        lposture = rposture;
        min_llimits = min_rlimits;
        max_llimits = max_rlimits;

        #if HEAD == 1
                Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_vacuum_gripper,
                                        robot_head_specs, rposture, lposture,
                                        min_rlimits, max_rlimits,
                                        min_llimits, max_llimits);
        #elif HEAD == 0
                Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_vacuum_gripper,
                                        rposture, lposture,
                                        min_rlimits, max_rlimits,
                                        min_llimits, max_llimits);
        #endif // endif HEAD=1

                // **** Transformation matrices **** //
                rptr->setMatRight(mat_right);
                rptr->setMatLeft(mat_right);

                // **** Right joints **** //
                std::vector<double> rightp;
                rptr->getRightPosture(rightp);
                // subtract theta_offset from the range rightp.begin to rightp.end and save it from rightp.begin
                std::transform(rightp.begin(), rightp.end(),theta_offset.begin(), rightp.begin(), std::minus<double>());

                std::vector<string> rj = std::vector<string>(rightp.size());
                for(size_t i = 0; i < rightp.size(); ++i)
                {
                    if(i < rightp.size()) // size of rightp is (JOINTS_ARM+JOINTS_HAND)

                        rj.at(i) = string("right_joint "+ QString::number(i + 1).toStdString()+ ": "+
                                          QString::number(rightp.at(i) * 180 / static_cast<double>(M_PI)).toStdString() + " [deg]");
                    else
                        rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                          QString::number(rightp.at(i)).toStdString() + " [mm]");

                    Q_EMIT newJoint(rj.at(i));
                }

                rptr->getLeftPosture(rightp);

                // **** Create robot **** //
                // display info of the robot
                infoLine = rptr->getInfoLine();
                Q_EMIT newElement(infoLine);
                scene->addRobot(robotPtr(rptr));

#endif // Endif Hand =1
    }
    else
        throw string("Error while retrieving elements from the scenario");

    this->curr_scene = scene;

    // stop the simulation
    add_client = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvcc;
    add_client.call(srvcc);

    // we got all the elements of the scenario
    got_scene = true;


    return succ;
}


bool QNode::setWaypoint(vector<double> &waypoints)
{
    ros::NodeHandle n;
    vector <double> offset_robot {0,-M_PI_2,0,-M_PI_2,0,0};
    vector <double> robot_wp_offset =  vector <double> (JOINTS_ARM); // size of the number of dof

    string topic = "/joint_states";
    subJointsStateRobotUR = n.subscribe(topic, 1, &QNode::URJointsCallback, this);
    sleep(2);
    ros::spinOnce();

    robot_wp_offset = robotPosture_wp;
    std::transform(robot_wp_offset.begin(), robot_wp_offset.end(),offset_robot.begin(), robot_wp_offset.begin(), std::minus<double>());

    waypoints = robot_wp_offset;
    this->robot_waypoints.push_back(waypoints);

    return true;
}

void QNode::updateWaypoints(vector<vector<double>> robot_wps)
{
    this->robot_waypoints.clear();
    this->robot_waypoints = robot_wps;
}



#if HAND == 0
void QNode::preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int arm, int attach, MatrixXd traj, string objName)
#elif HAND == 1
void QNode::preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int attach, string objName)
#elif HAND == 2
void QNode::preMovementOperation(ros::NodeHandle node, int movType, bool retreat, int attach, string objName)
#endif
{
    if(movType == 0) // Reach-to-grasp
    {
        if(retreat && obj_in_hand)
        {
            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
            vrep_common::simRosSetObjectParent srvset_parent;
            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
            srvset_parent.request.parentHandle = attach;
            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
            add_client.call(srvset_parent);
            if(srvset_parent.response.result != 1)
                log(QNode::Error,string("Error in grasping the object "));

#if HAND == 0
            VectorXd init_pos = traj.block<1, JOINTS_HAND>(0, JOINTS_ARM);
            std::vector<double> handPos; handPos.resize(init_pos.size());
            VectorXd::Map(&handPos[0], init_pos.size()) = init_pos;
            this->closeBarrettHand_to_pos(arm, handPos);
#elif HAND == 1
            closed = true;
#endif
        }
    }
    else if(movType == 2 || movType == 3) // Transport, Engage
    {
        if(retreat)
        {
            if(std::strcmp(objName.c_str(), "") != 0)
            {
                add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                vrep_common::simRosSetObjectParent srvset_parent;
                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                srvset_parent.request.parentHandle = -1;
                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                add_client.call(srvset_parent);
                if(srvset_parent.response.result != 1)
                    log(QNode::Error,string("Error in releasing the object "));
            }

#if HAND == 0
            closed.at(0) = false;
            closed.at(1) = false;
            closed.at(2) = false;
#elif HAND == 1
            closed = false;
#endif
        }
    }
}


void QNode::posMovementOperation(ros::NodeHandle node, int movType, bool plan, int attach)
{
    if((movType == 0) && plan && obj_in_hand)
    {
        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
        srvset_parent.request.parentHandle = attach;
        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
        add_client.call(srvset_parent);

        if(srvset_parent.response.result != 1)
            log(QNode::Error,string("Error in grasping the object "));
    }
}


#if HAND == 0
void QNode::publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, MatrixXi hand_handles, int sceneID, double timeTot, double tol)
#elif HAND == 1
void QNode::publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, int sceneID, double timeTot, double tol)
#elif HAND == 2
void QNode::publishData(ros::NodeHandle node, MatrixXd traj, std::vector<double> timesteps, std::vector<int> handles, int sceneID, double timeTot, double tol)
#endif
{
    // **** Publishers **** //
    // publish in the topic /motion_manager/set_joints
    ros::Publisher pub = node.advertise<vrep_common::JointSetStateData>("/" + nodeName + "/set_joints", 1);
#if HAND == 0
    ros::Publisher pubHand = node.advertise<vrep_common::JointSetStateData>("/" + nodeName + "/set_pos_hand", 1);
#endif

    ros::spinOnce();
    // Get the current simulation time
    double t_next = simulationTime - timeTot;
    // Get final posture
    VectorXd fPos = traj.row(traj.rows() - 1);


    for(size_t row = 0; row < traj.rows() - 1 ; ++row)
    {
        bool interval = true;
        bool fPosReached = false;
        double pos_prev;

        // Get the position of the current and next step of the planned movement
        VectorXd pos_curr = traj.row(row);
        VectorXd pos_next = traj.row(row + 1);

        // Get the current timestep
        double t_step = timesteps.at(row);
        // Current step time
        double t_curr = t_next;

        if(t_step < 0.001)
            t_step = MIN_EXEC_TIMESTEP_VALUE;

        // Next step time
        t_next = t_curr + t_step;


        while(ros::ok() && simulationRunning && interval)
        {
            //rostopic message type
            vrep_common::JointSetStateData dataTraj;
#if HAND == 0
            vrep_common::JointSetStateData dataHand;
#endif

            // Current micro step time
            double t_mstep = simulationTime - timeTot;

            if(t_mstep > t_next)
                interval = false;
            else
            {
                // ******************************************* //
                // Diference betweem current and final posture //
                // ******************************************* //
                std::vector<double> rPos;
                double diff = 0.0;

                // Get the current posture of the robotic arm
                this->curr_scene->getRobot()->getRightPosture(rPos);

                for(int i = 0; i < JOINTS_ARM; ++i)
                    diff += pow((fPos(i) - rPos.at(i)), 2);

                if(sqrt(diff) < tol)
                {
                    fPosReached = true;
                    break;
                }
                else
                    fPosReached = false;


                // *************************** //
                // Joints linear interpolation //
                // *************************** //
                double m;
                double pos_mstep;

                for(size_t col = 0; col < traj.cols(); ++col)
                {
                    if(fPosReached)
                        pos_mstep = pos_prev;
                    else
                    {
                        if((t_next - t_curr) == 0)
                            m = 1;
                        else
                            // m is determined by the following formula: (x - x0) / (x1 - x0)
                            m = (t_mstep - t_curr) / (t_next - t_curr);

                        pos_mstep = interpolate(pos_curr(col), pos_next(col), m);
                        pos_prev = pos_mstep;
                    }

#if HAND == 0
                    bool handClosed = (closed[0] && closed[1] && closed[2]);
                    bool isArmJoint = ((col != traj.cols() - 1) && (col != traj.cols() - 2) && (col != traj.cols() - 3) && (col != traj.cols() - 4));
                    bool isHandJoint = ((col == traj.cols() - 1) || (col == traj.cols() - 2) || (col == traj.cols() - 3) || (col == traj.cols() - 4));
#elif HAND == 1
                    bool handClosed = closed;
                    //traj = 0..7 cols - 7 arm joints + 1 hand joint
                    //traj.cols() - 1 = 7
                    bool isArmJoint = (col != traj.cols() - 1);
                    bool isHandJoint = (col == traj.cols() - 1);
#elif HAND == 2
                    bool handClosed = closed;
                    //traj = 0..7 cols - 7 arm joints + 1 hand joint
                    //traj.cols() - 1 = 7
                    bool isArmJoint = (col != traj.cols() - 1);
                    bool isHandJoint = (col == traj.cols() - 1);
#endif

                    // ************************ //
                    // Set data to be published //
                    // ************************ //
                    // **** Joints handles to be changed during the movement  **** //
                    if(isArmJoint || (isHandJoint && !handClosed))
                        dataTraj.handles.data.push_back(handles.at(col));

                    if(sceneID != 0)
                    {
                        // **** Execution mode and joints values **** //
                        if(isArmJoint)
                        {
                            // Position mode
                            dataTraj.setModes.data.push_back(0);
                            // Joints values
                            dataTraj.values.data.push_back(pos_mstep);
                        }
                        else if(isHandJoint && !handClosed)
                        {
                            // Target position mode
                            dataTraj.setModes.data.push_back(1);
                            // Joints values
                            dataTraj.values.data.push_back(pos_mstep);

#if HAND == 0
                            // Hand joints to control: theta8, theta9, theta10
                            if(col >= (traj.cols() - 3))
                            {
                                dataHand.handles.data.push_back(hand_handles(col + 3 - traj.cols(), 2));
                                dataHand.setModes.data.push_back(1);
                                dataHand.values.data.push_back(pos_mstep / 3.0 + 45.0f * static_cast<double>(M_PI) / 180.0f);
                            }
#endif
                        }
                    }
                }

                pub.publish(dataTraj);
#if HAND == 0
                pubHand.publish(dataHand);
#endif
                interval = true;
            }

            ros::spinOnce();
        }

        if(fPosReached)
        {
            log(QNode::Info,string("Final posture reached."));
            break;
        }
    }
}


bool QNode::execMovement(std::vector<MatrixXd> &traj_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string> &traj_descr, movementPtr mov, scenarioPtr scene)
{
    ros::NodeHandle node;
    // **** Scenario **** //
    this->curr_scene = scene;
    this->curr_mov = mov;
    int scenarioID = scene->getID();
    int movType = mov->getType();
    // **** Arm code **** //
    int armCode = mov->getArm();
    // **** Movement settings **** //
    bool plan;
    bool retreat;
    double timeTot = 0.0;
    // **** Handles **** //
    int h_attach;
    std::vector<int> handles;
#if HAND == 0
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1); // matrix fingers x (phalanges + 1) with all elements set as 1
#endif


    // ******************************* //
    //           Subscribers           //
    // ******************************* //
    // set joints position or velocity
    ros::ServiceClient client_enableSubscriber = node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName = "/" + nodeName + "/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize = 1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd = simros_strmcmd_set_joint_state; // the subscriber type

#if HAND == 0
    // set the target postion of the 2nd phalanx of the fingers
    ros::ServiceClient client_enableSubscriber_hand = node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName = "/" + nodeName + "/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize = 1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd = simros_strmcmd_set_joint_state; // the subscriber type
#endif

    // ******************************* //
    //              Handles            //
    // ******************************* //
    switch (armCode)
    {
    case 0: // dual arm
        break;
    case 1: //right arm
        handles = right_handles;
#if HAND == 0
        hand_handles = right_hand_handles;
#endif
#if HAND != 2
        h_attach = right_attach;
#endif
        break;
    case 2: // left arm
        handles = left_handles;
#if HAND == 0
        hand_handles = left_hand_handles;
#endif
        h_attach = left_attach;
        break;
    }

    // ******************************* //
    //        Hand open or close       //
    // ******************************* //
    switch (movType)
    {
    case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
#if HAND == 0
        closed.at(0) = false;
        closed.at(1) = false;
        closed.at(2) = false;
#elif HAND == 1
        closed = false;
#endif
        break;
    case 2: case 3: case 4: // transport, engage, disengage
#if HAND == 0
        closed.at(0) = true;
        closed.at(1) = true;
        closed.at(2) = true;
#elif HAND == 1
        closed = true;
#endif
    case 6: // waypoints 
        break;
    }


    // ******************************* //
    //         Start Simulation        //
    // ******************************* //
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);

    ros::spinOnce();


    // ******************************* //
    //         Execute movement        //
    // ******************************* //
    std::vector<MatrixXd> traj_mov_planned = traj_mov;
#if UR == 0
    std::vector<MatrixXd> traj_mov_robot = this->robotJointPositions(traj_mov_planned);
#elif UR == 1
    std::vector<MatrixXd> traj_mov_robot =traj_mov_planned;
#endif
    // **** Join plan and approach stage **** //
    this->joinStages(traj_mov_robot, timesteps, traj_descr);


    for(size_t k = 0; k < traj_mov_robot.size(); ++k)
    {
        string mov_descr = traj_descr.at(k);

        if(strcmp(mov_descr.c_str(), "plan") == 0)
        {
            // Plan or plan + approach stage
            plan = true;
            retreat = false;
        }
        else
        {
            // Retreat stage
            plan = false;
            retreat = true;
        }

        // **** Check if the object is in the robot's hand and if it has been grasped correctly **** //
        string obj_name = mov->getObject()->getName();
#if HAND == 0
        MatrixXd tt = traj_mov_robot.at(k);
        this->preMovementOperation(node, movType, retreat, armCode, h_attach, tt, obj_name);
#elif HAND == 1
        this->preMovementOperation(node, movType, retreat, h_attach, obj_name);
#elif HAND == 2
        
#endif

        // Trajectory to be performed and timesteps
        MatrixXd traj_stage = traj_mov_robot.at(k);
        std::vector<double> timesteps_stage = timesteps.at(k);
        // Tolerances
        double tol_stop_stage = tols_stop.at(k);


#if HAND == 0
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1) &&
                (client_enableSubscriber_hand.call(srv_enableSubscriber_hand)) && (srv_enableSubscriber_hand.response.subscriberID != -1))
#elif HAND == 1
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
#elif HAND == 2
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
#endif
        {
            // **** Publish the data in the V-REP topics **** //
#if HAND == 0
            this->publishData(node, traj_stage, timesteps_stage, handles, hand_handles, scenarioID, timeTot, tol_stop_stage);
#elif HAND == 1
            this->publishData(node, traj_stage, timesteps_stage, handles, scenarioID, timeTot, tol_stop_stage);
#elif HAND == 2
            this->publishData(node, traj_stage, timesteps_stage, handles, scenarioID, timeTot, tol_stop_stage);

#endif
            // **** Post-movement operations **** //
            this->posMovementOperation(node, movType, plan, h_attach);
        }

        ros::spinOnce();
        // Total time of the movement
        timeTot = simulationTime;
    }


    // ******************************* //
    //         Pause simulation        //
    // ******************************* //
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);

    ros::spinOnce();
    log(QNode::Info, string("Movement completed"));
    mov->setExecuted(true);
    TotalTime = simulationTime;

    return true;
}


bool QNode::execTask(vector<vector<MatrixXd>> &traj_task, vector<vector<vector<double>>> &timesteps_task, vector<vector<double>> &tols_stop_task, vector<vector<string>> &traj_descr_task, taskPtr task, scenarioPtr scene)
{
    ros::NodeHandle node;
    // **** Scenario **** //
    this->curr_scene = scene;
    int scenarioID = scene->getID();
    vector<movementPtr> movTask;
    vector<int> nProb;
    // **** Movement settings **** //
    bool plan;
    bool retreat;
    double timeTot = 0.0;
    vector<vector<MatrixXd>> traj;
    vector<vector<vector<double>>> timesteps;
    vector<string> task_descr;
    // **** Handles **** //
    int h_attach;
    std::vector<int> handles;
#if HAND == 0
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1); // matrix fingers x (phalanges + 1) with all elements set as 1
#endif


    // ******************************* //
    //           Subscribers           //
    // ******************************* //

    //tell V-REP to subscribe to set_joints topic
    // set joints position or velocity
    ros::ServiceClient client_enableSubscriber = node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName = "/" + nodeName + "/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize = 1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd = simros_strmcmd_set_joint_state; // the subscriber type

#if HAND == 0
    // set the target postion of the 2nd phalanx of the fingers
    ros::ServiceClient client_enableSubscriber_hand = node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName = "/" + nodeName + "/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize = 1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd = simros_strmcmd_set_joint_state; // the subscriber type
#endif


    // ******************************* //
    //         Start Simulation        //
    // ******************************* //
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);

    ros::spinOnce();


    // ******************************* //
    //          Join Movements         //
    // ******************************* //
    // Problems that don't belong to this task
    int nUnsolProb = 0;

    for(int k = 0; k < task->getProblemNumber(); ++k)
    {
        if(task->getProblem(k)->getPartOfTask() && task->getProblem(k)->getSolved())
        {
            // Number of the problem in the task to be performed
            int kk = k - nUnsolProb;
            nProb.push_back(k);

            movementPtr mov = task->getProblem(kk)->getMovement();
            movTask.push_back(mov);

            vector<MatrixXd> traj_mov_planned = traj_task.at(kk);
#if UR ==0
            vector<MatrixXd> traj_mov_robot = this->robotJointPositions(traj_mov_planned);
#elif UR == 1
            vector<MatrixXd> traj_mov_robot = traj_mov_planned;
#endif
            vector<vector<double>> timesteps_mov = timesteps_task.at(kk);
            vector<string> traj_descr_mov = traj_descr_task.at(kk);

            // **** Join plan and approach stage **** //
            this->joinStages(traj_mov_robot, timesteps_mov, traj_descr_mov);

            traj.push_back(traj_mov_robot);
            timesteps.push_back(timesteps_mov);
        }
        else
            ++nUnsolProb;
    }

    // **** Join movements to be performed **** //
    this->joinMovements(traj, timesteps, task_descr);


    // ******************************* //
    //           Execute Task          //
    // ******************************* //
    for(size_t k = 0; k < traj.size(); ++k)
    {
        string mov_descr = task_descr.at(k);
        double tol_stop;
        int prob;


        // **** Movement: Current stage **** //
        if(strcmp(mov_descr.c_str(), "plan") == 0)
        {
            // Plan or plan + approach stage
            plan = true;
            retreat = false;
            this->curr_mov = movTask.at(k);
            tol_stop = tols_stop_task.at(k).at(0);
            prob = nProb.at(k);
        }
        else if((strcmp(mov_descr.c_str(), "retreat_plan") == 0) || (strcmp(mov_descr.c_str(), "retreat") == 0))
        {
            // Retreat or Retreat + plan or retreat + plan + approach stage
            plan = false;
            retreat = true;
            this->curr_mov = movTask.at(k - 1);
            tol_stop = tols_stop_task.at(k - 1).at(2);
            prob = nProb.at(k - 1);
        }


        // **** Handles **** //
        int armCode = this->curr_mov->getArm();

        switch (armCode)
        {
        case 0: // dual arm
            break;
        case 1: //right arm
            handles = right_handles;
#if HAND == 0
            hand_handles = right_hand_handles;
#endif
#if HAND != 2
            h_attach = right_attach;
#endif
             break;
        case 2: // left arm
            handles = left_handles;
#if HAND == 0
            hand_handles = left_hand_handles;
#endif
            h_attach = left_attach;
            break;
        }


        // **** Hand open or close **** //
        int movType = this->curr_mov->getType();

        switch (movType)
        {
        case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
#if HAND == 0
            closed.at(0) = false;
            closed.at(1) = false;
            closed.at(2) = false;
#elif HAND == 1
            closed = false;
#endif
            break;
        case 2: case 3: case 4: // transport, engage, disengage
#if HAND == 0
            closed.at(0) = true;
            closed.at(1) = true;
            closed.at(2) = true;
#elif HAND == 1
            closed = true;
#endif
            break;
        }


        // **** Check if the object is in the robot's hand and if it has been grasped correctly
        string obj_name = this->curr_mov->getObject()->getName();
#if HAND == 0
        MatrixXd tt = traj.at(k).at(0);
        this->preMovementOperation(node, movType, retreat, armCode, h_attach, tt, obj_name);
#elif HAND == 1
        this->preMovementOperation(node, movType, retreat, h_attach, obj_name);
#endif


#if HAND == 0
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1) &&
                (client_enableSubscriber_hand.call(srv_enableSubscriber_hand)) && (srv_enableSubscriber_hand.response.subscriberID != -1))
#elif HAND == 1
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
#elif HAND == 2
        if((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
#endif
        {
            // the subscriber was succesfully started on V-REP and V-REP is now listening
            // **** Publish the data in the V-REP topics **** //
#if HAND == 0
            this->publishData(node, traj.at(k).at(0), timesteps.at(k).at(0), handles, hand_handles, scenarioID, timeTot, tols_stop_task.at(k).at(0));
#elif HAND == 1
            this->publishData(node, traj.at(k).at(0), timesteps.at(k).at(0), handles, scenarioID, timeTot, tol_stop);
#elif HAND == 2
            this->publishData(node, traj.at(k).at(0), timesteps.at(k).at(0), handles, scenarioID, timeTot, tol_stop);

#endif
            // **** Post-movement operations **** //
            this->posMovementOperation(node, movType, plan, h_attach);
        }

        ros::spinOnce();
        // Total time of the movement
        timeTot = simulationTime;

        log(QNode::Info, string("Movement completed"));
        task->getProblem(prob)->getMovement()->setExecuted(true);
    }


    // ******************************* //
    //         Pause simulation        //
    // ******************************* //
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);

    log(QNode::Info, string("Task completed"));
    ros::spinOnce();
    TotalTime = simulationTime;

    return true;
}

//subtracts the offset of the joints to their position
vector<MatrixXd> QNode::robotJointPositions(std::vector<MatrixXd>& traj_mov)
{
    for(size_t k = 0; k < traj_mov.size(); ++k)
    {
        MatrixXd traj = traj_mov.at(k);
        RowVectorXd aux_theta_off = VectorXd::Map(theta_offset.data(), traj.cols());
        MatrixXd theta_off(traj.rows(), traj.cols());

        for(int i = 0; i < traj.rows(); ++i)
            theta_off.row(i) << aux_theta_off;

        traj = traj - theta_off;
        traj_mov.at(k) = traj;
    }

    return traj_mov;
}


void QNode::joinStages(std::vector<MatrixXd> &traj, std::vector<std::vector<double>> &timesteps, std::vector<string> &traj_descr)
{
    MatrixXd traj_plan_approach;
    std::vector<double> timesteps_plan_approach;
    std::vector<MatrixXd> traj_mov;
    std::vector<std::vector<double>> timesteps_mov;

    // **** Copy and save the trajectory and timesteps **** //
    std::copy(traj.begin(), traj.end(), back_inserter(traj_mov));
    std::copy(timesteps.begin(), timesteps.end(), back_inserter(timesteps_mov));
    traj.clear();
    timesteps.clear();


    if(traj_mov.size() == 1)
    {
        // The movement is composed only by the plan stage
        traj.push_back(traj_mov.at(0));
        timesteps.push_back(timesteps_mov.at(0));
    }
    else if((traj_mov.size() > 1) && ((strcmp(traj_descr.at(0).c_str(), "plan") == 0) && (strcmp(traj_descr.at(1).c_str(), "approach") == 0)))
    {
        // The movement is composed by the stages: plan + approach or plan + approach + retreat
        traj_plan_approach.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows() - 1), traj_mov.at(0).cols());

        for(size_t k = 0; k < traj_mov.size(); ++k)
        {
            string mov_descr = traj_descr.at(k);
            MatrixXd tt = traj_mov.at(k);
            std::vector<double> ttsteps = timesteps_mov.at(k);

            if(strcmp(mov_descr.c_str(), "plan") == 0)
            {
                MatrixXd tt_plan = tt.topRows(tt.rows() - 1);
                traj_plan_approach.topLeftCorner(tt_plan.rows(), tt_plan.cols()) = tt_plan;
                timesteps_plan_approach.insert(timesteps_plan_approach.end(), ttsteps.begin(), ttsteps.end() - 1);
            }
            else if(strcmp(mov_descr.c_str(), "approach") == 0)
            {
                traj_plan_approach.bottomLeftCorner(tt.rows(), tt.cols()) = tt;
                timesteps_plan_approach.insert(timesteps_plan_approach.end(), ttsteps.begin(), ttsteps.end());

                traj.push_back(traj_plan_approach);
                timesteps.push_back(timesteps_plan_approach);
            }
            else if(strcmp(mov_descr.c_str(), "retreat") == 0)
            {
                traj.push_back(tt);;
                timesteps.push_back(ttsteps);
            }
        }
    }
    else if((traj_mov.size() > 1) && ((strcmp(traj_descr.at(0).c_str(), "plan") == 0) && (strcmp(traj_descr.at(1).c_str(), "retreat") == 0))) // Plan + Retreat
    {
        // The movement is composed by the plan and retreat stage
        traj.push_back(traj_mov.at(0));
        traj.push_back(traj_mov.at(1));
        timesteps.push_back(timesteps_mov.at(0));
        timesteps.push_back(timesteps_mov.at(1));
    }
}


void QNode::joinMovements(vector<vector<MatrixXd>> &traj, vector<vector<vector<double>>> &timesteps, vector<string> &task_descr)
{
    vector<vector<MatrixXd>> traj_task;
    vector<vector<vector<double>>> timesteps_task;
    bool joinStages;

    // **** Copy and save the trajectories and timesteps **** //
    std::copy(traj.begin(), traj.end(), back_inserter(traj_task));
    std::copy(timesteps.begin(), timesteps.end(), back_inserter(timesteps_task));
    traj.clear();
    timesteps.clear();
    task_descr.clear();


    for(size_t k = 0; k < traj_task.size(); ++k)
    {
        vector<MatrixXd> traj_mov = traj_task.at(k);
        vector<vector<double>> timesteps_mov = timesteps_task.at(k);

        for(size_t kk = 0; kk < traj_mov.size(); ++kk)
        {
            vector<MatrixXd> new_traj_mov;
            vector<vector<double>> new_timesteps_mov;
            MatrixXd traj_stage = traj_mov.at(kk);
            vector<double> timesteps_stage = timesteps_mov.at(kk);

            if(((k == 0 || !joinStages) &&  kk == 0))
            {
                // First movement: Plan or Plan + Approach
                // Other movement: Plan or Plan + Approach without previous execution of the retreat stage
                if(traj_mov.size() == 1)
                    joinStages = false;
                new_traj_mov.push_back(traj_stage);
                new_timesteps_mov.push_back(timesteps_stage);
                task_descr.push_back("plan");
            }
            else if((kk == 0 && kk + 1 >= traj_mov.size()) && joinStages)
            {
                // If the movement doesn't have retreat stage and it has joined to the previous movement
                // Example: Reach to grasp + (Go Park) + Reach to grasp
                joinStages = false;
            }
            else if(kk == 1)
            {
                if(k + 1 < traj_task.size())
                {
                    // Join the stages:
                    //   -> Retreat + Plan
                    //   -> Retreat + Plan + Approach
                    vector<double> timesteps_ret_plan;
                    MatrixXd traj_ret_plan;
                    traj_ret_plan.resize((traj_stage.rows() + traj_task.at(k + 1).at(0).rows() - 1), traj_stage.cols());

                    // **** Current mov: Retreat Stage **** //
                    MatrixXd tc_ret = traj_stage.topRows(traj_stage.rows() - 1);
                    traj_ret_plan.topLeftCorner(tc_ret.rows(), tc_ret.cols()) = tc_ret;
                    timesteps_ret_plan.insert(timesteps_ret_plan.end(), timesteps_stage.begin(), timesteps_stage.end() - 1);

                    // **** Next mov: Plan or Plan + Approach stage **** //
                    MatrixXd tn_plan = traj_task.at(k + 1).at(0);
                    vector<double> tn_steps = timesteps_task.at(k + 1).at(0);
                    traj_ret_plan.bottomLeftCorner(tn_plan.rows(), tn_plan.cols()) = tn_plan;
                    timesteps_ret_plan.insert(timesteps_ret_plan.end(), tn_steps.begin(), tn_steps.end());

                    joinStages = true;
                    task_descr.push_back("retreat_plan");
                    new_traj_mov.push_back(traj_ret_plan);
                    new_timesteps_mov.push_back(timesteps_ret_plan);
                }
                else
                {
                    // Only Retreat stage (There is no further movement to be performed next)
                    joinStages = false;
                    new_traj_mov.push_back(traj_stage);
                    new_timesteps_mov.push_back(timesteps_stage);
                    task_descr.push_back("retreat");
                }
            }

            if(!new_traj_mov.empty())
            {
                traj.push_back(new_traj_mov);
                timesteps.push_back(new_timesteps_mov);
                new_traj_mov.clear();
                new_timesteps_mov.clear();
            }
        }
    }
}

//*****************************************************************************************************************************//
//                                                 Collaborative Robot Sawyer                                                  //
//*****************************************************************************************************************************//
#if ROBOT == 1
bool QNode::moveRobotToStartPos(VectorXd &goal, double tol)
{
    // **** Initial Posture = Current Robot Posture **** //
    vector<double> robotPos(robotPosture.begin(), robotPosture.end() - JOINTS_HAND);
    // **** Final Posture = Initial Simulation Posture **** //
    vector<double> finalPos(&goal[0], goal.data() + (goal.cols() * goal.rows() - JOINTS_HAND));

    // **** Diference betwwen simulation and robot posture **** //
    double diff = 0.0;
    for(size_t i = 0; i < JOINTS_ARM; ++i)
        diff += pow((finalPos.at(i) - robotPos.at(i)), 2);


    if(sqrt(diff) > tol)
    {
        // Speed and acceleration per joint
        vector<double> maxAcc(JOINTS_ARM, 0.05);
        intera_motion_msgs::WaypointOptions options;
        options.max_joint_speed_ratio = 0.1;
        options.max_joint_accel = maxAcc;

        // Initial point: current posture
        intera_motion_msgs::Waypoint initPoint;
        initPoint.options = options;
        initPoint.joint_positions = robotPos;
        // Final point: final posture
        intera_motion_msgs::Waypoint finalPoint;
        finalPoint.options = options;
        finalPoint.joint_positions = finalPos;

        // Trajectory to be performed
        intera_motion_msgs::Trajectory traj;
        traj.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
        traj.waypoints.push_back(initPoint);
        traj.waypoints.push_back(finalPoint);
        traj.trajectory_options.interpolation_type = "JOINT";

        // Define the goal message
        intera_motion_msgs::MotionCommandGoal goToStartPos;
        goToStartPos.command = goToStartPos.MOTION_START;
        goToStartPos.trajectory = traj;

        // Send the goal message to the action server "/motion/motion_command"
        motionComm->sendGoal(goToStartPos);
        motionComm->waitForResult(ros::Duration(45));

        if(motionComm->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            log(QNode::Error, string("Error in reaching the initial posture of the robotic arm."));
            return false;
        }
    }

    ros::spinOnce();;

    return true;
}


void QNode::moveHeadToStartPos(double tol)
{
    while(abs(static_cast<double>(this->pan) - 0.0) > tol)
    {
        ros::Rate contRate(100); // 100 Hz = 0.01 seconds

        intera_core_msgs::HeadPanCommand setHeadPos;
        setHeadPos.target = 0.0;
        setHeadPos.speed_ratio = 0.2f;
        setHeadPos.pan_mode = intera_core_msgs::HeadPanCommand::SET_ACTIVE_MODE;
        pubHeadState.publish(setHeadPos);

        contRate.sleep();
        ros::spinOnce();
    }
}


#if HAND == 1
void QNode::setGripperPosition(double newPos)
{
    ros::Rate rate(10); // 10 Hz = 0.1 seconds

    intera_core_msgs::IOComponentCommand setPosGrip;
    setPosGrip.time = ros::Time::now();
    setPosGrip.op = "set";
    setPosGrip.args = string("{\"signals\": {\"position_m\": {\"data\": [") +
            boost::str(boost::format("%.4f") % (newPos)) +
            string("], \"format\": {\"type\": \"float\"}}}}");
    pubCommGripper.publish(setPosGrip);

    rate.sleep();
    ros::spinOnce();
}


void QNode::moveGripperToStartPos(double goal, double tol)
{
    // **** Speed of the gripper joint **** //
    double maxVel = 0.15;


    if(sqrt(pow((goal - robotPosture.at(7)), 2)) > tol)
    {
        // Calibrate the gripper in order to set maximum and minimum travel distance
        if(!gripperCalibrated)
        {
            intera_core_msgs::IOComponentCommand calibGrip;
            calibGrip.time = ros::Time::now();
            calibGrip.op = "set";
            calibGrip.args = "{\"signals\": {\"calibrate\": {\"data\": [true], \"format\": {\"type\": \"bool\"}}}}";
            pubCommGripper.publish(calibGrip);

            while(!gripperCalibrated)
                ros::spinOnce();
        }


        // Set the speed at which the gripper position movement will execute
        intera_core_msgs::IOComponentCommand setSpeedGrip;
        setSpeedGrip.time = ros::Time::now();
        setSpeedGrip.op = "set";
        setSpeedGrip.args = string("{\"signals\": {\"speed_mps\": {\"data\": [") +
                boost::str(boost::format("%.4f") % (maxVel)) +
                string("], \"format\": {\"type\": \"float\"}}}}");
        pubCommGripper.publish(setSpeedGrip);

        while(robotVel.at(7) != maxVel)
            ros::spinOnce();

        // Set the position of gripper
        setGripperPosition(goal);
    }
}

void QNode::feedbackCb(const control_msgs::FollowJointTrajectoryFeedback::ConstPtr &feedback, std::vector<vector<double>> &trajToExecute, vector<double> &params, bool isClosed)
{
    if(!isClosed)
    {
        std::vector<double> currArmPos(feedback->actual.positions);
        ros::Duration currTime = feedback->actual.time_from_start;
        std::vector<double> tarArmPos(trajToExecute.at(trajToExecute.size() - 1));

        double error = 0.0;
        for(size_t i = 0; i < JOINTS_ARM; ++i)
            error += pow((tarArmPos.at(i) - currArmPos.at(i)), 2);

        // **** Determine the current step **** //
        // Normalized time is given by the following formula:
        //    -> normTime(t) = 1 - exp((-a * t_curr) /(tau * (1 + w * ||theta_tar - theta_curr|| ^ 2))
        double normTime = 1 - exp((- params.at(0) * currTime.toSec()) / (params.at(1) * (1 + params.at(2) * error)));

        // CurrStep = (nTotalSteps - 1) * normTime
        unsigned int step = static_cast<unsigned int>((trajToExecute.size() - 1) * normTime);

        // **** Move the electric gripper to the current position **** //
        setGripperPosition(trajToExecute.at(step).at(7));

//        std::cout << "**********************" << std::endl;
//        std::cout << "Step : " << step << std::endl;
//        std::cout << "Total steps : " << trajToExecute.size() - 1 << std::endl;
//        std::cout << "Gripper pos: " << trajToExecute.at(step).at(7) << std::endl;
    }
}
#endif


void QNode::jointInterpolation(int nMicroSteps, MatrixXd &traj_stage, std::vector<double> &timesteps_stage, std::vector<vector<double>> &trajToExecute, std::vector<double> &timeFromStart)
{
#if HAND == 0
    int nJoints = JOINTS_ARM;
    int aux = JOINTS_HAND;
#elif HAND == 1
    int nJoints = JOINTS_ARM + JOINTS_HAND;
    int aux = 0;
#endif

    for(size_t kk = 0; kk < traj_stage.rows() - 1; ++kk)
    {
        // Get the position of the current and next step of the planned movement
        VectorXd pos_step_curr = traj_stage.row(kk);
        VectorXd pos_step_next = traj_stage.row(kk + 1);
        vector<double> pos_curr(&pos_step_curr[0], pos_step_curr.data() + (pos_step_curr.cols() * pos_step_curr.rows() - aux));
        vector<double> pos_next(&pos_step_next[0], pos_step_next.data() + (pos_step_next.cols() * pos_step_next.rows() - aux));

        // Determine the time associated with execution of each microSteps
        double t_inc = timesteps_stage.at(kk) / (nMicroSteps + 1);

        if(kk == 0)
        {
            // Adds the position obtained for the first step in plan and retreat stage
            trajToExecute.push_back(pos_curr);
            timeFromStart.push_back(0.0);
        }

        for(int n = 0; n < nMicroSteps + 1; ++n)
        {
            vector<double> pos_microStep;

            double t_curr = 0.0;
            double t_next = timesteps_stage.at(kk);

            // Linear interpolation depends on the value of m
            // m is determined by the following formula: (x - x0) / (x1 - x0)
            // In this case x is the time (in sec)
            double m = (((n + 1) * t_inc) - t_curr) / (t_next - t_curr);

            // Linear interpolation of the joints position
            for(int i = 0; i < nJoints; ++i)
                pos_microStep.push_back(interpolate(pos_curr.at(i), pos_next.at(i), m));

            trajToExecute.push_back(pos_microStep);
            timeFromStart.push_back(timeFromStart.back() + t_inc);
        }
    }
}


bool QNode::jointTrajectoryAction(std::vector<vector<double>> &trajToExecute, std::vector<double> &timeFromStart, std::vector<double> params, bool closeGripper, bool isGripperClosed)
{
    // Define the trajectory message
    trajectory_msgs::JointTrajectory jointTraj;
    jointTraj.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
    jointTraj.header.stamp = ros::Time::now();

    for(size_t step = 0; step < trajToExecute.size(); ++step)
    {
        std::vector<double> posArm(trajToExecute.at(step).begin(), trajToExecute.at(step).end() - 1);

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = posArm;
        point.time_from_start = ros::Duration(timeFromStart.at(step));
        jointTraj.points.push_back(point);
    }

    // Define the goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = jointTraj;

    // Send the message defined previously and wait the result
#if HAND == 0
    folJointTraj->sendGoal(goal);
#elif HAND == 1
    folJointTraj->sendGoal(goal, followJointTrajectoryClient::SimpleDoneCallback(), followJointTrajectoryClient::SimpleActiveCallback(),
                           boost::bind(&QNode::feedbackCb, this, _1, trajToExecute, params, isGripperClosed));
#endif
    folJointTraj->waitForResult(ros::Duration(45));


    if(folJointTraj->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(closeGripper)
        {
            setGripperPosition(0.0);
            log(QNode::Info, string("The object was correctly grasped."));
        }

        sleep(1);

        return true;
    }
    else
    {
        if(closeGripper)
            log(QNode::Error, string("Error in grasping the object."));

        return false;
    }

    ros::spinOnce();
}


bool QNode::execMovementSawyer(std::vector<MatrixXd>& traj_mov, std::vector<std::vector<double>> timesteps_mov, std::vector<string>& traj_descr, movementPtr mov, vector<double> paramsTimeMapping)
{
    ros::NodeHandle node;
    int movType = mov->getType();
    double tolArm = 0.04; // radians = 2.29 degrees
    double tolHead = 0.01; // radians = 0.57 degress
    int nMicroSteps = 3;
#if HAND == 1
    // **** Electric Gripper **** //
    double tolGrip = 0.035; // radians = 2.01 degrees
    bool isGripperClosed;
#endif


    // ************************************************** //
    //        Publishers and Gripper configuration        //
    // ************************************************** //
    pubHeadState = node.advertise<intera_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1, true);

#if HAND == 1
    // **** Publisher to the topic "io/end_effector/right_gripper/command" **** //
    pubCommGripper = node.advertise<intera_core_msgs::IOComponentCommand>("/io/end_effector/right_gripper/command", 1, true);

    // **** Gripper state at the beginning of the movement to be executed **** //
    switch (movType)
    {
    case 0: case 1: case 4: case 5: // reach-to-grasp, reaching, disengage, go-park
        isGripperClosed = false;
        break;
    case 2: case 3: // transport, engage
        isGripperClosed = true;
        break;
    }
#endif


    // ************************************************** //
    //                  Initial Postures                  //
    // ************************************************** //
    vector<MatrixXd> traj_mov_planned = traj_mov;
    vector<MatrixXd> traj_mov_robot = robotJointPositions(traj_mov_planned);
    VectorXd initPos = traj_mov_robot.at(0).row(0);

    // **** Move the robotic arm to its initial posture **** //
    this->moveRobotToStartPos(initPos, tolArm);
    // **** Move the head to its initial posture **** //
    this->moveHeadToStartPos(tolHead);

#if HAND == 1
    if(!isGripperClosed)
    {
        // **** Move the electric gripper to its initial posture **** //
        this->moveGripperToStartPos(initPos[7], tolGrip);
    }
#endif


    // ************************************************** //
    //  Movement analysis: Join plan and approach stages  //
    // ************************************************** //
    this->joinStages(traj_mov_robot, timesteps_mov, traj_descr);


    // ************************************************** //
    //           Perform the planned trajectory           //
    // ************************************************** //
    sleep(5);

    for(size_t k = 0; k < traj_mov_robot.size(); ++k)
    {
        std::vector<vector<double>> trajToExecute;
        std::vector<double> timeFromStart;

        string mov_descr = traj_descr.at(k);

        // **** Hand state: close or opened **** //
        switch (movType)
        {
        case 0: case 4: // Reach-to-grasp, Disengage
            if(strcmp(mov_descr.c_str(), "plan") == 0)
                isGripperClosed = false;
            else
                isGripperClosed = true;
            break;
        case 2: case 3: // Transport, Engage
            if(strcmp(mov_descr.c_str(), "plan") == 0)
                isGripperClosed = true;
            else
                isGripperClosed = false;
            break;
        case 1: case 5: // Reaching, Go-Park
            isGripperClosed = false;
            break;
        }

        // **** Joints linear interpolation **** //
        MatrixXd traj_stage = traj_mov_robot.at(k);
        std::vector<double> timesteps_stage = timesteps_mov.at(k);
        this->jointInterpolation(nMicroSteps, traj_stage, timesteps_stage, trajToExecute, timeFromStart);

        // **** Joint Trajectory Action Server **** //
        bool closeGripper = (((strcmp(mov_descr.c_str(), "plan") == 0) && (movType == 0 || movType == 4)));
        this->jointTrajectoryAction(trajToExecute, timeFromStart, paramsTimeMapping, closeGripper, isGripperClosed);

        ros::spinOnce();
    }

    log(QNode::Info,string("Movement completed."));
    ros::spinOnce();

    return true;
}


bool QNode::execTaskSawyer(vector<vector<MatrixXd>> &traj_task, vector<vector<vector<double>>> &timesteps_task, vector<vector<double>> &tols_stop_task, vector<vector<string>> &traj_descr_task, taskPtr task, vector<vector<double>> &paramsTimeMapping)
{
    ros::NodeHandle node;
    // **** Movements Settings **** //
    int nMicroSteps = 3;
    bool plan;
    vector<vector<MatrixXd>> traj;
    vector<vector<vector<double>>> timesteps;
    vector<string> task_descr;
    vector<movementPtr> movTask;
    vector<int> nProb;
    // **** Tolerances **** //
    double tolArm =  0.035; // radians = 2.01 degrees
    double tolHead = 0.01; // radians = 0.57 degress
#if HAND == 1
    // **** Electric Gripper **** //
    double tolGrip = 0.003; // meters
    bool isGripperClosed;
#endif


    // ******************************* //
    //            Publishers           //
    // ******************************* //
    pubHeadState = node.advertise<intera_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1, true);
#if HAND == 1
    // **** Publisher to the topic "io/end_effector/right_gripper/command" **** //
    pubCommGripper = node.advertise<intera_core_msgs::IOComponentCommand>("/io/end_effector/right_gripper/command", 1, true);
#endif


    // ******************************* //
    //          Join Movements         //
    // ******************************* //
    // Problems that don't belong to this task
    int nUnsolProb = 0;

    for(int k = 0; k < task->getProblemNumber(); ++k)
    {
        if(task->getProblem(k)->getPartOfTask() && task->getProblem(k)->getSolved())
        {
            // Number of the problem in the task to be performed
            int kk = k - nUnsolProb;
            nProb.push_back(k);

            movementPtr mov = task->getProblem(kk)->getMovement();
            movTask.push_back(mov);

            vector<MatrixXd> traj_mov_planned = traj_task.at(kk);
            vector<MatrixXd> traj_mov_robot = this->robotJointPositions(traj_mov_planned);
            vector<vector<double>> timesteps_mov = timesteps_task.at(kk);
            vector<string> traj_descr_mov = traj_descr_task.at(kk);

            // **** Join plan and approach stage **** //
            this->joinStages(traj_mov_robot, timesteps_mov, traj_descr_mov);

            traj.push_back(traj_mov_robot);
            timesteps.push_back(timesteps_mov);
        }
        else
            ++nUnsolProb;
    }

    // **** Join movements to be performed **** //
    this->joinMovements(traj, timesteps, task_descr);


    // ******************************* //
    //         Initial Postures        //
    // ******************************* //
    // Move the robotic arm to its initial posture
    VectorXd initPos = traj.at(0).at(0).row(0);
    this->moveRobotToStartPos(initPos, tolArm);

    // Move the head to its initial posture
    this->moveHeadToStartPos(tolHead);

#if HAND == 1
    // Move the electric gripper to its initial posture
    this->moveGripperToStartPos(initPos[7], tolGrip);
#endif


    // ******************************* //
    //           Execute Task          //
    // ******************************* //
    sleep(10);

    for(size_t k = 0; k < traj.size(); ++k)
    {
        int prob;
        std::vector<vector<double>> trajToExecute;
        std::vector<double> timeFromStart;
        std::vector<double> params;

        string mov_descr = task_descr.at(k);

        // **** Movement: Current stage **** //
        if(strcmp(mov_descr.c_str(), "plan") == 0)
        {
            // Plan or plan + approach stage
            plan = true;
            this->curr_mov = movTask.at(k);
            prob = nProb.at(k);
        }
        else if((strcmp(mov_descr.c_str(), "retreat_plan") == 0) || (strcmp(mov_descr.c_str(), "retreat") == 0))
        {
            // Retreat or Retreat + plan or retreat + plan + approach stage
            plan = false;
            this->curr_mov = movTask.at(k - 1);
            prob = nProb.at(k - 1);
        }


        // **** Hand state: close or opened **** //
        int movType = this->curr_mov->getType();

        switch (movType)
        {
        case 0: case 4: // Reach-to-grasp, Disengage
            if(plan)
            {
                isGripperClosed = false;
                std::copy(paramsTimeMapping.at(1).begin(), paramsTimeMapping.at(1).end(), back_inserter(params));
            }
            else
                isGripperClosed = true;
            break;
        case 2: case 3: // Transport, Engage
            if(plan)
                isGripperClosed = true;
            else
            {
                isGripperClosed = false;
                std::copy(paramsTimeMapping.at(2).begin(), paramsTimeMapping.at(2).end(), back_inserter(params));
            }
            break;
        case 1: case 5: // Reaching, Go-Park
            isGripperClosed = false;
            std::copy(paramsTimeMapping.at(0).begin(), paramsTimeMapping.at(0).end(), back_inserter(params));
            break;
        }

        // **** Joints linear interpolation **** //
        MatrixXd traj_stage = traj.at(k).at(0);
        std::vector<double> timesteps_stage = timesteps.at(k).at(0);
        this->jointInterpolation(nMicroSteps, traj_stage, timesteps_stage, trajToExecute, timeFromStart);

        // **** Joint Trajectory Action Server **** //
        bool closeGripper = (plan && (movType == 0 || movType == 4));
        this->jointTrajectoryAction(trajToExecute, timeFromStart, params, closeGripper, isGripperClosed);

        ros::spinOnce();
        trajToExecute.clear();
        timeFromStart.clear();
        params.clear();
    }

    log(QNode::Info, string("Task completed"));
    ros::spinOnce();

    return true;
}
#endif


//*****************************************************************************************************************************//
//                                                           Logging                                                           //
//*****************************************************************************************************************************//

void QNode::log(const LogLevel &level, const string &msg)
{
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;

    switch (level)
    {
    case(Debug):
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << currentDateTime() << "]: " << msg;
        break;
    case(Info):
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
        break;
    case(Warn):
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << currentDateTime() << "]: " << msg;
        break;
    case(Error):
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << currentDateTime() << "]: " << msg;
        break;
    case(Fatal):
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << currentDateTime() << "]: " << msg;
        break;
    }

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated();
}


QStringListModel* QNode::loggingModel()
{
    return &logging_model;
}


const string QNode::currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


void QNode::init()
{
    logging::add_file_log
            (
                keywords::file_name = "QNode_%N.log",
                keywords::rotation_size = 10 * 1024 * 1024,
                keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0,0,0),
                keywords::format = "[%TimeStamp%]: %Message%",
                keywords::target = "Boost_logs"
            );

    logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::info);
}


void QNode::updateObjectInfo(int obj_id, string name, const geometry_msgs::PoseStamped &data)
{
    std::vector<double> rpy;
    objectPtr obj = this->curr_scene->getObject(name);

    // **** position **** //
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]
    obj->setPos(poss,true);

    // **** orientation **** //
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    Matrix3d Rot;
    Rot(0, 0) = 2 * (pow(w, 2) + pow(epx, 2)) - 1; Rot(0, 1) = 2 * (epx * epy - w * epz);         Rot(0, 2) = 2 * (epx * epz + w * epy);
    Rot(1, 0) = 2 * (epx * epy + w * epz);         Rot(1, 1) = 2 * (pow(w, 2) + pow(epy, 2)) - 1; Rot(1, 2) = 2 * (epy * epz - w * epx);
    Rot(2, 0) = 2 * (epx * epz - w * epy);         Rot(2, 1) = 2 * (epy * epz + w * epx);         Rot(2, 2) = 2 * (pow(w, 2) + pow(epz, 2)) - 1;

    Matrix4d trans_obj;
    trans_obj(0, 0) = Rot(0, 0); trans_obj(0, 1) = Rot(0, 1); trans_obj(0, 2) = Rot(0, 2); trans_obj(0, 3) = poss.Xpos;
    trans_obj(1, 0) = Rot(1, 0); trans_obj(1, 1) = Rot(1, 1); trans_obj(1, 2) = Rot(1, 2); trans_obj(1, 3) = poss.Ypos;
    trans_obj(2, 0) = Rot(2, 0); trans_obj(2, 1) = Rot(2, 1); trans_obj(2, 2) = Rot(2, 2); trans_obj(2, 3) = poss.Zpos;
    trans_obj(3, 0) = 0;         trans_obj(3, 1) = 0;         trans_obj(3, 2) = 0;         trans_obj(3, 3) = 1;

    if(this->getRPY(trans_obj,rpy))
    {
        orr.roll  = rpy.at(0);
        orr.pitch = rpy.at(1);
        orr.yaw = rpy.at(2);
        obj->setOr(orr,true);
    }

    string info = obj->getInfoLine();
    Q_EMIT updateElement(obj_id,info);

    if(this->curr_scene)
        this->curr_scene->setObject(obj_id,obj);
}


double QNode::interpolate(double ya, double yb, double m)
{
    return ya + (yb - ya) * m;
}


bool QNode::getRPY(Matrix4d Trans, std::vector<double> &rpy)
{
    rpy = std::vector<double>(3);

    if((abs(Trans(0, 0)) < 1e-5) && (abs(Trans(1, 0)) < 1e-5))
    {
        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2, 0), Trans(0, 0)); // [rad]
        rpy.at(2) = atan2(-Trans(1, 2), Trans(1, 1)); // [rad]

        return false;
    }
    else
    {
        rpy.at(0) = atan2(Trans(1, 0), Trans(0, 0)); // [rad]
        double sp = sin(rpy.at(0));
        double cp = cos(rpy.at(0));
        rpy.at(1) = atan2(- Trans(2, 0), cp * Trans(0, 0) + sp*Trans(1, 0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0, 2) - cp * Trans(1, 2), cp*Trans(1, 1) - sp * Trans(0,1)); // [rad]

        return true;
    }
}

bool QNode::RotgetRPY(Matrix3d rot, std::vector<double> &rpy)
{
    rpy = std::vector<double>(3);

    if((abs(rot(0, 0)) < 1e-5) && (abs(rot(1, 0)) < 1e-5))
    {
        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-rot(2, 0), rot(0, 0)); // [rad]
        rpy.at(2) = atan2(-rot(1, 2), rot(1, 1)); // [rad]

        return false;
    }
    else
    {
        rpy.at(0) = atan2(rot(1, 0), rot(0, 0)); // [rad]
        double sp = sin(rpy.at(0));
        double cp = cos(rpy.at(0));
        rpy.at(1) = atan2(- rot(2, 0), cp * rot(0, 0) + sp*rot(1, 0)); // [rad]
        rpy.at(2) = atan2(sp*rot(0, 2) - cp * rot(1, 2), cp*rot(1, 1) - sp * rot(0,1)); // [rad]

        return true;
    }
}


void QNode::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty())
    {
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        Rot(0, 0) = cos(roll) * cos(pitch);  Rot(0, 1) = cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw); Rot(0, 2) = sin(roll) * sin(yaw) + cos(roll) * sin(pitch) * cos(yaw);
        Rot(1, 0) = sin(roll) * cos(pitch);  Rot(1, 1) = cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw); Rot(1, 2) = sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw);
        Rot(2, 0) = -sin(pitch);             Rot(2, 1) = cos(pitch) * sin(yaw);                                    Rot(2, 2) = cos(pitch) * cos(yaw);
    }
}

//*****************************************************************************************************************************//
//                                                         Barrett Hand                                                        //
//*****************************************************************************************************************************//
#if HAND == 0
bool QNode::closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1);
    ros::NodeHandle node;
    std::vector<double> hand2_pos;

    switch (hand)
    {
    case 1: // right hand
        hand_handles = right_hand_handles;
        hand2_pos = right_2hand_pos;
        break;
    case 2: // left hand
        hand_handles = left_hand_handles;
        hand2_pos = left_2hand_pos;
        break;
    }

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


    for (size_t i = 0; i < HAND_FINGERS; ++i)
    {
        // **** First joint **** //
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
        client_setTarPos.call(srv_setTarPos);

        // **** Second joint **** //
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 2);
        srv_setTarPos.request.targetPosition = 45.0f * static_cast<double>(M_PI) / 180.0f + hand_posture.at(i + 1) / 3.0f;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = true;
        needFullOpening[i] = 1;
        closed[i] = true;
    }

    log(QNode::Info,string("Hand closed."));


    return true;
}
#endif


//*****************************************************************************************************************************//
//                                                       V-REP Callbacks                                                       //
//*****************************************************************************************************************************//

void QNode::infoCallback(const vrep_common::VrepInfoConstPtr& info)
{
    simulationTime = info->simulationTime.data;
    simulationTimeStep = info->timeStep.data;
    simulationRunning = (info->simulatorState.data&1) != 0;
}


void QNode::JointsCallback(const sensor_msgs::JointState &state)
{
    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());
    std::vector<double> joints_vel(state.velocity.begin(),state.velocity.end());
    std::vector<double> joints_force(state.effort.begin(),state.effort.end());

    std::vector<double> right_posture; std::vector<double> left_posture;
    std::vector<double> right_vel; std::vector<double> left_vel;
    std::vector<double> right_forces; std::vector<double> left_forces;

#if HAND == 0
    const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                             "right_BarrettHand_jointA_0","right_BarrettHand_jointB_0","right_BarrettHand_jointB_2","right_BarrettHand_jointB_1"};
    const char *r_2hand[]={"right_BarrettHand_jointC_0","right_BarrettHand_jointC_2","right_BarrettHand_jointC_1"};

    const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                             "left_BarrettHand_jointA_0","left_BarrettHand_jointB_0","left_BarrettHand_jointB_2","left_BarrettHand_jointB_1"};
    const char *l_2hand[]={"left_BarrettHand_jointC_0","left_BarrettHand_jointC_2","left_BarrettHand_jointC_1"};
#elif HAND == 1
    const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                             "right_gripper_jointClose"};
#elif HAND == 2
    const char *r_names[] = {"right_joint0", "right_joint1", "right_joint2", "right_joint3","right_joint4", "right_joint5", "right_joint6",
                             "right_gripper_jointClose"};

#endif

    for(int i = 0; i < JOINTS_ARM+JOINTS_HAND; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();

        if(r_index < joints_names.size())
        {
            right_posture.push_back(joints_pos.at(r_index));
            right_vel.push_back(joints_vel.at(r_index));
            right_forces.push_back(joints_force.at(r_index));
        }

#if HAND == 0
        if(this->curr_scene->getRobot()->getName() == "ARoS")
        {
            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_names[i]) - joints_names.begin();

            if(l_index < joints_names.size())
            {
                left_posture.push_back(joints_pos.at(l_index));
                left_vel.push_back(joints_vel.at(l_index));
                left_forces.push_back(joints_force.at(l_index));
            }
        }
#endif
    }

#if HAND == 0
    for(int i = 0; i < HAND_FINGERS; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_2hand[i]) - joints_names.begin();

        if(r_index < joints_names.size())
        {
            right_2hand_pos.at(i) = joints_pos.at(r_index);
            right_2hand_vel.at(i) = joints_vel.at(r_index);
            right_2hand_force.at(i) = joints_force.at(r_index);
        }

        if(this->curr_scene->getRobot()->getName() == "ARoS")
        {
            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_2hand[i]) - joints_names.begin();

            if(l_index < joints_names.size())
            {
                left_2hand_pos.at(i) = joints_pos.at(l_index);
                left_2hand_vel.at(i) = joints_vel.at(l_index);
                left_2hand_force.at(i) = joints_force.at(l_index);
            }
        }
    }
#endif

    if(this->curr_scene)
    {
        // joints offset
        std::transform(right_posture.begin(), right_posture.end(), theta_offset.begin(), right_posture.begin(), std::plus<double>());
        // add informations about robot
        this->curr_scene->getRobot()->setRightPosture(right_posture);
        this->curr_scene->getRobot()->setRightVelocities(right_vel);
        this->curr_scene->getRobot()->setRightForces(right_forces);

        if(this->curr_scene->getRobot()->getName() == "ARoS")
        {
            std::transform(left_posture.begin(), left_posture.end(), theta_offset.begin(), left_posture.begin(), std::plus<double>());
            this->curr_scene->getRobot()->setLeftPosture(left_posture);
            this->curr_scene->getRobot()->setLeftVelocities(left_vel);
            this->curr_scene->getRobot()->setLeftForces(left_forces);
        }
    }
}


void QNode::rightProxCallback(const vrep_common::ProximitySensorData& data)
{
    if(this->curr_mov)
    {
        int arm_code = this->curr_mov->getArm();
        int mov_type = this->curr_mov->getType();

        // **** Right arm in reach-to-grasp movement **** //
        if(arm_code == 1 && mov_type == 0)
        {
            // Visible handle of the object we want to grasp
            int h_obj_body = this->curr_mov->getObject()->getHandleBody();
            // Non visible handle of the object we want to grasp
            int h_obj = this->curr_mov->getObject()->getHandle();

            // Handle of the object currently detected
            h_detobj = data.detectedObject.data;
            obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
        }
    }
}


void QNode::leftProxCallback(const vrep_common::ProximitySensorData& data)
{

}


//*****************************************************************************************************************************//
//                                                    Toy Vehicles Callbacks                                                   //
//*****************************************************************************************************************************//

void QNode::BlueColumnCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 0;
    string name = string("BlueColumn");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::GreenColumnCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 1;
    string name = string("GreenColumn");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::RedColumnCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 2;
    string name = string("RedColumn");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::MagentaColumnCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 3;
    string name = string("MagentaColumn");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Nut1Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 4;
    string name = string("Nut1");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Nut2Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 5;
    string name = string("Nut2");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Wheel1Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 6;
    string name = string("Wheel1");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Wheel2Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 7;
    string name = string("Wheel2");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::BaseCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 8;
    string name = string("Base");

    this->updateObjectInfo(obj_id,name,data);
}


//*****************************************************************************************************************************//
//                                                  Human Assistance Callbacks                                                 //
//*****************************************************************************************************************************//

void QNode::BottleTeaCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 0;
    string name = string("BottleTea");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::BottleCoffeeCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 1;
    string name = string("BottleCoffee");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::BottleJuiceCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 2;
    string name = string("BottleJuice");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::CupCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 3;
    string name = string("Cup");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Cup1Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 4;
    string name = string("Cup1");

    this->updateObjectInfo(obj_id,name,data);
}


//*****************************************************************************************************************************//
//                                                    Robot Sawyer Callbacks                                                   //
//*****************************************************************************************************************************//
#if ROBOT == 1
void QNode::SawyerJointsCallback(const sensor_msgs::JointState &state)
{
    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());
    std::vector<double> joints_vel(state.velocity.begin(), state.velocity.end());

    const char *r_names[] = {"right_j0", "right_j1", "right_j2", "right_j3","right_j4", "right_j5", "right_j6"};

    for(int i = 0; i < JOINTS_ARM; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();

        if(r_index < joints_names.size())
        {
            robotPosture.at(i) = joints_pos.at(r_index);
            robotVel.at(i) = joints_vel.at(r_index);
        }
    }
}


void QNode::SawyerHeadCallback(const intera_core_msgs::HeadState &state)
{
    this->pan = state.pan;
}


#if HAND == 1
void QNode::SawyerGripperCallback(const intera_core_msgs::IODeviceStatus &state)
{
    std::vector<intera_core_msgs::IODataStatus> signals = state.signals;

    for(int i = 0; i < signals.size(); ++i)
    {
        string data = signals.at(i).data.substr(1, signals.at(i).data.size() - 2);

        if(!signals.at(i).name.compare("is_calibrated"))
            gripperCalibrated = (strcasecmp("true", data.c_str()) == 0);
        else if(!signals.at(i).name.compare("position_response_m"))
            robotPosture.at(7) = atof(data.c_str());
        else if(!signals.at(i).name.compare("speed_mps"))
            robotVel.at(7) = atof(data.c_str());
    }
}
#endif //  endif of if HAND == 1
#endif  //  endif of if robot == 1


//*****************************************************************************************************************************//
//                                                    Robot UR10 Callbacks                                                   //
//*****************************************************************************************************************************/

void QNode::URJointsCallback(const sensor_msgs::JointState &state)
{
    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());
    std::vector<double> joints_vel(state.velocity.begin(), state.velocity.end());

    const char *r_names[] = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint","wrist_3_joint"};

    for(int i = 0; i < JOINTS_ARM; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();

        if(r_index < joints_names.size())
        {
            //use this to get the UR joints positions from ros
            robotPosture_wp.at(i) = joints_pos.at(r_index);
            robotVel_wp.at(i) = joints_vel.at(r_index);
        }
    }

}




}  // namespace motion_manager
