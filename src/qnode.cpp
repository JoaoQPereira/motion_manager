/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>

#include <ros/network.h>
#include <string>
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


namespace motion_manager {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv)
{
    nodeName = "motion_manager";
    TotalTime = 0.0;
#if HAND == 0
    right_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    left_hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    right_2hand_pos.assign(3,0.0f);
    right_2hand_vel.assign(3,0.0f);
    right_2hand_force.assign(3,0.0f);
    left_2hand_pos.assign(3,0.0f);
    left_2hand_vel.assign(3,0.0f);
    left_2hand_force.assign(3,0.0f);
    firstPartLocked.assign(3,false);
    needFullOpening.assign(3,0);
    closed.assign(3,false);
#elif HAND == 1
    closed = false;
#endif
    robotPosture.assign(7,0.0f);
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

    if (!ros::master::check())
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

    if (!ros::master::check())
		return false;

    ros::start();
    start();
	return true;
}


void QNode::on_end()
{
}


bool QNode::loadScenario(const std::string& path,int id)
{
    ros::NodeHandle n;

#if ROBOT == 1
    // ------------------------------------------------------------------------------------------- //
    //                                   ROBOT SUBSCRIBERS                                         //
    // Sawyer scenarios
    if(id >= 2)
    {
        // Topic that contains the position of the Sawyer joints
        subJoints_state_robot = n.subscribe("/robot/joint_states", 1, &QNode::SawyerJointsCallback, this);

        // ------------------------------------------------------------------------------------------- //
        //                                   ROBOT ACTIONLIBS                                          //
        // Create the action client specifying the server name to connect: "/motion/motion_command"
        motionComm = new motionCommClient("/motion/motion_command", true);
        // The action client waits for the action server to start before continuing
        motionComm->waitForServer();

        // Create the action client specifying the server name to connect: "/robot/limb/right/follow_joint_trajectory"
        folJointTraj = new followJointTrajectoryClient("/robot/limb/right/follow_joint_trajectory", true);
        // The action client waits for the action server to start before continuing
        folJointTraj->waitForServer();
    }
#endif

    // ------------------------------------------------------------------------------------------- //
    //                                   VREP SUBSCRIBERS                                         //
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

    if (res == 1)
    {
        subInfo = n.subscribe("/vrep/info",1, &QNode::infoCallback,this);
        subJoints_state = n.subscribe("/vrep/joints_state",1, &QNode::JointsCallback, this);
        subRightProxSensor = n.subscribe("/vrep/right_prox_sensor",1,&QNode::rightProxCallback,this);

        if(id!=4 && id!=5)
            subLeftProxSensor = n.subscribe("/vrep/left_prox_sensor",1,&QNode::leftProxCallback,this);

        switch(id)
        {
        case 0: case 2: case 4:
            // Assembly scenario: the Toy vehicle with ARoS
            // Assembly scenario: the Toy vehicle with Jarde
            // Assembly scenario: the Toy vehicle with Sawyer
            // Blue Column (obj_id = 0)
            subBlueColumn = n.subscribe("/vrep/BlueColumn_pose",1,&QNode::BlueColumnCallback,this);
            // Green Column (obj_id = 1)
            subGreenColumn = n.subscribe("/vrep/GreenColumn_pose",1,&QNode::GreenColumnCallback,this);
            // RedColumn (obj_id = 2)
            subRedColumn = n.subscribe("/vrep/RedColumn_pose",1,&QNode::RedColumnCallback,this);
            // MagentaColumn (obj_id = 3)
            subMagentaColumn = n.subscribe("/vrep/MagentaColumn_pose",1,&QNode::MagentaColumnCallback,this);
            // Nut 1 (obj_id = 4)
            subNut1 = n.subscribe("/vrep/Nut1_pose",1,&QNode::Nut1Callback,this);
            // Nut 2 (obj_id = 5)
            subNut2 = n.subscribe("/vrep/Nut2_pose",1,&QNode::Nut2Callback,this);
            // Wheel 1 (obj_id = 6)
            subWheel1 = n.subscribe("/vrep/Wheel1_pose",1,&QNode::Wheel1Callback,this);
            // Wheel 2 (obj_id = 7)
            subWheel2 = n.subscribe("/vrep/Wheel2_pose",1,&QNode::Wheel2Callback,this);
            // Base (obj_id = 8)
            subBase = n.subscribe("/vrep/Base_pose",1,&QNode::BaseCallback,this);
            break;
        case 1: case 3:
            // Human assistance scenario: Serving a drink with ARoS
            // Human assistance scenario: Serving a drink with Sawyer
            // Bottle Tea (obj_id = 0)
            subBottleTea = n.subscribe("/vrep/BottleTea_pose",1,&QNode::BottleTeaCallback,this);
            // Bottle Coffee (obj_id = 1)
            subBottleCoffee = n.subscribe("/vrep/BottleCoffee_pose",1,&QNode::BottleCoffeeCallback,this);
            // Bottle Juice (obj_id = 2)
            subBottleJuice = n.subscribe("/vrep/BottleJuice_pose",1,&QNode::BottleJuiceCallback,this);
            // Cup (obj_id = 3)
            subCup = n.subscribe("/vrep/Cup_pose",1,&QNode::CupCallback,this);
            // Cup 1 (obj_id = 4)
            subCup1 = n.subscribe("/vrep/Cup1_pose",1,&QNode::Cup1Callback,this);
            break;
        }
#if MOVEIT == 1
        // planning scene of RViZ
        planning_scene_interface_ptr.reset(new moveit::planning_interface::PlanningSceneInterface());
#endif
        ros::spinOnce();
        return true;
    }
    else
        return false;
}


#if MOVEIT == 1
void QNode::loadRVizScenario(std::vector<objectPtr> &objs)
{
    vector<string> rem_object_ids;
    vector<moveit_msgs::CollisionObject> add_collision_objects;

    for(size_t i=0; i<objs.size();++i)
    {
        objectPtr obj = objs.at(i); string name = obj->getName();
        string mesh_file;
        if(strcmp(name.c_str(),"Table")==0)
            mesh_file = "table/table.dae";
        else if((strcmp(name.c_str(),"BlueColumn")==0)||
                (strcmp(name.c_str(),"GreenColumn")==0)||
                (strcmp(name.c_str(),"RedColumn")==0)||
                (strcmp(name.c_str(),"MagentaColumn")==0))
            mesh_file = "column/column.dae";
        else if((strcmp(name.c_str(),"Nut1")==0)||
                (strcmp(name.c_str(),"Nut2")==0))
            mesh_file = "nut/nut.dae";
        else if((strcmp(name.c_str(),"Wheel1")==0)||
                (strcmp(name.c_str(),"Wheel2")==0))
            mesh_file = "wheel/wheel.dae";
        else if(strcmp(name.c_str(),"Base")==0)
            mesh_file = "base/base.dae";
        else if((strcmp(name.c_str(),"BottleTea")==0)||
                (strcmp(name.c_str(),"BottleCoffee")==0)||
                (strcmp(name.c_str(),"BottleJuice")==0))
            mesh_file = "bottle/bottle.dae";
        else if((strcmp(name.c_str(),"Cup")==0)||
                (strcmp(name.c_str(),"Cup1")==0))
            mesh_file = "cup/cup.dae";
        else if((strcmp(name.c_str(),"Shelf")==0)||
                (strcmp(name.c_str(),"Shelf_1_b")==0))
            mesh_file = "shelf/shelf_1.dae";
        else if((strcmp(name.c_str(),"Shelf_2_a")==0)||
                (strcmp(name.c_str(),"Shelf_2_b")==0))
            mesh_file = "shelf/shelf_2.dae";
        else if(strcmp(name.c_str(),"Shelf_3")==0)
            mesh_file = "shelf/shelf_3.dae";
        else if((strcmp(name.c_str(),"Shelf_4_a")==0)||
                (strcmp(name.c_str(),"Shelf_4_b")==0)||
                (strcmp(name.c_str(),"Shelf_4_c")==0)||
                (strcmp(name.c_str(),"Shelf_4_d")==0))
            mesh_file = "shelf/shelf_4.dae";

        std::vector<double> rpy = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
        Matrix3d Rot; this->RPY_matrix(rpy,Rot); Quaterniond q(Rot);

        string mesh_path = string("package://models/meshes/")+mesh_file;

        moveit_msgs::CollisionObject co;
        co.header.stamp = ros::Time::now();
        co.header.frame_id = FRAME_ID;
        // remove
        co.id = name;
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        rem_object_ids.push_back(co.id);
        // add
        co.operation = moveit_msgs::CollisionObject::ADD;
        shapes::Mesh* table_shape = shapes::createMeshFromResource(mesh_path.c_str());
        shapes::ShapeMsg table_mesh_msg;
        shapes::constructMsgFromShape(table_shape,table_mesh_msg);
        shape_msgs::Mesh table_mesh = boost::get<shape_msgs::Mesh>(table_mesh_msg);
        co.meshes.resize(1);
        co.meshes[0] = table_mesh;
        co.mesh_poses.resize(1);
        co.mesh_poses[0].position.x = obj->getPos().Xpos/1000; // [m]
        co.mesh_poses[0].position.y = obj->getPos().Ypos/1000; // [m]
        co.mesh_poses[0].position.z = obj->getPos().Zpos/1000; // [m]
        co.mesh_poses[0].orientation.w= q.w();
        co.mesh_poses[0].orientation.x= q.x();
        co.mesh_poses[0].orientation.y= q.y();
        co.mesh_poses[0].orientation.z= q.z();
        add_collision_objects.push_back(co);
    }
    // remove
    planning_scene_interface_ptr->removeCollisionObjects(rem_object_ids);
    // add
    planning_scene_interface_ptr->addCollisionObjects(add_collision_objects);
    /* Sleep so we have time to see the object in RViz */
    ros::WallDuration(5.0).sleep();
}
#endif


void QNode::resetSimTime()
{
    this->TotalTime=0.0;
}


void QNode::resetGlobals()
{
#if HAND == 0
    for (int i =0; i < 3; ++i)
    {
        closed.at(i)=false;
        needFullOpening.at(i)=0;
        firstPartLocked.at(i)=false;
    }
#endif
    obj_in_hand = false;
}


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
#endif

    // **** torso parameters **** //
    robot_part torso;  // parameters of the torso (ARoS, Jarde and Sawyer)
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


    //*************************************************************************************************
    //                               OBJECTS IN THE SCENARIO
    //*************************************************************************************************
    add_client = n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal");
    srvi.request.signalName = NOBJECTS;
    add_client.call(srvi);
    if (srvi.response.result == 1)
        n_objs = srvi.response.signalValue;
    else
    {
        succ = false;
        throw string("Communication error");
    }

    if(scenarioID == 2 || scenarioID == 4)
    {
        srvi.request.signalName = NPOSES;
        add_client.call(srvi);
        if (srvi.response.result == 1)
            n_poses= srvi.response.signalValue;
        else
        {
            succ = false;
            throw string("Communication error");
        }
    }

    client_getHandle = n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    if(scenarioID == 1 || scenarioID ==  3 || scenarioID == 5) // Toy Vehicle scenarios (ARoS, Sawyer)
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
    else if (scenarioID == 2 || scenarioID == 4) // Drinking Service scenarios (ARoS, Sawyer)
    {
        objs_prefix.push_back("BottleTea");      // obj_id = 0
        objs_prefix.push_back("BottleCoffee");   // obj_id = 1
        objs_prefix.push_back("BottleJuice");    // obj_id = 2
        objs_prefix.push_back("Cup");            // obj_id = 3
        objs_prefix.push_back("Cup1");           // obj_id = 4
        objs_prefix.push_back("Table");          // obj_id = 5
    }

    while(cnt_obj < n_objs)
    {
        signPrefix = objs_prefix[cnt_obj];

        add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
        srvs.request.signalName = signPrefix + string("Info");
        add_client.call(srvs);
        if (srvs.response.result == 1)
            obj_info_str = srvs.response.signalValue;
        else
        {
            throw string("Error: Couldn't get the information of the object");
            succ = false;
        }

        if (succ)
        {
            floatCount = obj_info_str.size()/sizeof(float);

            if(!obj_info_vec.empty())
                obj_info_vec.clear();
            for (int k=0;k<floatCount;++k)
                obj_info_vec.push_back(static_cast<double>(((float*)obj_info_str.c_str())[k]));

            // position of the object
            obj_pos.Xpos = obj_info_vec.at(0)*1000; //[mm]
            obj_pos.Ypos = obj_info_vec.at(1)*1000; //[mm]
            obj_pos.Zpos = obj_info_vec.at(2)*1000; //[mm]
            // orientation of the object
            obj_or.roll = obj_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
            obj_or.pitch = obj_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
            obj_or.yaw = obj_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]
            // size of the object
            obj_size.Xsize = obj_info_vec.at(6)*1000; //[mm]
            obj_size.Ysize = obj_info_vec.at(7)*1000; //[mm]
            obj_size.Zsize = obj_info_vec.at(8)*1000; //[mm]
            // position of the target right
            tarRight_pos.Xpos = obj_info_vec.at(9)*1000;//[mm]
            tarRight_pos.Ypos = obj_info_vec.at(10)*1000;//[mm]
            tarRight_pos.Zpos = obj_info_vec.at(11)*1000;//[mm]
            // orientation of the target right
            tarRight_or.roll = obj_info_vec.at(12)*static_cast<double>(M_PI)/180;//[rad]
            tarRight_or.pitch = obj_info_vec.at(13)*static_cast<double>(M_PI)/180;//[rad]
            tarRight_or.yaw = obj_info_vec.at(14)*static_cast<double>(M_PI)/180;//[rad]
            // position of the target left
            tarLeft_pos.Xpos = obj_info_vec.at(15)*1000;//[mm]
            tarLeft_pos.Ypos = obj_info_vec.at(16)*1000;//[mm]
            tarLeft_pos.Zpos = obj_info_vec.at(17)*1000;//[mm]
            // orientation of the target left
            tarLeft_or.roll = obj_info_vec.at(18)*static_cast<double>(M_PI)/180;//[rad]
            tarLeft_or.pitch = obj_info_vec.at(19)*static_cast<double>(M_PI)/180;//[rad]
            tarLeft_or.yaw = obj_info_vec.at(20)*static_cast<double>(M_PI)/180;//[rad]
            // position of the engage point
            engage_pos.Xpos = obj_info_vec.at(21)*1000;//[mm]
            engage_pos.Ypos = obj_info_vec.at(22)*1000;//[mm]
            engage_pos.Zpos = obj_info_vec.at(23)*1000;//[mm]
            // orientation of the engage point
            engage_or.roll = obj_info_vec.at(24)*static_cast<double>(M_PI)/180;//[rad]
            engage_or.pitch = obj_info_vec.at(25)*static_cast<double>(M_PI)/180;//[rad]
            engage_or.yaw = obj_info_vec.at(26)*static_cast<double>(M_PI)/180;//[rad]

            Object* ob = new Object(signPrefix,obj_pos,obj_or,obj_size,
                                    new Target(signPrefix + signTarRight,tarRight_pos,tarRight_or),
                                    new Target(signPrefix + signTarLeft,tarLeft_pos,tarLeft_or),
                                    new EngagePoint(signPrefix + signEngage, engage_pos, engage_or));

            infoLine = ob->getInfoLine();
            Q_EMIT newElement(infoLine);
            Q_EMIT newObject(ob->getName());

            //handle of the object
            srv_get_handle.request.objectName = signPrefix;
            client_getHandle.call(srv_get_handle);
            ob->setHandle(srv_get_handle.response.handle);
            // handle of the visible object
            srv_get_handle.request.objectName = signPrefix+string("_body");
            client_getHandle.call(srv_get_handle);
            ob->setHandleBody(srv_get_handle.response.handle);
            // add the object to the scenario
            scene->addObject(objectPtr(ob));
            // add the pose to the scenario
            if(scenarioID == 2 || scenarioID == 4) // Drinking Service scenarios (ARoS, Sawyer)
            {
                Pose* ps = new Pose(signPrefix+string("_home"),tarRight_pos,tarRight_or,true,cnt_obj);
                Q_EMIT newPose(ps->getName());
                scene->addPose(posePtr(ps));
            }

            cnt_obj++;
        }
        else
        {
            throw string("Error while retrieving the objects of the scenario");
        }
    }


    //*************************************************************************************************
    //                               POSES IN THE SCENARIO
    //*************************************************************************************************
    if(scenarioID == 2 || scenarioID == 4) // Drinking Service scenarios (ARoS, Sawyer)
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

        while(cnt_pose < n_poses)
        {
            signPrefix = poses_prefix[cnt_pose];

            add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
            srvs.request.signalName = signPrefix + string("Info");
            add_client.call(srvs);
            if (srvs.response.result == 1)
                pose_info_str = srvs.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the pose");
                succ = false;
            }

            if (succ)
            {
                floatCount = pose_info_str.size()/sizeof(float);

                if(!pose_info_vec.empty())
                    pose_info_vec.clear();
                for (int k=0;k<floatCount;++k)
                    pose_info_vec.push_back(static_cast<double>(((float*)pose_info_str.c_str())[k]));

                // position of the pose
                pose_pos.Xpos = pose_info_vec.at(0)*1000; //[mm]
                pose_pos.Ypos = pose_info_vec.at(1)*1000; //[mm]
                pose_pos.Zpos = pose_info_vec.at(2)*1000; //[mm]
                // orientation of the pose
                pose_or.roll = pose_info_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.pitch = pose_info_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
                pose_or.yaw = pose_info_vec.at(5)*static_cast<double>(M_PI)/180;//[rad]

                Pose* ps = new Pose(signPrefix,pose_pos,pose_or,poses_rel[cnt_pose],poses_obj_id[cnt_pose]);

                Q_EMIT newPose(ps->getName());
                // add the pose to the scenario
                scene->addPose(posePtr(ps));

                cnt_pose++;
            }
            else
            {
                throw string("Error while retrieving the poses of the scenario");
            }
        }
    }


    //*************************************************************************************************
    //                                        ROBOT
    //*************************************************************************************************
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    if(scenarioID == 1 || scenarioID == 2)
        srvs.request.signalName = string("HumanoidName");
    else if (scenarioID >= 3)
        srvs.request.signalName = string("RobotName");
    add_client.call(srvs);
    if (srvs.response.result == 1)
        Hname = srvs.response.signalValue;
    else
    {
        throw string("Error: Couldn't get the name of the robot");
        succ = false;
    }


    //*************************************************************************************************
    //                                     ROBOT ARM (OR ARMS)
    //*************************************************************************************************
    // get the handles of both arms
    if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
        succ = getArmsHandles(0);
    else if(scenarioID >= 3) // Sawyer scenarios (Toy Vehicle, Drinking Service)
        succ = getArmsHandles(2);

    // transformation matrix for the arm
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");

    // ----------------------------------------
    //                  Right arm
    // ----------------------------------------
    srvs.request.signalName = string("mat_right_arm");
    add_client.call(srvs);
    if (srvs.response.result == 1)
        mat_right_arm_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the transformation matrix of the arms");
    }

    if(!mat_right_arm_vec.empty())
        mat_right_arm_vec.clear();
    floatCount = mat_right_arm_str.size()/sizeof(float);
    for (int k=0;k<floatCount;++k)
        mat_right_arm_vec.push_back(static_cast<double>(((float*)mat_right_arm_str.c_str())[k]));

    // ----------------------------------------
    //                  Left arm
    // ----------------------------------------
    if(scenarioID == 1 || scenarioID == 2) // ARoS
    {
        srvs.request.signalName = string("mat_left_arm");
        add_client.call(srvs);
        if (srvs.response.result == 1)
            mat_left_arm_str = srvs.response.signalValue;
        else
        {
            succ = false;
            throw string("Error: Couldn't get the transformation matrix of the arms");
        }

        if(!mat_left_arm_vec.empty())
            mat_left_arm_vec.clear();
        floatCount = mat_left_arm_str.size()/sizeof(float);
        for (int k=0;k<floatCount;++k)
            mat_left_arm_vec.push_back(static_cast<double>(((float*)mat_left_arm_str.c_str())[k]));
    }

    rows=0;
    for(int i=0;i<3;++i){
        for(int j=0;j<4;++j)
        {
            if(i==3 && j<3)
            {
                mat_right(i,j) = 0;
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = 0;
            }
            else if(i==3 && j==3)
            {
                mat_right(i,j) = 1;
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = 1;
            }
            else if(i<3 && j==3)
            {
                mat_right(i,j) = mat_right_arm_vec.at(j+rows*4)*1000; //[mm]
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4)*1000; //[mm]
            }
            else
            {
                mat_right(i,j) = mat_right_arm_vec.at(j+rows*4);
                if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios (Toy Vehicle, Drinking Service)
                    mat_left(i,j) = mat_left_arm_vec.at(j+rows*4);
            }
        }
        ++rows;
    }


    //*************************************************************************************************
    //                                     DH PARAMETERS
    //*************************************************************************************************
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("DH_params_arm");
    add_client.call(srvs);
    if (srvs.response.result == 1)
        DH_params_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the DH parameters of the arm");
    }

    floatCount = DH_params_str.size()/sizeof(float);
    if (!DH_params_vec.empty())
    {
        DH_params_vec.clear();
        theta_offset.clear();
    }
    for (int k=0;k<floatCount;++k)
        DH_params_vec.push_back(static_cast<double>(((float*)DH_params_str.c_str())[k]));

    robot_arm_specs.arm_specs.alpha = std::vector<double>(7);
    robot_arm_specs.arm_specs.a = std::vector<double>(7);
    robot_arm_specs.arm_specs.d = std::vector<double>(7);
    robot_arm_specs.arm_specs.theta = std::vector<double>(7);

    for(int i=0;i<7;++i)
    {
        robot_arm_specs.arm_specs.alpha.at(i) = DH_params_vec.at(i)*static_cast<double>(M_PI)/180; // [rad]
        robot_arm_specs.arm_specs.a.at(i) = DH_params_vec.at(i+7)*1000; // [mm]
        robot_arm_specs.arm_specs.d.at(i) = DH_params_vec.at(i+14)*1000; // [mm]
        theta_offset.push_back(DH_params_vec.at(i+21)*static_cast<double>(M_PI)/180); // [rad]
    }


#if HAND == 0
    //*************************************************************************************************
    //                                     BARRETT HAND
    //*************************************************************************************************

    // ****************** MAX APERTURE
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    srvf.request.signalName = string("maxAperture_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        maxAp = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the maximum aperture");
        succ = false;
    }

    // ****************** Aw
    srvf.request.signalName = string("Aw_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        Aw = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the Aw");
        succ = false;
    }

    // ****************** A1
    srvf.request.signalName = string("A1_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        A1 = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the A1");
        succ = false;
    }

    // ****************** A2
    srvf.request.signalName = string("A2_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        A2 = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the A2");
        succ = false;
    }

    // ****************** A3
    srvf.request.signalName = string("A3_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        A3 = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the A3");
        succ = false;
    }

    // ****************** D3
    srvf.request.signalName = string("D3_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        D3 = srvf.response.signalValue*1000;
    else
    {
        throw string("Error: Couldn't get the information of the D3");
        succ = false;
    }

    // ****************** PH2
    srvf.request.signalName = string("phi2_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        phi2 = srvf.response.signalValue;
    else
    {
        throw string("Error: Couldn't get the information of the phi2");
        succ = false;
    }

    // ****************** PH3
    srvf.request.signalName = string("phi3_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        phi3 = srvf.response.signalValue;
    else
    {
        throw string("Error: Couldn't get the information of the phi3");
        succ = false;
    }
#elif HAND == 1
    //*************************************************************************************************
    //                                  ELETRIC PARALLEL GRIPPER
    //*************************************************************************************************
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    // ****************** A1
    srvf.request.signalName = string("A1_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        A1 = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the A1");
        succ = false;
    }

    // ****************** D3
    srvf.request.signalName = string("D3_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        D3 = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the D3");
        succ = false;
    }

    // ****************** Max Aperture
    srvf.request.signalName = string("maxAperture_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        maxAp = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the maximum aperture");
        succ = false;
    }

    // ****************** Min Aperture
    srvf.request.signalName = string("minAperture_info");
    add_client.call(srvf);
    if (srvf.response.result == 1)
        minAp = srvf.response.signalValue * 1000;
    else
    {
        throw string("Error: Couldn't get the information of the minimum aperture");
        succ = false;
    }
#endif


    //*************************************************************************************************
    //                                          HEAD
    //*************************************************************************************************
#if HEAD == 1
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("HeadInfo");
    add_client.call(srvs);
    if (srvs.response.result == 1)
        head_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the information of the head");
    }

    floatCount = head_str.size()/sizeof(float);
    if (!head_vec.empty())
        head_vec.clear();
    for (int k=0;k<floatCount;++k)
        head_vec.push_back(static_cast<double>(((float*)head_str.c_str())[k]));

    //position of the head
    head.Xpos = head_vec.at(0)*1000;//[mm]
    head.Ypos = head_vec.at(1)*1000;//[mm]
    head.Zpos = head_vec.at(2)*1000;//[mm]
    //orientation of the head
    head.Roll = head_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
    head.Pitch = head_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
    head.Yaw = head_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
    //size of the head
    head.Xsize = head_vec.at(6)*1000;//[mm]
    head.Ysize = head_vec.at(7)*1000;//[mm]
    head.Zsize = head_vec.at(8)*1000;//[mm]
#endif


    //*************************************************************************************************
    //                                          TORSO
    //*************************************************************************************************
    add_client = n.serviceClient<vrep_common::simRosGetStringSignal>("/vrep/simRosGetStringSignal");
    srvs.request.signalName = string("TorsoInfo");
    add_client.call(srvs);
    if (srvs.response.result == 1)
        torso_str = srvs.response.signalValue;
    else
    {
        succ = false;
        throw string("Error: Couldn't get the information of the torso");
    }

    floatCount = torso_str.size()/sizeof(float);
    if (!torso_vec.empty())
        torso_vec.clear();
    for (int k=0;k<floatCount;++k)
        torso_vec.push_back(static_cast<double>(((float*)torso_str.c_str())[k]));

    //position of the torso
    torso.Xpos = torso_vec.at(0)*1000;//[mm]
    torso.Ypos = torso_vec.at(1)*1000;//[mm]
    torso.Zpos = torso_vec.at(2)*1000;//[mm]
    //orientation of the torso
    torso.Roll = torso_vec.at(3)*static_cast<double>(M_PI)/180; //[rad]
    torso.Pitch = torso_vec.at(4)*static_cast<double>(M_PI)/180; //[rad]
    torso.Yaw = torso_vec.at(5)*static_cast<double>(M_PI)/180; //[rad]
    //size of the torso
    torso.Xsize = torso_vec.at(6)*1000;//[mm]
    torso.Ysize = torso_vec.at(7)*1000;//[mm]
    torso.Zsize = torso_vec.at(8)*1000;//[mm]


    //*************************************************************************************************
    //                                  HOME POSTURE AND JOINTS LIMITS
    //*************************************************************************************************
    // ****************** RIGHT ARM
    add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
    // home posture
    for (size_t i = 0; i < rposture.size(); i++)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString());
        add_client.call(srvf);
        if (srvf.response.result == 1)
        {
            rposture.at(i) = srvf.response.signalValue;
        }
        else
        {
            throw string("Error: Couldn't get the information of the right home posture");
            succ = false;
        }
    }

    // minimum limits
    for (size_t i = 0; i < min_rlimits.size(); i++)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_min");
        add_client.call(srvf);
        if (srvf.response.result == 1)
        {
            min_rlimits.at(i) = srvf.response.signalValue;
        }
        else
        {
            throw string("Error: Couldn't get the information of the minimum right limits");
            succ = false;
        }
    }

    // maximum limits
    for (size_t i = 0; i < max_rlimits.size(); i++)
    {
        srvf.request.signalName = string("sright_joint"+QString::number(i).toStdString()+"_max");
        add_client.call(srvf);
        if (srvf.response.result == 1)
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
        // ****************** LEFT ARM
        add_client = n.serviceClient<vrep_common::simRosGetFloatSignal>("/vrep/simRosGetFloatSignal");
        //home posture
        for (size_t i = 0; i < lposture.size(); i++)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString());
            add_client.call(srvf);
            if (srvf.response.result == 1)
                lposture.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the left home posture");
                succ = false;
            }
        }

        // minimum limits
        for (size_t i = 0; i < min_llimits.size(); i++)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_min");
            add_client.call(srvf);
            if (srvf.response.result == 1)
                min_llimits.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the minimum left limits");
                succ = false;
            }
        }

        // maximum limits
        for (size_t i = 0; i < max_llimits.size(); i++)
        {
            srvf.request.signalName = string("sleft_joint"+QString::number(i).toStdString()+"_max");
            add_client.call(srvf);
            if (srvf.response.result == 1)
                max_llimits.at(i)= srvf.response.signalValue;
            else
            {
                throw string("Error: Couldn't get the information of the maximum left limits");
                succ = false;
            }
        }
    }


    //*************************************************************************************************
    //                                  CREATE ROBOT
    //*************************************************************************************************
    if (succ)
    {
        // ****************** INFO OF THE TORSO
        robot_torso_specs.Xpos = torso.Xpos;
        robot_torso_specs.Ypos = torso.Ypos;
        robot_torso_specs.Zpos = torso.Zpos;
        robot_torso_specs.Roll = torso.Roll;
        robot_torso_specs.Pitch = torso.Pitch;
        robot_torso_specs.Yaw = torso.Yaw;
        robot_torso_specs.Xsize = torso.Xsize;
        robot_torso_specs.Ysize = torso.Ysize;
        robot_torso_specs.Zsize = torso.Zsize;
#if HEAD == 1
        // ****************** INFO OF THE HEAD
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
        // ****************** BARRETT HAND
        robot_hand_specs.maxAperture = maxAp;
        robot_hand_specs.Aw = Aw;
        robot_hand_specs.A1 = A1;
        robot_hand_specs.A2 = A2;
        robot_hand_specs.A3 = A3;
        robot_hand_specs.D3 = D3 ;
        robot_hand_specs.phi2 = phi2;
        robot_hand_specs.phi3 = phi3;

        //add the joints offset
        std::transform(rposture.begin(), rposture.end(), theta_offset.begin(), rposture.begin(), std::plus<double>());
        if (scenarioID == 1 || scenarioID == 2) // ARoS
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
        // **************************** TRANSFORMATION MATRICES
        rptr->setMatRight(mat_right);
        if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios
            rptr->setMatLeft(mat_left);
        else if(scenarioID >= 3) // Sawyer scenarios
            rptr->setMatLeft(mat_right);

        // **************************** RIGHT JOINTS
        std::vector<double> rightp;
        rptr->getRightPosture(rightp);

        // without offsets
        std::transform(rightp.begin(), rightp.end(),theta_offset.begin(), rightp.begin(), std::minus<double>());

        std::vector<string> rj = std::vector<string>(rightp.size());
        for (size_t i=0; i<rightp.size(); i++ )
        {
            rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                              QString::number(rightp.at(i)*180/static_cast<double>(M_PI)).toStdString() + " [deg]");
            Q_EMIT newJoint(rj.at(i));
        }

        // **************************** LEFT JOINTS
        if(scenarioID == 1 || scenarioID == 2) // ARoS scenarios
        {
            std::vector<double> leftp;
            rptr->getLeftPosture(leftp);

            //without offsets
            std::transform(leftp.begin(), leftp.end(), theta_offset.begin(), leftp.begin(), std::minus<double>());

            std::vector<string> lj = std::vector<string>(leftp.size());
            for (size_t i=0; i<leftp.size(); i++ )
            {
                lj.at(i) = string("left_joint "+ QString::number(i+1).toStdString()+ ": "+
                                  QString::number(leftp.at(i)*180/static_cast<double>(M_PI)).toStdString() + " [deg]");
                Q_EMIT newJoint(lj.at(i));
            }
        }
        else if(scenarioID >= 3) // Sawyer scenarios
            rptr->getLeftPosture(rightp);

        // **************************** CREATE ROBOT
        // display info of the robot
        infoLine = rptr->getInfoLine();
        Q_EMIT newElement(infoLine);
        // create robot
        scene->addRobot(robotPtr(rptr));
#elif HAND == 1
        // ****************** ELECTRIC PARALLEL GRIPPER
        robot_gripper_specs.maxAperture = maxAp;
        robot_gripper_specs.minAperture = minAp;
        robot_gripper_specs.A1 = A1;
        robot_gripper_specs.D3 = D3;

        //add the joints offset
        std::transform(rposture.begin(), rposture.end(), theta_offset.begin(), rposture.begin(), std::plus<double>());
        lposture = rposture;
        min_llimits = min_rlimits;
        max_llimits = max_rlimits;
#if HEAD == 1
        Robot *rptr = new Robot(Hname, robot_torso_specs, robot_arm_specs, robot_gripper_specs,
                                robot_head_specs, rposture, lposture,
                                min_rlimits, max_rlimits,
                                min_llimits, max_llimits);
#endif
        // **************************** TRANSFORMATION MATRICES
        rptr->setMatRight(mat_right);
        rptr->setMatLeft(mat_right);

        // **************************** RIGHT JOINTS
        std::vector<double> rightp;
        rptr->getRightPosture(rightp);

        // without offsets
        std::transform(rightp.begin(), rightp.end(),theta_offset.begin(), rightp.begin(), std::minus<double>());

        std::vector<string> rj = std::vector<string>(rightp.size());
        for (size_t i = 0; i < rightp.size(); i++)
        {
            if (i < rightp.size() - 1)
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                  QString::number(rightp.at(i) * 180/ static_cast<double>(M_PI)).toStdString() + " [deg]");
            else
                rj.at(i) = string("right_joint "+ QString::number(i+1).toStdString()+ ": "+
                                  QString::number(rightp.at(i)).toStdString() + " [mm]");

            Q_EMIT newJoint(rj.at(i));
        }

        rptr->getLeftPosture(rightp);

        // **************************** CREATE ROBOT
        // display info of the robot
        infoLine = rptr->getInfoLine();
        Q_EMIT newElement(infoLine);
        // create robot
        scene->addRobot(robotPtr(rptr));
#endif
    }
    else
    {
        throw string("Error while retrieving elements from the scenario");
    }

    this->curr_scene = scene;

    // stop the simulation
    add_client = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvcc;
    add_client.call(srvcc);

    // we got all the elements of the scenario
    got_scene = true;

    return succ;
}


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


void QNode::Cup_shelfCallback(const geometry_msgs::PoseStamped& data)
{
    int obj_id = 0;
    string name = string("Cup");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::ShelfCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 1;
    string name = string("Shelf");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_1_bCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 2;
    string name = string("Shelf_1_b");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_2_aCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 3;
    string name = string("Shelf_2_a");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_2_bCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 4;
    string name = string("Shelf_2_b");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_3Callback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 5;
    string name = string("Shelf_3");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_4_aCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 6;
    string name = string("Shelf_4_a");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_4_bCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 7;
    string name = string("Shelf_4_b");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_4_cCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 8;
    string name = string("Shelf_4_c");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::Shelf_4_dCallback(const geometry_msgs::PoseStamped &data)
{
    int obj_id = 9;
    string name = string("Shelf_4_d");

    this->updateObjectInfo(obj_id,name,data);
}


void QNode::updateObjectInfo(int obj_id, string name, const geometry_msgs::PoseStamped &data)
{
    std::vector<double> rpy;
    objectPtr obj = this->curr_scene->getObject(name);

    // position
    pos poss;
    poss.Xpos = data.pose.position.x * 1000; //[mm]
    poss.Ypos = data.pose.position.y * 1000; //[mm]
    poss.Zpos = data.pose.position.z * 1000; //[mm]

    obj->setPos(poss,true);

    // orientation
    orient orr;
    // get the quaternion
    double epx = data.pose.orientation.x;
    double epy = data.pose.orientation.y;
    double epz = data.pose.orientation.z;
    double w = data.pose.orientation.w;

    Matrix3d Rot;
    Rot(0,0) = 2*(pow(w,2)+pow(epx,2))-1; Rot(0,1) = 2*(epx*epy-w*epz);         Rot(0,2) = 2*(epx*epz+w*epy);
    Rot(1,0) = 2*(epx*epy+w*epz);         Rot(1,1) = 2*(pow(w,2)+pow(epy,2))-1; Rot(1,2) = 2*(epy*epz-w*epx);
    Rot(2,0) = 2*(epx*epz-w*epy);         Rot(2,1) = 2*(epy*epz+w*epx);         Rot(2,2) = 2*(pow(w,2)+pow(epz,2))-1;

    Matrix4d trans_obj;
    trans_obj(0,0) = Rot(0,0); trans_obj(0,1) = Rot(0,1); trans_obj(0,2) = Rot(0,2); trans_obj(0,3) = poss.Xpos;
    trans_obj(1,0) = Rot(1,0); trans_obj(1,1) = Rot(1,1); trans_obj(1,2) = Rot(1,2); trans_obj(1,3) = poss.Ypos;
    trans_obj(2,0) = Rot(2,0); trans_obj(2,1) = Rot(2,1); trans_obj(2,2) = Rot(2,2); trans_obj(2,3) = poss.Zpos;
    trans_obj(3,0) = 0;        trans_obj(3,1) = 0;        trans_obj(3,2) = 0;        trans_obj(3,3) = 1;

    if (this->getRPY(trans_obj,rpy))
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


bool QNode::getRPY(Matrix4d Trans, std::vector<double> &rpy)
{
    rpy = std::vector<double>(3);

    if((abs(Trans(0,0)) < 1e-5) && (abs(Trans(1,0)) < 1e-5))
    {
        // singularity
        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

        return false;
    }
    else
    {
        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        double sp = sin(rpy.at(0));
        double cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

        return true;
    }
}


void QNode::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty()){
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);
    }
}


void QNode::infoCallback(const vrep_common::VrepInfoConstPtr& info)
{
    simulationTime = info->simulationTime.data;
    simulationTimeStep = info->timeStep.data;
    simulationRunning = (info->simulatorState.data&1)!=0;
}


void QNode::rightProxCallback(const vrep_common::ProximitySensorData& data)
{
    if (this->curr_mov)
    {
        int arm_code = this->curr_mov->getArm();

        if (arm_code == 1)
        {
            int h_obj;
            int h_obj_body;
            int mov_type = this->curr_mov->getType();

            switch (mov_type)
            {
            case 0: // reach-to-grasp
                h_obj_body = this->curr_mov->getObject()->getHandleBody(); // visible handle of the object we want to grasp
                h_obj = this->curr_mov->getObject()->getHandle(); // non visible handle of the object we want to grasp
                h_detobj = data.detectedObject.data; // handle of the object currently detected
                obj_in_hand = (h_obj == h_detobj) || (h_obj_body == h_detobj);
                break;
            case 1: // reaching
                break;
            case 2: // transport
                break;
            case 3: // engage
                break;
            case 4: // disengage
                break;
            case 5: // go park
                break;
            }
        }
    }
}


void QNode::leftProxCallback(const vrep_common::ProximitySensorData& data)
{
}


vector<MatrixXd> QNode::realJointsPosition(std::vector<MatrixXd>& traj_mov)
{
    for (size_t k=0; k< traj_mov.size();++k)
    {
        MatrixXd traj = traj_mov.at(k);
        RowVectorXd aux_theta_off = VectorXd::Map(theta_offset.data(), traj.cols());

        MatrixXd theta_off(traj.rows(), traj.cols());
        for(int i=0; i<traj.rows(); ++i)
            theta_off.row(i) << aux_theta_off;

        traj = traj - theta_off;
        traj_mov.at(k) = traj;
    }

    return traj_mov;
}


bool QNode::execMovement(std::vector<MatrixXd>& traj_mov, std::vector<MatrixXd>& vel_mov, std::vector<std::vector<double>> timesteps, std::vector<double> tols_stop, std::vector<string>& traj_descr,movementPtr mov, scenarioPtr scene)
{
    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                               Initialization of variables                                                //
    // ------------------------------------------------------------------------------------------------------------------------ //
    //scenario
    this->curr_scene = scene;
    int scenarioID = scene->getID();
    //movement
    this->curr_mov = mov;
    int mov_type = mov->getType();
    //arm code
    int arm_code = mov->getArm();
    //movement settings
    bool plan;
    bool approach;
    bool retreat;
    //Ros communication
    ros::NodeHandle node;
    //Time steps
    double ta;
    double tb = 0.0;
    double tx;
    double pre_time = 0.0;
    std::vector<double> timesteps_stage;
    //Final posture
    VectorXd f_posture;
    bool f_reached;
    double tol_stop_stage;
    //Planned movement
    MatrixXd traj;
    MatrixXd vel;
    double timeTot = 0.0;
    //Plan and Approach stages
    MatrixXd traj_plan_approach;
    MatrixXd vel_plan_approach;
    std::vector<double> timesteps_plan_approach;
    bool join_plan_approach = false;
    //Handles
    std::vector<int> handles;
    //Hand
#if HAND == 0
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS, N_PHALANGE + 1, 1); // matrix fingers x (phalanges + 1) with all elements set as 1
#endif
    int h_attach; //attachment point
    bool hand_closed;

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                       Subscribers                                                        //
    // ------------------------------------------------------------------------------------------------------------------------ //
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


    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                         Handles                                                          //
    // ------------------------------------------------------------------------------------------------------------------------ //
    switch (arm_code)
    {
    case 0: // dual arm
        // TODO
        break;
    case 1: //right arm
        handles = right_handles;
        h_attach = right_attach;
#if HAND == 0
        hand_handles = right_hand_handles;
#endif
        break;
    case 2: // left arm
        handles = left_handles;
        h_attach = left_attach;
#if HAND == 0
        hand_handles = left_hand_handles;
#endif
        break;
    }


    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                  HAND OPENED OR CLOSED                                                   //
    // ------------------------------------------------------------------------------------------------------------------------ //
    switch (mov_type)
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
        closed = false;
#endif
        break;
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                         Approach Stage (Join to the plan stage)                                          //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // reach-to-grasp, transport, engage, disengage
    if(mov_type == 0 || mov_type == 2 || mov_type == 3 || mov_type == 4)
    {
        if(traj_mov.size() > 1)
        {
            // plan stage
            string mov_descr_1 = traj_descr.at(0);
            // approach stage
            string mov_descr_2 = traj_descr.at(1);

            if((strcmp(mov_descr_1.c_str(), "plan") == 0) && (strcmp(mov_descr_2.c_str(), "approach") == 0))
            {
                // the movements will be joined, we need to resize the matrix
                join_plan_approach = true;
                traj_plan_approach.resize((traj_mov.at(0).rows() + traj_mov.at(1).rows() - 1), traj_mov.at(0).cols());
                vel_plan_approach.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows() - 1), vel_mov.at(0).cols());
            }
        }
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                    Start Simulation                                                      //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // start the simulation
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);
    ros::spinOnce(); // first handle ROS messages

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                   Execute Movement                                                       //
    // ------------------------------------------------------------------------------------------------------------------------ //
    //the trajectory obtained doesn't include the joints offsets
    std::vector<MatrixXd> traj_mov_w_offset;
    traj_mov_w_offset = traj_mov;
    //add the joints offsets
    std::vector<MatrixXd> traj_mov_real;
    traj_mov_real = realJointsPosition(traj_mov_w_offset);


    for(size_t k = 0; k < traj_mov_real.size(); ++k)
    {
        string mov_descr = traj_descr.at(k);

        // ********************************************************************** //
        //                        Trajectory to be executed                       //
        // ********************************************************************** //
        // ***** Compile the information about the movement to be executed in the differents stages
        if(strcmp(mov_descr.c_str(),"plan") == 0)
        {
            plan = true;
            approach = false;
            retreat = false;

            if(join_plan_approach)
            {
                MatrixXd tt = traj_mov_real.at(k);
                MatrixXd vv = vel_mov.at(k);
                std::vector<double> ttsteps = timesteps.at(k);

                // PLAN STAGE: add the trajectory, velocity and time steps in TOP of matrix
                traj_plan_approach.topLeftCorner(tt.rows(),tt.cols()) = tt;
                vel_plan_approach.topLeftCorner(vv.rows(),vv.cols()) = vv;
                timesteps_plan_approach.reserve(ttsteps.size());

                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
                continue;
            }
        }
        else if(strcmp(mov_descr.c_str(),"approach") == 0)
        {
            plan = false;
            approach = true;
            retreat = false;

            if(join_plan_approach)
            {
                MatrixXd tt = traj_mov_real.at(k);
                MatrixXd tt_red = tt.bottomRows(tt.rows() - 1);
                MatrixXd vv = vel_mov.at(k);
                MatrixXd vv_red = vv.bottomRows(vv.rows() - 1);
                std::vector<double> ttsteps = timesteps.at(k);

                // APPROACH STAGE: add the trajectory, velocity and time steps in BOTTOM of matrix
                traj_plan_approach.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                vel_plan_approach.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                timesteps_plan_approach.reserve(ttsteps.size());

                std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_plan_approach));
            }
        }
        else if(strcmp(mov_descr.c_str(),"retreat") == 0)
        {
            plan = false;
            approach = false;
            retreat = true;
        }


        // ***** Check if the object is in the robot's hand and if it has been grasped correctly
        switch (mov_type)
        {
        case 0: // reach-to-grasp
            if(retreat)
            {
                if(obj_in_hand)
                {
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent;
                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                    srvset_parent.request.parentHandle = h_attach;
                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                    add_client.call(srvset_parent);
                    if (srvset_parent.response.result != 1)
                        log(QNode::Error,string("Error in grasping the object "));

#if HAND == 0 && OPEN_CLOSE_HAND == 1
                    this->closeBarrettHand(arm_code);
#elif HAND == 0 && OPEN_CLOSE_HAND == 0
                    MatrixXd tt = traj_mov_real.at(k);
                    VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                    std::vector<double> hand_init_pos;
                    hand_init_pos.resize(init_h_posture.size());
                    VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;

                    //Close the barrett hand
                    this->closeBarrettHand_to_pos(arm_code, hand_init_pos);
#elif HAND == 1
                    closed = true;
#endif
                }
            }
            break;
        case 1: // reaching
            break;
        case 2: case 3: // transport, engage
            if(retreat)
            {
                if(std::strcmp(mov->getObject()->getName().c_str(),"") != 0)
                {
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent;
                    srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                    srvset_parent.request.parentHandle = -1;
                    srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                    add_client.call(srvset_parent);
                    if (srvset_parent.response.result != 1)
                        log(QNode::Error,string("Error in releasing the object "));
                }

#if HAND == 0 && OPEN_CLOSE_HAND == 1
                MatrixXd tt = traj_mov_real.at(k);
                VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                std::vector<double> hand_init_pos;
                hand_init_pos.resize(init_h_posture.size());
                VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;

                //open the barrett hand
                this->openBarrettHand_to_pos(arm_code, hand_init_pos);
#elif HAND == 0 && OPEN_CLOSE_HAND == 0
                closed.at(0) = false;
                closed.at(1) = false;
                closed.at(2) = false;
#elif HAND == 1
                closed = false;
#endif
            }
            break;
        case 4:// disengage
            break;
        case 5: // go-park
            break;
        }


        // ***** Save the information about the movement to be executed in the differents stages
        //          Information needed:
        //              -> time steps
        //              -> joints trajectory (position)
        //              -> joints velocity
        //              -> final posture to be reached
        //              -> tolerances to stop the movement

        if(join_plan_approach && (strcmp(mov_descr.c_str(),"approach") == 0))
        {
            traj = traj_plan_approach;
            vel = vel_plan_approach;
            timesteps_stage = timesteps_plan_approach;
        }
        else
        {
            traj = traj_mov_real.at(k);
            vel = vel_mov.at(k);
            timesteps_stage = timesteps.at(k);
        }

        tol_stop_stage = tols_stop.at(k);
        f_posture = traj.row(traj.rows()-1);
        f_reached = false;


#if HAND == 0
        if ((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1) &&
            (client_enableSubscriber_hand.call(srv_enableSubscriber_hand)) && (srv_enableSubscriber_hand.response.subscriberID != -1))
#elif HAND == 1
        if ((client_enableSubscriber.call(srv_enableSubscriber)) && (srv_enableSubscriber.response.subscriberID != -1))
#endif
        {
            // ***** V-REP is now listening to the desired values
            // Arm joints publisher
            ros::Publisher pub = node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints", 1);
            // Fingers joints publisher
#if HAND == 0
            ros::Publisher pubHand = node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand", 1);
#endif
            ros::spinOnce();

            // Total time of the movement
            pre_time = simulationTime - timeTot;
            tb = pre_time;

            for (int i = 0; i < vel.rows() - 1; ++i)
            {
                bool interval = true;
                double tx_prev;
                double yxt_prev;

                // ********************************************************************** //
                //                            Steps information                           //
                // ********************************************************************** //

                // **** For each step of the planned movement (each stage is divided into several steps),
                //          Get the current values of:
                //              -> position
                //              -> velocity
                VectorXd ya = vel.row(i);
                VectorXd yat = traj.row(i);
                //          Get the next values of:
                //              -> position
                //              -> velocity
                VectorXd yb = vel.row(i + 1);
                VectorXd ybt = traj.row(i + 1);

                // Get the time associated to the previous step of the planned movement
                ta = tb;
                // Get the values of time_step
                double tt_step = timesteps_stage.at(i);

                if(tt_step < 0.001)
                    tt_step = MIN_EXEC_TIMESTEP_VALUE;

                // Get the time associated to the current step of the planned movement
                tb = ta + tt_step;


                while (ros::ok() && simulationRunning && interval)
                {
                    // Data to be published in the topic "/motion_manager/set_joints"
                    vrep_common::JointSetStateData dataTraj;
#if HAND == 0
                    vrep_common::JointSetStateData dataHand;
#endif

                    // Total time of the movement
                    tx = simulationTime - timeTot;

                    // Checks if the total time is greater than planned
                    if (tx > tb)
                        interval = false;
                    else
                    {
                        // ********************************************************************** //
                        //                       Joints linear interpolation                      //
                        // ********************************************************************** //
                        double m;

                        // Linear interpolation depends on the value of m
                        // m is determined by the following formula: (x - x0) / (x1 - x0)
                        // In this case x is the time (in sec)
                        if((tb - ta) == 0)
                            m = 1;
                        else
                            // ta == time associated with the start of the current step
                            // tb == time associated with the start of the next step
                            // tx == curr total time of the movement
                            m = (tx - ta) / (tb - ta);

                        // Gets right posture of the robot
                        std::vector<double> r_post;
                        this->curr_scene->getRobot()->getRightPosture(r_post);

                        // Checks if the final posture has been reached, taking into account the tolerances
                        if(sqrt(pow((f_posture(0) - r_post.at(0)), 2) +
                                pow((f_posture(1) - r_post.at(1)), 2) +
                                pow((f_posture(2) - r_post.at(2)), 2) +
                                pow((f_posture(3) - r_post.at(3)), 2) +
                                pow((f_posture(4) - r_post.at(4)), 2) +
                                pow((f_posture(5) - r_post.at(5)), 2) +
                                pow((f_posture(6) - r_post.at(6)), 2)) < tol_stop_stage)
                        {
                            // if the final posture was reached in the previous movement: stop robot
                            f_reached = true;
                            break;
                        }
                        else
                            f_reached = false;

                        double yx;
                        double yxt;

                        for (int k = 0; k < vel.cols(); ++k)
                        {
                            // Checks if the final posture has been reached
                            if(f_reached)
                            {
                                // Get the joints velocity at the beginning of the current micro step
                                yx = 0;
                                // Get the joints position at the beginning of the current micro step
                                yxt = yxt_prev;
                            }
                            else
                            {
                                // Linear interpolation of the joints velocity
                                yx = interpolate(ya(k), yb(k), m);
                                // Linear interpolation of the joints position
                                yxt = interpolate(yat(k), ybt(k), m);
                                // Save the joints position at the beginning of the current micro step
                                yxt_prev = yxt;
                            }


#if HAND == 0
                            hand_closed = (closed[0] && closed[1] && closed[2]);
#elif HAND == 1
                            hand_closed = closed;
#endif
                            // Get and save the handles of arm joints
#if HAND == 0
                            if(((k != vel.cols() - 1) && (k != vel.cols() - 2) && ( k != vel.cols() - 3) && (k != vel.cols() - 4)) ||
                                    (((k == vel.cols() - 1) || (k == vel.cols() - 2) || (k == vel.cols() - 3) || (k == vel.cols() - 4)) && !hand_closed))
#elif HAND == 1
                            if((k != vel.cols() - 1) || ((k == vel.cols() - 1) && !hand_closed))
#endif
                                dataTraj.handles.data.push_back(handles.at(k));


                            // ********************************************************************** //
                            //                              Execution mode                            //
                            // ********************************************************************** //
                            int exec_mode;
                            double exec_value;
#if VEL == 0
                            // Execution mode: POSITION
                            exec_mode = 0;
                            // joints position
                            exec_value = yxt;
#elif VEL == 1
                            // Execution mode: VELOCITY
                            exec_mode = 2;
                            // joints velocity
                            exec_value = yx;
#endif

                            switch(scenarioID)
                            {
                            case 0:
                                break;
                                // scenarios:
                                //  -> Toy Vehicle with AROS, Sawyer with BarrettHand and Sawyer with Gripper
                                //  -> Driking Service  with AROS, Sawyer with BarrettHand and Sawyer with Gripper
                            case 1: case 2: case 3: case 4: case 5:
                                // **** Send the execution mode
                                //          Values:
                                //              -> 0 to set the POSITION
                                //              -> 1 to set the TARGET POSITION
                                //              -> 1 to set the TARGET VELOCITY
#if HAND == 0
                                if(((k != vel.cols() - 1) && (k != vel.cols() - 2) && ( k != vel.cols() - 3) && (k != vel.cols() - 4)))
#elif HAND == 1
                                if(k != vel.cols() - 1)
#endif
                                    dataTraj.setModes.data.push_back(exec_mode);
#if HAND == 0
                                else if((((k == vel.cols() - 1) || (k == vel.cols() - 2) || ( k == vel.cols() - 3) || (k == vel.cols() - 4))) && !hand_closed)
#elif HAND == 1
                                else if((k == vel.cols() - 1) && !hand_closed)
#endif
                                    dataTraj.setModes.data.push_back(1);



                                // **** Send the joints values
#if HAND == 0
                                if(((k != vel.cols() - 1) && (k != vel.cols() - 2) && ( k != vel.cols() - 3) && (k != vel.cols() - 4)))
#elif HAND == 1
                                if(k != vel.cols() - 1)
#endif
                                    dataTraj.values.data.push_back(exec_value);
#if HAND == 0
                                else if((((k == vel.cols() - 1) || (k == vel.cols() - 2) || ( k == vel.cols() - 3) || (k == vel.cols() - 4))) && !hand_closed)
#elif HAND == 1
                                else if((k == vel.cols() - 1) && !hand_closed)
#endif
                                    dataTraj.values.data.push_back(yxt);
                                break;
                            }

#if HAND == 0
                            if(((k == vel.cols() - 1) || (k == vel.cols() - 2) || (k == vel.cols() - 3)) && !hand_closed)
                            {
                                // the fingers are being addressed
                                dataHand.handles.data.push_back(hand_handles(k + 3 - vel.cols(), 2));
                                dataHand.setModes.data.push_back(1); // set the target position
                                dataHand.values.data.push_back(yxt / 3.0 + 45.0f * static_cast<double>(M_PI) / 180.0f);
                            }
#endif
                        }

                        // Publish the data in the topic "/motion_manager/set_joints"
                        pub.publish(dataTraj);
#if HAND == 0
                        pubHand.publish(dataHand);
#endif

                        interval = true;
                        tx_prev = tx;
                    }

                    // handle ROS messages:
                    ros::spinOnce();
                }

                if(f_reached)
                {
                    log(QNode::Info,string("Final posture reached."));
                    break;
                }
            }


            // ------------------------------------------------------------------------------------------------------------------------ //
            //                                               Post-Movement operations                                                   //
            // ------------------------------------------------------------------------------------------------------------------------ //
            switch (mov_type)
            {
            case 0: // reach-to grasp
                if(approach || (plan && (traj_mov_real.size() == 1)))
                {
                    if(obj_in_hand)
                    {
                        add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                        vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                        srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                        srvset_parent.request.parentHandle = h_attach;
                        srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                        add_client.call(srvset_parent);

                        if (srvset_parent.response.result != 1)
                            log(QNode::Error,string("Error in grasping the object "));
                    }
                }
                break;
            case 1: // reaching
                break;
            case 2: // transport
                break;
            case 3: // engage
                break;
            case 4: // disengage
                break;
            case 5: // go park
                break;
            }
        }


        // Handle ROS messages:
        ros::spinOnce();
        // Update the total time of the movement
        timeTot = simulationTime;
    }


    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                    Pause Simulation                                                      //
    // ------------------------------------------------------------------------------------------------------------------------ //
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);

    // Handle ROS messages:
    ros::spinOnce();
    // Update the total time of the movement
    TotalTime = simulationTime;

    log(QNode::Info,string("Movement completed"));
    mov->setExecuted(true);

    return true;
}


#if ROBOT == 1
bool QNode::execMovement_Sawyer(std::vector<MatrixXd>& traj_mov, std::vector<MatrixXd>& vel_mov, std::vector<MatrixXd>& acc_mov, std::vector<std::vector<double>> timesteps)
{
    ros::NodeHandle node;
    bool homePostureEqual;

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                        PUBLISHERS                                                        //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // Topics to enable/disable the movement of robot joints
    pubEnable_robot = node.advertise<std_msgs::Bool>("/robot/set_super_enable", 1);

    // Enables the robot before attempting to control any of the motors
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    pubEnable_robot.publish(enable_msg);

    // Handle ROS messages
    ros::spinOnce();

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                 MOVE TO THE INITIAL POSTURE: MOTION CONTROLLER INTERFACE                                 //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // Get the trajectory planned by the HUMP
    vector<MatrixXd> traj_mov_w_offset = traj_mov;
    // The trajectory planned doesn't include the joints offsets, so it's necessary to add these values.
    vector<MatrixXd> traj_mov_real = realJointsPosition(traj_mov_w_offset);

    // Get the initial position of the robot arm in the V-REP simulator
    MatrixXd traj = traj_mov_real.at(0);
    VectorXd iP = traj.row(0);
    vector<double> simulationPosture(&iP[0], iP.data() + (iP.cols() * iP.rows() - JOINTS_HAND));

    // Calculate the difference between the initial posture in simulation and the current posture of the robot
    vector<double> diff;
    for(int i = 0; i < JOINTS_ARM; ++i)
        diff.push_back(simulationPosture.at(i) - robotPosture.at(i));

    // Get the highest value in the vector with differences between the joints
    double max_diff = *std::max_element(diff.begin(), diff.end());
    // Clear the diff vector
    diff.clear();
    // Position threshold in radians across each joint when move is considered successful
    double tol_maxDiff = 0.005; // 0.005 rad => 0.28647889757º

    if(max_diff < tol_maxDiff)
        // ******************************************************************************* //
        //                              The postures are equal                             //
        // ******************************************************************************* //
        homePostureEqual = true;
    else
    {
        // ******************************************************************************* //
        //       The postures are different: Moving the robot to the initial posture       //
        // ******************************************************************************* //
        log(QNode::Info, string("moving the Sawyer robot to the initial posture... "));

        // For joint trajectories, we specify the maximum value of speed and acceleration per joint
        intera_motion_msgs::WaypointOptions wayPointOptions;
        wayPointOptions.max_joint_speed_ratio = 0.1; // speed: values defined between 0.01 and 1
        for(int i = 0; i < JOINTS_ARM; ++i)
            wayPointOptions.max_joint_accel.push_back(0.05); // acceleration: values defined between 0.001 and 1

        // Define the first waypoint of the trajectory - initial posture of the robot
        intera_motion_msgs::Waypoint initialPoint;
        initialPoint.options = wayPointOptions;
        initialPoint.joint_positions = robotPosture;
        // Define the last waypoint of the trajectory - desired posture = initial posture in simulator
        intera_motion_msgs::Waypoint finalPoint;
        finalPoint.options = wayPointOptions;
        finalPoint.joint_positions = simulationPosture;

        // Define trajectory to be planned and executed. This has to pass through all waypoints defined previously
        // The motion controller supports two basic methods for interpolation between waypoints: JOINT and CARTESIAN mode
        intera_motion_msgs::Trajectory trajectory;
        trajectory.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
        trajectory.waypoints.push_back(initialPoint);
        trajectory.waypoints.push_back(finalPoint);
        trajectory.trajectory_options.interpolation_type = "JOINT";

        // Define the goal message
        intera_motion_msgs::MotionCommandGoal newStartPosture;
        newStartPosture.command = newStartPosture.MOTION_START;
        newStartPosture.trajectory = trajectory;

        // Send the goal message to the action server "/motion/motion_command"
        motionComm->sendGoal(newStartPosture);
        // After 30 sec the function return false, if the goal hasn't reached
        motionComm->waitForResult(ros::Duration(45));

        if(motionComm->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // The initial posture was reached
            log(QNode::Info,string("Initial posture reached."));
            homePostureEqual = true;
        }
        else
        {
            // It isn't possible to reach the desired posture. We can check the list of possible errors which are
            // returned from action server in the result message
            log(QNode::Error, string("Error in reaching the initial posture of the robot."));
            homePostureEqual = false;
            return false;
        }

        // Sleep for the specified number of seconds
        sleep(5);
    }

    // Handle ROS messages
    ros::spinOnce();

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                               EXECUTE THE PLANNED TRAJECTORY: JOINT TRAJECTORY ACTION SERVER                             //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // If Sawyer robot is in its initial posture: the execution of the planned movements begins
    if(homePostureEqual)
    {
        // Position, velocity and acceleration of the arm along the planned movement
        vector<vector<double>> pos_arm;
        vector<vector<double>> vel_arm;
        vector<vector<double>> acc_arm;
        // The obtained values must be reached a few seconds after the start of the movement
        vector<double> timeFromStart;
        timeFromStart.push_back(0.0);
        // Number of steps to be executed along the planned movement
        int nTotalSteps = 1;

        for(size_t k = 0; k < traj_mov_real.size(); ++k)
        {
            // ************************************************************************** //
            //                             Stages information                             //
            // ************************************************************************** //
            // For each stage of the planned movement ("Plan", "Approach" or "Retreat"),
            // we get the values of: position, velocity, aceleration and time_steps
            MatrixXd pos_stage = traj_mov_real.at(k);
            MatrixXd vel_stage = vel_mov.at(k);
            MatrixXd acc_stage = acc_mov.at(k);
            vector<double> timesteps_stage = timesteps.at(k);

            for(int kk = 0; kk < pos_stage.rows() - 1; ++kk)
            {
                // ********************************************************************** //
                //                           Steps information                            //
                // ********************************************************************** //
                // For each step of the planned movement (each stage is divided into several steps),
                // Get the current values of: position, velocity, aceleration
                VectorXd pos_step_curr = pos_stage.row(kk);
                VectorXd vel_step_curr = vel_stage.row(kk);
                VectorXd acc_step_curr = acc_stage.row(kk);
                // Get the next values of: position, velocity, aceleration
                VectorXd pos_step_next = pos_stage.row(kk + 1);
                VectorXd vel_step_next = vel_stage.row(kk + 1);
                VectorXd acc_step_next = acc_stage.row(kk + 1);

                // Get only the position, velocity and acceleration of the robot arm joints
                // Current step
                vector<double> pos_arm_curr(&pos_step_curr[0], pos_step_curr.data() + (pos_step_curr.cols() * pos_step_curr.rows() - JOINTS_HAND));
                vector<double> vel_arm_curr(&vel_step_curr[0], vel_step_curr.data() + (vel_step_curr.cols() * vel_step_curr.rows() - JOINTS_HAND));
                vector<double> acc_arm_curr(&acc_step_curr[0], acc_step_curr.data() + (acc_step_curr.cols() * acc_step_curr.rows() - JOINTS_HAND));
                // Get only the position, velocity and acceleration of the robot arm joints
                // Next step
                vector<double> pos_arm_next(&pos_step_next[0], pos_step_next.data() + (pos_step_next.cols() * pos_step_next.rows() - JOINTS_HAND));
                vector<double> vel_arm_next(&vel_step_next[0], vel_step_next.data() + (vel_step_next.cols() * vel_step_next.rows() - JOINTS_HAND));
                vector<double> acc_arm_next(&acc_step_next[0], acc_step_next.data() + (acc_step_next.cols() * acc_step_next.rows() - JOINTS_HAND));

                // ********************************************************************** //
                //                       Joints linear interpolation                      //
                // ********************************************************************** //
                // Adds the position, velocity and acceleration obtained for the first step in plan stage
                if(k == 0 && kk == 0)
                {
                    pos_arm.push_back(pos_arm_curr);
                    vel_arm.push_back(vel_arm_curr);
                    acc_arm.push_back(acc_arm_curr);
                }

                // Divide each step into several micro steps (1s correspond to 5 MicroStep)
                int microSteps = (int)round(timesteps_stage.at(kk) * 5.0);
                // Determine the time associated with execution of each microSteps
                double t_inc = timesteps_stage.at(kk) / microSteps;

                for(int n = 1; n <= microSteps; ++n)
                {
                    // Position, velocity and acceleration for each micro step of the planned movement
                    vector<double> pos_arm_microSteps;
                    vector<double> vel_arm_microSteps;
                    vector<double> acc_arm_microSteps;

                    // Each micro step starts at 0 sec and ends after the time step determined by the HUMP
                    double t_curr = 0.0;
                    double t_next = timesteps_stage.at(kk);

                    // Linear interpolation depends on the value of m
                    // m is determined by the following formula: (x - x0) / (x1 - x0)
                    // In this case x is the time (in sec)
                    double m = ((n * t_inc) - t_curr) / (t_next - t_curr);

                    for(int i = 0; i < JOINTS_ARM; ++i)
                    {
                        // Linear interpolation of the joints' position
                        pos_arm_microSteps.push_back(interpolate(pos_arm_curr.at(i), pos_arm_next.at(i), m));

                        // At the end of the planned movement, the velocity and acceleration of the joints are
                        // set to zero, ensuring no noise. The planned value is close to 0!
                        if((k == traj_mov_real.size() - 1) && (kk == pos_stage.rows() - 2) && (n == microSteps))
                        {
                            vel_arm_microSteps.push_back(0.0);
                            acc_arm_microSteps.push_back(0.0);
                        }
                        else
                        {
                            // Linear interpolation of the joints' velocity
                            vel_arm_microSteps.push_back(interpolate(vel_arm_curr.at(i), vel_arm_next.at(i), m));
                            // Linear interpolation of the joints' acceleration
                            acc_arm_microSteps.push_back(interpolate(acc_arm_curr.at(i), acc_arm_next.at(i), m));
                        }
                    }

                    // Save the position, velocity and acceleration of the robot's joints
                    pos_arm.push_back(pos_arm_microSteps);
                    vel_arm.push_back(vel_arm_microSteps);
                    acc_arm.push_back(acc_arm_microSteps);

                    // Save the time associated with each micro step of the planned movement
                    timeFromStart.push_back(timeFromStart.back() + t_inc);
                    // Increment the number of steps
                    ++nTotalSteps;
                }
            }
        }

        // ************************************************************************** //
        //                             Message to publish                             //
        // ************************************************************************** //
        // Define trajectory to be planned and executed. This has to pass through all points defined previously
        trajectory_msgs::JointTrajectory trajPlannedHUMP;
        trajPlannedHUMP.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
        trajPlannedHUMP.header.stamp = ros::Time::now();

        for(int step = 0; step < nTotalSteps; ++step)
        {
            // Points to be reached along the execution of the planned movement
            trajectory_msgs::JointTrajectoryPoint pointTraj;
            pointTraj.positions = pos_arm.at(step);
            //pointTraj.velocities = vel_arm.at(step);
            //pointTraj.accelerations = acc_arm.at(step);
            pointTraj.time_from_start = ros::Duration(timeFromStart.at(step));

            // Adds the point to the trajectory to be executed by the robot
            trajPlannedHUMP.points.push_back(pointTraj);
        }

        // Define the goal message
        control_msgs::FollowJointTrajectoryGoal plannedTrajectory;
        plannedTrajectory.trajectory = trajPlannedHUMP;

        // Send the goal message to the action server "/robot/limb/right/follow_joint_trajectory"
        folJointTraj->sendGoal(plannedTrajectory);
        // After 30 sec the function return false, if the goal hasn't reached
        folJointTraj->waitForResult(ros::Duration(45));

        if(folJointTraj->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // The final posture was reached
            log(QNode::Info,string("Final posture reached."));
        else
        {
            // It isn't possible to reach the desired posture. We can check the list of possible errors which are
            // returned from action server in the result message
            log(QNode::Error, string("Error in reaching the final posture of the robot."));
            return false;
        }

        log(QNode::Info,string("Movement completed."));
        // Handle ROS messages
        ros::spinOnce();
    }

    return true;
}
#endif


bool QNode::execTask(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task,taskPtr task, scenarioPtr scene)
{
#if HAND == 0
    bool hand_closed;
    closed.at(0) = false;
    closed.at(1) = false;
    closed.at(2) = false;

    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    int arm_code;
    int mov_type;
    double timeTot = 0.0;

    this->curr_scene = scene;
    int scenarioID = scene->getID();

    std::vector<int> handles;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int h_attach; // handle of the attachment point of the hand
    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan;
    bool approach;
    bool retreat;
    double pre_time;

    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
    ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    // start the simulation
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task

    for(int kk=0; kk < task->getProblemNumber(); ++kk)
    {
        if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved())
        {
            int ii = kk - hh;
            //the trajectory obtained doesn't include the joints offsets
            vector<MatrixXd> traj_mov_w_offset = traj_task.at(ii);
            //add the joints offsets
            vector<MatrixXd>traj_mov_real = realJointsPosition(traj_mov_w_offset);
            vector<MatrixXd> vel_mov = vel_task.at(ii);
            vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
            vector<double> tols_stop_mov = tols_stop_task.at(ii);

            double tol_stop_stage;
            MatrixXd traj;
            MatrixXd vel;
            std::vector<double> timesteps_stage;

            movementPtr mov = task->getProblem(kk)->getMovement();
            vector<string> traj_descr_mov = traj_descr_task.at(ii);
            this->curr_mov = mov;
            arm_code = mov->getArm();
            mov_type = mov->getType();

            switch (mov_type)
            {
            case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
                closed.at(0)=false;
                closed.at(1)=false;
                closed.at(2)=false;
                break;
            case 2: case 3: case 4: // transport, engage, disengage
                closed.at(0)=true;
                closed.at(1)=true;
                closed.at(2)=true;
                break;
            }

            switch (arm_code)
            {
            case 0: // dual arm
                break;
            case 1: //right arm
                handles = right_handles;
                hand_handles = right_hand_handles;
                h_attach = right_attach;
                break;
            case 2: // left arm
                handles = left_handles;
                hand_handles = left_hand_handles;
                h_attach = left_attach;
                break;
            }

            for(size_t j=0; j <traj_mov_real.size();++j)
            {
                string mov_descr = traj_descr_mov.at(j);

                if(strcmp(mov_descr.c_str(),"plan")==0)
                {
                    plan=true;
                    approach=false;
                    retreat=false;
                }
                else if(strcmp(mov_descr.c_str(),"approach")==0)
                {
                    plan=false;
                    approach=true;
                    retreat=false;
                }
                else if(strcmp(mov_descr.c_str(),"retreat")==0)
                {
                    plan=false;
                    approach=false;
                    retreat=true;
                }

                switch (mov_type)
                {
                case 0: // reach-to-grasp
                    if(retreat)
                    {
                        if(obj_in_hand)
                        {
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1)
                                log(QNode::Error,string("Error in grasping the object "));

#if OPEN_CLOSE_HAND == 1
                            this->closeBarrettHand(arm_code);
#else
                            MatrixXd tt = traj_mov_real.at(j);
                            VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);

                            std::vector<double> hand_init_pos;
                            hand_init_pos.resize(init_h_posture.size());

                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                            this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
#endif
                        }
                    }
                    break;
                case 1: // reaching
                    break;
                case 2: case 3: // transport, engage
                    if(retreat)
                    {
                        if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0)
                        {
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = -1; // parentless object
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1)
                                log(QNode::Error,string("Error in releasing the object "));
                        }

#if OPEN_CLOSE_HAND == 1
                        MatrixXd tt = traj_mov_real.at(j);
                        VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);

                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());

                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(arm_code,hand_init_pos);
#else
                        closed.at(0)=false;
                        closed.at(1)=false;
                        closed.at(2)=false;
#endif
                    }
                    break;
                case 4:// disengage
                    break;
                case 5: // go-park
                    break;
                }

                traj = traj_mov_real.at(j);
                vel = vel_mov.at(j);
                timesteps_stage = timesteps_mov.at(j);
                f_posture = traj.row(traj.rows()-1);
                f_reached=false;
                tol_stop_stage = tols_stop_mov.at(j);

                ros::spinOnce(); // handle ROS messages
                pre_time = simulationTime - timeTot; // update the total time of the movement


                if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                     client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1))
                {
                    // ok, the service call was ok, and the subscriber was succesfully started on V-REP side
                    // V-REP is now listening to the desired values
                    ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
                    // 5. Let's prepare a publisher of those values:
                    ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                    tb = pre_time;

                    for (int i = 0; i< vel.rows()-1; ++i)
                    {
                        VectorXd ya = vel.row(i);
                        VectorXd yb = vel.row(i+1);
                        VectorXd yat = traj.row(i);
                        VectorXd ybt = traj.row(i+1);

                        ta = tb;
                        double tt_step = timesteps_stage.at(i);
                        if(tt_step<0.001)
                            tt_step = MIN_EXEC_TIMESTEP_VALUE;
                        tb = ta + tt_step;

                        bool interval = true;
                        double tx_prev;
                        double yxt_prev;
                        ros::spinOnce(); // get the simulationRunning value

                        while (ros::ok() && simulationRunning && interval)
                        {// ros is running, simulation is running
                            vrep_common::JointSetStateData dataTraj;
                            vrep_common::JointSetStateData data_hand;

                            tx = simulationTime - timeTot;
                            if (tx >= tb)
                                interval = false;
                            else
                            {
                                double m;
                                if((tb-ta)==0)
                                    m=1;
                                else
                                    m = (tx-ta)/(tb-ta);

                                std::vector<double> r_post;
                                this->curr_scene->getRobot()->getRightPosture(r_post);
                                double yx;
                                double yxt;
                                if(sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                        pow((f_posture(1)-r_post.at(1)),2)+
                                        pow((f_posture(2)-r_post.at(2)),2)+
                                        pow((f_posture(3)-r_post.at(3)),2)+
                                        pow((f_posture(4)-r_post.at(4)),2)+
                                        pow((f_posture(5)-r_post.at(5)),2)+
                                        pow((f_posture(6)-r_post.at(6)),2)) < tol_stop_stage)
                                {
                                    f_reached=true;
                                    log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                    break;
                                }
                                else
                                    f_reached=false;

                                hand_closed = (closed[0] && closed[1] && closed[2]);
                                for (int k = 0; k< vel.cols(); ++k)
                                {
                                    if(f_reached)
                                    {
                                        yx=0;
                                        yxt=yxt_prev;
                                    }
                                    else
                                    {
                                        yx = interpolate(ya(k),yb(k),m);
                                        yxt = interpolate(yat(k),ybt(k),m);
                                        yxt_prev=yxt;
                                    }
                                    if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                            (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed))
                                        dataTraj.handles.data.push_back(handles.at(k));

                                    int exec_mode;
                                    double exec_value;
#if VEL == 0
                                    // position
                                    exec_mode = 0;
                                    exec_value = yxt;
#elif VEL == 1
                                    //velocity
                                    exec_mode = 2;
                                    exec_value = yx;
#endif
                                    switch(scenarioID)
                                    {
                                    case 0:
                                        break;
                                    case 1: case 2: case 3: case 4: // Toy vehicle scenario with AROS, human assistance with ARoS, Toy vehicle scenario with Sawyer, human assistance with Sawyer
                                        if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                        else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)))
                                            dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity

                                        if((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))
                                            dataTraj.values.data.push_back(exec_value); // joints of the arm
                                        else if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)
                                            dataTraj.values.data.push_back(yxt);  // joints of the hand
                                        break;
                                    }

                                    if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2))))
                                    {
                                        // the fingers are being addressed
                                        data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                        data_hand.setModes.data.push_back(1); // set the target position
                                        data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                    }
                                }
                                pub.publish(dataTraj);
                                pub_hand.publish(data_hand);

                                interval = true;
                                tx_prev = tx;
                            }
                            // handle ROS messages:
                            ros::spinOnce();
                        }

                        if(f_reached)
                        {
                            log(QNode::Info,string("Final Posture reached."));
                            break;
                        }
                    }

                    // ----------------------------- post-movement operations ---------------------------------- //
                    // set the detected object child of the attach point
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object

                    switch (mov_type)
                    {
                    case 0: // reach-to grasp
                        // grasp the object
                        if(approach ||(plan && (traj_mov_real.size()==1)))
                        {
                            if(obj_in_hand)
                            {
                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                srvset_parent.request.parentHandle = h_attach;
                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                add_client.call(srvset_parent);
                                if (srvset_parent.response.result != 1)
                                    log(QNode::Error,string("Error in grasping the object "));
                            }
                        }
                        break;
                    case 1: // reaching
                        break;
                    case 2: // transport
                        break;
                    case 3: // engage
                        break;
                    case 4: // disengage
                        break;
                    case 5: // go-park
                        break;
                    }

                    // handle ROS messages:
                    ros::spinOnce();
                    timeTot = simulationTime; // update the total time of the movement
                }
            }

            // movement complete
            log(QNode::Info,string("Movement completed"));
            task->getProblem(kk)->getMovement()->setExecuted(true);
        }
        else
            hh++;
    }

    log(QNode::Info,string("Task completed"));

    // pause the simulation
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);

    // handle ROS messages:
    ros::spinOnce();
    TotalTime=simulationTime;

    return true;
#endif
}


#if ROBOT == 1
bool QNode::execTask_Sawyer(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<MatrixXd>>& acc_task, vector<vector<vector<double>>>& timesteps_task)
{
    ros::NodeHandle node;
    bool homePostureEqual;

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                                        PUBLISHERS                                                        //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // Topics to enable/disable the movement of robot joints
    pubEnable_robot = node.advertise<std_msgs::Bool>("/robot/set_super_enable", 1);

    // Enables the robot before attempting to control any of the motors
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    pubEnable_robot.publish(enable_msg);

    // Handle ROS messages
    ros::spinOnce();

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                                 MOVE TO THE INITIAL POSTURE: MOTION CONTROLLER INTERFACE                                 //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // Get the trajectory planned by the HUMP
    vector<vector<MatrixXd>> traj_task_w_offset = traj_task;
    vector<vector<MatrixXd>> traj_task_real;
    // The trajectory planned doesn't include the joints offsets, so it's necessary to add these values.
    for(size_t k = 0; k < traj_task_w_offset.size(); ++k)
    {
        vector<MatrixXd> traj_mov_real = realJointsPosition(traj_task_w_offset.at(k));
        traj_task_real.push_back(traj_mov_real);
    }

    // Get the initial position of the robot arm in the V-REP simulator
    // First movement (Example: Reach-to-grasp magenta column)
    vector<MatrixXd> traj_first_mov = traj_task_real.at(0);
    // First stage of the first movement (Plan)
    MatrixXd traj = traj_first_mov.at(0);
    // Arm's inital posture
    VectorXd iP = traj.row(0);
    vector<double> simulationPosture(&iP[0], iP.data() + (iP.cols() * iP.rows() - JOINTS_HAND));

    // Calculate the difference between the initial posture in simulation and the current posture of the robot
    vector<double> diff;
    for(int i = 0; i < JOINTS_ARM; ++i)
        diff.push_back(simulationPosture.at(i) - robotPosture.at(i));

    // Get the highest value in the vector with differences between the joints
    double max_diff = *std::max_element(diff.begin(), diff.end());
    // Clear the diff vector
    diff.clear();
    // Position threshold in radians across each joint when move is considered successful
    double tol_maxDiff = 0.005; // 0.005 rad => 0.28647889757º

    if(max_diff < tol_maxDiff)
        // ******************************************************************************* //
        //                              The postures are equal                             //
        // ******************************************************************************* //
        homePostureEqual = true;
    else
    {
        // ******************************************************************************* //
        //       The postures are different: Moving the robot to the initial posture       //
        // ******************************************************************************* //
        log(QNode::Info, string("moving the Sawyer robot to the initial posture... "));

        // For joint trajectories, we specify the maximum value of speed and acceleration per joint
        intera_motion_msgs::WaypointOptions wayPointOptions;
        wayPointOptions.max_joint_speed_ratio = 0.1; // speed: values defined between 0.01 and 1
        for(int i = 0; i < JOINTS_ARM; ++i)
            wayPointOptions.max_joint_accel.push_back(0.05); // acceleration: values defined between 0.001 and 1

        // Define the first waypoint of the trajectory - initial posture of the robot
        intera_motion_msgs::Waypoint initialPoint;
        initialPoint.options = wayPointOptions;
        initialPoint.joint_positions = robotPosture;
        // Define the last waypoint of the trajectory - desired posture = initial posture in simulator
        intera_motion_msgs::Waypoint finalPoint;
        finalPoint.options = wayPointOptions;
        finalPoint.joint_positions = simulationPosture;

        // Define trajectory to be planned and executed. This has to pass through all waypoints defined previously
        // The motion controller supports two basic methods for interpolation between waypoints: JOINT and CARTESIAN mode
        intera_motion_msgs::Trajectory trajectory;
        trajectory.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
        trajectory.waypoints.push_back(initialPoint);
        trajectory.waypoints.push_back(finalPoint);
        trajectory.trajectory_options.interpolation_type = "JOINT";

        // Define the goal message
        intera_motion_msgs::MotionCommandGoal newStartPosture;
        newStartPosture.command = newStartPosture.MOTION_START;
        newStartPosture.trajectory = trajectory;

        // Send the goal message to the action server "/motion/motion_command"
        motionComm->sendGoal(newStartPosture);
        // After 30 sec the function return false, if the goal hasn't reached
        motionComm->waitForResult(ros::Duration(45));

        if(motionComm->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // The initial posture was reached
            log(QNode::Info,string("Initial posture reached."));
            homePostureEqual = true;
        }
        else
        {
            // It isn't possible to reach the desired posture. We can check the list of possible errors which are
            // returned from action server in the result message
            log(QNode::Error, string("Error in reaching the initial posture of the robot."));
            homePostureEqual = false;
            return false;
        }

        // Sleep for the specified number of seconds
        sleep(5);
    }

    // Handle ROS messages
    ros::spinOnce();

    // ------------------------------------------------------------------------------------------------------------------------ //
    //                               EXECUTE THE PLANNED TRAJECTORY: JOINT TRAJECTORY ACTION SERVER                             //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // If Sawyer robot is in its initial posture: the execution of the planned movements begins
    if(homePostureEqual)
    {
        // Position, velocity and acceleration of the arm along the planned movement
        vector<vector<double>> pos_arm;
        vector<vector<double>> vel_arm;
        vector<vector<double>> acc_arm;
        // The obtained values must be reached a few seconds after the start of the movement
        vector<double> timeFromStart;
        timeFromStart.push_back(0.0);
        // Number of steps to be executed along the planned movement
        int nTotalSteps = 1;

        for(size_t j = 0; j < traj_task_real.size(); ++j)
        {
            // ********************************************************************** //
            //                         Movements information                          //
            // ********************************************************************** //
            // For each movement of the planned task (each task is divided into several movements),
            // Position, velocity, aceleration
            vector<MatrixXd> pos_mov = traj_task_real.at(j);
            vector<MatrixXd> vel_mov = vel_task.at(j);
            vector<MatrixXd> acc_mov = acc_task.at(j);
            vector<vector<double>> timesteps_mov = timesteps_task.at(j);

            for(size_t k = 0; k < pos_mov.size(); ++k)
            {
                // ************************************************************************** //
                //                             Stages information                             //
                // ************************************************************************** //
                // For each stage of the planned movement ("Plan", "Approach" or "Retreat"),
                // we get the values of: position, velocity, aceleration and time_steps
                MatrixXd pos_stage = pos_mov.at(k);
                MatrixXd vel_stage = vel_mov.at(k);
                MatrixXd acc_stage = acc_mov.at(k);
                vector<double> timesteps_stage = timesteps_mov.at(k);

                for(int kk = 0; kk < pos_stage.rows() - 1; ++kk)
                {
                    // ********************************************************************** //
                    //                           Steps information                            //
                    // ********************************************************************** //
                    // For each step of the planned movement (each stage is divided into several steps),
                    // Get the current values of: position, velocity, aceleration
                    VectorXd pos_step_curr = pos_stage.row(kk);
                    VectorXd vel_step_curr = vel_stage.row(kk);
                    VectorXd acc_step_curr = acc_stage.row(kk);
                    // Get the next values of: position, velocity, aceleration
                    VectorXd pos_step_next = pos_stage.row(kk + 1);
                    VectorXd vel_step_next = vel_stage.row(kk + 1);
                    VectorXd acc_step_next = acc_stage.row(kk + 1);

                    // Get only the position, velocity and acceleration of the robot arm joints
                    // Current step
                    vector<double> pos_arm_curr(&pos_step_curr[0], pos_step_curr.data() + (pos_step_curr.cols() * pos_step_curr.rows() - JOINTS_HAND));
                    vector<double> vel_arm_curr(&vel_step_curr[0], vel_step_curr.data() + (vel_step_curr.cols() * vel_step_curr.rows() - JOINTS_HAND));
                    vector<double> acc_arm_curr(&acc_step_curr[0], acc_step_curr.data() + (acc_step_curr.cols() * acc_step_curr.rows() - JOINTS_HAND));
                    // Get only the position, velocity and acceleration of the robot arm joints
                    // Next step
                    vector<double> pos_arm_next(&pos_step_next[0], pos_step_next.data() + (pos_step_next.cols() * pos_step_next.rows() - JOINTS_HAND));
                    vector<double> vel_arm_next(&vel_step_next[0], vel_step_next.data() + (vel_step_next.cols() * vel_step_next.rows() - JOINTS_HAND));
                    vector<double> acc_arm_next(&acc_step_next[0], acc_step_next.data() + (acc_step_next.cols() * acc_step_next.rows() - JOINTS_HAND));

                    // ********************************************************************** //
                    //                       Joints linear interpolation                      //
                    // ********************************************************************** //
                    // Adds the position, velocity and acceleration obtained for the first movement (first step in plan stage)
                    if(j == 0 && k == 0 && kk == 0)
                    {
                        pos_arm.push_back(pos_arm_curr);
                        vel_arm.push_back(vel_arm_curr);
                        acc_arm.push_back(acc_arm_curr);
                    }

                    // Divide each step into several micro steps (1s correspond to 5 MicroStep)
                    int microSteps = (int)round(timesteps_stage.at(kk) * 5.0);
                    // Determine the time associated with execution of each microSteps
                    double t_inc = timesteps_stage.at(kk) / microSteps;

                    for(int n = 1; n <= microSteps; ++n)
                    {
                        // Position, velocity and acceleration for each micro step of the planned movement
                        vector<double> pos_arm_microSteps;
                        vector<double> vel_arm_microSteps;
                        vector<double> acc_arm_microSteps;

                        // Each micro step starts at 0 sec and ends after the time step determined by the HUMP
                        double t_curr = 0.0;
                        double t_next = timesteps_stage.at(kk);

                        // Linear interpolation depends on the value of m
                        // m is determined by the following formula: (x - x0) / (x1 - x0)
                        // In this case x is the time (in sec)
                        double m = ((n * t_inc) - t_curr) / (t_next - t_curr);

                        for(int i = 0; i < JOINTS_ARM; ++i)
                        {
                            // Linear interpolation of the joints' position
                            pos_arm_microSteps.push_back(interpolate(pos_arm_curr.at(i), pos_arm_next.at(i), m));

                            // At the end of the planned movement, the velocity and acceleration of the joints are
                            // set to zero, ensuring no noise. The planned value is close to 0!
                            if((j == traj_task_real.size() - 1) && (k == pos_mov.size() - 1) &&
                                    (kk == pos_stage.rows() - 2) && (n == microSteps))
                            {
                                vel_arm_microSteps.push_back(0.0);
                                acc_arm_microSteps.push_back(0.0);
                            }
                            else
                            {
                                // Linear interpolation of the joints' velocity
                                vel_arm_microSteps.push_back(interpolate(vel_arm_curr.at(i), vel_arm_next.at(i), m));
                                // Linear interpolation of the joints' acceleration
                                acc_arm_microSteps.push_back(interpolate(acc_arm_curr.at(i), acc_arm_next.at(i), m));
                            }
                        }

                        // Save the position, velocity and acceleration of the robot's joints
                        pos_arm.push_back(pos_arm_microSteps);
                        vel_arm.push_back(vel_arm_microSteps);
                        acc_arm.push_back(acc_arm_microSteps);

                        // Save the time associated with each micro step of the planned movement
                        timeFromStart.push_back(timeFromStart.back() + t_inc);
                        // Increment the number of steps
                        ++nTotalSteps;
                    }
                }
            }
        }

        // ************************************************************************** //
        //                             Message to publish                             //
        // ************************************************************************** //
        // Define trajectory to be planned and executed. This has to pass through all points defined previously
        trajectory_msgs::JointTrajectory trajPlannedHUMP;
        trajPlannedHUMP.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
        trajPlannedHUMP.header.stamp = ros::Time::now();

        for(int step = 0; step < nTotalSteps; ++step)
        {
            // Points to be reached along the execution of the planned movement
            trajectory_msgs::JointTrajectoryPoint pointTraj;
            pointTraj.positions = pos_arm.at(step);
            //pointTraj.velocities = vel_arm.at(step);
            //pointTraj.accelerations = acc_arm.at(step);
            pointTraj.time_from_start = ros::Duration(timeFromStart.at(step));

            // Adds the point to the trajectory to be executed by the robot
            trajPlannedHUMP.points.push_back(pointTraj);
        }

        // Define the goal message
        control_msgs::FollowJointTrajectoryGoal plannedTrajectory;
        plannedTrajectory.trajectory = trajPlannedHUMP;

        // Send the goal message to the action server "/robot/limb/right/follow_joint_trajectory"
        folJointTraj->sendGoal(plannedTrajectory);
        // After 30 sec the function return false, if the goal hasn't reached
        folJointTraj->waitForResult(ros::Duration(45));

        if(folJointTraj->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // The final posture was reached
            log(QNode::Info,string("Final posture reached."));
        else
        {
            // It isn't possible to reach the desired posture. We can check the list of possible errors which are
            // returned from action server in the result message
            log(QNode::Error, string("Error in reaching the final posture of the robot."));
            return false;
        }

        log(QNode::Info,string("Task completed."));
        // Handle ROS messages
        ros::spinOnce();
    }

    return true;
}
#endif


bool QNode::execTask_complete(vector<vector<MatrixXd>>& traj_task, vector<vector<MatrixXd>>& vel_task, vector<vector<vector<double>>>& timesteps_task, vector<vector<double>>& tols_stop_task, vector<vector<string>>& traj_descr_task, taskPtr task, scenarioPtr scene)
{
#if HAND == 0
    bool hand_closed;
    closed.at(0) = false;
    closed.at(1) = false;
    closed.at(2) = false;

    ros::NodeHandle node;
    double ta;
    double tb = 0.0;
    double tx;
    int arm_code;
    int mov_type;
    double timeTot = 0.0;

    this->curr_scene = scene;
    int scenarioID = scene->getID();

    std::vector<int> handles;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE+1,1);
    int h_attach; // handle of the attachment point of the hand
    bool f_reached;
    VectorXd f_posture; // the final posture of the movement
    bool plan;
    bool approach;
    bool retreat;
    double pre_time;

    // set joints position or velocity (it depends on the scenario)
    ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber;
    srv_enableSubscriber.request.topicName="/"+nodeName+"/set_joints"; // the topic name
    srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    // set joints position (it is used to set the target postion of the 2nd phalanx of the fingers)
    ros::ServiceClient client_enableSubscriber_hand=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
    vrep_common::simRosEnableSubscriber srv_enableSubscriber_hand;
    srv_enableSubscriber_hand.request.topicName="/"+nodeName+"/set_pos_hand"; // the topic name
    srv_enableSubscriber_hand.request.queueSize=1; // the subscriber queue size (on V-REP side)
    srv_enableSubscriber_hand.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

    // previous retreat trajectories
    MatrixXd traj_prev_retreat;
    MatrixXd vel_prev_retreat;
    std::vector<double> ttsteps_prev_retreat;

    // start the simulation
    add_client = node.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srvstart;
    add_client.call(srvstart);
    ros::spinOnce(); // first handle ROS messages

    int hh=0; // it counts problems that do not belong to the task

    for(int kk=0; kk < task->getProblemNumber(); ++kk)
    {
        if(task->getProblem(kk)->getPartOfTask() && task->getProblem(kk)->getSolved())
        {
            int ii = kk - hh;
            //the trajectory obtained doesn't include the joints offsets
            vector<MatrixXd> traj_mov_w_offset = traj_task.at(ii);
            //add the joints offsets
            vector<MatrixXd>traj_mov_real = realJointsPosition(traj_mov_w_offset);
            vector<MatrixXd> vel_mov = vel_task.at(ii);
            vector<vector<double>> timesteps_mov = timesteps_task.at(ii);
            vector<double> tols_stop_mov = tols_stop_task.at(ii);

            double tol_stop_stage;
            MatrixXd traj;
            MatrixXd vel;
            std::vector<double> timesteps_stage;

            movementPtr mov = task->getProblem(kk)->getMovement();
            vector<string> traj_descr_mov = traj_descr_task.at(ii);
            this->curr_mov = mov;
            arm_code = mov->getArm();
            mov_type = mov->getType();

            switch (mov_type)
            {
            case 0: case 1: case 5: // reach-to-grasp, reaching, go-park
                closed.at(0) = false;
                closed.at(1) = false;
                closed.at(2) = false;
                break;
            case 2: case 3: case 4: // transport, engage, disengage
                closed.at(0) = true;
                closed.at(1) = true;
                closed.at(2) = true;
                break;
            }

            switch (arm_code)
            {
            case 0: // dual arm
                break;
            case 1: //right arm
                handles = right_handles;
                hand_handles = right_hand_handles;
                h_attach = right_attach;
                break;
            case 2: // left arm
                handles = left_handles;
                hand_handles = left_hand_handles;
                h_attach = left_attach;
                break;
            }

            MatrixXd traj_ret_plan_app;
            MatrixXd vel_ret_plan_app;
            std::vector<double> timesteps_ret_plan_app;
            bool join_ret_plan_app = false;
            bool join_ret_plan = false;
            bool join_plan_app = false;

            if(mov_type==0 || mov_type==2 || mov_type==3 || mov_type==4)
            {
                // reach-to-grasp, transport, engage, disengage
                if(traj_mov_real.size() > 1)
                {
                    // there is more than one stage
                    string mov_descr_1 = traj_descr_mov.at(0);
                    string mov_descr_2 = traj_descr_mov.at(1);
                    if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"approach")==0))
                    {
                        if(ii==0 || traj_prev_retreat.rows()==0)
                        {
                            // first movement of the task => there is no previous retreat
                            join_plan_app = true;
                            traj_ret_plan_app.resize((traj_mov_real.at(0).rows() + traj_mov_real.at(1).rows()-1),traj_mov_real.at(0).cols());
                            vel_ret_plan_app.resize((vel_mov.at(0).rows() + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                        }
                        else
                        {
                            join_ret_plan_app = true;
                            traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov_real.at(0).rows()-1 + traj_mov_real.at(1).rows()-1),traj_mov_real.at(0).cols());
                            vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1 + vel_mov.at(1).rows()-1),vel_mov.at(0).cols());
                        }
                    }
                    else if((strcmp(mov_descr_1.c_str(),"plan")==0) && (strcmp(mov_descr_2.c_str(),"retreat")==0))
                    {
                        if(ii!=0 && traj_prev_retreat.rows()!=0)
                        {
                            join_ret_plan = true;
                            traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov_real.at(0).rows()-1),traj_mov_real.at(0).cols());
                            vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                        }
                    }
                }
                else
                {
                    // there is only the plan stage
                    if(ii!=0 && traj_prev_retreat.rows()!=0)
                    {
                        join_ret_plan = true;
                        traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov_real.at(0).rows()-1),traj_mov_real.at(0).cols());
                        vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                    }
                }
            }
            else
            {
                // reaching, go-park
                if(ii!=0 && traj_prev_retreat.rows()!=0)
                {
                    join_ret_plan = true;
                    traj_ret_plan_app.resize((traj_prev_retreat.rows() + traj_mov_real.at(0).rows()-1),traj_mov_real.at(0).cols());
                    vel_ret_plan_app.resize((vel_prev_retreat.rows() + vel_mov.at(0).rows()-1),vel_mov.at(0).cols());
                }
            }

            for(size_t j=0; j <traj_mov_real.size();++j)
            {
                string mov_descr = traj_descr_mov.at(j);
                if(strcmp(mov_descr.c_str(),"plan")==0)
                {
                    plan=true;
                    approach=false;
                    retreat=false;

                    if(join_ret_plan_app)
                    {
                        traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                        vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                        timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                        std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                        MatrixXd tt = traj_mov_real.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                        MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                        std::vector<double> ttsteps = timesteps_mov.at(j);
                        traj_ret_plan_app.block(traj_prev_retreat.rows(),0,tt_red.rows(),tt_red.cols()) = tt_red;
                        vel_ret_plan_app.block(vel_prev_retreat.rows(),0,vv_red.rows(),vv_red.cols()) = vv_red;
                        timesteps_ret_plan_app.reserve(ttsteps.size());
                        std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                        traj_prev_retreat.resize(0,0);
                        continue;
                    }
                    else if(join_ret_plan)
                    {
                        traj_ret_plan_app.topLeftCorner(traj_prev_retreat.rows(),traj_prev_retreat.cols()) = traj_prev_retreat;
                        vel_ret_plan_app.topLeftCorner(vel_prev_retreat.rows(),vel_prev_retreat.cols()) = vel_prev_retreat;
                        timesteps_ret_plan_app.reserve(ttsteps_prev_retreat.size());
                        std::copy (ttsteps_prev_retreat.begin(), ttsteps_prev_retreat.end(), std::back_inserter(timesteps_ret_plan_app));
                        MatrixXd tt = traj_mov_real.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                        MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                        std::vector<double> ttsteps = timesteps_mov.at(j);
                        traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                        vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                        timesteps_ret_plan_app.reserve(ttsteps.size());
                        std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                        traj_prev_retreat.resize(0,0);
                    }
                    else if(join_plan_app)
                    {
                        MatrixXd tt = traj_mov_real.at(j);
                        MatrixXd vv = vel_mov.at(j);
                        std::vector<double> ttsteps = timesteps_mov.at(j);
                        traj_ret_plan_app.topLeftCorner(tt.rows(),tt.cols()) = tt;
                        vel_ret_plan_app.topLeftCorner(vv.rows(),vv.cols()) = vv;
                        timesteps_ret_plan_app.reserve(ttsteps.size());
                        std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                        continue;
                    }
                }
                else if(strcmp(mov_descr.c_str(),"approach")==0)
                {
                    plan=false;
                    approach=true;
                    retreat=false;

                    if(join_ret_plan_app || join_plan_app)
                    {
                        MatrixXd tt = traj_mov_real.at(j); MatrixXd tt_red = tt.bottomRows(tt.rows()-1);
                        MatrixXd vv = vel_mov.at(j); MatrixXd vv_red = vv.bottomRows(vv.rows()-1);
                        std::vector<double> ttsteps = timesteps_mov.at(j);
                        traj_ret_plan_app.bottomLeftCorner(tt_red.rows(),tt_red.cols()) = tt_red;
                        vel_ret_plan_app.bottomLeftCorner(vv_red.rows(),vv_red.cols()) = vv_red;
                        timesteps_ret_plan_app.reserve(ttsteps.size());
                        std::copy (ttsteps.begin(), ttsteps.end(), std::back_inserter(timesteps_ret_plan_app));
                    }
                }
                else if(strcmp(mov_descr.c_str(),"retreat")==0)
                {
                    plan=false; approach=false; retreat=true;
                    traj_prev_retreat = traj_mov_real.at(j);
                    vel_prev_retreat = vel_mov.at(j);
                    ttsteps_prev_retreat = timesteps_mov.at(j);
                }

                switch (mov_type)
                {
                case 0: // reach-to-grasp
                    if(retreat)
                    {
                        if(obj_in_hand)
                        {
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = h_attach;
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1)
                                log(QNode::Error,string("Error in grasping the object "));
#if OPEN_CLOSE_HAND == 1
                            this->closeBarrettHand(arm_code);
#else
                            MatrixXd tt = traj_mov_real.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                            std::vector<double> hand_init_pos;
                            hand_init_pos.resize(init_h_posture.size());
                            VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                            this->closeBarrettHand_to_pos(arm_code,hand_init_pos);
#endif
                        }
                        continue;
                    }
                    break;
                case 1: // reaching
                    break;
                case 2: case 3: // transport, engage
                    if(retreat)
                    {
                        if(std::strcmp(mov->getObject()->getName().c_str(),"")!=0)
                        {
                            add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                            vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object
                            srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                            srvset_parent.request.parentHandle = -1; // parentless object
                            srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                            add_client.call(srvset_parent);
                            if (srvset_parent.response.result != 1)
                                log(QNode::Error,string("Error in releasing the object "));
                        }
#if OPEN_CLOSE_HAND == 1
                        MatrixXd tt = traj_mov_real.at(j); VectorXd init_h_posture = tt.block<1,JOINTS_HAND>(0,JOINTS_ARM);
                        std::vector<double> hand_init_pos;
                        hand_init_pos.resize(init_h_posture.size());
                        VectorXd::Map(&hand_init_pos[0], init_h_posture.size()) = init_h_posture;
                        this->openBarrettHand_to_pos(arm_code,hand_init_pos);
#else
                        closed.at(0) = false;
                        closed.at(1) = false;
                        closed.at(2) = false;
#endif
                        continue;
                    }
                    break;
                case 4:// disengage
                    break;
                case 5: // go-park
                    break;
                }

                if((join_ret_plan && (strcmp(mov_descr.c_str(),"plan")==0)) || ((join_ret_plan_app || join_plan_app) && (strcmp(mov_descr.c_str(),"approach")==0)))
                {
                    traj = traj_ret_plan_app;
                    vel = vel_ret_plan_app;
                    timesteps_stage = timesteps_ret_plan_app;
                }
                else
                {
                    traj = traj_mov_real.at(j);
                    vel = vel_mov.at(j);
                    timesteps_stage = timesteps_mov.at(j);
                }

                f_posture = traj.row(traj.rows()-1);
                f_reached=false;
                tol_stop_stage = tols_stop_mov.at(j);

                ros::spinOnce(); // handle ROS messages
                pre_time = simulationTime - timeTot; // update the total time of the movement


                if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) &&
                     client_enableSubscriber_hand.call(srv_enableSubscriber_hand) && (srv_enableSubscriber_hand.response.subscriberID!=-1))
                {
                    // ok, the service call was ok, and the subscriber was succesfully started on V-REP side
                    // V-REP is now listening to the desired values
                    ros::Publisher pub_hand=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_pos_hand",1);
                    // 5. Let's prepare a publisher of those values:
                    ros::Publisher pub=node.advertise<vrep_common::JointSetStateData>("/"+nodeName+"/set_joints",1);
                    tb = pre_time;

                    for (int i = 0; i< vel.rows()-1; ++i)
                    {
                        VectorXd ya = vel.row(i);
                        VectorXd yb = vel.row(i+1);
                        VectorXd yat = traj.row(i);
                        VectorXd ybt = traj.row(i+1);

                        ta = tb;
                        double tt_step = timesteps_stage.at(i);
                        if(tt_step<0.001)
                            tt_step = MIN_EXEC_TIMESTEP_VALUE;
                        tb = ta + tt_step;

                        bool interval = true;
                        double tx_prev;
                        double yxt_prev;
                        ros::spinOnce(); // get the simulationRunning value

                        while (ros::ok() && simulationRunning && interval)
                        {
                            // ros is running, simulation is running
                            vrep_common::JointSetStateData dataTraj;
                            vrep_common::JointSetStateData data_hand;

                            tx = simulationTime - timeTot;
                            if (tx >= tb)
                                interval = false;
                            else
                            {
                                double m;
                                if((tb-ta)==0)
                                    m=1;
                                else
                                    m = (tx-ta)/(tb-ta);

                                std::vector<double> r_post;
                                this->curr_scene->getRobot()->getRightPosture(r_post);
                                double yx;
                                double yxt;
                                if(sqrt(pow((f_posture(0)-r_post.at(0)),2)+
                                        pow((f_posture(1)-r_post.at(1)),2)+
                                        pow((f_posture(2)-r_post.at(2)),2)+
                                        pow((f_posture(3)-r_post.at(3)),2)+
                                        pow((f_posture(4)-r_post.at(4)),2)+
                                        pow((f_posture(5)-r_post.at(5)),2)+
                                        pow((f_posture(6)-r_post.at(6)),2)) < tol_stop_stage)
                                {
                                    f_reached=true;
                                    log(QNode::Info,string("Final posture reached, movement: ")+mov->getStrType());
                                    break;
                                }
                                else
                                    f_reached=false;

                                hand_closed = (closed[0] && closed[1] && closed[2]);
                                for (int k = 0; k< vel.cols(); ++k)
                                {
                                    if(f_reached)
                                    {
                                        yx=0;
                                        yxt=yxt_prev;
                                    }
                                    else
                                    {
                                        yx = interpolate(ya(k),yb(k),m);
                                        yxt = interpolate(yat(k),ybt(k),m);
                                        yxt_prev=yxt;
                                    }

                                    if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)) || // joints of the arm
                                            (((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed))
                                        dataTraj.handles.data.push_back(handles.at(k));

                                    int exec_mode;
                                    double exec_value;
#if VEL == 0
                                    // position
                                    exec_mode = 0;
                                    exec_value = yxt;
#elif VEL == 1
                                    //velocity
                                    exec_mode = 2;
                                    exec_value = yx;
#endif
                                    switch(scenarioID)
                                    {
                                    case 0:
                                        break;
                                    case 1: case 2: case 3: case 4: // Toy vehicle scenario with AROS, human assistance with ARoS, Toy vehicle scenario with Sawyer, human assistance with Sawyer
                                        if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)
                                            dataTraj.setModes.data.push_back(1); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                        else if(((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4)))
                                            dataTraj.setModes.data.push_back(exec_mode); // 0 to set the position, 1 to set the target position, 2 to set the target velocity
                                        if((k!=vel.cols()-1) && (k!=vel.cols()-2) && (k!=vel.cols()-3) && (k!=vel.cols()-4))
                                            dataTraj.values.data.push_back(exec_value); //joints of the arm
                                        else if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3) || (k==vel.cols()-4)) && !hand_closed)
                                            dataTraj.values.data.push_back(yxt); //joints of the hand
                                        break;
                                    }

                                    if(((k==vel.cols()-1) || (k==vel.cols()-2) || (k==vel.cols()-3)) && ((!closed.at(0)) && (!closed.at(1)) && (!closed.at(2))))
                                    {
                                        // the fingers are being addressed
                                        data_hand.handles.data.push_back(hand_handles(k+3-vel.cols(),2));
                                        data_hand.setModes.data.push_back(1); // set the target position
                                        data_hand.values.data.push_back(yxt/3.0 + 45.0f*static_cast<double>(M_PI) / 180.0f);
                                    }
                                }

                                pub.publish(dataTraj);
                                pub_hand.publish(data_hand);

                                interval = true;
                                tx_prev = tx;
                            }

                            // handle ROS messages:
                            ros::spinOnce();
                        }

                        if(f_reached)
                        {
                            log(QNode::Info,string("Final Posture reached."));
                            break;
                        }
                    } // for loop step

                    // ---------------------------------- post-movement operations -------------------------------- //
                    // set the detected object child of the attach point
                    add_client = node.serviceClient<vrep_common::simRosSetObjectParent>("/vrep/simRosSetObjectParent");
                    vrep_common::simRosSetObjectParent srvset_parent; // service to set a parent object

                    switch (mov_type)
                    {
                    case 0: // reach-to grasp
                        // grasp the object
                        if(approach ||(plan && (traj_mov_real.size()==1)))
                        {
                            if(obj_in_hand)
                            {
                                srvset_parent.request.handle = this->curr_mov->getObject()->getHandle();
                                srvset_parent.request.parentHandle = h_attach;
                                srvset_parent.request.keepInPlace = 1; // the detected object must stay in the same place
                                add_client.call(srvset_parent);
                                if (srvset_parent.response.result != 1)
                                    log(QNode::Error,string("Error in grasping the object "));
                            }
                        }
                        break;
                    case 1: // reaching
                        break;
                    case 2: // transport
                        break;
                    case 3: // engage
                        break;
                    case 4: // disengage
                        break;
                    case 5: // go-park
                        break;
                    }

                    // handle ROS messages:
                    ros::spinOnce();
                    timeTot = simulationTime; // update the total time of the movement
                }
            }

            // movement complete
            log(QNode::Info,string("Movement completed"));
            task->getProblem(kk)->getMovement()->setExecuted(true);
        }
        else
            hh++;
    }

    // for loop movements
    log(QNode::Info,string("Task completed"));

    // pause the simulation
    add_client = node.serviceClient<vrep_common::simRosPauseSimulation>("/vrep/simRosPauseSimulation");
    vrep_common::simRosPauseSimulation srvpause;
    add_client.call(srvpause);

    // handle ROS messages:
    ros::spinOnce();
    TotalTime=simulationTime;

    return true;
#endif
}


double QNode::interpolate(double ya, double yb, double m)
{
    // linear interpolation
    return ya + (yb - ya) * m;
}


void QNode::stopSim()
{
    ros::NodeHandle node;

    // stop the simulation
    add_client = node.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    vrep_common::simRosStopSimulation srvstop;
    add_client.call(srvstop);
}


bool QNode::checkRViz()
{
    bool online = false;
    std::string vrep_node = "/move_group";

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


void QNode::log(const LogLevel &level, const std::string &msg)
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
    const char *l_names[] = {"left_joint0", "left_joint1", "left_joint2", "left_joint3","left_joint4", "left_joint5", "left_joint6",
                            "left_gripper_jointClose"};
#endif

    for (int i = 0; i < JOINTS_ARM+JOINTS_HAND; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();

        if (r_index < joints_names.size())
        {
            right_posture.push_back(joints_pos.at(r_index));
            right_vel.push_back(joints_vel.at(r_index));
            right_forces.push_back(joints_force.at(r_index));
        }

        if(this->curr_scene->getRobot()->getName() == "ARoS")
        {
            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_names[i]) - joints_names.begin();

            if (l_index < joints_names.size())
            {
                left_posture.push_back(joints_pos.at(l_index));
                left_vel.push_back(joints_vel.at(l_index));
                left_forces.push_back(joints_force.at(l_index));
            }
        }
    }

#if HAND == 0
    for(int i = 0; i < HAND_FINGERS; ++i)
    {
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_2hand[i]) - joints_names.begin();

        if (r_index < joints_names.size())
        {
            right_2hand_pos.at(i) = joints_pos.at(r_index);
            right_2hand_vel.at(i) = joints_vel.at(r_index);
            right_2hand_force.at(i) = joints_force.at(r_index);
        }

        if(this->curr_scene->getRobot()->getName() == "ARoS")
        {
            size_t l_index = std::find(joints_names.begin(), joints_names.end(), l_2hand[i]) - joints_names.begin();

            if (l_index < joints_names.size())
            {
                left_2hand_pos.at(i) = joints_pos.at(l_index);
                left_2hand_vel.at(i) = joints_vel.at(l_index);
                left_2hand_force.at(i) = joints_force.at(l_index);
            }
        }
    }
#endif

    if (this->curr_scene)
    {
        //add the joints offset
        std::transform(right_posture.begin(), right_posture.end(), theta_offset.begin(), right_posture.begin(), std::plus<double>());
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


#if ROBOT == 1
void QNode::SawyerJointsCallback(const sensor_msgs::JointState &state)
{
    // Save the names and positions of the joints
    std::vector<std::string> joints_names = state.name;
    std::vector<double> joints_pos(state.position.begin(),state.position.end());

    // Name of arm joints
    const char *r_names[] = {"right_j0", "right_j1", "right_j2", "right_j3","right_j4", "right_j5", "right_j6"};

    for (int i = 0; i < JOINTS_ARM; ++i)
    {
        // Index of the joint in vector
        size_t r_index = std::find(joints_names.begin(), joints_names.end(), r_names[i]) - joints_names.begin();

        if (r_index < joints_names.size())
            robotPosture.at(i) = joints_pos.at(r_index);
    }
}
#endif


const std::string QNode::currentDateTime()
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

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
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
        srvgetHandle.request.objectName = string("right_joint")+QString::number(k).toStdString();
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1)
            right_handles.push_back(srvgetHandle.response.handle);
        else
        {
            throw string("Error: Couldn't get the information about of right arm joints");
            succ = false;
        }

        // ARoS and Jade
        if(robot != 2)
        {
            srvgetHandle.request.objectName = string("left_joint")+QString::number(k).toStdString();
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1)
                left_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the left arm joints");
                succ = false;
            }
        }
    }


    // **** Get the object handle of the hand joints
    for (int k = 0; k < JOINTS_HAND; ++k)
    {
        if(k == 0)
        {
#if HAND == 0
            srvgetHandle.request.objectName = string("right_BarrettHand_jointA_0");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
                right_handles.push_back(srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_0");
                succ = false;
            }

            if(robot !=2)
            {
                srvgetHandle.request.objectName = string("left_BarrettHand_jointB_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_0");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_0");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_0");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_0");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_2");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_2");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_2");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_2");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1)
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
            if (srvgetHandle.response.handle !=-1)
                right_hand_handles(k, 1) = (srvgetHandle.response.handle);
            else
            {
                throw string("Error: Couldn't get the information of the right_BarrettHand_jointB_1");
                succ = false;
            }

            srvgetHandle.request.objectName = string("right_BarrettHand_jointC_1");
            add_client.call(srvgetHandle);
            if (srvgetHandle.response.handle !=-1)
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
                if (srvgetHandle.response.handle !=-1)
                    left_hand_handles(k, 1) = (srvgetHandle.response.handle);
                else
                {
                    throw string("Error: Couldn't get the information of the left_BarrettHand_jointB_1");
                    succ = false;
                }

                srvgetHandle.request.objectName = string("left_BarrettHand_jointC_1");
                add_client.call(srvgetHandle);
                if (srvgetHandle.response.handle !=-1)
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
    if (srvgetHandle.response.handle !=-1)
        right_sensor = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_BarrettHand_attachProxSensor");
        succ = false;
    }

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_BarrettHand_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1)
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
        if (srvgetHandle.response.handle !=-1)
            left_sensor = srvgetHandle.response.handle;
        else
        {
            throw string("Error: Couldn't get the information of the left_BarrettHand_attachProxSensor");
            succ = false;
        }

        add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
        srvgetHandle.request.objectName = string("left_BarrettHand_attachPoint");
        add_client.call(srvgetHandle);
        if (srvgetHandle.response.handle !=-1)
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
    if (srvgetHandle.response.handle !=-1)
        right_sensor = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_gripper_attachProxSensor");
        succ = false;
    }

    add_client = node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    srvgetHandle.request.objectName = string("right_gripper_attachPoint");
    add_client.call(srvgetHandle);
    if (srvgetHandle.response.handle !=-1)
        right_attach = srvgetHandle.response.handle;
    else
    {
        throw string("Error: Couldn't get the information of the right_gripper_attachPoint");
        succ = false;
    }
#endif

    return succ;
}


#if HAND == 0
bool QNode::closeBarrettHand(int hand)
{
    int cnt = 0;
    std::vector<int> firstPartTorqueOvershootCount(3, 0);

    firstPartLocked.at(0) = false;
    firstPartLocked.at(1) = false;
    firstPartLocked.at(2) = false;

    needFullOpening.at(0) = 0;
    needFullOpening.at(1) = 0;
    needFullOpening.at(2) = 0;

    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE + 1, 1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    //set the force
    ros::ServiceClient client_setForce = node.serviceClient<vrep_common::simRosSetJointForce>("/vrep/simRosSetJointForce");
    vrep_common::simRosSetJointForce srv_setForce;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;


    while (ros::ok() && simulationRunning && (!closed[0] || !closed[1] || !closed[2]) && cnt < 1000)
    {
        cnt++;

        switch (hand)
        {
        case 1: // right hand
            hand_handles = right_hand_handles;
            this->curr_scene->getRobot()->getRightHandForces(hand_forces);
            this->curr_scene->getRobot()->getRightHandPosture(hand_posture);
            break;
        case 2: // left hand
            hand_handles = left_hand_handles;
            this->curr_scene->getRobot()->getLeftHandForces(hand_forces);
            this->curr_scene->getRobot()->getLeftHandPosture(hand_posture);
            break;
        }

        for (size_t i = 0; i < HAND_FINGERS; i++)
        {
            if (firstPartLocked.at(i))
            {
                closed[i] = true;

                // set the velocity of the second phalanx (1/3 of the velocity of the first phalanx)
                srv_setObjInt.request.handle = hand_handles(i,2);
                srv_setObjInt.request.parameter = 2001;
                srv_setObjInt.request.parameterValue = 0;
                client_setIntParam.call(srv_setObjInt);

                srv_setTarVel.request.handle = hand_handles(i,2);
                srv_setTarVel.request.targetVelocity = closingVel / 3.0f;
                client_setTarVel.call(srv_setTarVel);
            }
            else if (!firstPartLocked.at(i))
            {
                double t = 0.0f;
                t = hand_forces.at(i+1);

                if (abs(t) > firstPartMaxTorque)
                    firstPartTorqueOvershootCount[i] ++;
                else
                    firstPartTorqueOvershootCount[i] = 0;

                if (firstPartTorqueOvershootCount[i] >= firstPartTorqueOvershootCountRequired)
                {
                    needFullOpening[i] = 1;
                    firstPartLocked[i] = true;

                    // lock the first part
                    // set the position control
                    srv_setObjInt.request.handle = hand_handles(i, 1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 1;
                    client_setIntParam.call(srv_setObjInt);
                    // set the force
                    srv_setForce.request.handle = hand_handles(i, 1);
                    srv_setForce.request.forceOrTorque = closingOpeningTorque * 100.0f;
                    client_setForce.call(srv_setForce);
                    // set the target position
                    srv_setTarPos.request.handle = hand_handles(i, 1);
                    srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
                    client_setTarPos.call(srv_setTarPos);

                    // go on with the second part
                    srv_setObjInt.request.handle = hand_handles(i, 2);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i, 2);
                    srv_setTarVel.request.targetVelocity = closingVel / 3.0f;
                    client_setTarVel.call(srv_setTarVel);
                }
                else
                {
                    //make first joint to close with a predefined velocity
                    srv_setObjInt.request.handle = hand_handles(i, 1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i, 1);
                    srv_setTarVel.request.targetVelocity = closingVel;
                    client_setTarVel.call(srv_setTarVel);

                    //second joint position is 1/3 of the first
                    srv_setTarPos.request.handle = hand_handles(i, 2);
                    srv_setTarPos.request.targetPosition = 45.0f * static_cast<double>(M_PI) / 180.0f + hand_posture.at(i + 1) / 3.0f;
                    client_setTarPos.call(srv_setTarPos);
                }
            }
        }

        ros::spinOnce();
    } // while loop


    for (size_t i = 0; i < HAND_FINGERS; i++)
    {
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
        client_setTarPos.call(srv_setTarPos);

        srv_setObjInt.request.handle = hand_handles(i, 2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
    }

    log(QNode::Info,string("Hand closed."));


    return (closed[0] && closed[1] && closed[2]);
}


bool QNode::openBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE + 1, 1);
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
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 2);
        srv_setTarPos.request.targetPosition = 45.0f * static_cast<double>(M_PI) / 180.0f + hand_posture.at(i + 1) / 3.0f ;
        client_setTarPos.call(srv_setTarPos);

        firstPartLocked[i] = false;
        needFullOpening[i] = 0;
        closed[i] = false;
    }

    log(QNode::Info,string("Hand open."));


    return true;
}


bool QNode::closeBarrettHand_to_pos(int hand, std::vector<double>& hand_posture)
{
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE + 1, 1);
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
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
        client_setTarPos.call(srv_setTarPos);

        // second joint in position control
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


bool QNode::openBarrettHand(int hand)
{
    int cnt = 0;
    MatrixXi hand_handles = MatrixXi::Constant(HAND_FINGERS,N_PHALANGE + 1, 1);
    ros::NodeHandle node;
    std::vector<double> hand_forces;
    std::vector<double> hand_posture;
    std::vector<double> hand2_pos;

    // set the target position
    ros::ServiceClient client_setTarPos = node.serviceClient<vrep_common::simRosSetJointTargetPosition>("/vrep/simRosSetJointTargetPosition");
    vrep_common::simRosSetJointTargetPosition srv_setTarPos;

    // set the target velocity
    ros::ServiceClient client_setTarVel = node.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
    vrep_common::simRosSetJointTargetVelocity srv_setTarVel;

    // set Object int parameter
    ros::ServiceClient client_setIntParam = node.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter srv_setObjInt;

    while (ros::ok() && simulationRunning && (closed[0] || closed[1] || closed[2]) && cnt < 1000)
    {
        cnt++;

        switch (hand)
        {
        case 1: // right hand
            hand_handles = right_hand_handles;
            hand2_pos = right_2hand_pos;
            this->curr_scene->getRobot()->getRightHandForces(hand_forces);
            this->curr_scene->getRobot()->getRightHandPosture(hand_posture);
            break;
        case 2: // left hand
            hand_handles = left_hand_handles;
            hand2_pos = left_2hand_pos;
            this->curr_scene->getRobot()->getLeftHandForces(hand_forces);
            this->curr_scene->getRobot()->getLeftHandPosture(hand_posture);
            break;
        }

        for (size_t i = 0; i < HAND_FINGERS; i++)
        {
            srv_setObjInt.request.handle = hand_handles(i, 2);
            srv_setObjInt.request.parameter = 2001;
            srv_setObjInt.request.parameterValue = 0;
            client_setIntParam.call(srv_setObjInt);

            srv_setTarVel.request.handle = hand_handles(i, 2);
            srv_setTarVel.request.targetVelocity = openingVel / 3.0f;
            client_setTarVel.call(srv_setTarVel);

            if (firstPartLocked[i])
            {
                if(hand2_pos.at(i) < 45.5 * static_cast<double>(M_PI) / 180.0)
                {
                    // unlock the first part
                    // set the velocity control
                    srv_setObjInt.request.handle = hand_handles(i, 1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    //make first joints to open with a predefined velocity
                    srv_setTarVel.request.handle = hand_handles(i, 1);
                    srv_setTarVel.request.targetVelocity = openingVel;
                    client_setTarVel.call(srv_setTarVel);

                    firstPartLocked[i] = false;
                }
            }
            else
            {
                if (needFullOpening[i] != 0)
                {
                    // full opening is needed
                    if ((hand2_pos.at(i) < (45.5f * static_cast<double>(M_PI) / 180.0f)) && (hand_posture.at(i + 1) < (0.5f * static_cast<double>(M_PI) / 180.0f)))
                    {
                        needFullOpening[i] = 0;
                        // second joint in position control
                        // set the position control
                        srv_setObjInt.request.handle = hand_handles(i, 2);
                        srv_setObjInt.request.parameter = 2001;
                        srv_setObjInt.request.parameterValue = 1;
                        client_setIntParam.call(srv_setObjInt);

                        // set the target position
                        srv_setTarPos.request.handle = hand_handles(i, 2);
                        srv_setTarPos.request.targetPosition = 45.0f * static_cast<double>(M_PI) / 180.0f + hand_posture.at(i + 1) / 3.0f ;
                        client_setTarPos.call(srv_setTarPos);
                    }
                }
                else
                {
                    // full opening is NOT needed
                    //make first joint to open with a predefined velocity
                    srv_setObjInt.request.handle = hand_handles(i, 1);
                    srv_setObjInt.request.parameter = 2001;
                    srv_setObjInt.request.parameterValue = 0;
                    client_setIntParam.call(srv_setObjInt);

                    srv_setTarVel.request.handle = hand_handles(i, 1);
                    srv_setTarVel.request.targetVelocity = openingVel;
                    client_setTarVel.call(srv_setTarVel);

                    if (hand_posture.at(i + 1) <= (36.0f * static_cast<double>(M_PI) / 180.0f))
                        closed[i] = false;
                }
            }
        }

        ros::spinOnce();
    }


    for (size_t i = 0; i < HAND_FINGERS; i++)
    {
        // set the position control
        srv_setObjInt.request.handle = hand_handles(i, 1);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
        // set the target position
        srv_setTarPos.request.handle = hand_handles(i, 1);
        srv_setTarPos.request.targetPosition = hand_posture.at(i + 1);
        client_setTarPos.call(srv_setTarPos);

        srv_setObjInt.request.handle = hand_handles(i, 2);
        srv_setObjInt.request.parameter = 2001;
        srv_setObjInt.request.parameterValue = 1;
        client_setIntParam.call(srv_setObjInt);
    }

    log(QNode::Info,string("Hand open."));


    return true;
}

#endif

}  // namespace motion_manager
