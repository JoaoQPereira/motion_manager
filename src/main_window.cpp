#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/motion_manager/main_window.hpp"


using namespace Qt;
using namespace std;

namespace motion_manager
{

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    ui.setupUi(this);

    // qApp is a global variable for the application
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

    // create Ros Communication dialog
    mrosCommdlg = new RosCommDialog(&qnode, this);
    mrosCommdlg->setModal(true);

    // create Vrep Communication dialog
    mvrepCommdlg = new VrepCommDialog(&qnode, this);
    mvrepCommdlg->setModal(true);

    //create HUMP Tuning dialog
    mTolHumpdlg = new TolDialogHUMP(this);
    mTolHumpdlg->setModal(true);

    //create Define Waypoints Platform Tunning dialog
    mMovExecutedlg = new Mov_ExecuteDialog(&qnode, this);
    mMovExecutedlg->setModal(true);

    //create Execute Movement Tunning dialog
    PlatWaypointdlg = new set_waypointsdialog(&qnode, this);
    PlatWaypointdlg->setModal(true);

    //create Execute Movement Tunning dialog
    mTaskExecutedlg = new Task_ExecuteDialog(&qnode, this);
    mTaskExecutedlg->setModal(true);

    //create the Results Joints dialog
    mResultsJointsdlg = new ResultsJointsDialog(this);
    mResultsJointsdlg->setModal(false);

    // create the velocity components dialog
    mCompVeldlg = new CompVelocityDialog(this);
    mCompVeldlg->setModal(false);

    ReadSettings();
    setWindowIcon(QIcon(":/images/motion_managerIcon.png"));

    ///   QObject::connect- connect a signal to a slot- when an event happens ( signal) some code will run ( slot)
    ///   SIGNAL(rosShutdown()) -> rosShutdown() is the event
    ///   SLOT(close()) -> close() is the function that is executed when that event happens

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    // ROS connected signal
    QObject::connect(mrosCommdlg, SIGNAL(rosConnected(bool)), this, SLOT(updateRosStatus(bool)));

    // V-REP connected signal
    QObject::connect(mvrepCommdlg, SIGNAL(vrepConnected(bool)), this, SLOT(updateVrepStatus(bool)));

    // Plataform used to execute the planned movement
    QObject::connect(PlatWaypointdlg, SIGNAL(addPlat_setWps(int)), this, SLOT(SetWaypoints(int)));

    // Plataform used to execute the selected task
    QObject::connect(mTaskExecutedlg, SIGNAL(addPlat_execTask(int, bool)), this, SLOT(execTask(int, bool)));

    // Plataform used to execute the planned movement
    QObject::connect(mMovExecutedlg, SIGNAL(addPlat_execMove(int, bool)), this, SLOT(execMove(int, bool)));


    // new element in the scenario signal
    QObject::connect(&qnode, SIGNAL(newElement(string)),this,SLOT(addElement(string)));
    QObject::connect(&qnode, SIGNAL(newObject(string)),this,SLOT(addObject(string)));
    QObject::connect(&qnode, SIGNAL(newPose(string)),this,SLOT(addPose(string)));
    QObject::connect(&qnode, SIGNAL(newWaypoint(string)),this,SLOT(addWaypoint(string)));

    // update an element in the scenario signal
    QObject::connect(&qnode, SIGNAL(updateElement(int,string)),this,SLOT(updateElement(int,string)));

    // new joint in the home posture
    QObject::connect(&qnode, SIGNAL(newJoint(string)),this,SLOT(updateHomePosture(string)));

    //description of the selected scenario
    QObject::connect(ui.listWidget_scenario, SIGNAL(itemClicked(QListWidgetItem*)),this, SLOT(onListScenarioItemClicked(QListWidgetItem*)));

    //new waypoint selected signal
    //QObject::connect(&qnode, SIGNAL(newWaypoint(string)),this,SLOT(addWaypoint(string)));

    /*********************
    ** Start settings
    **********************/
    ui.tabWidget_main->setCurrentIndex(0);
    ui.tabWidget_sol->setCurrentIndex(0);
    ui.comboBox_planner->clear();
    ui.comboBox_planner->addItem(QString("HUMP"));

    // scenarios
    scenarios.clear();

#if HAND == 0
    scenarios.push_back(QString("Assembly scenario: Toy vehicle with ARoS and Bill"));
    scenarios.push_back(QString("Human assistance scenario: Serving a drink with ARoS"));
    scenarios.push_back(QString("Assembly scenario: Toy vehicle with Sawyer and Bill"));
    scenarios.push_back(QString("Human assistance scenario: Serving a drink with Sawyer"));

#elif HAND == 1
    scenarios.push_back(QString("Assembly scenario: Toy vehicle with Sawyer and Bill"));
    scenarios.push_back(QString("Assembly scenario: Interaction to build the base of the Toy Vehicle : 1"));
    scenarios.push_back(QString("Assembly scenario: Interaction to build the base of the Toy Vehicle : 2"));
    scenarios.push_back(QString("waypoints scenario with sawyer"));
#elif HAND == 2
    scenarios.push_back(QString("waypoints scenario: Human-Robot Collaboration (UR10)"));

#endif

    for (size_t i=0; i< scenarios.size();++i)
        ui.listWidget_scenario->addItem(scenarios.at(i));
}


MainWindow::~MainWindow()
{
}


void MainWindow::updateLoggingView()
{
    ui.view_logging->scrollToBottom();
}


void MainWindow::updateRosStatus(bool c)
{
    if (c)
    {
        ui.labelRosComm->setText(QString("connected"));
        ui.actionVrep_Communication->setEnabled(true);
        ui.labelStatusVrep->setEnabled(true);
        ui.labelVrepComm->setEnabled(true);
    }
    else
    {
        ui.labelRosComm->setText(QString("disconnected"));
        ui.actionVrep_Communication->setEnabled(false);
        ui.labelStatusVrep->setEnabled(false);
        ui.labelVrepComm->setEnabled(false);
    }
}


void MainWindow::updateVrepStatus(bool c)
{
    if (c)
    {
        ui.labelVrepComm->setText(QString("on-line"));
        ui.tab_scenario->setEnabled(true);
        ui.groupBox_selectScenario->setEnabled(true);
    }
    else
    {
        ui.labelVrepComm->setText(QString("off-line"));
        ui.actionRViz_Communication->setEnabled(false);
        ui.labelStatusRViz->setEnabled(false);
        ui.labelRVizComm->setEnabled(false);
        ui.tab_scenario->setEnabled(false);
        ui.groupBox_selectScenario->setEnabled(false);
    }
    ui.pushButton_loadScenario->setEnabled(false);
    ui.groupBox_getElements->setEnabled(false);
}


void MainWindow::updateRVizStatus(bool c)
{
    if (c)
    {
        ui.labelRVizComm->setText(QString("on-line"));
        ui.tab_scenario->setEnabled(true);
        ui.groupBox_selectScenario->setEnabled(true);
    }
    else
    {
        ui.labelRVizComm->setText(QString("off-line"));
        ui.tab_scenario->setEnabled(false);
        ui.groupBox_selectScenario->setEnabled(false);
    }
    ui.pushButton_loadScenario->setEnabled(false);
    ui.groupBox_getElements->setEnabled(false);

}

void MainWindow::SetWaypoints(int c)
{
    if(c == 0)  //Define waypoints using Coppelia
    {

    }
    else if(c == 1) //Define waypoints using Polyscope
    {
        qnode.ConnectPolyscope();

        try
        {
            //if(qnode.moveRobotCoppelia(this->curr_scene))
            //{

            //}
            // initialize a thread to move the coppelia robot while the waypoints are defined in Polyscope
            qnode.ThreadState(true);
            qnode.start();
        }
        catch(std::string str)
        {
            qnode.log(QNode::Error,str);
        }
        catch(std::exception e)
        {
            qnode.log(QNode::Error,e.what());
        }

    }
}

void MainWindow::execMove(int c, bool a)
{
    if(c == 0)  //Execute the planned movement in V-REP simulator
        qnode.execMovement(this->jointsPosition_mov, this->timesteps_mov, this->tols_stop_mov, this->traj_descr_mov, this->curr_mov, this->curr_scene);
    else if(c == 1) //Execute the planned movement in Robot
    {
#if ROBOT == 1
#if UR == 0
        if(this->curr_scene->getRobot()->getName() == "Sawyer")
        {
            std::vector<double> paramsTimeMapping;
            mMovExecutedlg->getTimeMappingParams(paramsTimeMapping);

            qnode.execMovementSawyer(this->jointsPosition_mov, this->timesteps_mov, this->traj_descr_mov, this->curr_mov, paramsTimeMapping);
        }
#else

         std::vector<double> paramsTimeMapping;
         mMovExecutedlg->getTimeMappingParams(paramsTimeMapping);
         qnode.execMovementUR(this->jointsPosition_mov, this->timesteps_mov, this->traj_descr_mov, this->curr_mov, paramsTimeMapping);
#endif
#endif
    }

    execSettings_move = a;
    //The user chooses the "Don't ask again" option
    if(execSettings_move == true)
        usedPlat_move = c;
}


void MainWindow::execTask(int c, bool a)
{
    if(c == 0) //Execute the task in V-REP simulator
        qnode.execTask(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, this->curr_scene);
    else if(c == 1) //Execute the task in Robot
    {
#if ROBOT == 1
#if UR ==0
        if(this->curr_scene->getRobot()->getName() == "Sawyer")
        {
            std::vector<vector<double>> paramsTimeMapping;
            mTaskExecutedlg->getTimeMappingParams(paramsTimeMapping);

            qnode.execTaskSawyer(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, paramsTimeMapping);
        }
#else
           // qnode.execTaskUR(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, paramsTimeMapping);

#endif
#endif
    }

    execSettings_task = a;
    //The user chooses the "Don't ask again" option
    if(execSettings_task == true)
        usedPlat_task = c;
}


void MainWindow::addElement(string value)
{
    ui.listWidget_elements->addItem(QString(value.c_str()));
    ui.listWidget_elements->setCurrentRow(0);
}


void MainWindow::updateElement(int id, string value)
{
    QListWidgetItem* curr_item = ui.listWidget_elements->takeItem(id);
    delete curr_item;
    ui.listWidget_elements->insertItem(id,QString(value.c_str()));
}


void MainWindow::addWaypoint(string value)
{
    ui.comboBox_waypoints->addItem(QString(value.c_str()));
}


void MainWindow::addObject(string value)
{
    ui.comboBox_objects->addItem(QString(value.c_str()));
    ui.comboBox_objects_eng->addItem(QString(value.c_str()));
}


void MainWindow::addPose(string value)
{
    if(this->scenario_id != 5 || value == "GreenColumn_Pose1")
        ui.comboBox_poses->addItem(QString(value.c_str()));
}

void MainWindow::updateHomePosture(string value)
{
    ui.listWidget_homePosture->addItem(QString(value.c_str()));
    ui.listWidget_homePosture->setCurrentRow(0);
}


void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About the motion manager"),tr("<h2>motion_manager version 1.10</h2><p>Copyright: Gianpaolo Gulletta</p><p>The motion manager is a ROS package."
                                                               "This software is designed to plan the movements of the arms for any robot</p>"));
}


void MainWindow::on_actionRos_Communication_triggered()
{
    mrosCommdlg->show();
}


void MainWindow::on_actionVrep_Communication_triggered()
{
    mvrepCommdlg->show();
}


void MainWindow::on_pushButton_tuning_clicked()
{
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();

    switch(planner_id)
    {
    case 0: // HUMP
        mTolHumpdlg->setInitJointsVel(this->jointsEndVelocity_mov);
        mTolHumpdlg->setInitJointsAcc(this->jointsEndAcceleration_mov);
        mTolHumpdlg->setPointsOfArm(this->curr_scene->getRobot()->getDH_rightArm(), this->curr_scene->getRobot()->getName());
        mTolHumpdlg->show();
        break;
    case 1: // RRT
        break;
    case 2: //RRTConnect
        break;
    case 3: //RRTstar
        break;
    case 4: //PRM
        break;
    case 5: // PRMstar
        break;
    }
}

// when clicked on button Load Scenario
void MainWindow::on_pushButton_loadScenario_clicked()
{
    ui.listWidget_elements->clear();
    ui.listWidget_homePosture->clear();

    ui.tab_plan->setEnabled(false);
    ui.listWidget_movs->clear();
    ui.tableWidget_sol_mov->clear();
    ui.tableWidget_sol_task->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();

    ui.tab_results->setEnabled(false);

    ui.plot_hand_vel_mov->plotLayout()->clear();
    ui.plot_hand_vel_mov->clearGraphs();
    ui.plot_hand_vel_mov->replot();
    ui.plot_hand_vel_task->plotLayout()->clear();
    ui.plot_hand_vel_task->clearGraphs();
    ui.plot_hand_vel_task->replot();
    ui.label_cost_hand_value->clear();
    ui.label_cost_hand_value_task->clear();
    ui.label_nmu->clear();
    ui.label_nmu_task->clear();
    ui.label_solving_time->clear();
    ui.label_solving_time_task->clear();

    //Deletes all the items in the range
    qDeleteAll(ui.plot_hand_pos_mov->children());
    qDeleteAll(ui.plot_hand_pos_task->children());

    //saved to scenario_text - the clicked scenario in the listwidget_scenario
    QString scenario_text = ui.listWidget_scenario->currentItem()->text();

    int equal;

    for(int i=0; i<scenarios.size();++i)
    {
        //scenarios.at(i) - Returns the string of the given index
        equal = scenario_text.compare(scenarios.at(i));
        // compares scenario_text with the string at scenarios.at
        // =0 equal

        if(equal==0)
        {
            //get the scenarios path

            // Toy vehicle scenario with ARoS
            string path_vrep_toyscene_aros = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_aros_bill.ttt");
            // Drinking Service task with ARoS
            string path_vrep_drinking_aros = PATH_SCENARIOS + string("/vrep/DrinkingServiceTask_aros_bill.ttt");
            // Toy vehicle scenario with Jarde
            string path_vrep_toyscene_jarde = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_jarde.ttt");
            // Toy vehicle scenario with Sawyer
            string path_vrep_toyscene_sawyer = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_sawyer_bill.ttt");
            // Drinking Service task with Sawyer
            string path_vrep_drinking_sawyer = PATH_SCENARIOS + string("/vrep/DrinkingServiceTask_sawyer_bill.ttt");
            // Toy vehicle scenario with Sawyer with Eletric Parallel Gripper
            string path_vrep_toyscene_sawyer_gripper = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_sawyer_gripper_bill.ttt");
            // Toy vehicle scenario with Sawyer with Eletric Parallel Gripper
            string path_vrep_toyscene_sawyer_gripper_interaction_1 = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_sawyer_gripper_bill_interaction_1.ttt");
            // 29.12.2019
            //  purple and blue collum engaged
            string path_vrep_toyscene_sawyer_gripper_interaction_2 = PATH_SCENARIOS + string("/vrep/ToyVehicleTask_sawyer_gripper_bill_interaction_2.ttt");

            string path_vrep_HRColab_UR = PATH_SCENARIOS + string("/vrep/UR_HRColab.ttt");

            string path_vrep_wp_sawyer = PATH_SCENARIOS + string("/vrep/sawyer_waypoints.ttt");
            switch(i)
            {
            case 0: // Assembly scenario
#if HAND == 0
                // Assembly scenario: the Toy vehicle with ARoS
                this->scenario_id = 0;

                if (qnode.loadScenario(path_vrep_toyscene_aros,this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Assembly scenario: the Toy vehicle with ARoS");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
                break;
#elif HAND == 1
                this->scenario_id = 4;

                if (qnode.loadScenario(path_vrep_toyscene_sawyer_gripper,this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Assembly scenario: the Toy vehicle with Sawyer");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
                break;
#elif HAND == 2
                this->scenario_id = 6;
                //load scnenario to vrep and all topics36

                if (qnode.loadScenario(path_vrep_HRColab_UR, this->scenario_id))
                {
                    qnode.log(QNode::Info,string("waypoints scenario: Human-Robot Collaboration (UR10)- HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(false);
                    ui.pushButton_getWaypoints->setEnabled(true);
                    ui.pushButton_deleteWaypoint->setEnabled(true);
                    ui.pushButton_saveWaypoints->setEnabled(true);
                    ui.pushButton_setWaypoint->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("waypoints scenario with UR10 robot");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                 }
                 else
                 {
                    qnode.log(QNode::Error,std::string("waypoints scenario: Human-Robot Collaboration (UR10)- HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.pushButton_getWaypoints->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                 }

#endif
            case 1:// Human assistance with ARoS
#if HAND == 0
                this->scenario_id = 1;
                if (qnode.loadScenario(path_vrep_drinking_aros,this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Human assistance scenario: Serving a drink with ARoS HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Human assistance scenario: Serving a drink with ARoS");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Human assistance scenario: Serving a drink with ARoS HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
#elif HAND == 1
                // Assembly scenario: the Toy vehicle with Sawyer
                //29.12.2019

                this->scenario_id = 5;
                //load scnenario to vrep and all topics36

                // scnerario - no column mounted
                if (qnode.loadScenario(path_vrep_toyscene_sawyer_gripper_interaction_1, this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Assembly scenario: the Toy vehicle with Sawyer");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
                break;
#endif
            case 2: // 29.12.2019
                    //Assembly scenario: the Toy vehicle with Sawyer
                    // scneario - purple and blue collumn mounted
                this->scenario_id = 5;
#if HAND == 1
                if (qnode.loadScenario(path_vrep_toyscene_sawyer_gripper_interaction_2, this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Assembly scenario: the Toy vehicle with Sawyer");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
                break;
#endif

            case 3:
#if HAND == 0  // Human assistance with Sawyer
                this->scenario_id = 3;

                if (qnode.loadScenario(path_vrep_drinking_sawyer,this->scenario_id))
                {
                    qnode.log(QNode::Info,string("Human assistance scenario: Serving a drink with Sawyer HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("Human assistance scenario: Serving a drink with Sawyer");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                }
                else
                {
                    qnode.log(QNode::Error,std::string("Human assistance scenario: Serving a drink with Sawyer HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                }
#endif
#if HAND == 1
                this->scenario_id = 6;
                //load scnenario to vrep and all topics36

                if (qnode.loadScenario(path_vrep_wp_sawyer, this->scenario_id))
                {
                    qnode.log(QNode::Info,string("waypoints scenario with Sawyer - HAS BEEN LOADED"));
                    ui.pushButton_getElements->setEnabled(false);
                    ui.pushButton_getWaypoints->setEnabled(true);
                    ui.pushButton_deleteWaypoint->setEnabled(true);
                    ui.pushButton_saveWaypoint->setEnabled(true);
                    ui.pushButton_setWaypoint->setEnabled(true);
                    ui.groupBox_getElements->setEnabled(true);
                    ui.groupBox_homePosture->setEnabled(true);
                    string title = string("waypoints scenario with Sawyer robot");
                    init_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                    curr_scene = scenarioPtr(new Scenario(title,this->scenario_id+1));
                 }
                 else
                 {
                    qnode.log(QNode::Error,std::string("waypoints scenario with Sawyer Robot - HAS NOT BEEN LOADED. You probaly have to stop the simulation"));
                    ui.groupBox_getElements->setEnabled(false);
                    ui.groupBox_homePosture->setEnabled(false);
                    ui.pushButton_loadScenario->setEnabled(true);
                 }
#endif
                 break;

            }
        }
    }
}

void MainWindow::on_pushButton_setWaypoint_clicked()
{
    vector <double> wp;
    if(qnode.setWaypoint(wp))// get the waypoint
    {
        string info_wp = string("Waypoint:{");
        for (int i = 0; i < wp.size(); i++){
            info_wp.append(to_string(wp[i]*180/M_PI));
            info_wp.append(string(", "));
        }
        ui.listWidget_waypoints->addItem(QString(info_wp.c_str()));
    }
    robot_waypoints.push_back(wp);
}

void MainWindow::on_pushButton_SaveTrajectory_clicked()
{
    mov_wps.push_back(robot_waypoints);
    mov_name.push_back(ui.qline_edit_MovementName->text());
    robot_waypoints.clear();
    qnode.setMovWps(mov_wps,mov_name);
}



void MainWindow::on_pushButton_deleteWaypoint_clicked()
{
    robot_waypoints.pop_back();
    ui.listWidget_waypoints->clear();
    vector<double> wp;
    for(int k=0; k<robot_waypoints.size();k++){
        wp = robot_waypoints.at(k);
        string info_wp = string("Waypoint:{");
        for (int i = 0; i < wp.size(); i++){
            info_wp.append(to_string(wp[i]*180/M_PI));
            info_wp.append(string(", "));
        }
        ui.listWidget_waypoints->addItem(QString(info_wp.c_str()));
    }
    qnode.updateWaypoints(robot_waypoints);
}

void MainWindow::on_pushButton_getWaypoints_clicked()
{
    //ui.listWidget_waypoints->clear();
    //show platform to decide where the waypoints are defined
    PlatWaypointdlg->show();

}

void MainWindow::on_pushButton_getWaypoints_pressed()
{
    qnode.log(QNode::Info,string("getting the waypoints . . ."));
    ui.pushButton_getElements->setEnabled(false);
    ui.pushButton_deleteWaypoint->setEnabled(true);
    ui.pushButton_saveWaypoints->setEnabled(true);
    ui.pushButton_setWaypoint->setEnabled(true);
    ui.pushButton_SaveTrajectory->setEnabled(true);
    ui.groupBox_2->setEnabled(true);
    ui.qline_edit_MovementName->setEnabled(true);

}


void MainWindow::on_pushButton_saveWaypoints_clicked()
{

    ui.pushButton_getElements->setEnabled(true);
    ui.pushButton_deleteWaypoint->setEnabled(false);
    ui.pushButton_saveWaypoints->setEnabled(false);
    ui.pushButton_setWaypoint->setEnabled(false);
    //qnode.moveRobotToInitPos();

    // stop the thread of moving the vrep robot, since the waypoints are already defined
    qnode.ThreadState(false);
}


void MainWindow::on_pushButton_getElements_pressed()
{
    qnode.log(QNode::Info,string("getting the elements of the scenario . . ."));
    ui.pushButton_getElements->setCheckable(true);
    ui.pushButton_loadScenario->setEnabled(false);
    ui.pushButton_setWaypoint->setEnabled(false);
    ui.pushButton_deleteWaypoint->setEnabled(false);
}


void MainWindow::on_pushButton_plan_pressed()
{
    qnode.log(QNode::Info,string("planning the selected movement. . ."));
    ui.pushButton_plan->setCheckable(true);
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_mov->clear();
}

// enter here when button Get elements is clicked
void MainWindow::on_pushButton_getElements_clicked()
{
    ui.listWidget_elements->clear();
    //qnode.setWaypoint();
    try
    {
        if (qnode.getElements(this->curr_scene))
        {
            this->init_scene = scenarioPtr(new Scenario(*(this->curr_scene.get()))); //set the init scenario
            this->curr_task = taskPtr(new Task());
            ui.pushButton_getElements->setEnabled(false);
            ui.tab_plan->setEnabled(true);
            ui.tab_results->setEnabled(true);
            ui.groupBox_specs->setEnabled(true);
            ui.groupBox_task->setEnabled(false);
            ui.tabWidget_sol->setEnabled(false);

            if(scenario_id == 2 || scenario_id == 3 || scenario_id == 4 || scenario_id == 5 || scenario_id == 6)
            {
                ui.radioButton_right->setEnabled(false);
                ui.radioButton_left->setEnabled(false);
                ui.comboBox_Task->setItemData(1, 0, Qt::UserRole-1);
                ui.pushButton_plan_trials->setEnabled(false);
            }

            std::vector<objectPtr> objs;
            this->curr_scene->getObjects(objs);
            // load the objects into RViz
            qnode.log(QNode::Info,string("The elements of the scenario are now available"));
        }
        else
        {
            ui.pushButton_getElements->setEnabled(true);
            qnode.log(QNode::Error,string("Error in getting the elements of the scenario"));
        }
        ui.comboBox_objects_eng->setEnabled(false);
    }
    catch(std::string str)
    {
        qnode.log(QNode::Error,str);
    }
    catch(std::exception e)
    {
        qnode.log(QNode::Error,e.what());
    }
}


void MainWindow::on_pushButton_addMov_clicked()
{
    ui.pushButton_save_task->setEnabled(false);
    int planner_id = ui.comboBox_planner->currentIndex();
    bool add = true;

    if(add)
    {
        bool success = false;
        int mov_id = ui.comboBox_mov->currentIndex();
        int arm_sel;

        if (ui.comboBox_Task->currentIndex()==0)
        {
            if(ui.radioButton_right->isChecked())
                arm_sel=1; // right arm
            else
                arm_sel=2; // left arm
        }
        else
            arm_sel=0;

        if (ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled())
        {
            // ----------------------- Engage Movements -------------------------------//
            string obj_name = ui.comboBox_objects->currentText().toStdString();
            string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString(); 
            objectPtr obj = curr_scene->getObject(obj_name);
            objectPtr obj_eng = curr_scene->getObject(obj_eng_name);

            if(obj!=NULL && obj_eng!=NULL)
            {
                bool prec = ui.radioButton_prec->isChecked();

                switch (arm_sel)
                {
                case 0: // dual arm
                    break;
                case 1: // right arm
                    obj->setTargetRightEnabled(true);
                    obj->setTargetLeftEnabled(false);
                    break;
                case 2: // left arm
                    obj->setTargetLeftEnabled(true);
                    obj->setTargetRightEnabled(false);
                    break;
                }

                if(planner_id==0)
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj, obj_eng, prec),new Scenario(*(this->curr_scene.get()))));

                success=true;
            }
            else
                qnode.log(QNode::Error,std::string("The movement requires two objects"));
        }
        else if(ui.comboBox_objects->isEnabled() && ui.comboBox_objects_eng->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled())
        {
            // ----------------------- Disengage Movements -------------------------------//
            string obj_name = ui.comboBox_objects->currentText().toStdString();
            string obj_eng_name = ui.comboBox_objects_eng->currentText().toStdString();
            objectPtr obj = curr_scene->getObject(obj_name);
            objectPtr obj_eng = curr_scene->getObject(obj_eng_name);
            string pose_name = ui.comboBox_poses->currentText().toStdString();
            posePtr pose = curr_scene->getPose(pose_name);

            if(obj!=NULL && obj_eng!=NULL && pose!=NULL)
            {
                bool prec = ui.radioButton_prec->isChecked();

                switch (arm_sel)
                {
                case 0: // dual arm
                    break;
                case 1: // right arm
                    obj->setTargetRightEnabled(true);
                    obj->setTargetLeftEnabled(false);
                    break;
                case 2: // left arm
                    obj->setTargetLeftEnabled(true);
                    obj->setTargetRightEnabled(false);
                    break;
                }

                if(planner_id==0)
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,obj_eng,pose,prec),new Scenario(*(this->curr_scene.get()))));

                success=true;
            }
            else
                qnode.log(QNode::Error,std::string("The movement requires two objects and a pose"));
        }
        else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && !ui.comboBox_poses->isEnabled())
        {
            // ----------------------- Reach-to-grasp Movements -------------------------------//

            string obj_name = ui.comboBox_objects->currentText().toStdString();
            objectPtr obj = curr_scene->getObject(obj_name);

            if(obj!=NULL)
            {
                bool prec = ui.radioButton_prec->isChecked();

                switch (arm_sel)
                {
                case 0: // dual arm
                    break;
                case 1: // right arm
                    obj->setTargetRightEnabled(true);
                    obj->setTargetLeftEnabled(false);
                    break;
                case 2: // left arm
                    obj->setTargetLeftEnabled(true);
                    obj->setTargetRightEnabled(false);
                    break;
                }

                if(planner_id==0)
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,prec),new Scenario(*(this->curr_scene.get()))));

                success=true;
            }
            else
                qnode.log(QNode::Error,std::string("The movement requires an object"));


        }
        else if(ui.comboBox_objects->isEnabled() && ui.groupBox_grip->isEnabled() && ui.comboBox_poses->isEnabled())
        {
            // ----------------------- Transport Movements -------------------------------//
            string obj_name = ui.comboBox_objects->currentText().toStdString();
            objectPtr obj = curr_scene->getObject(obj_name);
            string pose_name = ui.comboBox_poses->currentText().toStdString();
            posePtr pose = curr_scene->getPose(pose_name);

            if(obj!=NULL || pose!=NULL)
            {
                bool prec = ui.radioButton_prec->isChecked();

                if(planner_id==0)
                    curr_task->addProblem(new Problem(planner_id,new Movement(mov_id, arm_sel, obj,pose,prec),new Scenario(*(this->curr_scene.get()))));

                success=true;
            }
            else
                qnode.log(QNode::Error,std::string("The movement requires an object and a pose"));
        }
        else if(ui.comboBox_waypoints->isEnabled())
        {
            // ----------------------- waypoints Movements -------------------------------//
            std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);
            string wp_name = ui.comboBox_waypoints->currentText().toStdString();
            waypointPtr wp = this->curr_scene->getWaypoint_traj(wp_name);
            //std::vector<waypointPtr> wps; this->curr_scene->getWaypoints(wps); //get all waypoints
            for(size_t i=0;i<objects.size();++i)
            {
                this->curr_scene->getObject(i)->setTargetRightEnabled(false);
                this->curr_scene->getObject(i)->setTargetLeftEnabled(false);
            }

            if(planner_id==0){
                Waypoint *wppp = new Waypoint(*(wp.get()));
                //curr_task->addProblem(new Problem(planner_id, new Waypoint(*(wp.get())),new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));
                curr_task->addProblem(new Problem(planner_id, wppp,new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));
             }
            success=true;

        }
        else{
            // ----------------------- Go-Park and Reaching Movements -------------------------------//
            std::vector<objectPtr> objects; this->curr_scene->getObjects(objects);

            for(size_t i=0;i<objects.size();++i)
            {
                this->curr_scene->getObject(i)->setTargetRightEnabled(false);
                this->curr_scene->getObject(i)->setTargetLeftEnabled(false);
            }
            if(planner_id==0)
                curr_task->addProblem(new Problem(planner_id, new Movement(mov_id, arm_sel),new Scenario(*(this->curr_scene.get()))));

            success=true;
        }

        if(success)
        {
            qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
            ui.groupBox_task->setEnabled(true);
            ui.listWidget_movs->clear();

            for (int i = 0; i < curr_task->getProblemNumber();i++ )
                ui.listWidget_movs->addItem(QString(curr_task->getProblemInfo(i).c_str()));

            ui.listWidget_movs->setCurrentRow(ui.listWidget_movs->count()-1);
        }
    }
}


void MainWindow::on_pushButton_plan_clicked()
{
    ui.tabWidget_sol->setCurrentIndex(0);
    problemPtr prob = curr_task->getProblem(ui.listWidget_movs->currentRow());
    int planner_id = prob->getPlannerID();
    HUMotion::hump_params tols;
    std::vector<double> move_target;
    std::vector<double> move_final_hand;
    std::vector<double> move_final_arm;
    bool use_final;
    bool solved = false;

    try
    {
        switch(planner_id)
        {
        case 0: // HUMP
            //handle of the HUMP tuning dialog
            mTolHumpdlg->setInfo(prob->getInfoLine());
            // --- Tolerances for the final posture selection ---- //
            tols.tolTarPos = mTolHumpdlg->getTolTarPos(); // target position tolerances
            tols.tolTarOr = mTolHumpdlg->getTolTarOr(); // target orientation tolerances
            mTolHumpdlg->getTolsArm(tols.tolsArm);// tolerances of the arm : radius in [mm]
            mTolHumpdlg->getTolsHand(tols.tolsHand);// tolerances of the hand: radius in [mm]
            tols.target_avoidance = mTolHumpdlg->getTargetAvoidance();// target avoidance
            tols.obstacle_avoidance = mTolHumpdlg->getObstacleAvoidance(); //obstacle avoidance
            mTolHumpdlg->getLambda(tols.lambda_final); // joint expense factors
            mTolHumpdlg->getLambda(tols.lambda_bounce); // joint expense factors
            mTolHumpdlg->getLambda(tols.lambda_wp); // joint expense factors

            // --- Tolerances for the bounce posture selection ---- //
#if HAND == 0
            tols.w_max = std::vector<double>(JOINTS_ARM+JOINTS_HAND,(mTolHumpdlg->getWMax()*M_PI/180)); // max joint velocity
            tols.alpha_max = std::vector<double>(JOINTS_ARM+JOINTS_HAND,(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration
#elif HAND == 1
            tols.w_max = std::vector<double>(JOINTS_ARM,(mTolHumpdlg->getWMax()*M_PI/180)); // max joint velocity (arm)
            for(int i = 0; i < JOINTS_HAND; ++ i)
                    tols.w_max.push_back(mTolHumpdlg->getWMaxGripper());
            tols.alpha_max = std::vector<double>(JOINTS_ARM,(mTolHumpdlg->getAlphaMax()*M_PI/180)); // max joint acceleration (arm)
            for(int i = 0; i < JOINTS_HAND; ++ i)
                    tols.alpha_max.push_back(mTolHumpdlg->getAlphaMaxGripper());
#elif HAND == 2
            tols.w_max = std::vector<double>(JOINTS_ARM,(mTolHumpdlg->getWMaxUR()*M_PI/180)); // max joint velocity (arm)
            tols.alpha_max = std::vector<double>(JOINTS_ARM,(mTolHumpdlg->getAlphaMaxUR()*M_PI/180)); // max joint acceleration (arm)

#endif
            mTolHumpdlg->getInitVel(tols.bounds.vel_0); // initial velocity
            mTolHumpdlg->getFinalVel(tols.bounds.vel_f); // final velocity
            mTolHumpdlg->getInitAcc(tols.bounds.acc_0); // initial acceleration
            mTolHumpdlg->getFinalAcc(tols.bounds.acc_f); // final acceleration
            // --- Tolerances for the obstacles ---- //
            mTolHumpdlg->getTolsObstacles(tols.final_tolsObstacles); // final posture tols
            tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
            tols.singleArm_tolsObstacles.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(0));
            mTolHumpdlg->getTolsObstacles(tols.singleArm_tolsObstacles.at(1));
            // --- Tolerances for the target ---- //
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1)); // bounce posture tols
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
            tols.singleArm_tolsTarget.push_back(MatrixXd::Constant(3,6,1));
            mTolHumpdlg->getTolsTarget(tols.singleArm_tolsTarget.at(0));
            tols.singleArm_tolsTarget.at(1) = tols.singleArm_tolsTarget.at(0)/100;
            tols.singleArm_tolsTarget.at(2) = 0*tols.singleArm_tolsTarget.at(0);
            // --- Pick and place settings  ---- //
            tols.mov_specs.approach = mTolHumpdlg->getApproach();
            tols.mov_specs.retreat = mTolHumpdlg->getRetreat();
            mTolHumpdlg->getPreGraspApproach(tols.mov_specs.pre_grasp_approach); // pick approach
            mTolHumpdlg->getPostGraspRetreat(tols.mov_specs.post_grasp_retreat); // pick retreat
            mTolHumpdlg->getPrePlaceApproach(tols.mov_specs.pre_place_approach); // place approach
            mTolHumpdlg->getPostPlaceRetreat(tols.mov_specs.post_place_retreat); // place retreat
            tols.mov_specs.rand_init = mTolHumpdlg->getRandInit(); // random initialization for "plan" stages
            tols.mov_specs.coll = mTolHumpdlg->getColl(); // collisions option
            tols.mov_specs.straight_line = mTolHumpdlg->get_straight_line(); // hand straight line trajectory
            tols.mov_specs.w_red_app_max = mTolHumpdlg->getW_red_app(); // set the max velocity reduction when approaching
            tols.mov_specs.w_red_ret_max = mTolHumpdlg->getW_red_ret(); // set the max velocity reduction when retreating
            // --- Move settings ---- //
            mTolHumpdlg->getTargetMove(move_target);
            mTolHumpdlg->getFinalHand(move_final_hand);
            mTolHumpdlg->getFinalArm(move_final_arm);
            use_final = mTolHumpdlg->get_use_final_posture();
            prob->setMoveSettings(move_target,move_final_hand,move_final_arm,use_final);
            tols.mov_specs.use_move_plane = mTolHumpdlg->get_add_plane();
            mTolHumpdlg->getPlaneParameters(tols.mov_specs.plane_params);

            h_results = prob->solve(tols); // plan the movement
            ui.pushButton_plan->setCheckable(false);

            if(h_results!=nullptr)
            {
                if(h_results->status==0)
                {
                    qnode.log(QNode::Info,std::string("The movement has been planned successfully"));
                    this->curr_mov = prob->getMovement();
                    this->timesteps_mov.clear();

                    this->jointsPosition_mov.clear();
                    this->jointsPosition_mov = h_results->trajectory_stages;

#if HAND == 1
                    //convert the planned gripper joint to [m]
                    std::vector<MatrixXd> jointsPosition_mov_grripper;

                    for(size_t j = 0; j < this->jointsPosition_mov.size(); ++j)
                    {
                        MatrixXd jointPosition_stage_gripper = this->jointsPosition_mov.at(j);

                        for(int i = 0; i < jointPosition_stage_gripper.rows(); ++i)
                            jointPosition_stage_gripper(i, 7) = jointPosition_stage_gripper(i, 7) / 1000;

                        jointsPosition_mov_grripper.push_back(jointPosition_stage_gripper);
                    }

                    this->jointsPosition_mov = jointsPosition_mov_grripper;
#endif

                    this->jointsVelocity_mov.clear();
                    this->jointsVelocity_mov = h_results->velocity_stages;

                    this->jointsAcceleration_mov.clear();
                    this->jointsAcceleration_mov = h_results->acceleration_stages;

                    this->traj_descr_mov.clear();
                    this->traj_descr_mov = h_results->trajectory_descriptions;


                    std::vector<double> timesteps_stage_aux;

                    for(size_t i=0; i< h_results->trajectory_stages.size();++i)
                    {
                        timesteps_stage_aux.clear();
                        double t_stage = h_results->time_steps.at(i);
                        MatrixXd traj_stage = h_results->trajectory_stages.at(i);

                        for(int j=0;j<traj_stage.rows();++j)
                        {
                            if(j==traj_stage.rows()-1)
                                timesteps_stage_aux.push_back(0.0);
                            else
                                timesteps_stage_aux.push_back(t_stage);
                        }
                        this->timesteps_mov.push_back(timesteps_stage_aux);
                    }


                    solved=true;
                }
                else
                {
                    ui.tableWidget_sol_mov->clear();
                    qnode.log(QNode::Error,std::string("The planning has failed: ")+h_results->status_msg);
                }
            }
            else
            {
                ui.tableWidget_sol_mov->clear();
                qnode.log(QNode::Error,std::string("The planning has failed: unknown status"));
            }
            break;
        case 1: // RRT
            break;
        case 2: // RRT Connect
            break;
        case 3: // RRT star
            break;
        case 4: // PRM
            break;
        case 5: // PRM star
            break;
        }
    }
    catch (const std::string message)
    {
        qnode.log(QNode::Error,std::string("Plan failure: ")+message);
    }
    catch(const std::exception exc)
    {
        qnode.log(QNode::Error,std::string("Plan failure: ")+exc.what());
    }

    // --- RESULTS --- //
    if(solved)
    {
        // time taken to solve the problem
        this->prob_time_mov = prob->getTime();
        ui.label_solving_time->setText(QString::number(this->prob_time_mov));

        uint tot_steps=0;
        QStringList h_headers; bool h_head=false; QStringList v_headers;
        double mov_duration = 0.0;
        vector<double> time; QVector<double> tot_timesteps;
        std::vector<std::vector<QString>> mov_steps;
        std::vector<MatrixXd> jointsPosition_mov_real;
        std::vector<MatrixXd> jointsPosition_mov_w_offset;

        //UR doesnt have offsets
        //the trajectory obtained doesn't include the joints offsets
        jointsPosition_mov_w_offset = this->jointsPosition_mov;
        //add the joints offsets
#if UR == 0
        jointsPosition_mov_real = qnode.robotJointPositions(jointsPosition_mov_w_offset);
#else
       // jointsPosition_mov_real = qnode.robotJointPositions(jointsPosition_mov_w_offset);
        jointsPosition_mov_real = this->jointsPosition_mov;
#endif
        for (size_t k=0; k< jointsPosition_mov_real.size();++k)
        {
            MatrixXd jointPosition_stage = jointsPosition_mov_real.at(k);
            MatrixXd jointVelocity_stage = this->jointsVelocity_mov.at(k);
            MatrixXd jointAcceleration_stage = this->jointsAcceleration_mov.at(k);
            std::vector<double> timestep_stage = this->timesteps_mov.at(k);
            std::vector<QString> stage_step;

            double time_init;
            if(time.empty())
                time_init=0.0;
            else
                time_init=time.at(time.size()-1);

            vector<double> time_stage(timestep_stage.size());
            time_stage.at(0) = time_init;
            double stage_duration = 0.0;

            for(int i = 0; i< jointPosition_stage.rows(); ++i)
            {
                tot_steps++;
                tot_timesteps.push_back(timestep_stage.at(i));

                if(i>0)
                {
                    stage_duration += timestep_stage.at(i);
                    time_stage.at(i) = time_stage.at(i-1) + timestep_stage.at(i-1);
                }
                stage_step.clear();

                v_headers.push_back(QString("Step ")+QString::number(i));
                for (int j=0; j<jointPosition_stage.cols();++j)
                {
#if HAND == 0
                    stage_step.push_back(
                                QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
#elif HAND == 1
                    if(j < JOINTS_ARM)
                        stage_step.push_back(
                                    QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                    else
                        stage_step.push_back(
                                    QString::number(jointPosition_stage(i,j)*1000,'g',3)+"|"+
                                    QString::number(jointVelocity_stage(i,j)*1000,'g',3)+"|"+
                                    QString::number(jointAcceleration_stage(i,j)*1000,'g',3));
#elif HAND == 2
                    stage_step.push_back(
                                QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
#endif
                    if(!h_head){h_headers.push_back(QString("Joint ")+QString::number(j+1));}
                }
                h_head = true;
                mov_steps.push_back(stage_step);
            }
            mov_duration += stage_duration;
            time.reserve(time_stage.size());
            std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time));
        }
        this->qtime_mov = QVector<double>::fromStdVector(time);

        // show the results
        ui.tableWidget_sol_mov->setColumnCount(h_headers.size());
        ui.tableWidget_sol_mov->setHorizontalHeaderLabels(h_headers);
        ui.tableWidget_sol_mov->setRowCount(v_headers.size());
        ui.tableWidget_sol_mov->setVerticalHeaderLabels(v_headers);

        for(int i =0; i < v_headers.size(); ++i)
        {
            std::vector<QString> row = mov_steps.at(i);
            for(int j=0; j < h_headers.size(); ++j)
            {
                QString item = row.at(j);
                ui.tableWidget_sol_mov->setItem(i,j,new QTableWidgetItem(item));
            }
        }
        ui.label_totalTime_value_mov->setText(QString::number(mov_duration).toStdString().c_str());
        ui.tabWidget_sol->setEnabled(true);

        this->tols_stop_mov.clear();
        double tol_stop = ui.lineEdit_tol_stop_mov->text().toDouble();
        for (size_t k=0; k< this->jointsPosition_mov.size();++k)
            this->tols_stop_mov.push_back(tol_stop);

        // compute the hand values, positions and accelerations
        //hand
        this->handPosition_mov.resize(tot_steps); this->handVelocityNorm_mov.resize(tot_steps);
        this->handLinearVelocity_mov.resize(tot_steps); this->handAngularVelocity_mov.resize(tot_steps);
        // wrist
        this->wristVelocityNorm_mov.resize(tot_steps);
        this->wristLinearVelocity_mov.resize(tot_steps); this->wristAngularVelocity_mov.resize(tot_steps);
#if UR == 1
        // wrist1
        this->wrist1VelocityNorm_mov.resize(tot_steps);
        this->wrist1LinearVelocity_mov.resize(tot_steps); this->wrist1AngularVelocity_mov.resize(tot_steps);
        // wrist2
        this->wrist2VelocityNorm_mov.resize(tot_steps);
        this->wrist2LinearVelocity_mov.resize(tot_steps); this->wrist2AngularVelocity_mov.resize(tot_steps);
        // wrist3
        this->wrist3VelocityNorm_mov.resize(tot_steps);
        this->wrist3LinearVelocity_mov.resize(tot_steps); this->wrist3AngularVelocity_mov.resize(tot_steps);
#endif
        // elbow
        this->elbowVelocityNorm_mov.resize(tot_steps);
        this->elbowLinearVelocity_mov.resize(tot_steps); this->elbowAngularVelocity_mov.resize(tot_steps);
        //shoulder
        this->shoulderVelocityNorm_mov.resize(tot_steps);
        this->shoulderLinearVelocity_mov.resize(tot_steps); this->shoulderAngularVelocity_mov.resize(tot_steps);

        int step = 0;
        int arm_code = prob->getMovement()->getArm();
        for (size_t k=0; k< this->jointsPosition_mov.size();++k)
        {
            MatrixXd pos_stage = this->jointsPosition_mov.at(k);
            MatrixXd vel_stage = this->jointsVelocity_mov.at(k);
            MatrixXd acc_stage = this->jointsAcceleration_mov.at(k);

            for(int i=0;i<pos_stage.rows();++i)
            {

                // position
                VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> posture;
                posture.resize(pos_row.size());
                VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
#if UR == 1
                this->curr_scene->getRobot()->getDHposture_UR(posture);
#endif
                this->curr_scene->getRobot()->getHandPos(arm_code,this->handPosition_mov.at(step),posture);

                // velocities
                VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                vector<double> velocities; velocities.resize(vel_row.size());
                VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;

                // hand velocity
                this->handVelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getHandVelNorm(arm_code,posture,velocities);
                vector<double> hand_vel; this->curr_scene->getRobot()->getHandVel(arm_code,hand_vel,posture,velocities);
                this->handLinearVelocity_mov.at(step) = {hand_vel.at(0),hand_vel.at(1),hand_vel.at(2)};
                this->handAngularVelocity_mov.at(step) = {hand_vel.at(3),hand_vel.at(4),hand_vel.at(5)};
#if UR == 0
                // wrist velocity
                this->wristVelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getWristVelNorm(arm_code,posture,velocities);
                vector<double> wrist_vel; this->curr_scene->getRobot()->getWristVel(arm_code,wrist_vel,posture,velocities);
                this->wristLinearVelocity_mov.at(step) = {wrist_vel.at(0),wrist_vel.at(1),wrist_vel.at(2)};
                this->wristAngularVelocity_mov.at(step) = {wrist_vel.at(3),wrist_vel.at(4),wrist_vel.at(5)};
#elif UR == 1
                // wrist1 velocity
                this->wrist1VelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getWristURVelNorm(arm_code,posture,velocities,1);
                vector<double> wrist_vel; this->curr_scene->getRobot()->getWristURVel(arm_code,wrist_vel,posture,velocities,1);
                this->wrist1LinearVelocity_mov.at(step) = {wrist_vel.at(0),wrist_vel.at(1),wrist_vel.at(2)};
                this->wrist1AngularVelocity_mov.at(step) = {wrist_vel.at(3),wrist_vel.at(4),wrist_vel.at(5)};

                // wrist2 velocity
                this->wrist2VelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getWristURVelNorm(arm_code,posture,velocities,2);
                vector<double> wrist2_vel; this->curr_scene->getRobot()->getWristURVel(arm_code,wrist2_vel,posture,velocities,2);
                this->wrist2LinearVelocity_mov.at(step) = {wrist2_vel.at(0),wrist2_vel.at(1),wrist2_vel.at(2)};
                this->wrist2AngularVelocity_mov.at(step) = {wrist2_vel.at(3),wrist2_vel.at(4),wrist2_vel.at(5)};

                // wrist3 velocity
                this->wrist3VelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getWristURVelNorm(arm_code,posture,velocities,3);
                vector<double> wrist3_vel; this->curr_scene->getRobot()->getWristURVel(arm_code,wrist3_vel,posture,velocities,3);
                this->wrist3LinearVelocity_mov.at(step) = {wrist3_vel.at(0),wrist3_vel.at(1),wrist3_vel.at(2)};
                this->wrist3AngularVelocity_mov.at(step) = {wrist3_vel.at(3),wrist3_vel.at(4),wrist3_vel.at(5)};

#endif
                // elbow velocity
                this->elbowVelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getElbowVelNorm(arm_code,posture,velocities);
                vector<double> elbow_vel; this->curr_scene->getRobot()->getElbowVel(arm_code,elbow_vel,posture,velocities);
                this->elbowLinearVelocity_mov.at(step) = {elbow_vel.at(0),elbow_vel.at(1),elbow_vel.at(2)};
                this->elbowAngularVelocity_mov.at(step) = {elbow_vel.at(3),elbow_vel.at(4),elbow_vel.at(5)};

                // shoulder velocity
                this->shoulderVelocityNorm_mov.at(step) = this->curr_scene->getRobot()->getShoulderVelNorm(arm_code,posture,velocities);
                vector<double> shoulder_vel; this->curr_scene->getRobot()->getShoulderVel(arm_code,shoulder_vel,posture,velocities);
                this->shoulderLinearVelocity_mov.at(step) = {shoulder_vel.at(0),shoulder_vel.at(1),shoulder_vel.at(2)};
                this->shoulderAngularVelocity_mov.at(step) = {shoulder_vel.at(3),shoulder_vel.at(4),shoulder_vel.at(5)};

                step++;
            }
        }

        // normalized jerk cost of the hand
        QVector<double> handPosition_mov_x; QVector<double> handPosition_mov_y; QVector<double> handPosition_mov_z;
        QVector<double> der_1_handPosition_mov_x; QVector<double> der_1_handPosition_mov_y; QVector<double> der_1_handPosition_mov_z;
        QVector<double> der_2_handPosition_mov_x; QVector<double> der_2_handPosition_mov_y; QVector<double> der_2_handPosition_mov_z;
        QVector<double> der_3_handPosition_mov_x; QVector<double> der_3_handPosition_mov_y; QVector<double> der_3_handPosition_mov_z;

        for(size_t i=0; i<this->handPosition_mov.size();++i)
        {
            vector<double> position_i = this->handPosition_mov.at(i);
            handPosition_mov_x.push_back(position_i.at(0));
            handPosition_mov_y.push_back(position_i.at(1));
            handPosition_mov_z.push_back(position_i.at(2));
        }

        // derivatives
        this->getDerivative(handPosition_mov_x,tot_timesteps,der_1_handPosition_mov_x);
        this->getDerivative(der_1_handPosition_mov_x,tot_timesteps,der_2_handPosition_mov_x);
        this->getDerivative(der_2_handPosition_mov_x,tot_timesteps,der_3_handPosition_mov_x);
        this->getDerivative(handPosition_mov_y,tot_timesteps,der_1_handPosition_mov_y);
        this->getDerivative(der_1_handPosition_mov_y,tot_timesteps,der_2_handPosition_mov_y);
        this->getDerivative(der_2_handPosition_mov_y,tot_timesteps,der_3_handPosition_mov_y);
        this->getDerivative(handPosition_mov_z,tot_timesteps,der_1_handPosition_mov_z);
        this->getDerivative(der_1_handPosition_mov_z,tot_timesteps,der_2_handPosition_mov_z);
        this->getDerivative(der_2_handPosition_mov_z,tot_timesteps,der_3_handPosition_mov_z);

        QVector<double> jerk_hand;
        for(size_t i=0;i<handPosition_mov_x.size();++i)
            jerk_hand.push_back(sqrt(pow(der_3_handPosition_mov_x.at(i),2)+pow(der_3_handPosition_mov_y.at(i),2)+pow(der_3_handPosition_mov_z.at(i),2)));

        double duration = this->qtime_mov.at(this->qtime_mov.size()-1);
        double length = sqrt(pow((handPosition_mov_x.at(handPosition_mov_x.size()-1)-handPosition_mov_x.at(0)),2)+
                             pow((handPosition_mov_y.at(handPosition_mov_y.size()-1)-handPosition_mov_y.at(0)),2)+
                             pow((handPosition_mov_z.at(handPosition_mov_z.size()-1)-handPosition_mov_z.at(0)),2));
        double total_cost_jerk_hand=0.0;

        for(size_t i=0;i<tot_timesteps.size();++i)
            total_cost_jerk_hand += pow(jerk_hand.at(i),2)*tot_timesteps.at(i);

        total_cost_jerk_hand = sqrt(0.5*total_cost_jerk_hand*(pow(duration,5)/pow(length,2)));
        ui.label_cost_hand_value->setText(QString::number(total_cost_jerk_hand));
        this->njs_mov = total_cost_jerk_hand;

        // -- compute the number of movement units -- //
        this->nmu_mov = this->getNumberMovementUnits(this->handVelocityNorm_mov,this->qtime_mov);
        ui.label_nmu->setText(QString::number(this->nmu_mov));

    }

}


void MainWindow::on_pushButton_plan_trials_clicked()
{
    int trials = 100;
    int success = 0;

    for (int i =0; i<trials;++i)
    {
        this->on_pushButton_plan_clicked();
        if(this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getSolved())
        {
            success++;
            this->on_pushButton_append_mov_clicked();
        }
    }
    double rate = 100*success/trials;
    ui.label_rate_task->setText(QString::number(rate));
}


void MainWindow::on_pushButton_execMov_clicked()
{
    //If the dialog hasn't been displayed or the user hasn't chosen the "don't ask again" option
    if(execSettings_move == false)
        mMovExecutedlg->show();
    else
    {
        if(usedPlat_move == 0) //Execute the planned movement in V-Rep simulator
            qnode.execMovement(this->jointsPosition_mov, this->timesteps_mov, this->tols_stop_mov, this->traj_descr_mov, this->curr_mov, this->curr_scene);
        else if(usedPlat_move == 1) //Execute the planned movement in Robot
        {
#if ROBOT == 1
#if UR == 0
            if(this->curr_scene->getRobot()->getName() == "Sawyer")
            {
                std::vector<double> paramsTimeMapping;
                mMovExecutedlg->getTimeMappingParams(paramsTimeMapping);

                qnode.execMovementSawyer(this->jointsPosition_mov, this->timesteps_mov, this->traj_descr_mov, this->curr_mov, paramsTimeMapping);
             }
#else
            std::vector<double> paramsTimeMapping;
            mMovExecutedlg->getTimeMappingParams(paramsTimeMapping);
            qnode.execMovementUR(this->jointsPosition_mov, this->timesteps_mov, this->traj_descr_mov, this->curr_mov, paramsTimeMapping);
#endif
#endif
        }
    }
}


void MainWindow::on_pushButton_stop_mov_clicked()
{
    qnode.stopSim();
    qnode.resetSimTime();
    qnode.resetGlobals();
}


void MainWindow::on_pushButton_stop_task_clicked()
{
    qnode.stopSim();
    qnode.resetSimTime();
    qnode.resetGlobals();
}


void MainWindow::on_pushButton_save_end_posture_clicked()
{
    this->jointsEndAcceleration_mov.clear();
    this->jointsEndVelocity_mov.clear();
    this->jointsEndPosition_mov.clear();

    if(!this->jointsPosition_mov.empty())
    {
        MatrixXd joints_pos = this->jointsPosition_mov.back();
        MatrixXd joints_vel = this->jointsVelocity_mov.back();
        MatrixXd joints_acc = this->jointsAcceleration_mov.back();

        VectorXd end_pos = joints_pos.row(joints_pos.rows()-1);
        VectorXd end_vel = joints_vel.row(joints_vel.rows()-1);
        VectorXd end_acc = joints_acc.row(joints_acc.rows()-1);

        this->jointsEndPosition_mov.resize(end_pos.size());
        VectorXd::Map(&this->jointsEndPosition_mov[0], end_pos.size()) = end_pos;

        this->jointsEndVelocity_mov.resize(end_vel.size());
        VectorXd::Map(&this->jointsEndVelocity_mov[0], end_vel.size()) = end_vel;
        this->jointsEndAcceleration_mov.resize(end_acc.size());
        VectorXd::Map(&this->jointsEndAcceleration_mov[0], end_acc.size()) = end_acc;
    }
}


void MainWindow::on_pushButton_execTask_clicked()
{
    //If the dialog hasn't been displayed or the user hasn't chosen the "don't ask again" option
    if(execSettings_task == false)
        mTaskExecutedlg->show();
    else
    {
        if(usedPlat_task == 0) //Execute the task in Vrep simulator
            qnode.execTask(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, this->curr_scene);
        else if(usedPlat_task == 1) //Execute the task in Robot
        {
#if ROBOT == 1
#if UR == 0
            if(this->curr_scene->getRobot()->getName() == "Sawyer")
            {
                std::vector<vector<double>> paramsTimeMapping;
                mTaskExecutedlg->getTimeMappingParams(paramsTimeMapping);
                qnode.execTaskSawyer(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, paramsTimeMapping);
            }
#else
           // std::vector<vector<double>> paramsTimeMapping;
           // mTaskExecutedlg->getTimeMappingParams(paramsTimeMapping);
           // qnode.execTaskUR(this->jointsPosition_task, this->timesteps_task, this->tols_stop_task, this->traj_descr_task, this->curr_task, paramsTimeMapping);
#endif
#endif
        }
    }
}


void MainWindow::on_pushButton_load_task_clicked()
{
    int plan_id;  QString plan_type;
    int mov_id;  QString mov_type;
    int arm_code; QString arm_type;

    QString obj_str; objectPtr obj;
    QString obj_eng_str; objectPtr obj_eng;
    QString pose_str; posePtr pose;

    bool prec;
    QString grip_type;
    int row=0;
    MatrixXd pos_stage;
    MatrixXd vel_stage;
    MatrixXd acc_stage;

    // Clear all scene
    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->traj_descr_task.clear();
    this->timesteps_task.clear();
    this->tols_stop_task.clear();
    this->njs_task.clear();
    this->nmu_task.clear();
    this->prob_time_task.clear();
    this->jointsPosition_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsAcceleration_mov.clear();
    this->traj_descr_mov.clear();
    this->timesteps_mov.clear();
    this->tols_stop_mov.clear();
    ui.tableWidget_sol_task->clear();
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->curr_task->clearProblems();

    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load the task trajectory"),
                                                    QString(MAIN_PATH)+"/Tasks",
                                                    "All Files (*.*);; Task Files (*.task)");
    QFile f(filename);
    if(f.open(QIODevice::ReadOnly))
    {
        QTextStream stream( &f );
        QString line;

        vector<MatrixXd> t_mov;
        vector<MatrixXd> w_mov;
        vector<MatrixXd> a_mov;
        vector<vector<double>> timesteps_mov;
        vector<double> timesteps_stage;
        vector<double> tols_stop_mov;
        vector<string> descr_mov;

        while(!stream.atEnd())
        {
            line = f.readLine();

            if(line.at(0)==QChar('#'))
            {
                // the previous stage has finished
                if(pos_stage.rows()!=0)
                {
                    t_mov.push_back(pos_stage);
                    w_mov.push_back(vel_stage);
                    a_mov.push_back(acc_stage);
                    timesteps_mov.push_back(timesteps_stage);
                }
                // the previous movement has finished
                if(!t_mov.empty())
                {
                    this->jointsPosition_task.push_back(t_mov);
                    this->jointsVelocity_task.push_back(w_mov);
                    this->jointsAcceleration_task.push_back(a_mov);
                    this->timesteps_task.push_back(timesteps_mov);
                    this->tols_stop_task.push_back(tols_stop_mov);
                    this->traj_descr_task.push_back(descr_mov);
                }
                // new movement in the task
                t_mov.clear();
                w_mov.clear();
                a_mov.clear();
                timesteps_mov.clear();
                tols_stop_mov.clear();
                descr_mov.clear();
                // new stage in the movement
                pos_stage.resize(0,0);
                vel_stage.resize(0,0);
                acc_stage.resize(0,0);
                timesteps_stage.clear();

                if((line.at(1)==QChar('E')) && (line.at(2)==QChar('N')) && (line.at(3)==QChar('D')))
                {
                    break;
                }

                QStringList fields = line.split(",");
                QString tmp = fields.at(0);
                tmp.remove(QChar('#'));
                fields[0]=tmp;

                // ------------------------- Informations about movement ----------------------------------- //
                for(int i=0; i< fields.size(); ++i)
                {
                    QStringList fields1 = fields.at(i).split(":");

                    if (QString::compare(fields1.at(0).simplified(),QString("Planner"),Qt::CaseInsensitive)==0)
                        plan_type = fields1.at(1).simplified();
                    else if (QString::compare(fields1.at(0).simplified(),QString("Movement"),Qt::CaseInsensitive)==0)
                        mov_type = fields1.at(1).simplified();
                    else if(QString::compare(fields1.at(0).simplified(),QString("Arm"),Qt::CaseInsensitive)==0)
                        arm_type=fields1.at(1).simplified();
                    else if(QString::compare(fields1.at(0).simplified(),QString("Object"),Qt::CaseInsensitive)==0)
                        obj_str=fields1.at(1).simplified();
                    else if(QString::compare(fields1.at(0).simplified(),QString("Object Engaged"),Qt::CaseInsensitive)==0)
                        obj_eng_str=fields1.at(1).simplified();
                    else if(QString::compare(fields1.at(0).simplified(),QString("Pose"),Qt::CaseInsensitive)==0)
                        pose_str=fields1.at(1).simplified();
                    else if(QString::compare(fields1.at(0).simplified(),QString("Grip Type"),Qt::CaseInsensitive)==0)
                        grip_type=fields1.at(1).simplified();
                }

                //Planner ID
                if(QString::compare(plan_type,QString("HUMP"),Qt::CaseInsensitive)==0)
                    plan_id=0;

                // Grip type
                if(QString::compare(grip_type,QString("Precision"),Qt::CaseInsensitive)==0)
                    prec=true;
                else
                    prec=false;

                // Arm (both, single right, single left)
                if(QString::compare(arm_type,QString("both"),Qt::CaseInsensitive)==0)
                    arm_code=0;
                else if(QString::compare(arm_type,QString("right"),Qt::CaseInsensitive)==0)
                    arm_code=1;
                else if(QString::compare(arm_type,QString("left"),Qt::CaseInsensitive)==0)
                    arm_code=2;

                // Movement type
                if(QString::compare(mov_type,QString("Reach-to-grasp"),Qt::CaseInsensitive)==0)
                {
                    mov_id=0;
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());

                    switch (arm_code)
                    {
                    case 0: // dual arm
                        break;
                    case 1: // right arm
                        obj->setTargetRightEnabled(true);
                        obj->setTargetLeftEnabled(false);
                        break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        break;
                    }

                    problemPtr prob;
                    if(plan_id==0)
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,prec),new Scenario(*(this->curr_scene.get()))));

                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }
                else if(QString::compare(mov_type,QString("Reaching"),Qt::CaseInsensitive)==0)
                {
                    mov_id=1;
                    problemPtr prob;

                    if(plan_id==0)
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));

                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }
                else if(QString::compare(mov_type,QString("Transport"),Qt::CaseInsensitive)==0)
                {
                    mov_id=2;
                    problemPtr prob;
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                    // get the pose
                    pose = this->curr_scene->getPose(pose_str.toStdString());

                    if(plan_id==0)
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,pose,prec),new Scenario(*(this->curr_scene.get()))));

                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }
                else if(QString::compare(mov_type,QString("Engage"),Qt::CaseInsensitive)==0)
                {
                    mov_id=3;
                    //get the object
                    obj = this->curr_scene->getObject(obj_str.toStdString());
                    // get the object engaged
                    obj_eng = this->curr_scene->getObject(obj_eng_str.toStdString());

                    switch (arm_code)
                    {
                    case 0: // dual arm
                        break;
                    case 1: // right arm
                        obj->setTargetRightEnabled(true);
                        obj->setTargetLeftEnabled(false);
                        break;
                    case 2: // left arm
                        obj->setTargetLeftEnabled(true);
                        obj->setTargetRightEnabled(false);
                        break;
                    }

                    problemPtr prob;
                    if(plan_id==0)
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code, obj,obj_eng,prec),new Scenario(*(this->curr_scene.get()))));

                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }
                else if(QString::compare(mov_type,QString("Disengage"),Qt::CaseInsensitive)==0)
                    mov_id=4;
                else if(QString::compare(mov_type,QString("Go park"),Qt::CaseInsensitive)==0)
                {
                    mov_id=5;
                    problemPtr prob;

                    if(plan_id==0)
                        prob = problemPtr(new Problem(plan_id,new Movement(mov_id, arm_code),new Scenario(*(this->curr_scene.get()))));

                    prob->setSolved(true);
                    prob->setPartOfTask(true);
                    this->curr_task->addProblem(prob.get());
                }

                //logging
                qnode.log(QNode::Info,std::string("The movement has been added to the current task"));
                ui.groupBox_task->setEnabled(true);
                ui.listWidget_movs->clear();
                for (int i = 0; i < this->curr_task->getProblemNumber();i++ )
                    ui.listWidget_movs->addItem(QString(this->curr_task->getProblemInfo(i).c_str()));

                ui.listWidget_movs->setCurrentRow(0);
            }
            else if(line.at(0)==QChar('M'))
            {
                QStringList fields = line.split(":");
                if(QString::compare(fields.at(0).simplified(),QString("Movement stage"),Qt::CaseInsensitive)==0)
                    descr_mov.push_back((fields.at(1).simplified()).toStdString());

                // the previous stage has finished
                if(pos_stage.rows()!=0)
                {
                    t_mov.push_back(pos_stage);
                    w_mov.push_back(vel_stage);
                    a_mov.push_back(acc_stage);
                    timesteps_mov.push_back(timesteps_stage);
                }
                // new stage in the movement
                pos_stage.resize(0,0);
                vel_stage.resize(0,0);
                acc_stage.resize(0,0);
                timesteps_stage.clear();
                row=0;
            }
            else if(line.at(0)==QChar('t'))
            {
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("tol stop"),Qt::CaseInsensitive)==0)
                    tols_stop_mov.push_back(fields.at(1).toDouble());
            }
            else if((line.at(0)==QChar('n')) && (line.at(1)==QChar('j')))
            {
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("njs"),Qt::CaseInsensitive)==0)
                    this->njs_task.push_back(fields.at(1).toDouble());
            }
            else if((line.at(0)==QChar('n')) && (line.at(1)==QChar('m')))
            {
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("nmu"),Qt::CaseInsensitive)==0)
                    this->nmu_task.push_back(fields.at(1).toDouble());
            }
            else if(line.at(0)==QChar('p'))
            {
                QStringList fields = line.split("=");
                if(QString::compare(fields.at(0).simplified(),QString("prob_time"),Qt::CaseInsensitive)==0)
                    this->prob_time_task.push_back(fields.at(1).toDouble());
            }
            else
            {
                pos_stage.conservativeResize(pos_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                vel_stage.conservativeResize(vel_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);
                acc_stage.conservativeResize(acc_stage.rows()+1,JOINTS_ARM+JOINTS_HAND);

                QStringList fields = line.split(",");
                for(int i=0; i <fields.size();++i)
                {
                    QStringList fields1 = fields.at(i).split("=");
                    if(QString::compare(fields1.at(0).simplified(),QString("time step"),Qt::CaseInsensitive)==0)
                        timesteps_stage.push_back(fields1.at(1).toDouble());

                    for(int k=0; k < JOINTS_ARM + JOINTS_HAND; ++k)
                    {
                        if(QString::compare(fields1.at(0).simplified(),QString("Joint ")+QString::number(k+1),Qt::CaseInsensitive)==0)
                        {
                            QStringList fields2 = fields1.at(1).split("|");

#if HAND == 0
                            pos_stage(row,k) = ((double)fields2.at(0).toDouble()*M_PI)/180;
                            vel_stage(row,k) = ((double)fields2.at(1).toDouble()*M_PI)/180;
                            acc_stage(row,k) = ((double)fields2.at(2).toDouble()*M_PI)/180;
#elif HAND == 1
                            if (k < JOINTS_ARM)
                            {
                                pos_stage(row,k) = ((double)fields2.at(0).toDouble()*M_PI)/180;
                                vel_stage(row,k) = ((double)fields2.at(1).toDouble()*M_PI)/180;
                                acc_stage(row,k) = ((double)fields2.at(2).toDouble()*M_PI)/180;
                            }
                            else
                            {
                                pos_stage(row,k) = ((double)fields2.at(0).toDouble()/1000);
                                vel_stage(row,k) = ((double)fields2.at(1).toDouble()/1000);
                                acc_stage(row,k) = ((double)fields2.at(2).toDouble()/1000);
                            }
#endif
                        }
                        else if(QString::compare(fields1.at(0).simplified(),QString("step"),Qt::CaseInsensitive)==0)
                        {
                            break;
                        }
                    }
                }
                row++;
            }
        }

        qnode.log(QNode::Info,std::string("The task has been loaded"));

        QStringList h_headers;
        bool h_head=false;
        QStringList v_headers;

        std::vector<std::vector<QString>> task_steps;
        vector<MatrixXd> pos_mov_w_offset;
        vector<MatrixXd> pos_mov;
        vector<MatrixXd> vel_mov;
        vector<MatrixXd> acc_mov;

        vector<vector<double>> tstep_mov;
        vector<double> tstep_stage;
        double task_duration = 0.0;
        double mov_duration = 0.0;
        double stage_duration = 0.0;
        vector<double> time_task;
        uint tot_steps = 0;

        for(size_t h=0; h< this->jointsPosition_task.size();++h)
        {
            pos_mov_w_offset = this->jointsPosition_task.at(h);
            pos_mov = qnode.robotJointPositions(pos_mov_w_offset);
            vel_mov = this->jointsVelocity_task.at(h);
            acc_mov = this->jointsAcceleration_task.at(h);
            tstep_mov = this->timesteps_task.at(h);
            mov_duration = 0.0;

            for (size_t k=0; k< pos_mov.size();++k)
            {
                MatrixXd jointPosition_stage = pos_mov.at(k);
                MatrixXd jointVelocity_stage = vel_mov.at(k);
                MatrixXd jointAcceleration_stage = acc_mov.at(k);
                tstep_stage = tstep_mov.at(k);

                vector<double> time_stage(tstep_stage.size());
                double time_init;
                if(time_task.empty())
                    time_init=0.0;
                else
                    time_init=time_task.at(time_task.size()-1);

                time_stage.at(0) = time_init;
                stage_duration = 0.0;
                std::vector<QString> stage_step;
                stage_duration = 0.0;

                for(int i =0; i< jointPosition_stage.rows(); ++i)
                {
                    tot_steps++;
                    if(i>0)
                    {
                        time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                        stage_duration += tstep_stage.at(i);
                    }
                    stage_step.clear();
                    v_headers.push_back(QString("Step ")+QString::number(i));

                    for (int j=0; j<jointPosition_stage.cols();++j)
                    {
#if HAND == 0
                        stage_step.push_back(
                                    QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
#elif HAND == 1
                        if( j < JOINTS_ARM)
                            stage_step.push_back(
                                        QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                        QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                        QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                        else
                            stage_step.push_back(
                                        QString::number(jointPosition_stage(i,j)*1000,'g',3)+"|"+
                                        QString::number(jointVelocity_stage(i,j)*1000,'g',3)+"|"+
                                        QString::number(jointAcceleration_stage(i,j)*1000,'g',3));
#endif

                        if(!h_head)
                            h_headers.push_back(QString("Joint ")+QString::number(j+1));
                    }
                    h_head = true;
                    task_steps.push_back(stage_step);
                }
                mov_duration += stage_duration;
                time_task.reserve(time_stage.size());
                std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_task));
            }
            task_duration +=mov_duration;
        }

        this->qtime_task = QVector<double>::fromStdVector(time_task);

        ui.tableWidget_sol_task->setColumnCount(h_headers.size());
        ui.tableWidget_sol_task->setHorizontalHeaderLabels(h_headers);
        ui.tableWidget_sol_task->setRowCount(v_headers.size());
        ui.tableWidget_sol_task->setVerticalHeaderLabels(v_headers);

        for(int i =0; i < v_headers.size(); ++i)
        {
            std::vector<QString> row = task_steps.at(i);
            for(int j=0; j < h_headers.size(); ++j)
            {
                QString item = row.at(j);
                ui.tableWidget_sol_task->setItem(i,j,new QTableWidgetItem(item));
            }
        }

        ui.label_totalTime_value_task->setText(QString::number(task_duration).toStdString().c_str());
        ui.tabWidget_sol->setEnabled(true);
        ui.tabWidget_sol->setCurrentIndex(1);

        // compute the hand values
        this->handPosition_task.resize(tot_steps); this->handVelocityNorm_task.resize(tot_steps);
        int step = 0;

        for(size_t j=0;j<this->jointsPosition_task.size();++j)
        {
            vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
            vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);

            for (size_t k=0; k< pos_mov.size();++k)
            {
                MatrixXd pos_stage = pos_mov.at(k);
                MatrixXd vel_stage = vel_mov.at(k);

                for(int i=0;i<pos_stage.rows();++i)
                {
                    // position
                    VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                    vector<double> posture; posture.resize(pos_row.size());
                    VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
                    this->curr_scene->getRobot()->getHandPos(arm_code,this->handPosition_task.at(step),posture);
                    // velocity norm
                    VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                    vector<double> velocities; velocities.resize(vel_row.size());
                    VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                    this->handVelocityNorm_task.at(step) = this->curr_scene->getRobot()->getHandVelNorm(arm_code,posture,velocities);
                    step++;
                }
            }
        }

        // --------------------------  Compute njs, nmu and planning time  ------------------------------- //
        // njs
        double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
        double mean_njs = ((double)sum_njs) / this->njs_task.size();
        string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
        boost::replace_all(mean_njs_str,",",".");
        double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
        double stdev_njs = std::sqrt((((double)sq_sum_njs) / this->njs_task.size()) - pow(mean_njs,2));
        string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
        boost::replace_all(stdev_njs_str,",",".");
        ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));

        // nmu
        double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
        double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
        string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
        boost::replace_all(mean_nmu_str,",",".");
        double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
        double stdev_nmu = std::sqrt((((double)sq_sum_nmu) / this->nmu_task.size()) - pow(mean_nmu,2));
        string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
        boost::replace_all(stdev_nmu_str,",",".");
        ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));

        // planning time
        double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
        double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
        string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
        boost::replace_all(mean_prob_str,",",".");
        double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
        double stdev_prob = std::sqrt((((double)sq_sum_prob) / this->prob_time_task.size()) - pow(mean_prob,2));
        string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
        boost::replace_all(stdev_prob_str,",",".");
        ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
    }
    f.close();
}



void MainWindow::on_pushButton_save_task_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the task trajectory"),
                                                    QString(MAIN_PATH)+"/Tasks",
                                                    "All Files (*.*);;Task Files (*.task)");
    QFile f(filename);
    if(f.open(QIODevice::WriteOnly))
    {
        QTextStream stream(&f);
        int h=0; // numbers of problems that are not in the current task


        for(int i=0; i <ui.listWidget_movs->count(); ++i)
        {
            if((curr_task->getProblem(i)->getSolved()) && curr_task->getProblem(i)->getPartOfTask())
            {
                stream << "# " << ui.listWidget_movs->item(i)->text().toStdString().c_str()<< endl;
                vector< MatrixXd > traj_mov = this->jointsPosition_task.at(i-h);
                vector< MatrixXd > vel_mov = this->jointsVelocity_task.at(i-h);
                vector< MatrixXd > acc_mov = this->jointsAcceleration_task.at(i-h);
                vector< vector< double > > timesteps_mov = this->timesteps_task.at(i-h);
                vector< double > tols_stop_mov = this->tols_stop_task.at(i-h);
                vector< string > traj_descr_mov = this->traj_descr_task.at(i-h);

                double njs = this->njs_task.at(i-h);
                int nmu = this->nmu_task.at(i-h);
                double prob_time = this->prob_time_task.at(i-h);
                stream << "njs="<< QString::number(njs).toStdString().c_str()<< endl;
                stream << "nmu="<< QString::number(nmu).toStdString().c_str()<< endl;
                stream << "prob_time="<< QString::number(prob_time).toStdString().c_str()<< endl;

                for(size_t j=0;j < traj_mov.size(); ++j)
                {
                    string descr_stage = traj_descr_mov.at(j);
                    stream << "Movement stage: "<< descr_stage.c_str()<< endl;
                    MatrixXd traj = traj_mov.at(j);
                    MatrixXd vel = vel_mov.at(j);
                    MatrixXd acc = acc_mov.at(j);

                    vector< double > timestep_stage  = timesteps_mov.at(j);
                    double tol_stop = tols_stop_mov.at(j);
                    stream << "tol stop="<< QString::number(tol_stop).toStdString().c_str()<< endl;

                    for(int r=0; r < traj.rows(); ++r)
                    {
                        double timestep = timestep_stage.at(r);
                        stream << "step="<< QString::number(r).toStdString().c_str()<<", ";
                        stream << "time step="<< QString::number(timestep).toStdString().c_str()<< ", ";
                        for(int c=0; c < traj.cols(); ++c)
                        {
#if HAND == 0
                            stream << "Joint "<<QString::number(c+1).toStdString().c_str()<<"="<<
                                      QString::number(((double)traj(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                      QString::number(((double)vel(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                      QString::number(((double)acc(r,c)*180)/M_PI,'f',6).toStdString().c_str();
#elif HAND == 1
                            if(c < JOINTS_ARM)
                                stream << "Joint "<<QString::number(c+1).toStdString().c_str()<<"="<<
                                          QString::number(((double)traj(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                          QString::number(((double)vel(r,c)*180)/M_PI,'f',6).toStdString().c_str()<<"|"<<
                                          QString::number(((double)acc(r,c)*180)/M_PI,'f',6).toStdString().c_str();
                            else
                                stream << "Joint "<<QString::number(c+1).toStdString().c_str()<<"="<<
                                          QString::number(((double)traj(r,c)) * 1000,'f',6).toStdString().c_str()<<"|"<<
                                          QString::number(((double)vel(r,c)) * 1000,'f',6).toStdString().c_str()<<"|"<<
                                          QString::number(((double)acc(r,c)) * 1000,'f',6).toStdString().c_str();
#endif
                            if(c==traj.cols()-1)
                                stream << endl;
                            else
                                stream<<", ";
                        }
                    }
                }
            }
            else
                h++;
        }
        stream << "#END" <<endl;
    }
    f.close();
}


void MainWindow::on_pushButton_scene_reset_clicked()
{
    // reset the movements
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_mov->clear();
    this->jointsAcceleration_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsPosition_mov.clear();
    this->timesteps_mov.clear();
    this->tols_stop_mov.clear();

    this->curr_scene = scenarioPtr(new Scenario(*(this->init_scene.get())));
    qnode.resetSimTime();
    qnode.resetGlobals();
    qnode.log(QNode::Info,std::string("The scenario has been reset"));

    int scene_id = this->scenario_id;
    string path;
    string title;
    string success;
    string failure;

    // Toy vehicle scenario with ARoS
    string path_vrep_toyscene_aros = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_aros_bill.ttt");
    // Drinking Service task with ARoS
    string path_vrep_drinking_aros = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_aros_bill.ttt");
    // Toy vehicle scenario with Jarde
    string path_vrep_toyscene_jarde = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_jarde.ttt");
    // Toy vehicle scenario with Sawyer
    string path_vrep_toyscene_sawyer = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_sawyer_bill.ttt");
    // Drinking Service task with Sawyer
    string path_vrep_drinking_sawyer = PATH_SCENARIOS+string("/vrep/DrinkingServiceTask_sawyer_bill.ttt");
    // Toy vehicle scenario with Sawyer
    string path_vrep_toyscene_sawyer_gripper = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_sawyer_gripper_bill.ttt");
    // Toy vehicle scenario with Sawyer
    string path_vrep_toyscene_sawyer_gripper_interaction = PATH_SCENARIOS+string("/vrep/ToyVehicleTask_sawyer_gripper_bill_interaction.ttt");
    // Pick and Place scenario with Universal Robot
    string path_vrep_HRColab_UR = PATH_SCENARIOS + string("/vrep/HRColab_IKEA.ttt");


    switch(scene_id)
    {
    case 0:
        // Assembly scenario: the Toy vehicle with ARoS
        path = path_vrep_toyscene_aros;
        title = string("Assembly scenario: the Toy vehicle with ARoS");
        success = string("Assembly scenario: the Toy vehicle with ARoS HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with ARoS HAS NOT BEEN LOADED");
        break;
    case 1:
        // Assistive scenario: beverages with ARoS
        path = path_vrep_drinking_aros;
        title = string("Human assistance scenario: Serving a drink with ARoS");
        success = string("Human assistance scenario: Serving a drink with ARoS HAS BEEN LOADED");
        failure = string("Human assistance scenario: Serving a drink with ARoS HAS NOT BEEN LOADED");
        break;
    case 2:
        // Assembly scenario: the Toy vehicle with Sawyer
        path = path_vrep_toyscene_sawyer;
        title = string("Assembly scenario: the Toy vehicle with Sawyer");
        success = string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED");
        break;
    case 3:
        // Assistive scenario: beverages with Sawyer
        path = path_vrep_drinking_sawyer;
        title = string("Human assistance scenario: Serving a drink with Sawyer");
        success = string("Human assistance scenario: Serving a drink with Sawyer HAS BEEN LOADED");
        failure = string("Human assistance scenario: Serving a drink with Sawyer HAS NOT BEEN LOADED");
        break;
    case 4:
        // Assembly scenario: the Toy vehicle with Sawyer (with Gripper)
        path = path_vrep_toyscene_sawyer_gripper;
        title = string("Assembly scenario: the Toy vehicle with Sawyer");
        success = string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED");
        break;
    case 5:
        // Assembly scenario: the Toy vehicle with Sawyer (with Gripper)
        path = path_vrep_toyscene_sawyer_gripper_interaction;
        title = string("Assembly scenario: the Toy vehicle with Sawyer");
        success = string("Assembly scenario: the Toy vehicle with Sawyer HAS BEEN LOADED");
        failure = string("Assembly scenario: the Toy vehicle with Sawyer HAS NOT BEEN LOADED");
        break;
    case 6:
        // Pick and Place scenario with Universal Robot
        path = path_vrep_HRColab_UR;
        title = string("Pick and Place scenario with Universal Robot");
        success = string("Pick and Place scenario with Universal Robot HAS BEEN LOADED");
        failure = string("Pick and Place scenario with Universal Robot HAS NOT BEEN LOADED");
        break;
    }


    if (qnode.loadScenario(path,1))
    {
        qnode.log(QNode::Info,success);
        ui.groupBox_getElements->setEnabled(true);
        ui.groupBox_homePosture->setEnabled(true);
        std::vector<objectPtr> objs; this->curr_scene->getObjects(objs);
    }
    else
    {
        qnode.log(QNode::Error,failure);
        ui.groupBox_getElements->setEnabled(false);
        ui.groupBox_homePosture->setEnabled(false);
        ui.pushButton_loadScenario->setEnabled(true);
    }
}


void MainWindow::on_pushButton_append_mov_clicked()
{
    ui.tableWidget_sol_task->clear();
    ui.pushButton_save_task->setEnabled(true);

    if(curr_task->getProblem(ui.listWidget_movs->currentRow())->getSolved())
    {
        this->jointsPosition_task.push_back(this->jointsPosition_mov);
        this->jointsVelocity_task.push_back(this->jointsVelocity_mov);
        this->jointsAcceleration_task.push_back(this->jointsAcceleration_mov);
        this->timesteps_task.push_back(this->timesteps_mov);
        this->tols_stop_task.push_back(this->tols_stop_mov);
        this->traj_descr_task.push_back(this->traj_descr_mov);
        this->njs_task.push_back(this->njs_mov);
        this->nmu_task.push_back(this->nmu_mov);
        this->prob_time_task.push_back(this->prob_time_mov);

        QStringList h_headers;
        bool h_head=false;
        QStringList v_headers;
        std::vector<std::vector<QString>> task_steps;
        vector<MatrixXd> pos_mov;
        std::vector<MatrixXd> pos_mov_real;
        std::vector<MatrixXd> pos_mov_w_offset;
        vector<MatrixXd> vel_mov;
        vector<MatrixXd> acc_mov;
        vector<vector<double>> tstep_mov;
        double task_duration = 0.0;
        double mov_duration = 0.0;
        double stage_duration = 0.0;
        vector<double> time_task;
        uint tot_steps = 0;

        for(size_t h=0; h< this->jointsPosition_task.size();++h)
        {
            pos_mov = this->jointsPosition_task.at(h);
            //the trajectory obtained doesn't include the joints offs
            pos_mov_w_offset = pos_mov;
#if UR==0
            //add the joints offsets
            pos_mov_real = qnode.robotJointPositions(pos_mov_w_offset);
#elif UR == 1
            pos_mov_real = pos_mov;
#endif
            vel_mov = this->jointsVelocity_task.at(h);
            acc_mov = this->jointsAcceleration_task.at(h);
            tstep_mov = this->timesteps_task.at(h);
            mov_duration = 0.0;

            for (size_t k=0; k< pos_mov_real.size();++k)
            {
                MatrixXd jointPosition_stage = pos_mov_real.at(k);
                MatrixXd jointVelocity_stage = vel_mov.at(k);
                MatrixXd jointAcceleration_stage = acc_mov.at(k);
                vector<double> tstep_stage = tstep_mov.at(k);
                vector<double> time_stage(tstep_stage.size());

                double time_init;
                if(time_task.empty())
                    time_init=0.0;
                else
                    time_init=time_task.at(time_task.size()-1);

                time_stage.at(0) = time_init;
                stage_duration = 0.0;
                std::vector<QString> stage_step;

                for(int i =0; i< jointPosition_stage.rows(); ++i)
                {
                    tot_steps++;
                    if(i>0)
                    {
                        time_stage.at(i) = time_stage.at(i-1) + tstep_stage.at(i-1);
                        stage_duration += tstep_stage.at(i);
                    }

                    stage_step.clear();
                    v_headers.push_back(QString("Step ")+QString::number(i));

                    for (int j=0; j<jointPosition_stage.cols();++j)
                    {
#if HAND == 0
                        stage_step.push_back(
                                    QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                    QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
#elif HAND == 1
                        if (j < JOINTS_ARM)
                            stage_step.push_back(
                                        QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                        QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                        QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
                        else
                            stage_step.push_back(
                                        QString::number(jointPosition_stage(i,j)*1000,'g',3)+"|"+
                                        QString::number(jointVelocity_stage(i,j)*1000,'g',3)+"|"+
                                        QString::number(jointAcceleration_stage(i,j)*1000,'g',3));
#elif HAND == 2
                    stage_step.push_back(
                                QString::number(jointPosition_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointVelocity_stage(i,j)*180/M_PI,'g',3)+"|"+
                                QString::number(jointAcceleration_stage(i,j)*180/M_PI,'g',3));
#endif
                        if(!h_head)
                            h_headers.push_back(QString("Joint ")+QString::number(j+1));
                    }
                    h_head = true;
                    task_steps.push_back(stage_step);
                }
                mov_duration +=stage_duration;
                time_task.reserve(time_stage.size());
                std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_task));
            }
            task_duration +=mov_duration;
        }
        this->qtime_task = QVector<double>::fromStdVector(time_task);

        ui.tableWidget_sol_task->setColumnCount(h_headers.size());
        ui.tableWidget_sol_task->setHorizontalHeaderLabels(h_headers);
        ui.tableWidget_sol_task->setRowCount(v_headers.size());
        ui.tableWidget_sol_task->setVerticalHeaderLabels(v_headers);

        for(int i =0; i < v_headers.size(); ++i)
        {
            std::vector<QString> row = task_steps.at(i);
            for(int j=0; j < h_headers.size(); ++j)
            {
                QString item = row.at(j);
                ui.tableWidget_sol_task->setItem(i,j,new QTableWidgetItem(item));
            }
        }
        ui.label_totalTime_value_task->setText(QString::number(task_duration).toStdString().c_str());
        ui.tabWidget_sol->setCurrentIndex(1);

        // set part of the task
        curr_task->getProblem(ui.listWidget_movs->currentRow())->setPartOfTask(true);

        // compute the hand values
        this->handPosition_task.resize(tot_steps); this->handVelocityNorm_task.resize(tot_steps);
        int step = 0;
        int arm_code = this->curr_task->getProblem(ui.listWidget_movs->currentRow())->getMovement()->getArm();

        for(size_t j=0;j<this->jointsPosition_task.size();++j)
        {
            vector<MatrixXd> pos_mov = this->jointsPosition_task.at(j);
            vector<MatrixXd> vel_mov = this->jointsVelocity_task.at(j);

            for (size_t k=0; k< pos_mov.size();++k)
            {
                MatrixXd pos_stage = pos_mov.at(k);
                MatrixXd vel_stage = vel_mov.at(k);
                for(int i=0;i<pos_stage.rows();++i)
                {
                    // position
                    VectorXd pos_row = pos_stage.block<1,JOINTS_ARM>(i,0);
                    vector<double> posture; posture.resize(pos_row.size());
                    VectorXd::Map(&posture[0], pos_row.size()) = pos_row;
#if UR == 1
                    this->curr_scene->getRobot()->getDHposture_UR(posture);
#endif
                    this->curr_scene->getRobot()->getHandPos(arm_code,this->handPosition_task.at(step),posture);
                    // velocity norm
                    VectorXd vel_row = vel_stage.block<1,JOINTS_ARM>(i,0);
                    vector<double> velocities; velocities.resize(vel_row.size());
                    VectorXd::Map(&velocities[0], vel_row.size()) = vel_row;
                    this->handVelocityNorm_task.at(step) = this->curr_scene->getRobot()->getHandVelNorm(arm_code,posture,velocities);

                    step++;
                }
            }
        }

        // --------------------------  Compute njs, nmu and planning time  ------------------------------- //
        // njs
        double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
        double mean_njs = ((double)sum_njs) / this->njs_task.size();
        string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
        boost::replace_all(mean_njs_str,",",".");
        double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
        double stdev_njs = std::sqrt((((double)sq_sum_njs) / this->njs_task.size()) - pow(mean_njs,2));
        string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
        boost::replace_all(stdev_njs_str,",",".");
        ui.label_cost_hand_value_task->setText(QString::fromStdString(mean_njs_str)+QString("(")+QString::fromStdString(stdev_njs_str)+QString(")"));

        // nmu
        double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
        double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
        string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
        boost::replace_all(mean_nmu_str,",",".");
        double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
        double stdev_nmu = std::sqrt((((double)sq_sum_nmu) / this->nmu_task.size()) - pow(mean_nmu,2));
        string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
        boost::replace_all(stdev_nmu_str,",",".");
        ui.label_nmu_task->setText(QString::fromStdString(mean_nmu_str)+QString("(")+QString::fromStdString(stdev_nmu_str)+QString(")"));

        // planning time
        double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
        double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
        string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
        boost::replace_all(mean_prob_str,",",".");
        double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
        double stdev_prob = std::sqrt((((double)sq_sum_prob) / this->prob_time_task.size()) - pow(mean_prob,2));
        string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
        boost::replace_all(stdev_prob_str,",",".");
        ui.label_solving_time_task->setText(QString::fromStdString(mean_prob_str)+QString("(")+QString::fromStdString(stdev_prob_str)+QString(")"));
    }
}


void MainWindow::on_pushButton_clear_task_clicked()
{
    this->jointsAcceleration_task.clear();
    this->jointsVelocity_task.clear();
    this->jointsPosition_task.clear();
    this->jointsAcceleration_mov.clear();
    this->jointsVelocity_mov.clear();
    this->jointsPosition_mov.clear();
    this->timesteps_task.clear();
    this->timesteps_mov.clear();
    this->tols_stop_task.clear();
    this->tols_stop_mov.clear();
    this->handPosition_mov.clear();
    this->handOrientation_mov.clear();
    this->handLinearVelocity_mov.clear();
    this->handAngularVelocity_mov.clear();
    this->handVelocityNorm_mov.clear();
    this->handPosition_task.clear();
    this->handOrientation_task.clear();
    this->handLinearVelocity_task.clear();
    this->handAngularVelocity_task.clear();
    this->handVelocityNorm_task.clear();
    this->nmu_task.clear();
    this->njs_task.clear();
    this->prob_time_task.clear();
    ui.tableWidget_sol_task->clear();
    ui.tableWidget_sol_mov->clear();
    ui.label_totalTime_value_task->clear();
    ui.label_totalTime_value_mov->clear();
    ui.listWidget_movs->clear();
    this->curr_task->clearProblems();
}


void MainWindow::on_comboBox_Task_currentIndexChanged(int i)
{
    switch (i)
    {
    case 0:
        // Single- arm task
        ui.radioButton_right->setEnabled(true);
        ui.radioButton_left->setEnabled(true);

        if(scenario_id == 2 || scenario_id == 3 || scenario_id == 4 || scenario_id == 5)
        {
            ui.radioButton_right->setEnabled(false);
            ui.radioButton_left->setEnabled(false);
        }
        break;
    case 1:
        //Dual-arm task
        ui.radioButton_right->setEnabled(false);
        ui.radioButton_left->setEnabled(false);
        break;
    }
}


void MainWindow::on_comboBox_mov_currentIndexChanged(int i)
{
    switch (i)
    {
    case 0:
        // Reach-to-grasp
        ui.comboBox_objects->setEnabled(true);
        ui.comboBox_objects_eng->setEnabled(false);
        ui.label_objects->setEnabled(true);
        ui.groupBox_grip->setEnabled(true);
        ui.comboBox_poses->setEnabled(false);
        ui.label_poses->setEnabled(false);
        break;
    case 1:
        // Reaching
        ui.comboBox_objects->setEnabled(false);
        ui.comboBox_objects_eng->setEnabled(false);
        ui.label_objects->setEnabled(false);
        ui.groupBox_grip->setEnabled(false);
        ui.comboBox_poses->setEnabled(true);
        ui.label_poses->setEnabled(true);
        break;
    case 2:
        // Transport
        ui.comboBox_objects->setEnabled(true);
        ui.comboBox_objects_eng->setEnabled(false);
        ui.label_objects->setEnabled(true);
        ui.groupBox_grip->setEnabled(true);
        ui.comboBox_poses->setEnabled(true);
        ui.label_poses->setEnabled(true);
        break;
    case 3:
        //Engage
        ui.comboBox_objects->setEnabled(true);
        ui.comboBox_objects_eng->setEnabled(true);
        ui.label_objects->setEnabled(true);
        ui.groupBox_grip->setEnabled(true);
        ui.comboBox_poses->setEnabled(false);
        ui.label_poses->setEnabled(false);
        break;
    case 4:
        //Disengage
        ui.comboBox_objects->setEnabled(true);
        ui.comboBox_objects_eng->setEnabled(true);
        ui.label_objects->setEnabled(true);
        ui.groupBox_grip->setEnabled(true);
        ui.comboBox_poses->setEnabled(true);
        ui.label_poses->setEnabled(true);
        break;
    case 5:
        // Go park
        ui.comboBox_objects->setEnabled(false);
        ui.comboBox_objects_eng->setEnabled(false);
        ui.label_objects->setEnabled(false);
        ui.groupBox_grip->setEnabled(false);
        ui.comboBox_poses->setEnabled(false);
        ui.label_poses->setEnabled(false);
        break;
    case 6:
        // waypoints
        ui.comboBox_objects->setEnabled(false);
        ui.label_objects_eng->setEnabled(false);
        ui.comboBox_objects_eng->setEnabled(false);
        ui.label_objects->setEnabled(false);
        ui.groupBox_grip->setEnabled(false);
        ui.comboBox_poses->setEnabled(false);
        ui.label_poses->setEnabled(false);
        ui.comboBox_waypoints->setEnabled(true);
        ui.label_waypoints->setEnabled(true);
        break;
    }
}

// enter here when clicked on ListScenario
void MainWindow::onListScenarioItemClicked(QListWidgetItem *item)
{
    //enable LoadScenario button
    ui.pushButton_loadScenario->setEnabled(true);

#if HAND == 0
    for(int i=0; i<ui.listWidget_scenario->size().height(); ++i)
    {
        if (ui.listWidget_scenario->item(i)== item)
        {
            switch(i)
            {
            case 0:
                // Assembly scenario: the Toy vehicle with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS has to assemble a toy vehicle on a table in front of him"));
                break;
            case 1:
                //Human assistance scenario: beverages with ARoS
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "ARoS serves a drink to a human patient"));
                break;
            case 2:
                //Assembly scenario: the Toy vehicle with Sawyer
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "Sawyer has to assemble a toy vehicle on a table in front of him"));
                break;
            case 3:
                //Human assistance scenario: beverages with Sawyer
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "Sawyer serves a drink to a human patient"));
                break;
            }
        }
    }
#elif HAND == 1

    for(int i=0; i<ui.listWidget_scenario->size().height(); ++i)
    {
        if (ui.listWidget_scenario->item(i)== item)
        {
            switch(i)
            {
            case 0:
                // Assembly scenario: the Toy vehicle with Sawyer
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "Sawyer has to assemble a toy vehicle on a table in front of him"));
                break;
            }
        }
    }
#elif HAND == 2

    for(int i=0; i<ui.listWidget_scenario->size().height(); ++i)
    {
        if (ui.listWidget_scenario->item(i)== item)
        {
            switch(i)
            {
            case 0:
                // waypoints scenario with UR10
                ui.textBrowser_scenario->setText(QString("Description of the selected scenario:\n"
                                                         "The user has to define waypoints and then the robot must interpolate them in a Human-like manner"));
                break;
            }
        }
    }
#endif
}


void MainWindow::on_pushButton_plot_mov_clicked()
{
    // plot the 3D hand position
    //handPosPlot_mov_ptr- pointer to the hand position plot of the movement
    this->handPosPlot_mov_ptr.reset(new HandPosPlot(this->handPosition_mov));
    this->handPosPlot_mov_ptr->setParent(this->ui.plot_hand_pos_mov);
    this->handPosPlot_mov_ptr->resize(522,329);
    this->handPosPlot_mov_ptr->show();
    // plot the hand velocity norm
    if(!this->handVelocityNorm_mov.empty())
    {
        QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_mov);
        ui.plot_hand_vel_mov->plotLayout()->clear();
        ui.plot_hand_vel_mov->clearGraphs();
        ui.plot_hand_vel_mov->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator

        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_mov);
        wideAxisRect->setupFullAxesBox(true);

        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_mov);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);

        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_hand_vel_mov->axisRects())
        {
            for (QCPAxis *axis : rect->axes())
            {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
            }
        }
        QString title("Hand velocity");
        ui.plot_hand_vel_mov->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_mov,title));
        ui.plot_hand_vel_mov->plotLayout()->addElement(1, 0, wideAxisRect);
        ui.plot_hand_vel_mov->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_hand_vel_mov->graph(0)->setPen(QPen(Qt::red));
        ui.plot_hand_vel_mov->graph(0)->setName(title);
        ui.plot_hand_vel_mov->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_hand_vel_mov->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_hand_vel_mov->graph(0)->setData(this->qtime_mov, qhand_vel);
        ui.plot_hand_vel_mov->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                              *std::max_element(qhand_vel.begin(), qhand_vel.end()));
        ui.plot_hand_vel_mov->graph(0)->rescaleAxes();
        ui.plot_hand_vel_mov->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_hand_vel_mov->replot();
    }
    else
    {
        ui.plot_hand_vel_mov->plotLayout()->clear();
        ui.plot_hand_vel_mov->clearGraphs();
    }
}


void MainWindow::on_pushButton_plot_task_clicked()
{
    // plot the 3D hand position
    this->handPosPlot_task_ptr.reset( new HandPosPlot(this->handPosition_task));
    this->handPosPlot_task_ptr->setParent(this->ui.plot_hand_pos_task);
    this->handPosPlot_task_ptr->resize(522,329);
    this->handPosPlot_task_ptr->show();

    // plot the hand velocity norm
    if(!this->handVelocityNorm_task.empty())
    {
        QVector<double> qhand_vel = QVector<double>::fromStdVector(this->handVelocityNorm_task);
        ui.plot_hand_vel_task->plotLayout()->clear();
        ui.plot_hand_vel_task->clearGraphs();
        ui.plot_hand_vel_task->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator

        QCPAxisRect *wideAxisRect = new QCPAxisRect(ui.plot_hand_vel_task);
        wideAxisRect->setupFullAxesBox(true);

        QCPMarginGroup *marginGroup = new QCPMarginGroup(ui.plot_hand_vel_task);
        wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);

        // move newly created axes on "axes" layer and grids on "grid" layer:
        for (QCPAxisRect *rect : ui.plot_hand_vel_task->axisRects())
        {
            for (QCPAxis *axis : rect->axes())
            {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
            }
        }
        QString title("Hand velocity");
        ui.plot_hand_vel_task->plotLayout()->addElement(0,0, new QCPPlotTitle(ui.plot_hand_vel_task,title));
        ui.plot_hand_vel_task->plotLayout()->addElement(1, 0, wideAxisRect);
        ui.plot_hand_vel_task->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
        ui.plot_hand_vel_task->graph(0)->setPen(QPen(Qt::red));
        ui.plot_hand_vel_task->graph(0)->setName(title);
        ui.plot_hand_vel_task->graph(0)->valueAxis()->setLabel("hand velocity [mm/s]");
        ui.plot_hand_vel_task->graph(0)->keyAxis()->setLabel("time [s]");
        ui.plot_hand_vel_task->graph(0)->setData(this->qtime_task, qhand_vel);
        ui.plot_hand_vel_task->graph(0)->valueAxis()->setRange(*std::min_element(qhand_vel.begin(), qhand_vel.end()),
                                                               *std::max_element(qhand_vel.begin(), qhand_vel.end()));
        ui.plot_hand_vel_task->graph(0)->rescaleAxes();
        ui.plot_hand_vel_task->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui.plot_hand_vel_task->replot();
    }
    else
    {
        ui.plot_hand_vel_task->plotLayout()->clear();
        ui.plot_hand_vel_task->clearGraphs();
    }
}


void MainWindow::on_pushButton_joints_results_mov_clicked()
{
    if(!this->jointsPosition_mov.empty())
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_mov,this->jointsVelocity_mov,this->jointsAcceleration_mov,this->timesteps_mov);
    this->mResultsJointsdlg->show();
}


void MainWindow::on_pushButton_joints_results_task_clicked()
{
    if(!this->jointsPosition_task.empty())
        this->mResultsJointsdlg->setupPlots(this->jointsPosition_task,this->jointsVelocity_task,this->jointsAcceleration_task,this->timesteps_task);
    this->mResultsJointsdlg->show();
}


void MainWindow::on_pushButton_comp_vel_mov_clicked()
{
    if(!this->shoulderLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->shoulderLinearVelocity_mov,this->shoulderAngularVelocity_mov,this->qtime_mov,0);
    if(!this->elbowLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->elbowLinearVelocity_mov,this->elbowAngularVelocity_mov,this->qtime_mov,1);
#if UR == 0
    if(!this->wristLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wristLinearVelocity_mov,this->wristAngularVelocity_mov,this->qtime_mov,2);
    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handLinearVelocity_mov,this->handAngularVelocity_mov,this->qtime_mov,3);
#elif UR == 1
    if(!this->wrist1LinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wrist1LinearVelocity_mov,this->wrist1AngularVelocity_mov,this->qtime_mov,2);
    if(!this->wrist2LinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wrist2LinearVelocity_mov,this->wrist2AngularVelocity_mov,this->qtime_mov,3);
    if(!this->wrist3LinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->wrist3LinearVelocity_mov,this->wrist3AngularVelocity_mov,this->qtime_mov,4);

    if(!this->handLinearVelocity_mov.empty())
        this->mCompVeldlg->setupPlots(this->handLinearVelocity_mov,this->handAngularVelocity_mov,this->qtime_mov,5);
#endif
    this->mCompVeldlg->show();
}


void MainWindow::on_pushButton_save_res_mov_clicked()
{
    struct stat st = {0};

    if (stat("results", &st) == -1)
        mkdir("results", 0700);
    if (stat("results/planning", &st) == -1)
        mkdir("results/planning", 0700);
    if (stat("results/planning/mov", &st) == -1)
        mkdir("results/planning/mov", 0700);

    QString path("results/planning/mov/");

    ui.plot_hand_vel_mov->savePdf(path+QString("hand_vel_mov.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));
    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");
    string hand_pos_file = path.toStdString()+string("hand_pos_mov.pdf");
    if(this->handPosPlot_mov_ptr!=nullptr)
        IO::save(this->handPosPlot_mov_ptr.get(), hand_pos_file.c_str(),  "PDF" );

    // --------------------------------------- RESULTS -------------------------------------- //
    string filename("results_mov.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    results << string("# NORMALIZED JERK SCORE \n");
    string njs_str =  boost::str(boost::format("%.2f") % (this->njs_mov));
    boost::replace_all(njs_str,",",".");
    results << string("njs =")+njs_str+string(";\n");

    results << string("# NUMBER OF MOVEMENT UNITS \n");
    string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_mov));
    boost::replace_all(nmu_str,",",".");
    results << string("nmu =")+nmu_str+string(";\n");

    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [s] \n");
    string time_str =  boost::str(boost::format("%.2f") % (this->prob_time_mov));
    boost::replace_all(time_str,",",".");
    results << string("prob_time =")+time_str+string(";\n");
    results.close();

    // --------------------------------------- HAND POSITION -------------------------------------- //
    if(!this->handPosition_mov.empty())
    {
        string filename_hand_pos("hand_pos_mov.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_mov.size();++i)
        {
            vector<double> point = this->handPosition_mov.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // --------------------------------------- HAND VELOCITY -------------------------------------- //
    if(!this->handVelocityNorm_mov.empty())
    {
        string filename_hand_vel("hand_vel_mov.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_mov.size();++i)
        {
            double vel = this->handVelocityNorm_mov.at(i);
            double time = this->qtime_mov.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr;
    string pdf_str;
    QString svg_qstr;
    string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_mov.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_mov.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());
    pdf_qstr = path+QString("hand_vel_mov.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_mov.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());
}


void MainWindow::on_pushButton_save_res_task_clicked()
{
    struct stat st = {0};

    if (stat("results", &st) == -1)
        mkdir("results", 0700);
    if (stat("results/planning", &st) == -1)
        mkdir("results/planning", 0700);
    if (stat("results/planning/task", &st) == -1)
        mkdir("results/planning/task", 0700);

    QString path("results/planning/task/");

    // --------------------------------------- results.TXT -------------------------------------- //
    string filename("results_task.txt");
    ofstream results;
    results.open(path.toStdString()+filename);

    //*****************************************************************************************
    //                                  NORMALIZED JERK SCORE
    results << string("# NORMALIZED JERK SCORE \n");
    results << string("njs = ");
    for(size_t i=0;i<this->njs_task.size();++i)
    {
        string njs_str =  boost::str(boost::format("%.2f") % (this->njs_task.at(i)));
        boost::replace_all(njs_str,",",".");
        if(i==(this->njs_task.size()-1))
            results << njs_str+" \n";
        else
            results << njs_str+" ";
    }

    // Mean
    double sum_njs = std::accumulate(this->njs_task.begin(), this->njs_task.end(), 0.0);
    double mean_njs = ((double)sum_njs) / this->njs_task.size();
    string mean_njs_str =  boost::str(boost::format("%.2f") % (mean_njs));
    boost::replace_all(mean_njs_str,",",".");
    results << string("mean njs = ")+mean_njs_str+string(" \n");

    // Standard deviation
    double sq_sum_njs = std::inner_product(this->njs_task.begin(), this->njs_task.end(), this->njs_task.begin(), 0.0);
    double stdev_njs = std::sqrt(((double)sq_sum_njs) / this->njs_task.size() - mean_njs * mean_njs);
    string stdev_njs_str =  boost::str(boost::format("%.2f") % (stdev_njs));
    boost::replace_all(stdev_njs_str,",",".");
    results << string("sd njs = ")+stdev_njs_str+string(" \n");

    // Median
    double median_njs = this->getMedian(this->njs_task);
    string median_njs_str =  boost::str(boost::format("%.2f") % (median_njs));
    boost::replace_all(median_njs_str,",",".");
    results << string("median njs = ")+median_njs_str+string(" \n");

    // 1st quartile
    double first_quartile_njs = this->getFirstQuartile(this->njs_task);
    string first_quartile_njs_str =  boost::str(boost::format("%.2f") % (first_quartile_njs));
    boost::replace_all(first_quartile_njs_str,",",".");
    results << string("first quartile njs = ")+first_quartile_njs_str+string(" \n");

    // 3rd quartile
    double third_quartile_njs = this->getThirdQuartile(this->njs_task);
    string third_quartile_njs_str =  boost::str(boost::format("%.2f") % (third_quartile_njs));
    boost::replace_all(third_quartile_njs_str,",",".");
    results << string("third quartile njs = ")+third_quartile_njs_str+string(" \n");

    //*****************************************************************************************
    //                                              NMU
    results << string("# NUMBER OF MOVEMENT UNITS \n");
    results << string("nmu = ");
    for(size_t i=0;i<this->nmu_task.size();++i)
    {
        string nmu_str =  boost::str(boost::format("%.2f") % (this->nmu_task.at(i)));
        boost::replace_all(nmu_str,",",".");
        if(i==(this->nmu_task.size()-1))
            results << nmu_str+" \n";
        else
            results << nmu_str+" ";
    }

    // Mean
    double sum_nmu = std::accumulate(this->nmu_task.begin(), this->nmu_task.end(), 0.0);
    double mean_nmu = ((double)sum_nmu) / this->nmu_task.size();
    string mean_nmu_str =  boost::str(boost::format("%.2f") % (mean_nmu));
    boost::replace_all(mean_nmu_str,",",".");
    results << string("mean nmu = ")+mean_nmu_str+string(" \n");

    // Standard deviation
    double sq_sum_nmu = std::inner_product(this->nmu_task.begin(), this->nmu_task.end(), this->nmu_task.begin(), 0.0);
    double stdev_nmu = std::sqrt(((double)sq_sum_nmu) / this->nmu_task.size() - mean_nmu * mean_nmu);
    string stdev_nmu_str =  boost::str(boost::format("%.2f") % (stdev_nmu));
    boost::replace_all(stdev_nmu_str,",",".");
    results << string("sd nmu = ")+stdev_nmu_str+string(" \n");

    //Median
    double median_nmu = this->getMedian(this->nmu_task);
    string median_nmu_str =  boost::str(boost::format("%.2f") % (median_nmu));
    boost::replace_all(median_nmu_str,",",".");
    results << string("median nmu = ")+median_nmu_str+string(" \n");

    // 1st quartile
    double first_quartile_nmu = this->getFirstQuartile(this->nmu_task);
    string first_quartile_nmu_str =  boost::str(boost::format("%.2f") % (first_quartile_nmu));
    boost::replace_all(first_quartile_nmu_str,",",".");
    results << string("first quartile nmu = ")+first_quartile_nmu_str+string(" \n");

    // 3rd quartile
    double third_quartile_nmu = this->getThirdQuartile(this->nmu_task);
    string third_quartile_nmu_str =  boost::str(boost::format("%.2f") % (third_quartile_nmu));
    boost::replace_all(third_quartile_nmu_str,",",".");
    results << string("third quartile nmu = ")+third_quartile_nmu_str+string(" \n");

    //*****************************************************************************************
    //                                           TIME
    results << string("# TIME TAKEN TO PLAN THE MOVEMENT [s] \n");
    results << string("prob_time = ");
    for(size_t i=0;i<this->prob_time_task.size();++i)
    {
        string prob_str =  boost::str(boost::format("%.2f") % (this->prob_time_task.at(i)));
        boost::replace_all(prob_str,",",".");
        if(i==(this->prob_time_task.size()-1))
            results << prob_str+" \n";
        else
            results << prob_str+" ";
    }

    // Mean
    double sum_prob = std::accumulate(this->prob_time_task.begin(), this->prob_time_task.end(), 0.0);
    double mean_prob = ((double)sum_prob) / this->prob_time_task.size();
    string mean_prob_str =  boost::str(boost::format("%.2f") % (mean_prob));
    boost::replace_all(mean_prob_str,",",".");
    results << string("mean plan time = ")+mean_prob_str+string(" \n");

    // Standard deviation
    double sq_sum_prob = std::inner_product(this->prob_time_task.begin(), this->prob_time_task.end(), this->prob_time_task.begin(), 0.0);
    double stdev_prob = std::sqrt(((double)sq_sum_prob) / this->prob_time_task.size() - mean_prob * mean_prob);
    string stdev_prob_str =  boost::str(boost::format("%.2f") % (stdev_prob));
    boost::replace_all(stdev_prob_str,",",".");
    results << string("sd plan time = ")+stdev_prob_str+string(" \n");

    //Median
    double median_prob = this->getMedian(this->prob_time_task);
    string median_prob_str =  boost::str(boost::format("%.2f") % (median_prob));
    boost::replace_all(median_prob_str,",",".");
    results << string("median prob = ")+median_prob_str+string(" \n");

    // 1st quartile
    double first_quartile_prob = this->getFirstQuartile(this->prob_time_task);
    string first_quartile_prob_str =  boost::str(boost::format("%.2f") % (first_quartile_prob));
    boost::replace_all(first_quartile_prob_str,",",".");
    results << string("first quartile plan time = ")+first_quartile_prob_str+string(" \n");

    // 3rd quartile
    double third_quartile_prob = this->getThirdQuartile(this->prob_time_task);
    string third_quartile_prob_str =  boost::str(boost::format("%.2f") % (third_quartile_prob));
    boost::replace_all(third_quartile_prob_str,",",".");
    results << string("third quartile plan time = ")+third_quartile_prob_str+string(" \n");

    string rate_success = ui.label_rate_task->text().toStdString();
    results << string("rate of success [%] = ")+rate_success+string(" \n");
    results.close();


    // --------------------------------------- results.CVS -------------------------------------- //
    string filename_csv("results_task.csv");
    ofstream results_csv;
    results_csv.open(path.toStdString()+filename_csv);
    results_csv << "TRAJ,NJS,NMU,PLANNING TIME [ms] \n";
    for(size_t i=0;i<this->njs_task.size();++i)
    {
        string njs_str =  boost::str(boost::format("%.8f") % (this->njs_task.at(i)));
        string nmu_str =  boost::str(boost::format("%.8f") % (this->nmu_task.at(i)));
        string prob_str =  boost::str(boost::format("%.8f") % (this->prob_time_task.at(i)));
        boost::replace_all(njs_str,",","."); boost::replace_all(nmu_str,",","."); boost::replace_all(prob_str,",",".");
        results_csv << QString::number(i+1).toStdString()+","+njs_str+","+nmu_str+","+prob_str+" \n";
    }
    results_csv.close();

    // --------------------------------------- hand_vel.PDF -------------------------------------- //
    ui.plot_hand_vel_task->savePdf(path+QString("hand_vel_task.pdf"),true,0,0,QString(),QString("Module of the Hand velocity"));
    VectorWriter* handler = (VectorWriter*)IO::outputHandler("PDF");
    handler->setTextMode(VectorWriter::NATIVE);
    handler->setFormat("PDF");

    // --------------------------------------- hand_pos.PDF -------------------------------------- //
    string hand_pos_file = path.toStdString()+string("hand_pos_task.pdf");
    if(this->handPosPlot_task_ptr!=nullptr)
        IO::save(this->handPosPlot_task_ptr.get(), hand_pos_file.c_str(),  "PDF" );

    // --------------------------------------- hand_pos.TXT -------------------------------------- //
    if(!this->handPosition_task.empty())
    {
        string filename_hand_pos("hand_pos_task.txt");
        ofstream hand_pos;
        hand_pos.open(path.toStdString()+filename_hand_pos);

        hand_pos << string("# HAND POSITION \n");
        hand_pos << string("# x [mm], y [mm], z [mm] \n");

        for(size_t i=0;i<this->handPosition_task.size();++i)
        {
            vector<double> point = this->handPosition_task.at(i);
            string x_str =  boost::str(boost::format("%.2f") % (point.at(0)));
            boost::replace_all(x_str,",",".");
            string y_str =  boost::str(boost::format("%.2f") % (point.at(1)));
            boost::replace_all(y_str,",",".");
            string z_str =  boost::str(boost::format("%.2f") % (point.at(2)));
            boost::replace_all(z_str,",",".");
            hand_pos << x_str+string(", ")+y_str+string(", ")+z_str+string("\n");
        }
        hand_pos.close();
    }

    // --------------------------------------- hand_pos.TXT -------------------------------------- //
    if(!this->handVelocityNorm_task.empty())
    {
        string filename_hand_vel("hand_vel_task.txt");
        ofstream hand_vel;
        hand_vel.open(path.toStdString()+filename_hand_vel);

        hand_vel << string("# HAND VELOCITY NORM \n");
        hand_vel << string("# velocity [mm/s], time [s] \n");

        for(size_t i=0;i<this->handVelocityNorm_task.size();++i)
        {
            double vel = this->handVelocityNorm_task.at(i);
            double time = this->qtime_task.at(i);
            string vel_str =  boost::str(boost::format("%.2f") % (vel));
            boost::replace_all(vel_str,",",".");
            string t_str =  boost::str(boost::format("%.2f") % (time));
            boost::replace_all(t_str,",",".");
            hand_vel << vel_str+string(", ")+t_str+string("\n");
        }
        hand_vel.close();
    }

    QString pdf_qstr;
    string pdf_str;
    QString svg_qstr;
    string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos_task.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos_task.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());
    pdf_qstr = path+QString("hand_vel_task.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_vel_task.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());
}


void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "motion_manager");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
#if ROBOT == 1
#if UR == 0
    QString master_url = ("http://021604CP00018.local:11311/");
    QString host_url = ("192.168.197.115");
    //something is wrong with 2 lines, the variables does not keep the following values.
    //QString master_url = settings.value("master_url",QString("http://021604CP00018.local:11311/")).toString();
    //QString host_url = settings.value("host_url", QString("192.168.197.115")).toString();
    mrosCommdlg->setMasterUrl(master_url);
    mrosCommdlg->setHostUrl(host_url);
#endif
#endif
#if ROBOT == 0
    QString master_url = settings.value("master_url",QString("http://127.0.0.1:11311/")).toString();
    QString host_url = settings.value("host_url", QString("127.0.0.1")).toString();
    mrosCommdlg->setMasterUrl(master_url);
    mrosCommdlg->setHostUrl(host_url);
#endif
    bool remember = settings.value("remember_settings", false).toBool();
    mrosCommdlg->setRememberCheckbox(remember);

    bool checked = settings.value("use_environment_variables", false).toBool();
    mrosCommdlg->setUseEnvCheckbox(checked);

    mrosCommdlg->enableMasterUrl(!checked);
    mrosCommdlg->enableHostUrl(!checked);
}


void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "motion_manager");
    settings.setValue("master_url", mrosCommdlg->getMasterUrl());
    settings.setValue("host_url", mrosCommdlg->getHostUrl());
    settings.setValue("use_environment_variables",QVariant(mrosCommdlg->getUseEnvCheckbox()));
    settings.setValue("remember_settings",QVariant(mrosCommdlg->getRememberCheckbox()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}


void MainWindow::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
{
    const double MIN_STEP_VALUE = 0.1;
    int h = 1;
    int tnsample;
    double f0;
    double f1;
    double f2;
    double f3;
    double f4;
    double step_value;

    // 1st point
    // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
    tnsample = 0;
    f0 = function.at(tnsample);
    f1 = function.at(tnsample+1);
    f2 = function.at(tnsample+2);
    f3 = function.at(tnsample+3);
    f4 = function.at(tnsample+4);
    step_value = step_values.at(tnsample);
    if(step_value==0)
        step_value=MIN_STEP_VALUE;
    derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));

    // 2nd point
    // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
    tnsample = 1;
    f0 = function.at(tnsample-1);
    f1 = function.at(tnsample);
    f2 = function.at(tnsample+1);
    f3 = function.at(tnsample+2);
    f4 = function.at(tnsample+3);
    step_value = step_values.at(tnsample);
    if(step_value==0)
        step_value=MIN_STEP_VALUE;
    derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));

    // 3rd point
    // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
    for (int i=2; i< function.size() -2;++i)
    {
        f0 = function.at(i-2);
        f1 = function.at(i-1);
        f2 = function.at(i);
        f3 = function.at(i+1);
        f4 = function.at(i+2);
        step_value = step_values.at(i);
        if(step_value==0)
            step_value=0.01;
        derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
    }

    // 4th point
    // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
    tnsample = function.size()-2;
    f0 = function.at(tnsample-3);
    f1 = function.at(tnsample-2);
    f2 = function.at(tnsample-1);
    f3 = function.at(tnsample);
    f4 = function.at(tnsample+1);
    step_value = step_values.at(tnsample);
    if(step_value==0)
        step_value=MIN_STEP_VALUE;
    derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));

    // 5th point
    // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
    tnsample = function.size()-1;
    f0 = function.at(tnsample-4);
    f1 = function.at(tnsample-3);
    f2 = function.at(tnsample-2);
    f3 = function.at(tnsample-1);
    f4 = function.at(tnsample);
    step_value = step_values.at(tnsample);
    if(step_value==0)
        step_value=MIN_STEP_VALUE;
    derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));
}


int MainWindow::getNumberMovementUnits(vector<double> &function, QVector<double> &time)
{
    int nmu = 0;
    vector<float> ffunc(function.begin(),function.end());
    double maxfunc = *std::max_element(ffunc.begin(), ffunc.end());
    double perc = ((double)10)/100;
    double threshold = maxfunc*perc;

    Persistence1D p;
    p.RunPersistence(ffunc);

    //Get all extrema with a persistence larger than perst.
    double perst = 5;
    vector< TPairedExtrema > Extrema;
    p.GetPairedExtrema(Extrema, perst);

    //Print all found pairs - pairs are sorted ascending wrt. persistence.
    for(vector< TPairedExtrema >::iterator it = Extrema.begin(); it != Extrema.end(); it++)
    {
        double slope = (ffunc.at((*it).MaxIndex)-ffunc.at((*it).MinIndex))/(time.at((*it).MaxIndex)-time.at((*it).MinIndex));
        if(abs(slope)>threshold)
            nmu++;
    }

    return nmu;
}


double MainWindow::getMedian(vector<double> v)
{
    double median;
    size_t size = v.size();
    sort(v.begin(), v.end());

    if (size  % 2 == 0)
        median = (v[size / 2 - 1] + v[size / 2]) / 2;
    else
        median = v[size / 2];

    return median;
}


double MainWindow::getFirstQuartile(vector<double> v)
{
    double first_quartile;
    size_t size = v.size()/4;
    sort(v.begin(), v.end());

    if (size  % 2 == 0)
        first_quartile = (v[size / 2 - 1] + v[size / 2]) / 2;
    else
        first_quartile = v[size / 2];

    return first_quartile;
}


double MainWindow::getThirdQuartile(vector<double> v)
{
    double third_quartile;
    size_t size = 3*(v.size()/4);
    size_t size_1 = v.size()/4;
    sort(v.begin(), v.end());

    if (size_1  % 2 == 0)
        third_quartile = (v[size + size_1/2 - 1] + v[size + size_1/2]) / 2;
    else
        third_quartile = v[size + size_1/2];

    return third_quartile;
}


double MainWindow::getMedian(vector<int> v)
{
    double median;
    size_t size = v.size();
    sort(v.begin(), v.end());

    if (size  % 2 == 0)
        median = (v[size / 2 - 1] + v[size / 2]) / 2;
    else
        median = v[size / 2];

    return median;
}


double MainWindow::getFirstQuartile(vector<int> v)
{
    double first_quartile;
    size_t size = v.size()/4;
    sort(v.begin(), v.end());

    if (size  % 2 == 0)
        first_quartile = (v[size / 2 - 1] + v[size / 2]) / 2;
    else
        first_quartile = v[size / 2];

    return first_quartile;
}


double MainWindow::getThirdQuartile(vector<int> v)
{
    double third_quartile;
    size_t size = 3*(v.size()/4);
    size_t size_1 = v.size()/4;
    sort(v.begin(), v.end());

    if (size_1  % 2 == 0)
        third_quartile = (v[size + size_1/2 - 1] + v[size + size_1/2]) / 2;
    else
        third_quartile = v[size + size_1/2];

    return third_quartile;
}




}  // namespace motion_manager
