#include "../include/motion_manager/problem.hpp"


namespace motion_manager {

Problem::Problem():
    mov(nullptr),scene(nullptr)
{
    this->rightFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<double>(JOINTS_HAND);
    this->leftFinalHand = std::vector<double>(JOINTS_HAND);

    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;
}


Problem::Problem(int planner_id,Movement* mov,Scenario* scene)
{
    this->rightFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_diseng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftFinalPosture_eng = std::vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightFinalHand = std::vector<double>(JOINTS_HAND);
    this->leftFinalHand = std::vector<double>(JOINTS_HAND);

    this->targetAxis = 0;
    this->solved=false;
    this->part_of_task=false;
    this->err_log=0;

    this->mov=movementPtr(mov);
    this->scene=scenarioPtr(scene);
    this->planner_id=planner_id;

    bool hump = false;

    if (planner_id==0)
    {
        hump = true;
        this->planner_name = "HUMP";
    }

    string scene_name = this->scene->getName();

    if (hump)
    {
        // --- Human-like movement planner settings --- //
        HUMotion::HUMPlanner::hand_fingers = HAND_FINGERS;
        HUMotion::HUMPlanner::joints_arm = JOINTS_ARM;
        HUMotion::HUMPlanner::joints_hand = JOINTS_HAND;
        HUMotion::HUMPlanner::n_phalange = N_PHALANGE;

        this->h_planner.reset(new HUMotion::HUMPlanner(scene_name));
        // set the current obstacles and targets of the scenario
        vector<objectPtr> scene_objects;
        if(this->scene->getObjects(scene_objects))
        {
            HUMotion::objectPtr hump_obj; // object of the planner
            objectPtr obj;
            for(size_t i=0; i < scene_objects.size(); ++i)
            {
                obj = scene_objects.at(i);
                std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
                std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
                std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
                hump_obj.reset(new HUMotion::Object(obj->getName()));
                hump_obj->setParams(position,orientation,dimension);
                if(!obj->isTargetRightEnabled() && !obj->isTargetLeftEnabled())
                    this->h_planner->addObstacle(hump_obj); // the object is an obstacle for the planner
            }
        }

        // set the robot
        Matrix4d mat_right_arm;
        Matrix4d mat_right_hand;
        vector<double> min_rlimits;
        vector<double> max_rlimits;

        Matrix4d mat_left_arm;
        Matrix4d mat_left_hand;
        vector<double> min_llimits;
        vector<double> max_llimits;

        this->scene->getRobot()->getMatRight(mat_right_arm);
        this->scene->getRobot()->getMatRightHand(mat_right_hand);
        this->scene->getRobot()->getMatLeft(mat_left_arm);
        this->scene->getRobot()->getMatLeftHand(mat_left_hand);
        this->scene->getRobot()->getRightMinLimits(min_rlimits);
        this->scene->getRobot()->getRightMaxLimits(max_rlimits);
        this->scene->getRobot()->getLeftMinLimits(min_llimits);
        this->scene->getRobot()->getLeftMaxLimits(max_llimits);

        h_planner->setMatRightArm(mat_right_arm);
        h_planner->setMatRightHand(mat_right_hand);
        h_planner->setRightMaxLimits(max_rlimits);
        h_planner->setRightMinLimits(min_rlimits);
        h_planner->setMatLeftArm(mat_left_arm);
        h_planner->setMatLeftHand(mat_left_hand);
        h_planner->setLeftMaxLimits(max_llimits);
        h_planner->setLeftMinLimits(min_llimits);

        robot_part torso = this->scene->getRobot()->getTorso();
        HUMotion::RobotPart hump_torso;

        hump_torso.Xpos = torso.Xpos;
        hump_torso.Ypos = torso.Ypos;
        hump_torso.Zpos = torso.Zpos;
        hump_torso.Roll = torso.Roll;
        hump_torso.Pitch = torso.Pitch;
        hump_torso.Yaw = torso.Yaw;
        hump_torso.Xsize = torso.Xsize;
        hump_torso.Ysize = torso.Ysize;
        hump_torso.Xpos = torso.Xpos;
        hump_torso.Zsize = torso.Zsize;

        h_planner->setTorso(hump_torso);

        DHparams rDH = this->scene->getRobot()->getDH_rightArm();
        DHparams lDH = this->scene->getRobot()->getDH_leftArm();

        HUMotion::DHparameters right_arm_DH;
        HUMotion::DHparameters left_arm_DH;

        right_arm_DH.a = rDH.a;
        right_arm_DH.alpha = rDH.alpha;
        right_arm_DH.d = rDH.d;
        right_arm_DH.theta = rDH.theta;
        left_arm_DH.a = lDH.a;
        left_arm_DH.alpha = lDH.alpha;
        left_arm_DH.d = lDH.d;
        left_arm_DH.theta = lDH.theta;

        h_planner->setDH_rightArm(right_arm_DH);
        h_planner->setDH_leftArm(left_arm_DH);

#if HAND == 0
        barrett_hand b_hand = this->scene->getRobot()->getBarrettHand();
        std::vector<int> rk; this->scene->getRobot()->getRK(rk);
        std::vector<int> jk; this->scene->getRobot()->getJK(jk);
        HUMotion::BarrettHand hump_bhand;

        hump_bhand.A1 = b_hand.A1;
        hump_bhand.A2 = b_hand.A2;
        hump_bhand.A3 = b_hand.A3;
        hump_bhand.Aw = b_hand.Aw;
        hump_bhand.D3 = b_hand.D3;
        hump_bhand.maxAperture = b_hand.maxAperture;
        hump_bhand.phi2 = b_hand.phi2;
        hump_bhand.phi3 = b_hand.phi3;
        hump_bhand.rk = rk;
        hump_bhand.jk = jk;

        h_planner->setBarrettHand(hump_bhand);
#elif HAND == 1
        electric_gripper e_hand = this->scene->getRobot()->getElectricGripper();
        std::vector<int> rk; this->scene->getRobot()->getRK(rk);
        HUMotion::ElectricGripper hump_ehand;

        hump_ehand.A1 = e_hand.A1;
        hump_ehand.D3 = e_hand.D3;
        hump_ehand.maxAperture = e_hand.maxAperture;
        hump_ehand.minAperture = e_hand.minAperture;
        hump_ehand.rk = rk;

        h_planner->setElectricGripper(hump_ehand);
#endif
#if HEAD==1
        robot_part head = this->scene->getRobot()->getHead();
        HUMotion::RobotPart hump_head;

        hump_head.Xpos = head.Xpos;
        hump_head.Ypos = head.Ypos;
        hump_head.Zpos = head.Zpos;
        hump_head.Roll = head.Roll;
        hump_head.Pitch = head.Pitch;
        hump_head.Yaw = head.Yaw;
        hump_head.Xsize = head.Xsize;
        hump_head.Ysize = head.Ysize;
        hump_head.Xpos = head.Xpos;
        hump_head.Zsize = head.Zsize;

        h_planner->setHead(hump_head);
#endif
    }
}


Problem::Problem(const Problem& s)
{
    this->rightFinalPosture = s.rightFinalPosture;
    this->rightFinalPosture_diseng = s.rightFinalPosture_diseng;
    this->rightFinalPosture_eng = s.rightFinalPosture_eng;
    this->rightFinalHand = s.rightFinalHand;
    this->leftFinalPosture = s.leftFinalPosture;
    this->leftFinalPosture_eng = s.leftFinalPosture_eng;
    this->leftFinalPosture_diseng = s.leftFinalPosture_diseng;
    this->leftFinalHand = s.leftFinalHand;

    this->dFF = s.dFF;
    this->dFH = s.dFH;
    this->dHOl = s.dHOl;
    this->dHOr = s.dHOr;
    this->solved=s.solved;
    this->part_of_task=s.part_of_task;
    this->err_log=s.err_log;

    this->targetAxis = s.targetAxis;
    this->mov = movementPtr(new Movement(*s.mov.get()));
    this->scene = scenarioPtr(new Scenario(*s.scene.get()));

    if(s.h_planner!=nullptr)
        this->h_planner = h_plannerPtr(new HUMotion::HUMPlanner(*s.h_planner.get()));

    this->planner_id=s.planner_id;
    this->planner_name=s.planner_name;
}


Problem::~Problem()
{

}


void Problem::setPlannerID(int id)
{
    this->planner_id=id;

    switch(id)
    {
    case 0:
        this->planner_name = "HUMP";
        break;
    case 1:
        this->planner_name = "RRT";
        break;
    case 2:
        this->planner_name = "RRTConnect";
        break;
    case 3:
        this->planner_name = "RRTstar";
        break;
    case 4:
        this->planner_name = "PRM";
        break;
    case 5:
        this->planner_name = "PRMstar";
        break;
    }
}


int Problem::getPlannerID()
{
    return this->planner_id;
}


string Problem::getPlannerName()
{
    return this->planner_name;
}


int Problem::getErrLog()
{
    return this->err_log;
}


bool Problem::getSolved()
{
    return this->solved;
}


bool Problem::getPartOfTask()
{
    return this->part_of_task;
}


string Problem::getInfoLine()
{
    return string("Planner: ")+this->getPlannerName()+string(" ,Movement: ")+this->mov->getInfoLine();
}


objectPtr Problem::getObjectEngaged()
{
    return this->obj_eng;
}


void Problem::setSolved(bool s)
{
    this->solved=s;
}


void Problem::setPartOfTask(bool p)
{
    this->part_of_task=p;
}


void Problem::setMoveSettings(std::vector<double> &tar, std::vector<double> &final_hand, std::vector<double> &final_arm, bool use_posture)
{
    this->move_target=tar;
    this->move_final_hand=final_hand;
    this->move_final_arm=final_arm;
    this->use_posture=use_posture;
}


bool Problem::finalPostureHand()
{
    bool success;
    // get the robot used in this scenario
    robotPtr hh = this->scene->getRobot();
    // get the object(s) involved in this movement
    objectPtr obj = this->mov->getObject();
    // compute the diameter (plus tolerance) of the object to be grasped
    double d_obj;
    // get the type of grip for this movement
    bool prec = this->mov->getGrip();

#if HAND == 0

    if(prec)
    {
        d_obj = obj->getRadius() * 2.0 + TOL_GRIP;

        if (d_obj > hh->getBarrettHand().maxAperture)
        {
            success = false;
            throw string("impossible to grasp the object ") + obj->getName() +
                    string(" with the grip ") + this->mov->getGripStr() +
                    string(". The object is too large");
        }
    }
    else
    {
        d_obj = min(hh->getBarrettHand().maxAperture,double(1.2)*obj->getRadius()*2+TOL_GRIP);

        if (obj->getRadius() * 2 + TOL_GRIP > hh->getBarrettHand().maxAperture)
        {
            success = false;
            throw string("impossible to grasp the object ") + obj->getName() +
                    string(" with the grip ") + this->mov->getGripStr() +
                    string(". The object is too large");
        }
    }
#elif HAND == 1

    if(prec)
        d_obj = obj->getRadius() * 2.0 + TOL_GRIP;
    else
        d_obj = double(1.035) * obj->getRadius() * 2.0 + TOL_GRIP;

    if(d_obj > hh->getElectricGripper().maxAperture)
    {
        success = false;
        throw string("impossible to grasp the object ") + obj->getName() +
                string(" with the grip ") + this->mov->getGripStr() +
                string(". The object is too large");
    }
    else if(d_obj < hh->getElectricGripper().minAperture + TOL_GRIP)
    {
        success = false;
        throw string("impossible to grasp the object ") + obj->getName() +
                string(" with the grip ") + this->mov->getGripStr() +
                string(". The object is too small");
    }
#endif

    try
    {
        success = this->invKinHand(d_obj);

        if(!success)
            throw string("Error: Hand inverse kinematic not solved");
    }
    catch(const string str)
    {
        success = false;
        throw str;
    }

    return success;
}


bool Problem::invKinHand(double d_obj)
{
    robotPtr hh = this->scene->getRobot();
    bool success = true;

#if HAND == 0
    double A1 = hh->getBarrettHand().A1;
    double A2 = hh->getBarrettHand().A2;
    double A3 = hh->getBarrettHand().A3;
    double D3 = hh->getBarrettHand().D3;
    double phi2 = hh->getBarrettHand().phi2;
    double phi3 = hh->getBarrettHand().phi3;

    double fnew;
    double dfnew;

    double x0 = 60.0 * M_PI / 180.0; // initial approximation
    double xold = x0;
    double theta = xold;
    double xnew = 140.0 * M_PI / 180.0;
    int cnt = 0;

    while(abs(xnew - xold) > 1e-4)
    {
        xold = theta;
        fnew = 2 * cos((4/3) * theta + phi2 + phi3) * A3 - 2 * sin((4/3) * theta + phi2 + phi3) * D3 +
               2 * cos(theta + phi2) * A2 + 2 * A1 - d_obj;
        dfnew = -(8/3) * sin((4/3) * theta + phi2 + phi3) * A3- (8/3) * cos((4/3) * theta + phi2 + phi3) * D3-
                2 * sin(theta + phi2) * A2;
        xnew = xold - fnew / dfnew;
        theta = xnew;
        cnt++;
    }

    if (cnt > 100)
        success = false;

    this->dFF = 2 * cos((4/3) * theta + phi2 + phi3) * A3 - 2 * sin((4/3) * theta + phi2 + phi3) * D3 +
                2 * cos(theta + phi2) * A2 + 2 * A1;
    this->dFH = sin((4/3) * theta + phi2 + phi3) * A3 + cos((4/3) * theta + phi2 + phi3) * D3 + sin(theta + phi2) * A2;


#elif HAND == 1
    double A1 = hh->getElectricGripper().A1;
    double minAp = hh->getElectricGripper().minAperture;

    double d = -(minAp - d_obj);
    this->dFH = A1;
#endif


    if(success)
    {
        bool prec = this->mov->getGrip();
        int arm = this->mov->getArm();

        switch(arm)
        {
        case 0: // both arms
            break;
        case 1: // right arm
#if HAND == 0
            this->rightFinalHand.at(0) = THETA8_FINAL;
            this->rightFinalPosture.at(JOINTS_ARM) = THETA8_FINAL;
            this->rightFinalPosture_diseng.at(JOINTS_ARM) = THETA8_FINAL;
            this->rightFinalPosture_eng.at(JOINTS_ARM) = THETA8_FINAL;

            for (int i = 0; i < HAND_FINGERS; ++i)
            {
                 this->rightFinalHand.at(i+1) = theta;
                 this->rightFinalPosture.at(JOINTS_ARM + i + 1) = theta;
            }
#elif HAND == 1
            this->rightFinalHand.at(0) = d;
            this->rightFinalPosture.at(JOINTS_ARM) = d;
            this->rightFinalPosture_diseng.at(JOINTS_ARM) = d;
            this->rightFinalPosture_eng.at(JOINTS_ARM) = d;
#endif
            if(prec)
                this->dHOr = this->dFH;
            else
                this->dHOr = d_obj / 2 + TOL_GRIP;
            break;
        case 2: // left arm
#if HAND == 0
            this->leftFinalHand.at(0) = THETA8_FINAL; // spread of F1 and F2
            this->leftFinalPosture.at(JOINTS_ARM) = THETA8_FINAL;

            for (int i = 0; i < HAND_FINGERS; ++i)
            {
                 this->leftFinalHand.at(i + 1) = theta;
                 this->leftFinalPosture.at(JOINTS_ARM + i + 1) = theta;
            }
#elif HAND == 1
            this->leftFinalHand.at(0) = d;
            this->leftFinalPosture.at(JOINTS_ARM) = d;
#endif
            if(prec)
                this->dHOl = this->dFH - 3;
            else
                this->dHOl = d_obj / 2 + TOL_GRIP;
            break;
        }
    }

    return success;
}


bool Problem::getRPY(std::vector<double>& rpy, Matrix3d& Rot)
{
    if((Rot.cols()==3) && (Rot.rows()==3))
    {// the matrix is not empy
        rpy.resize(3,0);
        if((Rot(0,0)<1e-10) && (Rot(1,0)<1e-10))
        {// singularity
            rpy.at(0) = 0; // roll
            rpy.at(1) = std::atan2(-Rot(2,0),Rot(0,0)); // pitch
            rpy.at(2) = std::atan2(-Rot(1,2),Rot(1,1)); // yaw
            return false;
        }
        else
        {
            rpy.at(0) = std::atan2(Rot(1,0),Rot(0,0)); // roll
            double sp = std::sin(rpy.at(0)); double cp = std::cos(rpy.at(0));
            rpy.at(1) = std::atan2(-Rot(2,0),cp*Rot(0,0)+sp*Rot(1,0)); // pitch
            rpy.at(2) = std::atan2(sp*Rot(0,2)-cp*Rot(1,2),cp*Rot(1,1)-sp*Rot(0,1)); // yaw
            return true;
        }
    }
    else
        return false;
}


bool Problem::RPY_matrix(std::vector<double>& rpy, Matrix3d& Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty())
    {
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

        return true;
    }
    else
        return false;
}


movementPtr Problem::getMovement()
{
    return this->mov;
}


HUMotion::planning_result_ptr Problem::solve(HUMotion::hump_params &params)
{
    this->solved = false;
    int arm_code =  this->mov->getArm();
    int mov_type = this->mov->getType();
    int sceneID = this->scene->getID();
    params.mov_specs.support_obj = "Table";

#if HAND == 0
    // Barrett Hand
    int hand_code = 1;
#elif HAND == 1
    // Electric Parallel Gripper
    int hand_code = 2;
#endif

    std::vector<double> finalHand;
    std::vector<double> homePosture;
    std::vector<double> initPosture;
    std::vector<double> eng_to_tar;

    double dHO;
    targetPtr tar;
    objectPtr obj; engagePtr eng;
    objectPtr obj_eng; engagePtr eng1;
    posePtr pose;

    std::vector<double> target;
    std::vector<double> tar_pose;
    std::vector<double> place_location;

    if(mov_type != 1 && mov_type != 5)
    {
        try
        {
            this->finalPostureHand();
        }
        catch(const string message)
        {
            throw message;
        }

        obj = this->mov->getObject();
        pose = this->mov->getPose();

        // engaging info
        obj_eng = this->mov->getObjectEng();
        eng = obj->getEngagePoint();
        eng1 = obj_eng->getEngagePoint();
    }


    switch(arm_code)
    {
    case 0: // both arms
        break;
    case 1://right arm
        if(obj != nullptr)
            obj->getEngTarRight(eng_to_tar);

        this->scene->getRobot()->getRightPosture(initPosture);
        this->scene->getRobot()->getRightArmHomePosture(homePosture);

        if(mov_type == 5)
            this->scene->getRobot()->getRightHandHomePosture(finalHand);
        else if(mov_type == 1)
            finalHand=this->move_final_hand;
        else
        {
            dHO = this->dHOr;
            tar = obj->getTargetRight();
            finalHand = this->rightFinalHand;
        }
        break;
    case 2:// left arm
        if(obj != nullptr)
            obj->getEngTarLeft(eng_to_tar);

        this->scene->getRobot()->getLeftPosture(initPosture);
        this->scene->getRobot()->getLeftArmHomePosture(homePosture);

        if(mov_type == 5)
             this->scene->getRobot()->getLeftHandHomePosture(finalHand);
        else if(mov_type == 1)
            finalHand = this->move_final_hand;
        else
        {
            dHO = this->dHOl;
            finalHand = this->leftFinalHand;
            tar = obj->getTargetLeft();
        }
        break;
    }


    if(mov_type != 1 && mov_type != 5)
    {
        // compute the position of the target when the object will be engaged
        pos eng1_pos = eng1->getPos(); // position related to the world of the engage point of the other object
        orient eng1_or = eng1->getOr(); // orientation of the engage point of the other object
        std::vector<double> rpy_eng1 = {eng1_or.roll, eng1_or.pitch, eng1_or.yaw};
        Matrix3d Rot_eng1; this->RPY_matrix(rpy_eng1, Rot_eng1);

        pos new_tar;
        std::vector<double> rpy = {tar->getOr().roll, tar->getOr().pitch, tar->getOr().yaw};
        Matrix3d Rot_tar; this->RPY_matrix(rpy, Rot_tar);
        Vector3d v(eng_to_tar.at(0),eng_to_tar.at(1),eng_to_tar.at(2));
        Vector3d eng_to_tar_w = Rot_tar*v;
        new_tar.Xpos = eng1_pos.Xpos - eng_to_tar_w(0);
        new_tar.Ypos = eng1_pos.Ypos - eng_to_tar_w(1);
        new_tar.Zpos = eng1_pos.Zpos - eng_to_tar_w(2);

        orient eng_or = eng->getOr(); // orientation of the engage point of the object to engage
        std::vector<double> rpy_eng = {eng_or.roll, eng_or.pitch, eng_or.yaw};
        Matrix3d Rot_eng; this->RPY_matrix(rpy_eng,Rot_eng);
        Matrix3d Rot_eng_inv = Rot_eng.transpose();
        Matrix3d Rot_eng_tar = Rot_eng_inv * Rot_tar;
        Matrix3d Rot_new_tar = Rot_eng1 * Rot_eng_tar;
        std::vector<double> rpy_new_tar; this->getRPY(rpy_new_tar,Rot_new_tar);

        place_location.push_back(new_tar.Xpos);
        place_location.push_back(new_tar.Ypos);
        place_location.push_back(new_tar.Zpos);
        place_location.push_back(rpy_new_tar.at(0));
        place_location.push_back(rpy_new_tar.at(1));
        place_location.push_back(rpy_new_tar.at(2));

        HUMotion::objectPtr hump_obj;
        target = {tar->getPos().Xpos, tar->getPos().Ypos, tar->getPos().Zpos,tar->getOr().roll,tar->getOr().pitch,tar->getOr().yaw};
        tar_pose = {pose->getPos().Xpos, pose->getPos().Ypos, pose->getPos().Zpos, pose->getOr().roll, pose->getOr().pitch, pose->getOr().yaw};

        std::vector<double> position = {obj->getPos().Xpos,obj->getPos().Ypos,obj->getPos().Zpos};
        std::vector<double> orientation = {obj->getOr().roll,obj->getOr().pitch,obj->getOr().yaw};
        std::vector<double> dimension = {obj->getSize().Xsize,obj->getSize().Ysize,obj->getSize().Zsize};
        hump_obj.reset(new HUMotion::Object(obj->getName()));
        hump_obj->setParams(position, orientation, dimension);

        // movement settings
        params.mov_specs.dHO = dHO;
        params.mov_specs.obj = hump_obj;
    }

    // movement settings
    params.mov_specs.arm_code = arm_code;
    params.mov_specs.hand_code = hand_code;
    params.mov_specs.mov_infoline = this->mov->getInfoLine();
    params.mov_specs.finalHand = finalHand;

    HUMotion::planning_result_ptr res;
    long long curr_time;
    switch(mov_type)
    {
    case 0:// reach-to-grasp
        params.mov_specs.target = target;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_pick(params,initPosture);
        this->exec_time = double(this->GetTimeMs64() - curr_time);
        break;
    case 1:// reaching
        if(this->use_posture)
        {
            curr_time = this->GetTimeMs64();
            res = this->h_planner->plan_move(params,initPosture,this->move_final_arm);
            this->exec_time = double(this->GetTimeMs64() - curr_time);
        }
        else
        {
            params.mov_specs.target=this->move_target;
            curr_time = this->GetTimeMs64();
            res = this->h_planner->plan_move(params,initPosture);
            this->exec_time = double(this->GetTimeMs64() - curr_time);
        }
        break;
    case 2://transport
        if (sceneID==6)
            params.mov_specs.support_obj = "Shelf_2_a";
        params.mov_specs.target = tar_pose;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64() - curr_time);
        break;
    case 3://engage
        params.mov_specs.support_obj = obj_eng->getName();
        params.mov_specs.target = place_location;
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_place(params,initPosture);
        this->exec_time = double(this->GetTimeMs64() - curr_time);
        break;
    case 4:// disengage
        break;
    case 5:// go-park
        curr_time = this->GetTimeMs64();
        res = this->h_planner->plan_move(params,initPosture,homePosture);
        this->exec_time = double(this->GetTimeMs64() - curr_time);
        break;
    }

    this->h_params = params;

    if(res != nullptr)
        if(res->status==0)
            this->solved=true;

    return res;
}


long long Problem::GetTimeMs64()
{
#ifdef WIN32
    /* Windows */
    FILETIME ft;
    LARGE_INTEGER li;
    /* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
    * to a LARGE_INTEGER structure. */
    GetSystemTimeAsFileTime(&ft);
    li.LowPart = ft.dwLowDateTime;
    li.HighPart = ft.dwHighDateTime;
    unsigned long long ret = li.QuadPart;
    ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
    ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */
    return ret;
#else
    /* Linux */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t ret = tv.tv_usec;
    /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
    ret /= 1000;
    /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
    ret += (tv.tv_sec * 1000);

    return ret;
#endif
}


double Problem::getTime()
{
    return this->exec_time/1000;
}

}// motion_manager
