#include "../include/motion_manager/robot.hpp"


namespace motion_manager{

#if HAND == 0
Robot::Robot(string name, robot_part torsospecs, arm aspecs, barrett_hand hspecs)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_barrett_hand_specs = hspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


Robot::Robot(string name, robot_part torsospecs, arm aspecs, barrett_hand hspecs,
             vector<double>& r, vector<double>& l)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_barrett_hand_specs = hspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


Robot::Robot(string name, robot_part torsospecs, arm aspecs, barrett_hand hspecs,
             vector<double> &r, vector<double> &l,
             vector<double> &min_rl, vector<double> &max_rl,
             vector<double> &min_ll, vector<double> &max_ll)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_barrett_hand_specs = hspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


#if HEAD==1
Robot::Robot(string name, robot_part torsospecs, arm aspecs, barrett_hand hspecs,
             robot_part headspecs, vector<double> &r, vector<double> &l,
             vector<double> &min_rl, vector<double> &max_rl,
             vector<double> &min_ll, vector<double> &max_ll)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_barrett_hand_specs = hspecs;
    this->m_head = headspecs;

    this->rk.push_back(-1.0);
    this->rk.push_back(1.0);
    this->rk.push_back(0.0);

    this->jk.push_back(-1.0);
    this->jk.push_back(-1.0);
    this->jk.push_back(1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}
#endif


#elif HAND == 1
Robot::Robot(string name, robot_part torsospecs, arm aspecs, electric_gripper hspecs)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_electric_gripper_specs = hspecs;

    this->rk.push_back(1.0);
    this->rk.push_back(-1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


Robot::Robot(string name, robot_part torsospecs, arm aspecs, electric_gripper hspecs,
             vector<double>& r, vector<double>& l)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_electric_gripper_specs = hspecs;
    this->rk.push_back(1.0);
    this->rk.push_back(-1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
    {
        rightPosture.at(i + JOINTS_ARM) = rightPosture.at(i + JOINTS_ARM) * 1000;
        leftPosture.at(i + JOINTS_ARM) = leftPosture.at(i + JOINTS_ARM) * 1000;
        rightHomePosture.at(i + JOINTS_ARM) = rightHomePosture.at(i + JOINTS_ARM) * 1000;
        leftHomePosture.at(i + JOINTS_ARM) = leftHomePosture.at(i + JOINTS_ARM) * 1000;

        min_rightLimits.at(i + JOINTS_ARM) = min_rightLimits.at(i + JOINTS_ARM) * 1000;
        min_leftLimits.at(i + JOINTS_ARM) = min_leftLimits.at(i + JOINTS_ARM) * 1000;
        max_rightLimits.at(i + JOINTS_ARM) = max_rightLimits.at(i + JOINTS_ARM) * 1000;
        max_leftLimits.at(i + JOINTS_ARM) = max_leftLimits.at(i + JOINTS_ARM) * 1000;
    }
#endif

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


Robot::Robot(string name, robot_part torsospecs, arm aspecs, electric_gripper hspecs,
             vector<double> &r, vector<double> &l,
             vector<double> &min_rl, vector<double> &max_rl,
             vector<double> &min_ll, vector<double> &max_ll)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_electric_gripper_specs = hspecs;
    this->rk.push_back(1.0);
    this->rk.push_back(-1.0);

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
    {
        rightPosture.at(i + JOINTS_ARM) = rightPosture.at(i + JOINTS_ARM) * 1000;
        leftPosture.at(i + JOINTS_ARM) = leftPosture.at(i + JOINTS_ARM) * 1000;
        rightHomePosture.at(i + JOINTS_ARM) = rightHomePosture.at(i + JOINTS_ARM) * 1000;
        leftHomePosture.at(i + JOINTS_ARM) = leftHomePosture.at(i + JOINTS_ARM) * 1000;

        min_rightLimits.at(i + JOINTS_ARM) = min_rightLimits.at(i + JOINTS_ARM) * 1000;
        min_leftLimits.at(i + JOINTS_ARM) = min_leftLimits.at(i + JOINTS_ARM) * 1000;
        max_rightLimits.at(i + JOINTS_ARM) = max_rightLimits.at(i + JOINTS_ARM) * 1000;
        max_leftLimits.at(i + JOINTS_ARM) = max_leftLimits.at(i + JOINTS_ARM) * 1000;
    }
#endif

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}


#if HEAD==1
Robot::Robot(string name, robot_part torsospecs, arm aspecs, electric_gripper hspecs,
             robot_part headspecs, vector<double> &r, vector<double> &l,
             vector<double> &min_rl, vector<double> &max_rl,
             vector<double> &min_ll, vector<double> &max_ll)
{
    this->m_name = name;
    this->m_torso = torsospecs;
    this->m_arm_specs = aspecs;

    this->m_electric_gripper_specs = hspecs;
    this->rk.push_back(1.0);
    this->rk.push_back(-1.0);

    this->m_head = headspecs;

    this->rightPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftPosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftHomePosture = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(r.begin(),r.end(),this->rightPosture.begin());
    std::copy(l.begin(),l.end(),this->leftPosture.begin());
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

    this->min_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->min_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_rightLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->max_leftLimits = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
    {
        rightPosture.at(i + JOINTS_ARM) = rightPosture.at(i + JOINTS_ARM) * 1000;
        leftPosture.at(i + JOINTS_ARM) = leftPosture.at(i + JOINTS_ARM) * 1000;
        rightHomePosture.at(i + JOINTS_ARM) = rightHomePosture.at(i + JOINTS_ARM) * 1000;
        leftHomePosture.at(i + JOINTS_ARM) = leftHomePosture.at(i + JOINTS_ARM) * 1000;

        min_rightLimits.at(i + JOINTS_ARM) = min_rightLimits.at(i + JOINTS_ARM) * 1000;
        min_leftLimits.at(i + JOINTS_ARM) = min_leftLimits.at(i + JOINTS_ARM) * 1000;
        max_rightLimits.at(i + JOINTS_ARM) = max_rightLimits.at(i + JOINTS_ARM) * 1000;
        max_leftLimits.at(i + JOINTS_ARM) = max_leftLimits.at(i + JOINTS_ARM) * 1000;
    }
#endif

    this->rightVelocities= vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftVelocities = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->rightForces = vector<double>(JOINTS_ARM+JOINTS_HAND);
    this->leftForces = vector<double>(JOINTS_ARM+JOINTS_HAND);

    this->mat_right = Matrix4d::Identity(4,4);
    this->mat_left = Matrix4d::Identity(4,4);
    this->mat_r_hand = Matrix4d::Identity(4,4);
    this->mat_l_hand = Matrix4d::Identity(4,4);
}
#endif
#endif


Robot::Robot(const Robot &hh)
{
    this->m_name = hh.m_name;
    this->m_torso = hh.m_torso;
    this->m_arm_specs = hh.m_arm_specs;

#if HAND == 0
    this->m_barrett_hand_specs = hh.m_barrett_hand_specs;
    this->rk = hh.rk;
    this->jk = hh.jk;
#elif HAND == 1
    this->m_electric_gripper_specs = hh.m_electric_gripper_specs;
    this->rk = hh.rk;
#endif

    this->m_DH_rightArm = hh.m_DH_rightArm;
    this->m_DH_leftArm = hh.m_DH_leftArm;
    this->m_DH_rightHand = hh.m_DH_rightHand;
    this->m_DH_leftHand = hh.m_DH_leftHand;

#if HEAD==1
    this->m_head=hh.m_head;
#endif

    this->mat_right = hh.mat_right;
    this->mat_left = hh.mat_left;
    this->mat_r_hand = hh.mat_r_hand;
    this->mat_l_hand = hh.mat_l_hand;

    this->max_rightLimits = hh.max_rightLimits;
    this->min_rightLimits = hh.min_rightLimits;
    this->max_leftLimits = hh.max_leftLimits;
    this->min_leftLimits=hh.min_leftLimits;
    this->rightPosture = hh.rightPosture;
    this->leftPosture = hh.leftPosture;
    this->rightVelocities = hh.rightVelocities;
    this->leftVelocities = hh.leftVelocities;
    this->rightForces = hh.rightForces;
    this->leftForces = hh.leftForces;
    this->rightHomePosture = hh.rightHomePosture;
    this->leftHomePosture = hh.leftHomePosture;

    this->rightShoulderPos = hh.rightShoulderPos;
    this->rightShoulderOr = hh.rightShoulderOr;
    this->rightElbowPos = hh.rightElbowPos;
    this->rightElbowOr = hh.rightElbowOr;
    this->rightWristPos = hh.rightWristPos;
    this->rightWristOr = hh.rightWristOr;
    this->rightHandPos = hh.rightHandPos;
    this->rightHandOr = hh.rightHandOr;
    this->rightFingers = hh.rightFingers;

    this->leftShoulderPos = hh.leftShoulderPos;
    this->leftShoulderOr = hh.leftShoulderOr;
    this->leftElbowPos = hh.leftElbowPos;
    this->leftElbowOr = hh.leftElbowOr;
    this->leftWristPos = hh.leftWristPos;
    this->leftWristOr = hh.leftWristOr;
    this->leftHandPos = hh.leftHandPos;
    this->leftHandOr = hh.leftHandOr;
    this->leftFingers = hh.leftFingers;
}


Robot::~Robot()
{

}


void Robot::setName(string& name)
{
    this->m_name = name;
}


void Robot::setTorso(robot_part& torso)
{
    this->m_torso=torso;
}


void Robot::setArm(arm& specs)
{
    this->m_arm_specs = specs;
}


#if HAND == 0
void Robot::setBarrettHand(barrett_hand& specs)
{
    this->m_barrett_hand_specs=specs;
}


#elif HAND == 1
void Robot::setElectricGripper(electric_gripper& specs)
{
    this->m_electric_gripper_specs = specs;
}
#endif


void Robot::setRightPosture(vector<double> &r)
{
    std::copy(r.begin(),r.end(),this->rightPosture.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        rightPosture.at(i + JOINTS_ARM) = rightPosture.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setLeftPosture(vector<double> &l)
{
    std::copy(l.begin(),l.end(),this->leftPosture.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        leftPosture.at(i + JOINTS_ARM) = leftPosture.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setRightHomePosture(vector<double> &r)
{
    std::copy(r.begin(),r.end(),this->rightHomePosture.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        rightHomePosture.at(i + JOINTS_ARM) = rightHomePosture.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setLeftHomePosture(vector<double> &l)
{
    std::copy(l.begin(),l.end(),this->leftHomePosture.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        leftHomePosture.at(i + JOINTS_ARM) = leftHomePosture.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setRightMinLimits(vector<double> &min_rl)
{
    std::copy(min_rl.begin(),min_rl.end(),this->min_rightLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        min_rightLimits.at(i + JOINTS_ARM) = min_rightLimits.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setRightMaxLimits(vector<double> &max_rl)
{
    std::copy(max_rl.begin(),max_rl.end(),this->max_rightLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        max_rightLimits.at(i + JOINTS_ARM) = max_rightLimits.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setLeftMinLimits(vector<double> &min_ll)
{
    std::copy(min_ll.begin(),min_ll.end(),this->min_leftLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        min_leftLimits.at(i + JOINTS_ARM) = min_leftLimits.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setLeftMaxLimits(vector<double> &max_ll)
{
    std::copy(max_ll.begin(),max_ll.end(),this->max_leftLimits.begin());

#if HAND == 1
    for(size_t i = 0; i < JOINTS_HAND; ++i)
        max_leftLimits.at(i + JOINTS_ARM) = max_leftLimits.at(i + JOINTS_ARM) * 1000;
#endif
}


void Robot::setRightVelocities(vector<double> &r)
{
    std::copy(r.begin(),r.end(),this->rightVelocities.begin());
}


void Robot::setLeftVelocities(vector<double> &l)
{
    std::copy(l.begin(),l.end(),this->leftVelocities.begin());
}


void Robot::setRightForces(vector<double> &r)
{
    std::copy(r.begin(),r.end(),this->rightForces.begin());
}


void Robot::setLeftForces(vector<double> &l)
{
    std::copy(l.begin(),l.end(),this->leftForces.begin());

}


void Robot::setMatRight(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            this->mat_right(i, j) = m(i,j);
    }
}


void Robot::setMatLeft(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            this->mat_left(i, j) = m(i,j);
    }
}


void Robot::setMatRightHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            this->mat_r_hand(i, j) = m(i,j);
    }
}


void Robot::setMatLeftHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            this->mat_l_hand(i, j) = m(i,j);
    }
}

#if HEAD==1
void Robot::setHead(robot_part& head)
{
    this->m_head=head;
}
#endif


string Robot::getName()
{
    return this->m_name;
}


robot_part Robot::getTorso()
{
    return this->m_torso;
}


void Robot::getRK(vector<int> &rkk)
{
    rkk = this->rk;
}


#if HAND == 0
void Robot::getJK(vector<int> &jkk)
{
    jkk = this->jk;
}
#endif


arm Robot::getArm()
{
    return this->m_arm_specs;
}


#if HAND == 0
barrett_hand Robot::getBarrettHand()
{
    return this->m_barrett_hand_specs;
}


#elif HAND == 1
electric_gripper Robot::getElectricGripper()
{
    return this->m_electric_gripper_specs;
}
#endif


void Robot::getRightPosture(vector<double>& p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightPosture.begin(),this->rightPosture.end(),p.begin());
}


void Robot::getRightArmPosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->rightPosture.begin(),this->rightPosture.end()-JOINTS_HAND,p.begin());
}


void Robot::getRightHandPosture(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->rightPosture.begin()+JOINTS_ARM,this->rightPosture.end(),p.begin());
}


void Robot::getLeftPosture(vector<double>& p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftPosture.begin(),this->leftPosture.end(),p.begin());
}


void Robot::getLeftArmPosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->leftPosture.begin(),this->leftPosture.end()-JOINTS_HAND,p.begin());
}


void Robot::getLeftHandPosture(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->leftPosture.begin()+JOINTS_ARM,this->leftPosture.end(),p.begin());
}


void Robot::getRightHomePosture(vector<double>& p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightHomePosture.begin(),this->rightHomePosture.end(),p.begin());
}


void Robot::getRightHandHomePosture(vector<double> &p)
{
   p = vector<double>(JOINTS_HAND);

   std::copy(this->rightHomePosture.begin()+JOINTS_ARM,this->rightHomePosture.end(),p.begin());
}


void Robot::getRightArmHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->rightHomePosture.begin(),this->rightHomePosture.end()-JOINTS_HAND,p.begin());
}


void Robot::getLeftHandHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->leftHomePosture.begin()+JOINTS_ARM,this->leftHomePosture.end(),p.begin());
}


void Robot::getLeftArmHomePosture(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->leftHomePosture.begin(),this->leftHomePosture.end()-JOINTS_HAND,p.begin());
}


void Robot::getLeftHomePosture(vector<double>& p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftHomePosture.begin(),this->leftHomePosture.end(),p.begin());
}


void Robot::getRightMinLimits(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_rightLimits.begin(),this->min_rightLimits.end(),p.begin());
}


void Robot::getRightMaxLimits(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_rightLimits.begin(),this->max_rightLimits.end(),p.begin());
}


void Robot::getLeftMinLimits(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->min_leftLimits.begin(),this->min_leftLimits.end(),p.begin());
}


void Robot::getLeftMaxLimits(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->max_leftLimits.begin(),this->max_leftLimits.end(),p.begin());
}


void Robot::getRightVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightVelocities.begin(),this->rightVelocities.end(),p.begin());
}


void Robot::getRightArmVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->rightVelocities.begin(),this->rightVelocities.end()-JOINTS_HAND,p.begin());
}


void Robot::getRightHandVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->rightVelocities.begin()+JOINTS_ARM,this->rightVelocities.end(),p.begin());
}


void Robot::getLeftVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftVelocities.begin(),this->leftVelocities.end(),p.begin());
}


void Robot::getLeftArmVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->leftVelocities.begin(),this->leftVelocities.end()-JOINTS_HAND,p.begin());
}


void Robot::getLeftHandVelocities(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->leftVelocities.begin()+JOINTS_ARM,this->leftVelocities.end(),p.begin());
}


void Robot::getRightForces(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->rightForces.begin(),this->rightForces.end(),p.begin());
}


void Robot::getRightArmForces(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->rightForces.begin(),this->rightForces.end()-JOINTS_HAND,p.begin());
}


void Robot::getRightHandForces(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->rightForces.begin()+JOINTS_ARM,this->rightForces.end(),p.begin());
}


void Robot::getLeftForces(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM+JOINTS_HAND);

    std::copy(this->leftForces.begin(),this->leftForces.end(),p.begin());
}


void Robot::getLeftArmForces(vector<double> &p)
{
    p = vector<double>(JOINTS_ARM);

    std::copy(this->leftForces.begin(),this->leftForces.end()-JOINTS_HAND,p.begin());
}


void Robot::getLeftHandForces(vector<double> &p)
{
    p = vector<double>(JOINTS_HAND);

    std::copy(this->leftForces.begin()+JOINTS_ARM,this->leftForces.end(),p.begin());
}


void Robot::getMatRight(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            m (i, j) = this->mat_right(i, j);
    }
}


void Robot::getMatLeft(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            m (i, j) = this->mat_left(i, j);
    }
}


void Robot::getMatRightHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            m (i, j) = this->mat_r_hand(i, j);
    }
}


void Robot::getMatLeftHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i)
    {
        for (unsigned j = 0; j < m.cols(); ++ j)
            m (i, j) = this->mat_l_hand(i, j);
    }
}


string Robot::getInfoLine()
{
    return  this->m_name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % this->m_torso.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % this->m_torso.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % this->m_torso.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % this->m_torso.Roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % this->m_torso.Pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % this->m_torso.Yaw) + RAD + SEP+
            XsizeSTR + str(boost::format("%d") % this->m_torso.Xsize) + MILLIMETERS + SEP+
            YsizeSTR + str(boost::format("%d") % this->m_torso.Ysize) + MILLIMETERS + SEP+
            ZsizeSTR + str(boost::format("%d") % this->m_torso.Zsize)+ MILLIMETERS;
}


DHparams Robot::getDH_rightArm()
{
    this->computeRightArmDHparams();
    return this->m_DH_rightArm;
}


DHparams Robot::getDH_leftArm()
{
    this->computeLeftArmDHparams();
    return this->m_DH_leftArm;
}


void Robot::getRightShoulderPos(vector<double> &pos)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightShoulderPos;
}


double Robot::getRightShoulderNorm()
{
    vector<double> pos;
    this->getRightShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getRightShoulderOr(Matrix3d &orr)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr=rightShoulderOr;
}


void Robot::getRightElbowPos(vector<double> &pos)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightElbowPos;
}


double Robot::getRightElbowNorm()
{
    vector<double> pos;
    this->getRightElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getRightElbowOr(Matrix3d &orr)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightElbowOr;
}


void Robot::getRightWristPos(vector<double> &pos)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightWristPos;
}


double Robot::getRightWristNorm()
{
    vector<double> pos;
    this->getRightWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getRightWristOr(Matrix3d &orr)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightWristOr;
}


void Robot::getRightHandPos(vector<double> &pos)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    pos = rightHandPos;
}


double Robot::getRightHandNorm()
{
    vector<double> pos;
    this->getRightHandPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getRightHandOr(Matrix3d &orr)
{
    // direct kinematics of the right arm
    std::vector<double> posture;
    this->getRightArmPosture(posture);
    directKinematicsSingleArm(1,posture);

    orr = rightHandOr;
}


void Robot::getRightHandVel(vector<double> &vel)
{
    std::vector<double> posture; std::vector<double> velocities;
    this->getRightArmPosture(posture); this->getRightArmVelocities(velocities);
    directDiffKinematicsSingleArm(1,posture,velocities,vel,3);
}


double Robot::getRightHandVelNorm()
{
    std::vector<double> hand_vel;
    this->getRightHandVel(hand_vel);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}


void Robot::getLeftShoulderPos(vector<double> &pos)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftShoulderPos;
}


double Robot::getLeftShoulderNorm()
{
    vector<double> pos;
    this->getLeftShoulderPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getLeftShoulderOr(Matrix3d &orr)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftShoulderOr;
}


void Robot::getLeftElbowPos(vector<double> &pos)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftElbowPos;
}


double Robot::getLeftElbowNorm()
{
    vector<double> pos;
    this->getLeftElbowPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getLeftElbowOr(Matrix3d &orr)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftElbowOr;
}


void Robot::getLeftWristPos(vector<double> &pos)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftWristPos;
}


double Robot::getLeftWristNorm()
{
    vector<double> pos;
    this->getLeftWristPos(pos);

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getLeftWristOr(Matrix3d &orr)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftWristOr;
}


void Robot::getLeftHandPos(vector<double> &pos)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    pos = leftHandPos;
}


double Robot::getLeftHandNorm()
{
    vector<double> pos;
    this->getLeftHandPos(pos);
    pos = leftHandPos;

    return sqrt(pow(pos.at(0),2)+pow(pos.at(1),2)+pow(pos.at(2),2));
}


void Robot::getLeftHandOr(Matrix3d &orr)
{
    // direct kinematics of the left arm
    std::vector<double> posture;
    this->getLeftArmPosture(posture);
    directKinematicsSingleArm(2,posture);

    orr = leftHandOr;
}


void Robot::getLeftHandVel(vector<double> &vel)
{
    std::vector<double> posture; std::vector<double> velocities;
    this->getLeftArmPosture(posture); this->getLeftArmVelocities(velocities);
    directDiffKinematicsSingleArm(2,posture,velocities,vel,3);
}


double Robot::getLeftHandVelNorm()
{
    std::vector<double> hand_vel;
    this->getLeftHandVel(hand_vel);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}


void Robot::getHandPos(int arm, vector<double> &pos, vector<double> &posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(3);
    Matrix3d shoulderOr;
    vector<double> elbowPos = vector<double>(3);
    Matrix3d elbowOr;
    vector<double> wristPos = vector<double>(3);
    Matrix3d wristOr;
    vector<double> handPos = vector<double>(3);
    Matrix3d handOr;

    switch (arm)
    {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i)
    {
        this->transfMatrix(m_DH_arm.alpha.at(i), m_DH_arm.a.at(i), m_DH_arm.d.at(i), posture.at(i), T_aux);
        T = T * T_aux;
        Vector3d v;
        if (i == 1)
        {
            // get the shoulder
            shoulderOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
        }
        else if (i == 3)
        {
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
        }
        else if (i == 5)
        {
            // get the wrist
            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
        }
        else if (i == 6)
        {
            //get the hand
            T = T * mat_hand;
            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
        }
    }

    pos.clear();
    pos.push_back(handPos[0]);
    pos.push_back(handPos[1]);
    pos.push_back(handPos[2]);
}


void Robot::getHandVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,3);
}


double Robot::getHandVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> hand_vel;
    this->getHandVel(arm,hand_vel,posture,velocities);

    return sqrt(pow(hand_vel.at(0),2)+pow(hand_vel.at(1),2)+pow(hand_vel.at(2),2));
}


void Robot::getWristVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,2);
}


double Robot::getWristVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> wrist_vel;
    this->getWristVel(arm,wrist_vel,posture,velocities);

    return sqrt(pow(wrist_vel.at(0),2)+pow(wrist_vel.at(1),2)+pow(wrist_vel.at(2),2));
}


void Robot::getElbowVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,1);
}


double Robot::getElbowVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> elbow_vel;
    this->getElbowVel(arm,elbow_vel,posture,velocities);

    return sqrt(pow(elbow_vel.at(0),2)+pow(elbow_vel.at(1),2)+pow(elbow_vel.at(2),2));
}


void Robot::getShoulderVel(int arm, vector<double> &vel, vector<double> &posture, vector<double> &velocities)
{
    directDiffKinematicsSingleArm(arm,posture,velocities,vel,0);
}


double Robot::getShoulderVelNorm(int arm, vector<double> &posture, vector<double> &velocities)
{
    std::vector<double> shoulder_vel;
    this->getShoulderVel(arm,shoulder_vel,posture,velocities);

    return sqrt(pow(shoulder_vel.at(0),2)+pow(shoulder_vel.at(1),2)+pow(shoulder_vel.at(2),2));
}


#if HEAD==1
robot_part Robot::getHead()
{
    return this->m_head;
}
#endif


void Robot::directKinematicsDualArm()
{
    // direct kinematics of the right arm
    std::vector<double> rposture;
    this->getRightArmPosture(rposture);
    directKinematicsSingleArm(1,rposture);
    // direct kinematics of the left arm
    std::vector<double> lposture;
    this->getLeftArmPosture(lposture);
    directKinematicsSingleArm(2,lposture);
}


void Robot::computeRightArmDHparams()
{
    this->m_DH_rightArm.a.clear();
    this->m_DH_rightArm.d.clear();
    this->m_DH_rightArm.alpha.clear();
    this->m_DH_rightArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i)
    {
        // d [mm]
        m_DH_rightArm.d.push_back(m_arm_specs.arm_specs.d.at(i));
        //a [mm]
        m_DH_rightArm.a.push_back(m_arm_specs.arm_specs.a.at(i));
        //alpha [rad]
        m_DH_rightArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));
        //theta [rad]
        m_DH_rightArm.theta.push_back(rightPosture.at(i));
    }
}


void Robot::computeLeftArmDHparams()
{
    this->m_DH_leftArm.a.clear();
    this->m_DH_leftArm.d.clear();
    this->m_DH_leftArm.alpha.clear();
    this->m_DH_leftArm.theta.clear();

    for (int i = 0; i < JOINTS_ARM; ++i)
    {
        // d [mm]
        m_DH_leftArm.d.push_back(-m_arm_specs.arm_specs.d.at(i));
        //a [mm]
        m_DH_leftArm.a.push_back(m_arm_specs.arm_specs.a.at(i));
        //alpha [rad]
        if ((i == 0))
            m_DH_leftArm.alpha.push_back(m_arm_specs.arm_specs.alpha.at(i));
        else
            m_DH_leftArm.alpha.push_back(-m_arm_specs.arm_specs.alpha.at(i));
        //theta [rad]
        m_DH_leftArm.theta.push_back(leftPosture.at(i));
    }
}


void Robot::computeRightHandDHparams()
{
    this->m_DH_rightHand.clear();

    for (int i = 0; i < HAND_FINGERS; ++i)
    {
        vector<double> t;
        this->getRightHandPosture(t);
        DHparams f;
        vector<double> fing_pos;

#if HAND == 0
        f.a = vector<double>(4);
        f.d = vector<double>(4);
        f.alpha = vector<double>(4);
        f.theta = vector<double>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i) * (m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i) * t.at(0)) - 1.57 * jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2 + t.at(i + 1);
        f.theta.at(2) = m_barrett_hand_specs.phi3 + (1/3) * t.at(i + 1);
        f.theta.at(3) = 0.0;
#elif HAND == 1
        f.a = vector<double>(2);
        f.d = vector<double>(2);
        f.alpha = vector<double>(2);
        f.theta = vector<double>(2);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = 0;
        f.a.at(1) = m_electric_gripper_specs.A1;

        //d [mm]
        f.d.at(0) = - ((m_electric_gripper_specs.maxAperture - t.at(0) * 1000) + m_electric_gripper_specs.D3);
        f.d.at(1) = 0;

        //alpha [rad]
        f.alpha.at(0) = rk.at(i) * 1.57;
        f.alpha.at(1) = 0;

        //theta [rad]
        f.theta.at(0) = rk.at(i) * 1.57;
        f.theta.at(1) = 0;
#endif
        m_DH_rightHand.push_back(f);
        right_fing_pos.push_back(fing_pos);
    }
}


void Robot::computeLeftHandDHparams()
{
    this->m_DH_leftHand.clear();

    for (int i = 0; i< HAND_FINGERS; ++i)
    {
        DHparams f;
        vector<double> fing_pos;
        vector<double> t;
        this->getLeftHandPosture(t);

#if HAND == 0
        f.a = vector<double>(4);
        f.d = vector<double>(4);
        f.alpha = vector<double>(4);
        f.theta = vector<double>(4);

        // finger positions [mm]
        fing_pos.push_back(0);
        fing_pos.push_back(0);
        fing_pos.push_back(0);

        //a [mm]
        f.a.at(0) = (rk.at(i)*(m_barrett_hand_specs.Aw));
        f.a.at(1) = m_barrett_hand_specs.A1;
        f.a.at(2) = m_barrett_hand_specs.A2;
        f.a.at(3) = m_barrett_hand_specs.A3;

        //d [mm]
        f.d.at(0) = 0.0;
        f.d.at(1) = 0.0;
        f.d.at(2) = 0.0;
        f.d.at(3) = m_barrett_hand_specs.D3;

        //alpha [rad]
        f.alpha.at(0) = 0.0;
        f.alpha.at(1) = 1.57;
        f.alpha.at(2) = 0.0;
        f.alpha.at(3) = -1.57;

        //theta [rad]
        f.theta.at(0) = (rk.at(i)*t.at(0))-1.57*jk.at(i);
        f.theta.at(1) = m_barrett_hand_specs.phi2+t.at(i+1);
        f.theta.at(2) = m_barrett_hand_specs.phi3+(1/3)*t.at(i+1);
        f.theta.at(3) = 0.0;
#endif
        m_DH_leftHand.push_back(f);
        left_fing_pos.push_back(fing_pos);
    }
}


void Robot::directKinematicsSingleArm(int arm, std::vector<double>& posture)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    vector<double> shoulderPos = vector<double>(3);
    Matrix3d shoulderOr;
    vector<double> elbowPos = vector<double>(3);
    Matrix3d elbowOr;
    vector<double> wristPos = vector<double>(3);
    Matrix3d wristOr;
    vector<double> handPos = vector<double>(3);
    Matrix3d handOr;

    switch (arm)
    {
    // right arm
    case 1:
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    //left arm
    case 2:
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }


    T = mat_world;
    for (int i = 0; i < posture.size(); ++i)
    {
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i), T_aux);

        T = T * T_aux;
        Vector3d v;

        if (i==1)
        {
            // get the shoulder

            shoulderOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];

            switch (arm)
            {
            case 1: // right arm
                this->rightShoulderPos = shoulderPos;
                this->rightShoulderOr = shoulderOr;
                break;
            case 2: // left arm
                this->leftShoulderPos = shoulderPos;
                this->leftShoulderOr = shoulderOr;
                break;
            }
        }
        else if (i==3)
        {
            // get the elbow
            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];

            switch(arm)
            {
            case 1: // right arm
                this->rightElbowPos = elbowPos;
                this->rightElbowOr = elbowOr;
                break;
            case 2: // left arm
                this->leftElbowPos = elbowPos;
                this->leftElbowOr = elbowOr;
                break;
            }
        }
        else if (i==5)
        {
            // get the wrist
            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];

            switch(arm)
            {
            case 1: // right arm
                this->rightWristPos = wristPos;
                this->rightWristOr = wristOr;
                break;
            case 2: // left arm
                this->leftWristPos = wristPos;
                this->leftWristOr = wristOr;
                break;
            }
        }
        else if (i==6)
        {
            //get the hand
            T = T * mat_hand;

            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];

            switch(arm)
            {
            case 1: // right arm
                this->rightHandPos = handPos;
                this->rightHandOr = handOr;
                break;
            case 2: // left arm
                this->leftHandPos = handPos;
                this->leftHandOr = handOr;
                break;
            }
        }
    }

    // Direct kinematics of the fingers
    this->rightFingers.resize(HAND_FINGERS,12);
    this->leftFingers.resize(HAND_FINGERS,12);

    Matrix4d T_H_0_pos;
    vector<double> fing_pos;

    for (int i=0; i< HAND_FINGERS; ++i)
    {
        DHparams p = m_DH_hand.at(i);
        switch (arm)
        {
        // right arm
        case 1:
            fing_pos=this->right_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,rightFingers);
            break;
        // left arm
        case 2:
            fing_pos=this->left_fing_pos.at(i);
            T_H_0_pos(0,3)=fing_pos.at(0);
            T_H_0_pos(1,3)=fing_pos.at(1);
            T_H_0_pos(2,3)=fing_pos.at(2);
            this->directKinematicsFinger(p,T,T_H_0_pos,i,leftFingers);
            break;
        }
    }
}


void Robot::directDiffKinematicsSingleArm(int arm,vector<double> posture, vector<double> velocities, vector<double>& vel, int mod)
{
    VectorXd joint_velocities;
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm)
    {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);
    joint_velocities.resize(velocities.size());

    for (int i = 0; i < posture.size(); ++i)
    {
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i)
        {
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
        joint_velocities(i) = velocities.at(i);
    }
    MatrixXd Jac_tmp; VectorXd joint_vel_tmp;
    // shoulder velocity
    Jac_tmp = JacobianArm.block<6,2>(0,0);
    joint_vel_tmp = joint_velocities.block<2,1>(0,0);
    VectorXd shoulder_vel_xd = Jac_tmp*joint_vel_tmp;
    // elbow velocity
    Jac_tmp = JacobianArm.block<6,4>(0,0);
    joint_vel_tmp = joint_velocities.block<4,1>(0,0);
    VectorXd elbow_vel_xd = Jac_tmp*joint_vel_tmp;
    // wrist velocity
    Jac_tmp = JacobianArm.block<6,6>(0,0);
    joint_vel_tmp = joint_velocities.block<6,1>(0,0);
    VectorXd wrist_vel_xd = Jac_tmp*joint_vel_tmp;
    // hand velocity
    VectorXd hand_vel_xd = JacobianArm*joint_velocities;

    switch(mod)
    {
    case 0: // shoulder
        vel.clear();
        vel.resize(shoulder_vel_xd.size());
        VectorXd::Map(&vel[0], shoulder_vel_xd.size()) = shoulder_vel_xd;
        break;
    case 1:// elbow
        vel.clear();
        vel.resize(elbow_vel_xd.size());
        VectorXd::Map(&vel[0], elbow_vel_xd.size()) = elbow_vel_xd;
        break;
    case 2: // wrist
        vel.clear();
        vel.resize(wrist_vel_xd.size());
        VectorXd::Map(&vel[0], wrist_vel_xd.size()) = wrist_vel_xd;
        break;
    case 3: // hand
        vel.clear();
        vel.resize(hand_vel_xd.size());
        VectorXd::Map(&vel[0], hand_vel_xd.size()) = hand_vel_xd;
        break;
    default: // hand
        vel.clear();
        vel.resize(hand_vel_xd.size());
        VectorXd::Map(&vel[0], hand_vel_xd.size()) = hand_vel_xd;
        break;
    }
}


void Robot::inverseDiffKinematicsSingleArm(int arm, vector<double> posture, vector<double> hand_vel, vector<double> &velocities)
{
    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparams m_DH_arm;
    vector<DHparams> m_DH_hand;

    MatrixXd JacobianArm(6,JOINTS_ARM);

    Vector3d pos0;
    Vector3d z0;
    Vector3d pos1;
    Vector3d z1;
    Vector3d pos2;
    Vector3d z2;
    Vector3d pos3;
    Vector3d z3;
    Vector3d pos4;
    Vector3d z4;
    Vector3d pos5;
    Vector3d z5;
    Vector3d pos6;
    Vector3d z6;

    vector<double> handPos; Vector3d pos_hand;
    this->getHandPos(arm,handPos,posture);

    switch (arm)
    {
    case 1: // right arm
        mat_world = this->mat_right;
        mat_hand = this->mat_r_hand;
        this->computeRightArmDHparams();
        this->computeRightHandDHparams();
        m_DH_arm = this->m_DH_rightArm;
        m_DH_hand = this->m_DH_rightHand;
        break;
    case 2: //left arm
        mat_world = this->mat_left;
        mat_hand = this->mat_l_hand;
        this->computeLeftArmDHparams();
        this->computeLeftHandDHparams();
        m_DH_arm = this->m_DH_leftArm;
        m_DH_hand = this->m_DH_leftHand;
        break;
    }

    T = mat_world;
    pos_hand << handPos.at(0), handPos.at(1), handPos.at(2);

    for (int i = 0; i < posture.size(); ++i)
    {
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i), T_aux);
        T = T * T_aux;
        Vector3d diff;
        Vector3d cross;
        Vector3d zi;
        switch(i){
        case 0:
            z0 = T.block(0,2,3,1);
            pos0 = T.block(0,3,3,1);
            diff = pos_hand - pos0;
            cross = z0.cross(diff);
            zi=z0;
            break;
        case 1:
            z1 = T.block(0,2,3,1);
            pos1 = T.block(0,3,3,1);
            diff = pos_hand - pos1;
            cross = z1.cross(diff);
            zi=z1;
            break;
        case 2:
            z2 = T.block(0,2,3,1);
            pos2 = T.block(0,3,3,1);
            diff = pos_hand - pos2;
            cross = z2.cross(diff);
            zi=z2;
            break;
        case 3:
            z3 = T.block(0,2,3,1);
            pos3 = T.block(0,3,3,1);
            diff = pos_hand - pos3;
            cross = z3.cross(diff);
            zi=z3;
            break;
        case 4:
            z4 = T.block(0,2,3,1);
            pos4 = T.block(0,3,3,1);
            diff = pos_hand - pos4;
            cross = z4.cross(diff);
            zi=z4;
            break;
        case 5:
            z5 = T.block(0,2,3,1);
            pos5 = T.block(0,3,3,1);
            diff = pos_hand - pos5;
            cross = z5.cross(diff);
            zi=z5;
            break;
        case 6:
            z6 = T.block(0,2,3,1);
            pos6 = T.block(0,3,3,1);
            diff = pos_hand - pos6;
            cross = z6.cross(diff);
            zi=z6;
            break;
        }
        VectorXd column(6); column << cross, zi;
        JacobianArm.col(i) = column;
    }

    double k; // damping factor
    MatrixXd I = MatrixXd::Identity(6,6);
    MatrixXd JacobianArmT = JacobianArm.transpose();
    MatrixXd JJ = JacobianArm*JacobianArmT;
    if(abs(JJ.determinant())<0.001)
        k = 0.01;
    else
        k=0.0;
    MatrixXd JT = JacobianArmT*(JJ+pow(k,2)*I);
    VectorXd hand_vel_xd(6);
    hand_vel_xd << hand_vel.at(0),hand_vel.at(1),hand_vel.at(2),hand_vel.at(3),hand_vel.at(4),hand_vel.at(5);
    VectorXd joint_velocities = JT*hand_vel_xd;
    velocities.clear();
    velocities.resize(joint_velocities.size());
    VectorXd::Map(&velocities[0], joint_velocities.size()) = joint_velocities;
}


void Robot::transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T)
{
    T = Matrix4d::Zero();

    T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha); T(1,1) = cos(theta)*cos(alpha);  T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
    T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;
}


void Robot::directKinematicsFinger(DHparams& p, Matrix4d& T_ext, Matrix4d& T_H_0_pos, int id_fing, MatrixXd& Fingers)
{
#if HAND == 0
     Matrix4d T;
     vector<double> pos = vector<double>(3);
     Matrix4d T_aux;

     for(int i =0; i<T_aux.rows();++i)
     {
         for(int j=0; j<T_aux.cols();++j)
             T_aux(i,j)=T_ext(i,j);
     }

     // translate to the begenning of each finger
     T_aux = T_aux * T_H_0_pos;

     for (int i=0; i< N_PHALANGE+1; ++i)
     {
         double a = p.a.at(i);
         double d = p.d.at(i);
         double alpha = p.alpha.at(i);
         double theta = p.theta.at(i);

         T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
         T(1,0) = sin(theta)*cos(alpha); T(1,1) = cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
         T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
         T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;

         T_aux = T_aux * T;
         pos[0] = T_aux(0,3);
         pos[1] = T_aux(1,3);
         pos[2] = T_aux(2,3);

         Fingers(id_fing,3*i) = pos[0]; Fingers(id_fing,3*i+1) = pos[1]; Fingers(id_fing,3*i+2) = pos[2];
     }
#endif
}

} // namespace motion_manager
