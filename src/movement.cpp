#include "../include/motion_manager/movement.hpp"


namespace motion_manager {

Movement::Movement(int type, int arm)
{
    this->obj=objectPtr(new Object());
    this->obj_init=objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());
    this->type=type;

    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    case 6:
        this->strType = string("Waypoints");
        break;
    }

    this->prec=false;
    this->arm = arm;
    this->executed = false;
}


Movement::Movement(int type, int arm, objectPtr obj)
{
    this->type=type;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());
    this->pose=posePtr(new Pose());
    this->prec=false;
    this->grip_str=string("No Grip");
    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm, posePtr pose)
{
    this->type=type;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }

    this->obj = objectPtr(new Object());
    this->obj_init = objectPtr(new Object());
    this->obj_eng=objectPtr(new Object());
    this->pose=pose;
    this->prec=false;
    this->grip_str=string("No Grip");
    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm, objectPtr obj, bool prec)
{
    this->type=type;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng=objectPtr(new Object());
    this->pose = posePtr(new Pose());

    this->prec=prec;
    if(prec)
        this->grip_str=string("Precision");
    else
        this->grip_str=string("Full");

    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm,objectPtr obj, objectPtr obj_eng,bool prec)
{
    this->type=type;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }
    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = obj_eng;
    this->pose = posePtr(new Pose());

    this->prec=prec;
    if(prec)
        this->grip_str=string("Precision");
    else
        this->grip_str=string("Full");

    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm,objectPtr obj, objectPtr obj_eng, posePtr pose, bool prec)
{
    this->type=type;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = obj_eng;
    this->pose = pose;

    this->prec = prec;

    if(prec)
        this->grip_str=string("Precision");
    else
        this->grip_str=string("Full");

    this->arm=arm;
    this->executed = false;
}


Movement::Movement(int type, int arm, objectPtr obj, posePtr pose,bool prec)
{
    this->type=type;
    switch (type){
    case 0:
        this->strType =
                /*
                if (obj && obj_eng && grip_code!=0){

                    return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                            ", Object Engaged: "+obj_eng->getName()+", Grip Type: "+grip_str;

                }else if (obj && pose && grip_code!=0){

                    return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                            ", Pose: "+pose->getName()+", Grip Type: "+grip_str;

                }else if (obj && obj_eng && pose && grip_code!=0){

                    return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                            ", Object Engaged: "+obj_eng->getName() + ", Pose: "+pose->getName()+
                            ", Grip Type: "+grip_str;

                }else if (pose){

                    return strType +", Arm: "+arm_info+", Pose: "+pose->getName();

                }else if(obj && grip_code!=0){


                    return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
                            ", Grip Type: "+grip_str;

                }else if(obj){

                    return strType +", Arm: "+arm_info+", Object: "+obj->getName();

                }else{

                    return strType+", Arm: "+arm_info;

                }
                */
string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go home");
        break;
    }

    this->obj = obj;
    this->obj_init = obj;
    this->obj_eng = objectPtr(new Object());
    this->pose = pose;

    this->prec=prec;
    if(prec)
        this->grip_str=string("Precision");
    else
        this->grip_str=string("Full");

    this->arm=arm;
    this->executed = false;
}


Movement::Movement(const Movement &mov)
{
    this->arm = mov.arm;
    this->type = mov.type;
    this->strType = mov.strType;
    this->prec=mov.prec;
    this->grip_str = mov.grip_str;

    this->obj = objectPtr(new Object(*mov.obj.get()));
    this->obj_init = objectPtr(new Object(*mov.obj_init.get()));
    this->obj_eng = objectPtr(new Object(*mov.obj_eng.get()));
    this->pose = posePtr(new Pose(*mov.pose.get()));
    this->executed = mov.executed;
}


Movement::~Movement()
{
}


void Movement::setType(int t)
{
    this->type=t;
    switch (type)
    {
    case 0:
        this->strType = string("Reach-to-grasp");
        break;
    case 1:
        this->strType = string("Reaching");
        break;
    case 2:
        this->strType = string("Transport");
        break;
    case 3:
        this->strType = string("Engage");
        break;
    case 4:
        this->strType = string("Disengage");
        break;
    case 5:
        this->strType = string("Go park");
        break;
    case 6:
        this->strType = string("Waypoints");
        break;
    }
}


void Movement::setGrip(bool prec)
{
    this->prec=prec;
    if(prec)
        this->grip_str=string("Precision");
    else
        this->grip_str=string("Full");
}


void Movement::setObject(objectPtr obj)
{
     this->obj=obj;
}


void Movement::setObjectInit(objectPtr obj)
{
     this->obj_init=obj;
}


void Movement::setObjectEng(objectPtr obj_eng)
{
   this->obj_eng = obj_eng;
}


void Movement::setArm(int a)
{
    this->arm = a;
}


void Movement::setExecuted(bool exec)
{
    this->executed=exec;
}


int Movement::getType()
{
    return this->type;
}


bool Movement::getGrip()
{
    return this->prec;
}


string Movement::getGripStr()
{
    return this->grip_str;
}


objectPtr Movement::getObject()
{
    return obj;
}


posePtr Movement::getPose()
{
    return pose;
}


objectPtr Movement::getObjectInit()
{
    return obj_init;
}


objectPtr Movement::getObjectEng()
{
    return obj_eng;
}


string Movement::getStrType()
{
    return this->strType;
}


string Movement::getInfoLine()
{
    string arm_info;

    switch (arm)
    {
    case 0:
        arm_info = string("both");
        break;
    case 1:
        arm_info = string("right");
        break;
    case 2:
        arm_info = string("left");
        break;
    default:
        arm_info = string("right");
        break;
    }

    return strType +", Arm: "+arm_info+", Object: "+obj->getName()+
            ", Object Engaged: "+obj_eng->getName() + ", Pose: "+pose->getName()+
            ", Grip Type: "+grip_str;
}


int Movement::getArm()
{
    return this->arm;
}


bool Movement::getExecuted()
{
    return this->executed;
}

} // namespace motion_manager

