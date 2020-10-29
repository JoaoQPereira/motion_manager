#include "../include/motion_manager/waypoint.hpp"

namespace motion_manager {



Waypoint::Waypoint()
{
    //this->waypoints.clear();
}


Waypoint::Waypoint(vector <waypoint> wps, bool wks)
{
    this->workspace = wks;
    this->waypoints = vector<waypoint>(wps.size());
    std::copy(wps.begin(),wps.end(),this->waypoints.begin());
}

Waypoint::Waypoint(int wp_nr, vector <waypoint> wps, bool wks, std::string wp_name)
{
    this->nr_wp = wp_nr;
    this->name=wp_name;
    this->workspace = wks;
    this->waypoints = vector<waypoint>(wps.size());
    std::copy(wps.begin(),wps.end(),this->waypoints.begin());
}
Waypoint::Waypoint(int wp_nr, vector <waypoint> wps, bool wks)
{
    this->nr_wp = wp_nr;
    this->workspace = wks;
    this->waypoints = vector<waypoint>(wps.size());
    std::copy(wps.begin(),wps.end(),this->waypoints.begin());
}

//copy constructor
Waypoint::Waypoint(const Waypoint &wp)
{
    this->nr_wp=wp.nr_wp;
    this->name=wp.name;
    this->workspace=wp.workspace;
    this->waypoints = vector<waypoint>(wp.waypoints.size());
    std::copy(wp.waypoints.begin(),wp.waypoints.end(),this->waypoints.begin());
}

Waypoint::~Waypoint()
{
}

string Waypoint::getName() const
{
    return this->name;
}
void Waypoint::setName(string name)
{
    this->name=name;
}

bool Waypoint::getWaypoints(vector <waypoint> &wps){

    if(!this->waypoints.empty())
    {
        wps = std::vector<waypoint>(this->waypoints.size());
        std::copy(this->waypoints.begin(),this->waypoints.end(),wps.begin());
        return true;
    }
    else
        return false;
}

//return the structure of waypoint
waypoint Waypoint::get_waypoint(int index) const{
    return this->waypoints.at(index);
}
void Waypoint::setWaypoint(vector <waypoint> wp){
    this->waypoints.push_back(wp.back());
}

void Waypoint::setWPworkspace(int wp_space){
    this->workspace=wp_space; // 1-OP 0-Joint space
}

bool Waypoint::getWPworkspace(bool &wp_wks){
    wp_wks=this->workspace;
}

int Waypoint::getWPnumber(){
    nr_wp=this->nr_wp;
    return nr_wp;
}

string Waypoint::getInfoLine_OP(waypoint wp)
{

    return  wp.name + COLUMN + SPACE +
            XposSTR + str(boost::format("%d") % wp.OperatSpace.position.Xpos) + MILLIMETERS + SEP +
            YposSTR + str(boost::format("%d") % wp.OperatSpace.position.Ypos) + MILLIMETERS + SEP+
            ZposSTR + str(boost::format("%d") % wp.OperatSpace.position.Zpos) + MILLIMETERS + SEP+
            RollSTR + str(boost::format("%d") % wp.OperatSpace.or_rpy.roll) + RAD + SEP+
            PitchSTR + str(boost::format("%d") % wp.OperatSpace.or_rpy.pitch) + RAD + SEP+
            YawSTR + str(boost::format("%d") % wp.OperatSpace.or_rpy.yaw) + RAD + SEP //+
            /*VelocitySTR + str(boost::format("%d") % wp.OperatSpace.velocity) + VELOCITY + SEP+
            AccelerationSTR + str(boost::format("%d") % wp.OperatSpace.accelaration) + ACCELERATION*/ ;
}
string Waypoint::getInfoLine_Joint(waypoint wp)
{
    string text;
    for (int i = 0 ; i< wp.JointSpace.PosJoints.size();i++)
    {
        text = text + str(boost::format("%d") % wp.JointSpace.PosJoints[i]) + SEP + SPACE  ;

    }
    return  wp.name + COLUMN + SPACE +
            "Joints" + COLUMN + SPACE + text + DEG ;
           // "Joints velocities" + str(boost::format("%d") % wp.JointSpace.velocity) + VELOCITY + SEP+
           // "Joints Accelerations" + str(boost::format("%d") % wp.JointSpace.accelaration) + ACCELERATION ;

}

Eigen::Matrix3d Waypoint::setRotMatrixQuat(orient_q quat){

    Eigen::Quaterniond q;
    q.x() = quat.X;
    q.y() = quat.Y;
    q.z() = quat.Z;
    q.w() = quat.W;

    Eigen::Matrix3d Rot= q.normalized().toRotationMatrix();

    return Rot;
}





} //motion_manager namespace
