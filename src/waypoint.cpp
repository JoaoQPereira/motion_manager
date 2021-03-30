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

Waypoint::Waypoint(int wp_nr, vector <waypoint> wps, bool wks, std::string wp_name, bool gripper_state)
{
    this->nr_wp = wp_nr;
    this->name=wp_name;
    this->workspace = wks;
    this->waypoints = vector<waypoint>(wps.size());
    std::copy(wps.begin(),wps.end(),this->waypoints.begin());
    this->gripper_state = gripper_state;

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
    this->gripper_state = wp.gripper_state;
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
// return the vacuum state to be actuated after the current waypoints traj
int Waypoint::get_vacuumState(){
    return this->gripper_state;
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


bool Waypoint::SaveWaypointsFile(vector<vector<vector<double>>> mov_wps , vector <QString> mov_name,  vector <int> mov_gripper_vacuum)
{
    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1)
    {
        mkdir("Models", 0700);
    }
    QString path("Models/");

    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save the waypoints"),
                                                    "/home/joao/ros_ws/devel/lib/motion_manager/Models",
                                                    "All Files (*.*);; Tol Files (*.txt)");
    //ofstream wp;
    QFile f(filename);
    if(f.open(QIODevice::WriteOnly))
    {

        // ---- write the waypoints ---- //
        //string filename = namefile + string(".txt");
        //ofstream wp;
        QTextStream wp(&f);

        //wp.open(("/home/joao/ros_ws/devel/lib/motion_manager/Models"+filename).toStdString());

        for(int i=0; i<mov_name.size();i++)
        {
            wp << "# Trajectory" << mov_name[i].toStdString().c_str() << endl;
            wp << "mov_name=" << mov_name[i].toStdString().c_str() << endl;
            for(int k=0;k<mov_wps[i].size();k++)
            {
                wp << "#waypoint"<< to_string(k+1).c_str()<< endl;
                wp << "wp=";
                for(int j=0;j<mov_wps[i][k].size();j++){
                    // print joints
                    if(j==mov_wps[i][k].size()-1)
                        wp << to_string(mov_wps[i][k][j]).c_str();
                    else
                        wp << to_string(mov_wps[i][k][j]).c_str() << ",";
                }
                wp << endl;
            }
            wp << "# Vacuum activation at the end of the movement"<< endl;
            wp << "vacuum=" << to_string(mov_gripper_vacuum[i]).c_str()<< endl;
            wp << "end_mov"<< endl;
        }

        //close the file
        //wp.close();
    }
    return true;
}

bool Waypoint::LoadWaypointsFile(QString filename,vector<vector<vector<double>>> &mov_wps , vector <QString> &mov_name,  vector <int> &mov_gripper_vacuum)
{


    QFile f(filename);

    if(f.open( QIODevice::ReadOnly))
    {
        QTextStream stream( &f );
        QString line;
        int num_mov=0; // number of movements saved
        int num_wp=0; // number of waypoints by movement
        string line_aux;
        vector <double> wps_joint; // 1 waypoint - varias juntas
        vector <vector<double>> wps; // varios waypoints
    // mov_wps - varias trajetorias, cada com varios waypoints
        while(!stream.atEnd())
        {
            line = f.readLine();
            if(line.at(0)!=QChar('#'))
            {

                QStringList fields = line.split("=");
               // if (QString::compare(fields.at(0),QString("num_mov"),Qt::CaseInsensitive)==0)
               //     {num_mov = fields.at(1).toInt();}

               if (QString::compare(fields.at(0),QString("vacuum"),Qt::CaseInsensitive)==0)
                  {mov_gripper_vacuum.push_back(fields.at(1).toInt());}
               else if (QString::compare(fields.at(0),QString("mov_name"),Qt::CaseInsensitive)==0)
                  {mov_name.push_back(fields.at(1));}
               else if (QString::compare(fields.at(0),QString("num_wp"),Qt::CaseInsensitive)==0)
                  {num_wp = fields.at(1).toInt();}

               else if (QString::compare(fields.at(0),QString("wp"),Qt::CaseInsensitive)==0)
               {
                  QStringList line_wp = fields.at(1).split(",");
                  for(int k=0;k<JOINTS_ARM;k++)
                  {
                     wps_joint.push_back(line_wp.at(k).toDouble());
                  }
                  wps.push_back(wps_joint);
                  wps_joint.clear();
               }
               else if (line=="end_mov\n")
               {
                   mov_wps.push_back(wps);
                   wps.clear();

               }
            }
        }
        f.close();
     }
}

} //motion_manager namespace
