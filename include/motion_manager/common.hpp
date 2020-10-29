#ifndef COMMON_HPP
#define COMMON_HPP
//#include <math.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/smart_ptr.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>
//#include <Eigen/Geometry>
/** configuration */
#include "config.hpp"

/** definitions for printing output */
#define SEP ", "
#define METERS " [m]"
#define MILLIMETERS " [mm]"
#define VELOCITY " [mm/s]"
#define ACCELERATION " [mm/s²]"
#define DEG " [deg]"
#define RAD " [rad]"
#define COLUMN ":"
#define SPACE " "
#define XposSTR "Xpos = "
#define YposSTR "Ypos = "
#define ZposSTR "Zpos = "
#define RollSTR "Roll = "
#define PitchSTR "Pitch = "
#define YawSTR "Yaw = "
#define XsizeSTR "Xsize = "
#define YsizeSTR "Ysize = "
#define ZsizeSTR "Zsize = "
#define VelocitySTR "Velocity = "
#define AccelerationSTR "Acceleration = "
//definition of the macro ASSERT
#ifndef DEBUG
#define ASSERT(x)
#else
#define ASSERT(x) \
    if (! (x)) \
    { \
        cout << "ERROR!! Assert " << #x << " failed\n"; \
        cout << " on line " << __LINE__  << "\n"; \
        cout << " in file " << __FILE__ << "\n";  \
    }
#endif

using namespace std;
using namespace Eigen;

namespace motion_manager
{
#if UR == 0
  const int JOINTS_ARM = 7; /**< number of joints per arm */
#elif UR == 1
  const int JOINTS_ARM = 6; /**< number of joints per arm */
#endif
#if HAND == 0
//*************************************************************************************************
//                                 BARRETT HAND
    const double THETA8_HOME = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double THETA8_FINAL = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const double TOL_GRIP = 0.0; /**< tolerance on the grip in [mm] */
    const int firstPartTorqueOvershootCountRequired = 1;/**< number of time that the torque applied to the first phalanx is bigger than firstPartMaxTorque */
    const double firstPartMaxTorque = 0.9f;/**< max torque that can be applied to the first phalanx of the fingers */
    const double closingOpeningTorque = 1.0f;/**< torque applied to the fingers when they are opening/closing */
    const double closingVel = 60.0f * static_cast<double>(M_PI) / 180.0f; /**< joint velocity of the fingers when they are closing */
    const double openingVel = -120.0f * static_cast<double>(M_PI) / 180.0f;/**< joint velocity of the fingers when they are opening */

    const int HAND_FINGERS = 3; /**< number of fingers per hand */
    const int JOINTS_HAND = 4; /**< number of joints per hand */
    const int N_PHALANGE = 3; /**< number of phalanges per finger */
#elif HAND == 1
  //*************************************************************************************************
  //                                 ELECTRIC PARALLEL GRIPPER
    const double TOL_GRIP = 0; /**< tolerance on the grip in [mm] */
    const double TOL_TOOL_GRIP = - 0.315; /**< tolerance on the grip tool in [mm] */
    const int JOINTS_HAND = 1; /**< number of joints per hand */
    const int HAND_FINGERS = 2; /**< number of fingers per hand */

    const int N_PHALANGE = 0; /**< number of phalanges per finger */
#elif HAND == 2
    const double TOL_GRIP = 0; /**< tolerance on the grip in [mm] */
    const double TOL_TOOL_GRIP = - 0.315; /**< tolerance on the grip tool in [mm] */

    const int JOINTS_HAND = 0; /**< number of joints per hand */
    const int HAND_FINGERS = 0; /**< number of fingers per hand */
    const int N_PHALANGE = 0; /**< number of phalanges per finger */

#endif

    /** this struct defines the position in the Cartesian space*/
    typedef struct
    {
        double Xpos; /**< position along the x axis in [mm] */
        double Ypos; /**< position along the y axis in [mm] */
        double Zpos; /**< position along the z axis in [mm] */
    } pos;

    /** this struct defines the orientation in Roll-Pitch-Yaw */
    typedef struct
    {
        double roll; /**< rotarion around the z axis in [rad] */
        double pitch; /**< rotarion around the y axis in [rad] */
        double yaw; /**< rotarion around the x axis in [rad] */
    } orient;

    /** this struct defines the dimention of an object */
    typedef struct
    {
        double Xsize; /**< size of the object along the x axis in [mm] */
        double Ysize; /**< size of the object along the y axis in [mm] */
        double Zsize; /**< size of the object along the z axis in [mm] */
    } dim;

    /** this struct defines the Denavit-Hartenberg kinematic parameters */
    typedef struct
    {
        vector<double> d; /**< distances between consecutive frames along the x axes in [mm] */
        vector<double> a; /**< distances between concecutive frames along the z axes in [mm] */
        vector<double> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
        vector<double> theta; /**< angle around the z axes between consecutive x axes in [rad] */
    } DHparams;

    /** this struct defines the arm */
    typedef struct
    {
        DHparams arm_specs; /**< the Denavit-Hartenberg parameters of the arm */
    } arm;

    /** this struct defines the barrett hand */
    typedef struct
    {
        double maxAperture; /**< [mm] max aperture of the hand in [mm] */
        double Aw; /**< smallest distance between F1 and F2 in [mm] */
        double A1; /**< length of the 1st part of the finger in [mm] */
        double A2; /**< length of the first phalax in [mm] */
        double A3; /**< length of the second phalax in [mm] */
        double D3; /**< depth of the fingertip in [mm] */
        double phi2; /**< angular displacement between the 1st part of the finger and the 1st phalax in [rad] */
        double phi3; /**< angular displacement between the 1st and the 2nd phalax in [rad] */
    } barrett_hand;

    /** this struct defines the electric gripper */
    typedef struct
    {
        double maxAperture; /**< [mm] max aperture of the hand in [mm] */
        double minAperture; /**< [mm] max aperture of the hand in [mm] */
        double A1; /**< length of the finger in [mm] */
        double D3; /**< depth of the fingertip in [mm] */
    } electric_gripper;

    /** this struct defines the vacuum gripper */
    typedef struct
    {
        double D7; /**< length of the vaccum gripper [mm] */
    } vacuum_gripper;

    /** this struct defines a generic part of a robot body */
    typedef struct
    {
        double Xpos; /**< position of the part along the x axis in [mm] */
        double Ypos; /**< position of the part along the y axis in [mm] */
        double Zpos; /**< position of the part along the z axis in [mm] */
        double Roll; /**< orientation of the part around the z axis in [rad] */
        double Pitch; /**< orientation of the part around the y axis in [rad] */
        double Yaw; /**< orientation of the part around the x axis in [rad] */
        double Xsize; /**< size of the part along the x axis in [mm] */
        double Ysize; /**< size of the part along the y axis in [mm] */
        double Zsize; /**< size of the part along the z axis in [mm] */
    } robot_part;

    /** this struct defines a generic part of a robot body with orientation in quaternions*/
    typedef struct
    {
        double Xpos; /**< position of the part along the x axis in [mm] */
        double Ypos; /**< position of the part along the y axis in [mm] */
        double Zpos; /**< position of the part along the z axis in [mm] */
        float q_X;  /** The X value of the vector component of the quaternion. */
        float q_Y;  /** The Y value of the vector component of the quaternion. */
        float q_Z;  /** The Z value of the vector component of the quaternion. */
        float q_W; /** The rotation component of the quaternion. */
        double Xsize; /**< size of the part along the x axis in [mm] */
        double Ysize; /**< size of the part along the y axis in [mm] */
        double Zsize; /**< size of the part along the z axis in [mm] */
    } robot_part_q;


    /** this struct defines the orientation in quaternions
     quaternion \f$ w+xi+yj+zk - four scalar coefficients  w, x, y and z. */
    typedef struct{
        float X;  /** The X value of the vector component of the quaternion. */
        float Y;  /** The Y value of the vector component of the quaternion. */
        float Z;  /** The Z value of the vector component of the quaternion. */
        float W; /** The rotation component of the quaternion. */
    } orient_q;


    /** this struct defines the  waypoint in Operational Space */
    typedef struct{
        pos position;         /** Position X,Y,Z */
        orient_q or_quat;  /** Orientation in Quaternions  */
        orient or_rpy;
        vector <double> velocity; /**< vector of angular velocity of the end effector in the waypoint   */
        vector <double> accelaration; /**< vector of angular accelaration of the end effector in the waypoint */
                                       /** velocity  and acceleration in x,y,z and wx,wy,wz   linear and angular velocity??*/
    } WpOperatSpace;

    /** this struct defines the  waypoint in Joint Space */
    typedef struct{
        vector <double> PosJoints; /** vector of Joints Positions in Joint Space*/
        vector <double> velocity; /**< vector of angular velocity of each joint in the waypoint   */
        vector <double> accelaration; /**< vector of angular accelaration of each joint in the waypoint */
    } WpJointSpace;

    /** this struct defines the waypoint in Joint Space and Operational Space*/
    typedef struct
    {   string name;
        WpJointSpace JointSpace;
        WpOperatSpace OperatSpace;
    } waypoint;

}// namespace motion_manager

#endif // COMMON_HPP
