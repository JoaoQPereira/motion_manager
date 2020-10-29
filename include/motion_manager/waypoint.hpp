#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "common.hpp"

using namespace std;

namespace motion_manager {


class Waypoint
{
public:
    /**
     * @brief Waypoint, a constructor
     */
    Waypoint();
    /**
     * @brief Waypoint
     * @param wpoint
     * @param workspace
     */
    Waypoint(vector <waypoint> wpoint,bool workspace);
    /**
     * @brief Waypoint
     * @param wpoint
     * @param workspace
     */
    Waypoint(int wp_nr, vector <waypoint> wpoint,bool workspace,std::string name);

    Waypoint(int wp_nr, vector <waypoint> wps, bool wks);

    /**
     * @brief waypoint, a copy constructor.
     * @param wp
     */
    Waypoint(const Waypoint &wayp);
    /**
     * @brief ~waypoint, a destructor.
     */
    ~Waypoint();

    /**
     * @brief getName
     * @return
     */
    string getName() const;

    void setName(string name);
    /**
     * @brief getWaypoint
     * @return
     */
    bool getWaypoints(vector <waypoint>& wp);
    /**
     * @brief get_single_waypoint
     * @param index
     * @return
     */
    waypoint get_waypoint(int index) const;

    /**
     * @brief setWaypoint
     * @param wpoint
     */
    void setWaypoint(vector <waypoint> wpoint);
    /**
     * @brief setWPworkspace
     * @param wp_space
     */
    void setWPworkspace(int wp_space);
    /**
     * @brief getWPworkspace
     * @param wp_wks
     * @return
     */
    bool getWPworkspace(bool &wp_wks);
    /**
     * @brief getWPnumber
     * @param nr_wp
     * @return
     */
    int getWPnumber();
    /**
     * @brief getInfoLine_OP
     * @param wp
     * @return
     */
    string getInfoLine_OP(waypoint wp);
    /**
     * @brief getInfoLine_Joint
     * @param wp
     * @return
     */
    string getInfoLine_Joint(waypoint wp);
    /**
     * @brief setRotMatrixQuat
     * @param quat
     * @return
     */
    Eigen::Matrix3d setRotMatrixQuat(orient_q quat);
protected:
    string name; /** name of which trajectory these waypoints correspond */
    vector <waypoint> waypoints; //  vector of waypoints
    bool workspace;
    int nr_wp; // number of waypoints
};

}// namespace motion_manager

#endif // WAYPOINT_H
