#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "object.hpp"
#include "pose.hpp"


namespace motion_manager
{

typedef boost::shared_ptr<Object> objectPtr;
typedef boost::shared_ptr<Pose> posePtr;


class Movement
{

public:
    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     */
    Movement(int type, int arm);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param obj
     */
    Movement(int type, int arm, objectPtr obj);

    /**
     * @brief Movement, a constructor
     * @param type
     * @param arm
     * @param pose
     */
    Movement(int type, int arm, posePtr pose);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj,bool prec);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param obj_eng
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, bool prec);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param obj_eng
     * @param pose
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, objectPtr obj_eng, posePtr pose, bool prec);

    /**
     * @brief Movement
     * @param type
     * @param arm
     * @param obj
     * @param pose
     * @param prec
     */
    Movement(int type, int arm, objectPtr obj, posePtr pose,bool prec);

    /**
     * @brief Movement, a copy constructor
     * @param mov
     */
    Movement(const Movement& mov);

    /**
     * @brief ~Movement, a destructor
     */
    ~Movement();

    /**
    * @brief This method sets the type of the movement.
    *  @param t
    */
    void setType(int t);

    /**
     * @brief This method sets the type of grip related to the movement.
     * @param prec
     */
    void setGrip(bool prec);

    /**
     * @brief This method sets the object that is manipulated during the movement
     * @param obj
     */
    void setObject(objectPtr obj);

    /**
     * @brief This method sets the object that is manipulated before the movement starts
     * @param obj
     */
    void setObjectInit(objectPtr obj);

    /**
     * @brief This method sets the object that is involved in engaging/disengaging movements
     * @param obj_eng
     */
    void setObjectEng(objectPtr obj_eng);

    /**
     * @brief This method sets the arm/s in movement
     * @param a
     */
    void setArm(int a);

    /**
     * @brief This method gets the the status of the movement
     * @param exec
     */
    void setExecuted(bool exec);

    /**
     * @brief This method gets the code of the movement
     * @return
     */
    int getType();

    /**
     * @brief This method gets the description of the movement
     * @return
     */
    string getStrType();

    /**
     * @brief This method gets the type of the grip
     * @return
     */
    bool getGrip();

    /**
     * @brief This method gets the description of the grip
     * @return
     */
    string getGripStr();

    /**
     * @brief This method gets the object that is being manipulated during the movement
     * @return
     */
    objectPtr getObject();

    /**
     * @brief This method gets the object that is being manipulated during the movement
     * @return
     */
    posePtr getPose();

    /**
     * @brief This method gets the object that is being manipulated before the movement starts
     * @return
     */
    objectPtr getObjectInit();

    /**
     * @brief This method gets the object that is involved in engaging/disengaging movements
     * @return
     */
    objectPtr getObjectEng();

    /**
     * @brief This method gets information about the movement
     * @return
     */
    string getInfoLine();

    /**
     * @brief This method gets the arm/s in movement
     * @return
     */
    int getArm();

    /**
     * @brief This method gets the the status of the movement
     * @return
     */
    bool getExecuted();

private:
    int arm;  /**< 0 = both arms, 1 = right-arm, 2 = left-arm */
    int type; /** type of movement: 0 = reach-to-grasp, 1 = reaching, 2 = transport, 3 = engage, 4 = disengage, 5 = go-park */
    string strType; /**< type of the movement (string) */
    bool prec; /**< true for a precision grip, false otherwise*/
    string grip_str; /**< type of the grip (string) */
    objectPtr obj; /**< object being manipulated in the movement */
    objectPtr obj_init; /**< object being manipulated before the movement starts */
    objectPtr obj_eng; /**< object involved in engaging/disengaging movements */
    posePtr pose; /**< pose involved in transport/disengaging/reaching movements*/
    bool executed; /**< true if the movement has been executed, false otherwise */
};

} // namespace motion_manager

#endif // MOVEMENT_HPP
