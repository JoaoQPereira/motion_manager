#ifndef ENGAGEPOINT_HPP
#define ENGAGEPOINT_HPP

#include "point.hpp"


namespace motion_manager
{

class EngagePoint: public Point
{

public:
    /**
     * @brief EngagePoint, default constructor.
     */
    EngagePoint();

    /**
     * @brief EngagePoint, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    EngagePoint(string name, pos ppos, orient oor);

    /**
     * @brief EngagePoint, a copy constructor.
     * @param eng
     */
    EngagePoint(const EngagePoint& eng);

    /**
     * @brief ~EngagePoint, a destructor.
     */
    ~EngagePoint();
};

} // namespace motion_manager

#endif // ENGAGEPOINT_HPP
