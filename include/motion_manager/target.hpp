#ifndef TARGET_HPP
#define TARGET_HPP

#include "point.hpp"


namespace motion_manager
{

class Target:public Point
{
public:
    /**
     * @brief Target, default constructor.
     */
    Target();

    /**
     * @brief Target, a constructor.
     * @param name
     * @param ppos
     * @param oor
     */
    Target(string name, pos ppos, orient oor);

    /**
     * @brief Target, a copy constructor.
     * @param tar
     */
    Target(const Target& tar);

    /**
     * @brief ~Target, a destructor.
     */
    ~Target();
};

}// namespace motion_manager

#endif // TARGET_HPP
