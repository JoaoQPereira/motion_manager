#ifndef HANDPOSFUNCTION_H
#define HANDPOSFUNCTION_H

#include <qwt3d_surfaceplot.h>
#include <qwt3d_function.h>
#include <qwt3d_plot.h>
#include <qwt3d_enrichment_std.h>


using namespace Qwt3D;
using namespace std;

namespace motion_manager
{

class HandPosFunction: public Function
{

public:
    /**
     * @brief HandPosFunction, a constructor
     * @param pw
     * @param hand_pos
     */
    HandPosFunction(SurfacePlot* pw, vector<vector<double>>& hand_pos);

    /**
     * @brief operator ()
     * @param x
     * @param y
     * @return
     */
    double operator()(double x, double y);

    /**
     * @brief create
     * @return
     */
    bool create();

    /**
     * @brief get the minimum of the x coordinate of the hand position
     * @return
     */
    double getX_min();

    /**
     * @brief get the maximum of the x coordinate of the hand position
     * @return
     */
    double getX_max();

    /**
     * @brief get the minimum of the y coordinate of the hand position
     * @return
     */
    double getY_min();

    /**
     * @brief get the maximum of the y coordinate of the hand position
     * @return
     */
    double getY_max();

    /**
     * @brief get the minimum of the z coordinate of the hand position
     * @return
     */
    double getZ_min();

    /**
     * @brief get the maximum of the z coordinate of the hand position
     * @return
     */
    double getZ_max();

private:
    vector<vector<double>> handPos; /**< hand position. 0=x,1=y,2=z*/
    double x_min;/**< minimum value of the x coordinate of the hand position */
    double x_max;/**< maximum value of the x coordinate of the hand position */
    double y_min;/**< minimum value of the y coordinate of the hand position */
    double y_max;/**< maximum value of the y coordinate of the hand position */
    double z_min;/**< minimum value of the z coordinate of the hand position */
    double z_max;/**< maximum value of the z coordinate of the hand position */
};

}// namespace motion_manager

#endif // HANDPOSFUNCTION_H
