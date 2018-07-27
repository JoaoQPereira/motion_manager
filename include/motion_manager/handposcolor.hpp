#ifndef HANDPOSCOLOR_H
#define HANDPOSCOLOR_H

#include <qwt3d_color.h>
#include <qwt3d_surfaceplot.h>


using namespace Qwt3D;

namespace motion_manager
{

class HandPosColor: public StandardColor
{

public:
    /**
     * @brief HandPosColor, a constructor
     * @param pw
     * @param z_min
     */
    HandPosColor(SurfacePlot* pw, double z_min);

    /**
     * @brief operator ()
     * @param x
     * @param y
     * @param z
     * @return
     */
    RGBA operator()(double x,double y, double z) const;

private:
    double z_min_;
};

} // namespace motion_manager

#endif // HANDPOSCOLOR_H
