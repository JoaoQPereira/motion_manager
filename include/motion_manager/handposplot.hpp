#ifndef HANDPOSPLOT_H
#define HANDPOSPLOT_H

#include "handposfunction.hpp"
#include "handposcolor.hpp"


namespace motion_manager
{

class HandPosPlot : public SurfacePlot
{

public:
    /**
     * @brief HandPosPlot, a constructor
     * @param hand_pos
     */
    HandPosPlot(vector<vector<double>>& hand_pos);
};

} // namespace motion_manager

#endif // HANDPOSPLOT_H
