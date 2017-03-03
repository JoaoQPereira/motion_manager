#include "../include/motion_manager/power_law_dialog.hpp"

namespace motion_manager {

PowerLawDialog::PowerLawDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::powerLawDialog)
{
    ui->setupUi(this);
}

PowerLawDialog::~PowerLawDialog()
{
    delete ui;
}



void PowerLawDialog::setupPlots(vector<vector<double> > &hand_position, vector<vector<vector<double> > > &timesteps)
{
    vector<vector<vector<vector<double>>>> hand_position_task;
    // time
    vector<double> time_task; QVector<double> tot_timesteps;
    vector<int> index;
    QVector<double> pos_x; QVector<double> pos_y; //QVector<double> pos_z;
    QVector<double> C; // Curvature of the task
    QVector<double> lnC; // Curvature of the task
    //QVector<double> R; // Curvature radius
    //QVector<double> lnR; // Curvature radius
    QVector<double> vel_tan; // Velocity of the task
    QVector<double> ln_vel_tan; // Velocity of the task

    double coeff_tot=0; // sum of the coefficient
    int n_coeff=0; // number of the non-zero coefficient

    for(size_t i=0; i<timesteps.size();++i){
        vector<vector<double>> tsteps_mov = timesteps.at(i);
        double time_init;
        if(time_task.empty()){
            time_init=0.0;
        }else{
            time_init=time_task.at(time_task.size()-1);
        }
        int offset;
        if(i==0){
            offset=0;
        }else{
            vector<vector<vector<double>>> h_mov = hand_position_task.at(i-1);
            for(size_t ii=0; ii<h_mov.size();ii++){
                vector<vector<double>> h_stage = h_mov.at(ii);
                offset += h_stage.size();
            }
        }
        vector<double> time_mov;
        vector<vector<vector<double>>> hand_position_mov;
        for(size_t j=0; j<tsteps_mov.size();++j){
            vector<double> tsteps_stage = tsteps_mov.at(j);
            vector<double> time_stage(tsteps_stage.size());
            time_stage.at(0) = time_init;
            vector<vector<double>> hand_position_stage;
            for(size_t k=0;k<tsteps_stage.size();++k){
                tot_timesteps.push_back(tsteps_stage.at(k));
                if(k>0){time_stage.at(k) = time_stage.at(k-1) + tsteps_stage.at(k-1);}
                hand_position_stage.push_back(hand_position.at(k+offset));
            }// stage
            index.push_back(tot_timesteps.size());
            time_mov.reserve(time_stage.size());
            std::copy (time_stage.begin(), time_stage.end(), std::back_inserter(time_mov));
            hand_position_mov.push_back(hand_position_stage);
        }// mov
        time_task.reserve(time_mov.size());
        std::copy (time_mov.begin(), time_mov.end(), std::back_inserter(time_task));
        hand_position_task.push_back(hand_position_mov);
    }// task
    QVector<double> qtime = QVector<double>::fromStdVector(time_task);


    QVector<double> lnY_tot; QVector<double> lnX_tot;
    vector<double> q_tot; vector<double> m_tot; vector<double> r_tot;
    vector<double> q_tot_w; vector<double> m_tot_w; vector<double> r_tot_w;
    vector<double> q_tot_2w; vector<double> m_tot_2w; vector<double> r_tot_2w;

    for(size_t i=0; i<hand_position_task.size();++i){
        vector<vector<vector<double>>> hand_position_mov = hand_position_task.at(i);
        QVector<double> pos_u; QVector<double> pos_v; QVector<double> pos_w;
        vector<vector<double>> tsteps_mov = timesteps.at(i);
        QVector<double> timesteps_mov;
        for(size_t j=0; j<hand_position_mov.size();++j){
            vector<vector<double>> hand_position_stage = hand_position_mov.at(j);
            vector<double> tsteps_stage = tsteps_mov.at(j);
            for(size_t h=0; h<hand_position_stage.size();++h){
                vector<double> hand_point = hand_position_stage.at(h);
                timesteps_mov.push_back(tsteps_stage.at(h));
                pos_x.push_back(hand_point.at(0)); // [mm]
                pos_y.push_back(hand_point.at(1)); // [mm]
                //pos_z.push_back(hand_point.at(2)/1000); // [m]

                pos_u.push_back(hand_point.at(0)/1000); // [m] u = x
                pos_v.push_back(hand_point.at(1)/1000); // [m] v = y
                pos_w.push_back(hand_point.at(2)/1000); // [m] w = z
            }
        } // mov
        // first derivatives
        QVector<double> der_pos_u_1; QVector<double> der_pos_v_1; QVector<double> der_pos_w_1;
        this->getDerivative(pos_u,timesteps_mov,der_pos_u_1);
        this->getDerivative(pos_v,timesteps_mov,der_pos_v_1);
        this->getDerivative(pos_w,timesteps_mov,der_pos_w_1);
        // second derivatives
        QVector<double> der_pos_u_2; QVector<double> der_pos_v_2; QVector<double> der_pos_w_2;
        this->getDerivative(der_pos_u_1,timesteps_mov,der_pos_u_2);
        this->getDerivative(der_pos_v_1,timesteps_mov,der_pos_v_2);
        this->getDerivative(der_pos_w_1,timesteps_mov,der_pos_w_2);
        // third derivatives
        QVector<double> der_pos_u_3; QVector<double> der_pos_v_3; QVector<double> der_pos_w_3;
        this->getDerivative(der_pos_u_2,timesteps_mov,der_pos_u_3);
        this->getDerivative(der_pos_v_2,timesteps_mov,der_pos_v_3);
        this->getDerivative(der_pos_w_2,timesteps_mov,der_pos_w_3);

        // --- Curvature and Tangential Velocity --- //
        QVector<double> C_mov; // Curvature of the movement
        QVector<double> lnC_mov; // Curvature of the movement
        QVector<double> R_mov; // Radius of the Curvature of the movement
        QVector<double> lnR_mov; // Radius of the Curvature of the movement
        QVector<double> vel_tan_mov; // Velocity of the movement
        QVector<double> ln_vel_tan_mov; // Velocity of the movement
        for(int i=0; i<der_pos_u_1.size();++i){
            Vector2d der_1(der_pos_u_1.at(i),der_pos_v_1.at(i));
            Vector2d der_2(der_pos_u_2.at(i),der_pos_v_2.at(i));
            double num = abs((der_1(0)*der_2(1))-(der_1(1)*der_2(0)));
            double den = pow(der_1.norm(),3);

            if(den==0)
                den=0.0001;
            C.push_back((double)num/den); // [m^⁻1]
            lnC.push_back(log(C.at(i)));
            C_mov.push_back((double)num/den); // [m^⁻1]
            lnC_mov.push_back(log(C_mov.at(i)));
            R_mov.push_back(((double)1)/C_mov.at(i)); // [m]
            lnR_mov.push_back(log(R_mov.at(i)));

            vel_tan.push_back(der_1.norm()); // [m/s]
            ln_vel_tan.push_back(log(vel_tan.at(i)));
            vel_tan_mov.push_back(der_1.norm()); // [m/s]
            ln_vel_tan_mov.push_back(log(vel_tan_mov.at(i)));
        }

        // angular velocity
        QVector<double> theta_mov; QVector<double> der_theta_mov;
        for(size_t i=0; i< pos_u.size(); ++i){
            theta_mov.push_back(std::atan2(pos_v.at(i),pos_u.at(i)));
        }
        this->getDerivative(theta_mov,timesteps_mov,der_theta_mov);

        QVector<double> lnX; // Radius of the Curvature
        QVector<double> lnY; // Tangential Velocity
        for(size_t i=0;  i<C_mov.size();++i){
            if(C_mov.at(i)>=0.0001){
                lnX.push_back(log(abs(C_mov.at(i))));
                lnY.push_back(ln_vel_tan_mov.at(i));
                //lnX.push_back(lnR_mov.at(i));
                //lnY.push_back(log(abs(der_theta_mov.at(i))));
            }
        }

        if(!lnX.empty()){
            // R-squared regression
            double q,m,r;
            this->linreg(lnX,lnY,&q,&m,&r);
            if(lnX.size()!=0)
                n_coeff++;
            double coeff = ((double)lnX.size())/C_mov.size();
            q_tot.push_back(q); m_tot.push_back(m); r_tot.push_back(r);
            q_tot_w.push_back(coeff*q); m_tot_w.push_back(coeff*m); r_tot_w.push_back(coeff*r);
            q_tot_2w.push_back(coeff*pow(q,2)); m_tot_2w.push_back(coeff*pow(m,2)); r_tot_2w.push_back(coeff*pow(r,2));

            for(int i=0; i<lnX.size();++i){
                lnX_tot.push_back(lnX.at(i));
                lnY_tot.push_back(lnY.at(i));
            }
            coeff_tot +=coeff;
        }
    }// task

    double norm;
    if(n_coeff==1){
        norm=1;
    }else{
        norm=((double)n_coeff)/(n_coeff-1);
    }

    // q
    double sum_q = std::accumulate(q_tot_w.begin(), q_tot_w.end(), 0.0);
    //double mean_q = ((double)sum_q) / q_tot.size();
    double mean_q = ((double)sum_q) / coeff_tot;
    double sq_sum_q = std::accumulate(q_tot_2w.begin(), q_tot_2w.end(), 0.0);
    double stdev_q = std::sqrt(norm*((((double)sq_sum_q) / coeff_tot) - pow(mean_q,2)));
    // m
    double sum_m = std::accumulate(m_tot_w.begin(), m_tot_w.end(), 0.0);
    //double mean_m = ((double)sum_m) / m_tot.size();
    double mean_m = ((double)sum_m) / coeff_tot;
    double sq_sum_m = std::accumulate(m_tot_2w.begin(), m_tot_2w.end(), 0.0);
    double stdev_m = std::sqrt(norm*((((double)sq_sum_m) / coeff_tot) - pow(mean_m,2)));
    // r
    double sum_r = std::accumulate(r_tot_w.begin(), r_tot_w.end(), 0.0);
    //double mean_r = ((double)sum_r) / r_tot.size();
    double mean_r = ((double)sum_r) / coeff_tot;
    double sq_sum_r = std::accumulate(r_tot_2w.begin(), r_tot_2w.end(), 0.0);
    double stdev_r = std::sqrt(norm*((((double)sq_sum_r) / coeff_tot) - pow(mean_r,2)));

    QVector<double> ln_vel_tot_fit; QVector<double> best_line;
    double m_best = ((double)-1)/3;
    for(int i=0; i < lnX_tot.size(); ++i){
        ln_vel_tot_fit.push_back(mean_m*lnX_tot.at(i)+mean_q);
        best_line.push_back(m_best*lnX_tot.at(i)+mean_q);
    }

    // plot hand position
    ui->plot_hand_pos->plotLayout()->clear();
    ui->plot_hand_pos->clearGraphs();
    ui->plot_hand_pos->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_hand_pos);
    wideAxisRect->setupFullAxesBox(true);
    QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_hand_pos);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_hand_pos->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    QString title("Hand position");
    ui->plot_hand_pos->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_hand_pos,title));
    ui->plot_hand_pos->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_hand_pos->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_hand_pos->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_hand_pos->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot_hand_pos->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->plot_hand_pos->graph(0)->setName(title);
    ui->plot_hand_pos->graph(0)->valueAxis()->setLabel("y [mm]");
    ui->plot_hand_pos->graph(0)->keyAxis()->setLabel("x [mm]");
    ui->plot_hand_pos->graph(0)->setData(pos_x, pos_y);
    ui->plot_hand_pos->graph(0)->valueAxis()->setRange(*std::min_element(pos_y.begin(), pos_y.end()),
                                                      *std::max_element(pos_y.begin(), pos_y.end()));
    ui->plot_hand_pos->graph(0)->rescaleAxes();
    ui->plot_hand_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_hand_pos->replot();

    // plot the curvature
    ui->plot_curvature->plotLayout()->clear();
    ui->plot_curvature->clearGraphs();
    ui->plot_curvature->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_curvature);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_curvature);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_curvature->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Curvature";
    ui->plot_curvature->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_curvature,title));
    ui->plot_curvature->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_curvature->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_curvature->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_curvature->graph(0)->setName(title);
    ui->plot_curvature->graph(0)->valueAxis()->setLabel("curvature [1/m]");
    ui->plot_curvature->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_curvature->graph(0)->setData(qtime, C);
    ui->plot_curvature->graph(0)->valueAxis()->setRange(*std::min_element(C.begin(), C.end()),
                                                       *std::max_element(C.begin(), C.end()));
    ui->plot_curvature->graph(0)->rescaleAxes();


    ui->plot_curvature->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_curvature->replot();

    // plot the velocity
    ui->plot_ang_vel->plotLayout()->clear();
    ui->plot_ang_vel->clearGraphs();
    ui->plot_ang_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_ang_vel);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_ang_vel);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_ang_vel->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Velocity";
    ui->plot_ang_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_ang_vel,title));
    ui->plot_ang_vel->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_ang_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_ang_vel->graph(0)->setPen(QPen(Qt::blue));
    ui->plot_ang_vel->graph(0)->setName(title);
    ui->plot_ang_vel->graph(0)->valueAxis()->setLabel("velocity [m/s]");
    ui->plot_ang_vel->graph(0)->keyAxis()->setLabel("time [s]");
    ui->plot_ang_vel->graph(0)->setData(qtime, vel_tan);

    ui->plot_ang_vel->graph(0)->valueAxis()->setRange(*std::min_element(vel_tan.begin(), vel_tan.end()),
                                                       *std::max_element(vel_tan.begin(), vel_tan.end()));
    ui->plot_ang_vel->graph(0)->rescaleAxes();


    ui->plot_ang_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_ang_vel->replot();

    // plot power law
    ui->plot_23->plotLayout()->clear();
    ui->plot_23->clearGraphs();
    ui->plot_23->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
    wideAxisRect = new QCPAxisRect(ui->plot_23);
    wideAxisRect->setupFullAxesBox(true);
    marginGroup = new QCPMarginGroup(ui->plot_23);
    wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    // move newly created axes on "axes" layer and grids on "grid" layer:
    for (QCPAxisRect *rect : ui->plot_23->axisRects())
    {
      for (QCPAxis *axis : rect->axes())
      {
        axis->setLayer("axes");
        axis->grid()->setLayer("grid");
      }
    }
    title = "Two-third power law";
    ui->plot_23->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_23,title));
    ui->plot_23->plotLayout()->addElement(1, 0, wideAxisRect);

    ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_23->graph(0)->setPen(QPen(Qt::black));
    ui->plot_23->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot_23->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->plot_23->graph(0)->setName("ln(V)/ln(C)");
    ui->plot_23->graph(0)->valueAxis()->setLabel("ln(V) [m/s]");
    ui->plot_23->graph(0)->keyAxis()->setLabel("ln(C) [1/m]");
    ui->plot_23->graph(0)->setData(lnX_tot, lnY_tot);
    ui->plot_23->graph(0)->valueAxis()->setRange(*std::min_element(lnY_tot.begin(), lnY_tot.end()),
                                                 *std::max_element(lnY_tot.begin(), lnY_tot.end()));


    ui->plot_23->graph(0)->rescaleAxes();

    ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_23->graph(1)->setPen(QPen(Qt::red));
    string mean_m_str =  boost::str(boost::format("%.2f") % (mean_m)); boost::replace_all(mean_m_str,",",".");
    string mean_r_str =  boost::str(boost::format("%.2f") % (mean_r)); boost::replace_all(mean_r_str,",",".");
    string stdev_m_str =  boost::str(boost::format("%.2f") % (stdev_m)); boost::replace_all(stdev_m_str,",",".");
    string stdev_r_str =  boost::str(boost::format("%.2f") % (stdev_r)); boost::replace_all(stdev_r_str,",",".");
    QString name = QString::fromStdString(string("slope=")+mean_m_str+string("(")+stdev_m_str+string(")")+string(" R^2=")+mean_r_str+string("(")+stdev_r_str+string(")"));
    ui->plot_23->graph(1)->setName(name);

    //ui->plot_23->graph(1)->setData(lnR_mean, ln_vel_fit);
    ui->plot_23->graph(1)->setData(lnX_tot, ln_vel_tot_fit);
    //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                     // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
    ui->plot_23->graph(1)->rescaleAxes();

    ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
    ui->plot_23->graph(2)->setPen(QPen(Qt::blue));
    ui->plot_23->graph(2)->setName(QString("best fit slope: ")+QString::number(m_best));

    //ui->plot_23->graph(2)->setData(lnR_mean, best_line);
    ui->plot_23->graph(2)->setData(lnX_tot, best_line);
    //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                     // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
    ui->plot_23->graph(2)->rescaleAxes();

    // legend
    QCPLegend *legend = new QCPLegend();
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    ui->plot_23->plotLayout()->addElement(2, 0, subLayout);
    subLayout->setMargins(QMargins(5, 0, 5, 5));
    subLayout->addElement(0, 0, legend);
    // set legend's row stretch factor very small so it ends up with minimum height:
    ui->plot_23->plotLayout()->setRowStretchFactor(2, 0.001);
    legend->setLayer("legend");
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    legend->setFont(legendFont);
    legend->addElement(0,0,new QCPPlottableLegendItem(legend,ui->plot_23->graph(0)));
    legend->addElement(0,1,new QCPPlottableLegendItem(legend,ui->plot_23->graph(1)));
    legend->addElement(0,2,new QCPPlottableLegendItem(legend,ui->plot_23->graph(2)));


    ui->plot_23->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot_23->replot();


/*


    // PCA of the hand position
    vector<vector<double>> hand_position_red;
    int pca_hand = this->doPCA(hand_position,hand_position_red);
    if (pca_hand!=-1){
        // PCA successful
        if((!hand_position_red.empty())&&(hand_position_red.at(0).size()==2)){
            // dimensionality of the hand position reduced to 2 (movement on a plane)
            QVector<double> pos_u; QVector<double> pos_v;
            for(size_t i=0;i<hand_position_red.size();++i){
                vector<double> row = hand_position_red.at(i);
                pos_u.push_back(row.at(0)); // [mm]
                pos_v.push_back(row.at(1)); // [mm]
            }

            // ecc ellipse
            double ecc = 0.85;  double pos_u_0 = -400;
            double v_max = *std::max_element(pos_v.begin(), pos_v.end());
            double v_min = *std::min_element(pos_v.begin(), pos_v.end());
            double pos_v_0 = (v_max -v_min)/2;
            double ar = v_max - pos_v_0; double br=ar*sqrt(1-pow(ecc,2));
            for(size_t i=0; i<pos_v.size();++i){
                pos_u.push_back(pos_u_0+br*sqrt(1-pow(((pos_v.at(i)-pos_v_0)/ar),2)));
            }


            // plot hand position
            ui->plot_hand_pos->plotLayout()->clear();
            ui->plot_hand_pos->clearGraphs();
            ui->plot_hand_pos->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            QCPAxisRect *wideAxisRect = new QCPAxisRect(ui->plot_hand_pos);
            wideAxisRect->setupFullAxesBox(true);
            QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->plot_hand_pos);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_hand_pos->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            QString title("Hand position");
            ui->plot_hand_pos->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_hand_pos,title));
            ui->plot_hand_pos->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_hand_pos->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_hand_pos->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_hand_pos->graph(0)->setLineStyle(QCPGraph::lsNone);
            ui->plot_hand_pos->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
            ui->plot_hand_pos->graph(0)->setName(title);
            ui->plot_hand_pos->graph(0)->valueAxis()->setLabel("u [mm]");
            ui->plot_hand_pos->graph(0)->keyAxis()->setLabel("v [mm]");
            ui->plot_hand_pos->graph(0)->setData(pos_v, pos_u);
            ui->plot_hand_pos->graph(0)->valueAxis()->setRange(*std::min_element(pos_u.begin(), pos_u.end()),
                                                              *std::max_element(pos_u.begin(), pos_u.end()));
            ui->plot_hand_pos->graph(0)->rescaleAxes();
            ui->plot_hand_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_hand_pos->replot();

            // first derivatives
            QVector<double> der_pos_u_1; QVector<double> der_pos_v_1;
            this->getDerivative(pos_u,tot_timesteps,der_pos_u_1);
            this->getDerivative(pos_v,tot_timesteps,der_pos_v_1);
            // second derivatives
            QVector<double> der_pos_u_2; QVector<double> der_pos_v_2;
            this->getDerivative(der_pos_u_1,tot_timesteps,der_pos_u_2);
            this->getDerivative(der_pos_v_1,tot_timesteps,der_pos_v_2);
            // third derivatives
            QVector<double> der_pos_u_3; QVector<double> der_pos_v_3;
            this->getDerivative(der_pos_u_2,tot_timesteps,der_pos_u_3);
            this->getDerivative(der_pos_v_2,tot_timesteps,der_pos_v_3);


            // --- Curvature and Tangential Velocity --- //
            QVector<double> C; // Curvature
            QVector<double> lnC; // Curvature
            QVector<double> R; // Curvature radius
            QVector<double> lnR; // Curvature radius
            QVector<double> vel_tan; // Velocity
            QVector<double> ln_vel_tan; // Velocity
            for(int i=0; i<der_pos_u_1.size();++i){
                Vector2d der_1(der_pos_u_1.at(i),der_pos_v_1.at(i));
                Vector2d der_2(der_pos_u_2.at(i),der_pos_v_2.at(i));
                double num = abs((der_1(0)*der_2(1))-(der_1(1)*der_2(0)));
                double den = pow(der_1.norm(),3);
                C.push_back((double)num/den); // [m^⁻1]
                lnC.push_back(log(C.at(i)));                
                R.push_back(((double)1)/C.at(i)); // [m]
                lnR.push_back(log(R.at(i)));
                vel_tan.push_back(((double)sqrt(pow(der_1(0),2)+pow(der_1(1),2)))/1000); // [m/s]
                ln_vel_tan.push_back(log(vel_tan.at(i)));
            }

            QVector<double> lnX; // Curvature^2
            QVector<double> lnY; // Velocity
            QVector<int> index_t(index.size()); int k=0; int h=0; int hh;
            for(size_t i=0;  i<C.size();++i){
                int mov_size = index.at(k);
                if(i>=mov_size){
                    hh = index_t.at(h);
                    h++;
                    index_t.replace(h,hh);
                    k++;
                }
                if(C.at(i)>=0.001){
                    lnX.push_back(log(pow(C.at(i),2)));
                    lnY.push_back(ln_vel_tan.at(i));
                    index_t.replace(h,index_t.at(h)+1);
                }
            }


            QVector<double> C;// curvature
            QVector<double> lnC;
            QVector<double> R;// curvature radius
            QVector<double> lnR; //QVector<double> lnHand_vel;
            double eexp = ((double)3)/2; double num; double den;
            for(size_t i=0; i<pos_u.size();++i){
                num = pow(pow(der_pos_u_1.at(i),2)+pow(der_pos_v_1.at(i),2),eexp);
                den = 1000*abs((der_pos_u_1.at(i)*der_pos_v_2.at(i))-(der_pos_v_1.at(i)*der_pos_u_2.at(i)));
                R.push_back(((double)num)/den); // [m]
                C.push_back(((double)1)/R.at(i)); // [m^-1]
            }



            // angular and tangential velocity
            QVector<double> theta; QVector<double> der_theta; QVector<double> ln_theta;
            for(size_t i=0; i< pos_u.size(); ++i){
                theta.push_back(std::atan2(pos_v.at(i),pos_u.at(i)));
            }
            this->getDerivative(theta,tot_timesteps,der_theta);

            for(size_t i=0; i<der_theta.size();++i){
                vel_tan.push_back(R.at(i)*der_theta.at(i)); // [m/s]
                //vel_tan.push_back(((double)sqrt(pow(der_pos_u_1.at(i),2)+pow(der_pos_v_1.at(i),2)))/1000);// [m/s]
                //if(abs(der_theta.at(i))>0.05){
                    //ln_theta.push_back(abs(der_theta.at(i)));
                    //lnC.push_back(log(C.at(i)));
                ln_vel_tan.push_back(abs(vel_tan.at(i)));
                    //lnR.push_back(log(R.at(i)));
                //}
            }



            QVector<double> lnY_mean; QVector<double> lnX_mean;
            QVector<double> C_mean; // Medium Curvature
            QVector<double> V_mean; // Medium Velocity
            for(size_t i=0; i<index_t.size();++i){
                if(i==0){
                    //C_mean.push_back((double)accumulate( C.begin(), C.begin()+index_t.at(i), 0.0)/index_t.at(i));
                    //V_mean.push_back((double)accumulate( vel_tan.begin(), vel_tan.begin()+index_t.at(i), 0.0)/index_t.at(i));
                    lnX_mean.push_back((double)accumulate( lnX.begin(), lnX.begin()+index_t.at(i), 0.0)/index_t.at(i));
                    lnY_mean.push_back((double)accumulate( lnY.begin(), lnY.begin()+index_t.at(i), 0.0)/index_t.at(i));
                }else{
                    //C_mean.push_back((double)accumulate( C.begin()+index_t.at(i-1), C.begin()+index_t.at(i), 0.0)/(index_t.at(i)-index_t.at(i-1)));
                    //V_mean.push_back((double)accumulate( vel_tan.begin()+index_t.at(i-1), vel_tan.begin()+index_t.at(i), 0.0)/(index_t.at(i)-index_t.at(i-1)));
                    lnX_mean.push_back((double)accumulate( lnX.begin()+index_t.at(i-1), lnX.begin()+index_t.at(i), 0.0)/(index_t.at(i)-index_t.at(i-1)));
                    lnY_mean.push_back((double)accumulate( lnY.begin()+index_t.at(i-1), lnY.begin()+index_t.at(i), 0.0)/(index_t.at(i)-index_t.at(i-1)));
                }
            }
            /*

            for(size_t i=0;  i<index_t.size();++i){
                lnX_mean.push_back(log(pow(C_mean.at(i),2)));
                lnY_mean.push_back(log(V_mean.at(i)));
            }





            // plot the curvature
            ui->plot_curvature->plotLayout()->clear();
            ui->plot_curvature->clearGraphs();
            ui->plot_curvature->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_curvature);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_curvature);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_curvature->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            title = "Curvature";
            ui->plot_curvature->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_curvature,title));
            ui->plot_curvature->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_curvature->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_curvature->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_curvature->graph(0)->setName(title);
            ui->plot_curvature->graph(0)->valueAxis()->setLabel("curvature [1/m]");
            ui->plot_curvature->graph(0)->keyAxis()->setLabel("time [s]");
            ui->plot_curvature->graph(0)->setData(qtime, C);
            ui->plot_curvature->graph(0)->valueAxis()->setRange(*std::min_element(C.begin(), C.end()),
                                                               *std::max_element(C.begin(), C.end()));
            ui->plot_curvature->graph(0)->rescaleAxes();


            ui->plot_curvature->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_curvature->replot();

            // plot the velocity
            ui->plot_ang_vel->plotLayout()->clear();
            ui->plot_ang_vel->clearGraphs();
            ui->plot_ang_vel->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_ang_vel);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_ang_vel);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_ang_vel->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            title = "Velocity";
            ui->plot_ang_vel->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_ang_vel,title));
            ui->plot_ang_vel->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_ang_vel->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_ang_vel->graph(0)->setPen(QPen(Qt::blue));
            ui->plot_ang_vel->graph(0)->setName(title);
            ui->plot_ang_vel->graph(0)->valueAxis()->setLabel("velocity [m/s]");
            ui->plot_ang_vel->graph(0)->keyAxis()->setLabel("time [s]");
            ui->plot_ang_vel->graph(0)->setData(qtime, vel_tan);

            ui->plot_ang_vel->graph(0)->valueAxis()->setRange(*std::min_element(vel_tan.begin(), vel_tan.end()),
                                                               *std::max_element(vel_tan.begin(), vel_tan.end()));
            ui->plot_ang_vel->graph(0)->rescaleAxes();


            ui->plot_ang_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_ang_vel->replot();

            // R-squared regression
            double q,m,r;

            //this->linreg(lnX,lnY,&q,&m,&r);
            this->linreg(lnX_mean,lnY_mean,&q,&m,&r);

            std::cout << " m = " << m << " q = " << q << " R^2 = " << r << endl;
            QVector<double> ln_vel_fit; QVector<double> best_line;
            double m_best = ((double)-1)/6;
            for(size_t i=0; i < lnX.size(); ++i){
                ln_vel_fit.push_back(m*lnX.at(i)+q);
                best_line.push_back(m_best*lnX.at(i)+q);
            }

            // plot power law
            ui->plot_23->plotLayout()->clear();
            ui->plot_23->clearGraphs();
            ui->plot_23->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
            wideAxisRect = new QCPAxisRect(ui->plot_23);
            wideAxisRect->setupFullAxesBox(true);
            marginGroup = new QCPMarginGroup(ui->plot_23);
            wideAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
            // move newly created axes on "axes" layer and grids on "grid" layer:
            for (QCPAxisRect *rect : ui->plot_23->axisRects())
            {
              for (QCPAxis *axis : rect->axes())
              {
                axis->setLayer("axes");
                axis->grid()->setLayer("grid");
              }
            }
            title = "Two-third power law";
            ui->plot_23->plotLayout()->addElement(0,0, new QCPPlotTitle(ui->plot_23,title));
            ui->plot_23->plotLayout()->addElement(1, 0, wideAxisRect);

            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(0)->setPen(QPen(Qt::black));
            ui->plot_23->graph(0)->setLineStyle(QCPGraph::lsNone);
            ui->plot_23->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
            ui->plot_23->graph(0)->setName("ln(V)/ln(C^2)");
            ui->plot_23->graph(0)->valueAxis()->setLabel("ln(V) [m/s]");
            ui->plot_23->graph(0)->keyAxis()->setLabel("ln(C^2) [m^-2]");
            //ui->plot_23->graph(0)->setData(lnX, lnY);
            //ui->plot_23->graph(0)->valueAxis()->setRange(*std::min_element(lnY.begin(), lnY.end()),
              //                                           *std::max_element(lnY.begin(), lnY.end()));
            ui->plot_23->graph(0)->setData(lnX_mean, lnY_mean);
            ui->plot_23->graph(0)->valueAxis()->setRange(*std::min_element(lnY_mean.begin(), lnY_mean.end()),
                                                         *std::max_element(lnY_mean.begin(), lnY_mean.end()));

            ui->plot_23->graph(0)->rescaleAxes();


            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(1)->setPen(QPen(Qt::red));
            string m_str =  boost::str(boost::format("%.2f") % (m)); boost::replace_all(m_str,",",".");
            string r_str =  boost::str(boost::format("%.2f") % (r)); boost::replace_all(r_str,",",".");
            QString name = QString::fromStdString(string("slope=")+m_str+string(" R^2=")+r_str);
            ui->plot_23->graph(1)->setName(name);

            //ui->plot_23->graph(1)->setData(lnR_mean, ln_vel_fit);
            ui->plot_23->graph(1)->setData(lnX, ln_vel_fit);
            //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                             // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
            ui->plot_23->graph(1)->rescaleAxes();

            ui->plot_23->addGraph(wideAxisRect->axis(QCPAxis::atBottom), wideAxisRect->axis(QCPAxis::atLeft));
            ui->plot_23->graph(2)->setPen(QPen(Qt::blue));
            ui->plot_23->graph(2)->setName(QString("best fit slope: ")+QString::number(m_best));

            //ui->plot_23->graph(2)->setData(lnR_mean, best_line);
            ui->plot_23->graph(2)->setData(lnX, best_line);
            //ui.plot_power_law->graph(0)->valueAxis()->setRange(*std::min_element(lnHand_vel.begin(), lnHand_vel.end()),
                                                             // *std::max_element(lnHand_vel.begin(), lnHand_vel.end()));
            ui->plot_23->graph(2)->rescaleAxes();

            // legend
            QCPLegend *legend = new QCPLegend();
            QCPLayoutGrid *subLayout = new QCPLayoutGrid;
            ui->plot_23->plotLayout()->addElement(2, 0, subLayout);
            subLayout->setMargins(QMargins(5, 0, 5, 5));
            subLayout->addElement(0, 0, legend);
            // set legend's row stretch factor very small so it ends up with minimum height:
            ui->plot_23->plotLayout()->setRowStretchFactor(2, 0.001);
            legend->setLayer("legend");
            QFont legendFont = font();  // start out with MainWindow's font..
            legendFont.setPointSize(9); // and make a bit smaller for legend
            legend->setFont(legendFont);
            legend->addElement(0,0,new QCPPlottableLegendItem(legend,ui->plot_23->graph(0)));
            legend->addElement(0,1,new QCPPlottableLegendItem(legend,ui->plot_23->graph(1)));
            legend->addElement(0,2,new QCPPlottableLegendItem(legend,ui->plot_23->graph(2)));


            ui->plot_23->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            ui->plot_23->replot();


        }else{
            ui->plot_hand_pos->plotLayout()->clear();
            ui->plot_hand_pos->clearGraphs();
            ui->plot_23->plotLayout()->clear();
            ui->plot_23->clearGraphs();
            ui->plot_ang_vel->plotLayout()->clear();
            ui->plot_ang_vel->clearGraphs();
            ui->plot_curvature->plotLayout()->clear();
            ui->plot_curvature->clearGraphs();
        }
    }
*/

}




int PowerLawDialog::doPCA(vector<vector<double> > &data, vector<vector<double> > &data_red)
{
    MatrixXd mat(data.size(),data.at(0).size());
    for (size_t i = 0; i < data.size(); ++i) {
        vector<double> row = data.at(i);
      for (size_t j = 0; j < row.size(); ++j) {
          mat(i,j)=row.at(j);
      }
    }
    // Principal component analisys
    Pca *pca = new Pca();
    int init_result = pca->Calculate(mat,true,true,true);
    if (0 != init_result) {
      //There is an error during PCA calculation!
      return -1;
    }
    vector<double> sd = pca->sd(),
                  prop_of_var = pca->prop_of_var(),
                  cum_prop = pca->cum_prop(),
                  scores = pca->scores();
    vector<unsigned int> el_cols = pca->eliminated_columns();
    double         kaiser = pca->kaiser(),
                  thresh95 = pca->thresh95(),
                  thresh99 = pca->thresh99();
    unsigned int
                  ncols = pca->ncols(),
                  nrows = pca->nrows();
    string method = pca->method();
    delete pca;

    // Save the result to text file
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/pca", &st) == -1) {
        mkdir("results/planning/pca", 0700);
    }
    QString path("results/planning/pca/");
    QString fname = path+QString("pca_out.txt");

    ofstream outfile(fname.toStdString());
    if (!outfile) {
      cerr << "Can't create output file!" << endl;
      return -1;
    }
    outfile << "Initial matrix: " << endl;
    for (unsigned int i = 0; i < mat.rows(); ++i) {
      for (unsigned int j = 0; j < mat.cols(); ++j) {
        outfile << setw(7) << mat(i,j) << " ";
      }
      outfile << endl;
    }
    if (0 != el_cols.size()) {
      outfile << "\nNumbers of eliminated columns (0 based):\n";
      copy(el_cols.begin(), el_cols.end(), std::ostream_iterator<unsigned int>(outfile, " "));
      outfile << "\n\nMatrix after the eliminating: " << endl;
      for (unsigned int i = 0; i < mat.rows(); ++i) {
        for (unsigned int j = 0; j < mat.cols(); ++j) {
          if ( std::find(el_cols.begin(), el_cols.end(), j) == el_cols.end() ) {
            outfile << setw(7) << mat(i,j) << " ";
          }
        }
        outfile << endl;
      }
    }
    outfile << "\n\n" << method << " method was used\n";
    outfile << "\n\nStandard deviation:\n";
    copy(sd.begin(), sd.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nProportion of variance:\n";
    copy(prop_of_var.begin(), prop_of_var.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nCumulative proportion:\n";
    copy(cum_prop.begin(), cum_prop.end(), std::ostream_iterator<float>(outfile, " "));
    outfile << "\n\nKaiser criterion: " << kaiser;
    outfile << "\n\n95% threshold criterion: " << thresh95 << endl;
    outfile << "\n\n99% threshold criterion: " << thresh99 << endl;

    outfile << "\n\nRotated data: " << endl;
    unsigned int row_lim = nrows,
                 col_lim = ncols;
    if (scores.size() != nrows * ncols) {
      row_lim = (nrows < ncols)? nrows : ncols,
      col_lim = (ncols < nrows)? ncols : nrows;
    }
    for (unsigned int i = 0; i < row_lim; ++i) {
      for (unsigned int j = 0; j < col_lim; ++j) {
        outfile << setw(13) << scores[j + col_lim*i];
      }
      outfile << endl;
    }

    // export data reduced in dimentionality
    data_red.clear();
    data_red.resize(data.size());
    data_red.at(0).resize(max(kaiser,thresh99));

    for (size_t i = 0; i < data.size(); ++i) {
      vector<double> data_row = data.at(i);
      vector<double> data_red_row;
      for (size_t j = 0; j < data_row.size(); ++j) {
          if(kaiser<=thresh99){
              if(cum_prop.at(j)<=0.99)
                  data_red_row.push_back(data_row.at(j));
          }else{
              if(sd.at(j)>=1.0)
                  data_red_row.push_back(data_row.at(j));
          }
      }
      data_red.at(i) = data_red_row;
    }

    outfile << "\n\nMatrix reduced: " << endl;
    for (size_t i = 0; i < data_red.size(); ++i) {
      vector<double> data_red_row = data_red.at(i);
      for (size_t j = 0; j < data_red_row.size(); ++j) {
          outfile << setw(7) << data_red_row.at(j) << " ";
      }
      outfile << endl;
    }

    outfile.close();
    return 0;
}



void PowerLawDialog::getDerivative(QVector<double> &function, QVector<double> &step_values, QVector<double> &derFunction)
{
    // Formula of the numarical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


       const double MIN_STEP_VALUE = 0.1;
       const double MIN_DER_VALUE = 0.001;

       int h = 1;
       int tnsample;
       double f0;
       double f1;
       double f2;
       double f3;
       double f4;
       double step_value;

       // 1st point
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       tnsample = 0;
       f0 = function.at(tnsample);
       f1 = function.at(tnsample+1);
       f2 = function.at(tnsample+2);
       f3 = function.at(tnsample+3);
       f4 = function.at(tnsample+4);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
            derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));
       }

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = function.at(tnsample-1);
       f1 = function.at(tnsample);
       f2 = function.at(tnsample+1);
       f3 = function.at(tnsample+2);
       f4 = function.at(tnsample+3);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));
       }

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< function.size() -2;++i){     // centered
           f0 = function.at(i-2);
           f1 = function.at(i-1);
           f2 = function.at(i);
           f3 = function.at(i+1);
           f4 = function.at(i+2);
           step_value = step_values.at(i);
           if(step_value==0){
               //step_value=MIN_STEP_VALUE;
               derFunction.push_back(MIN_DER_VALUE);
           }else{
               derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
           }
       }

       // 4th point
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       tnsample = function.size()-2;
       f0 = function.at(tnsample-3);
       f1 = function.at(tnsample-2);
       f2 = function.at(tnsample-1);
       f3 = function.at(tnsample);
       f4 = function.at(tnsample+1);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));
       }

       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = function.size()-1;
       f0 = function.at(tnsample-4);
       f1 = function.at(tnsample-3);
       f2 = function.at(tnsample-2);
       f3 = function.at(tnsample-1);
       f4 = function.at(tnsample);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));
       }

}

int PowerLawDialog::linreg(const QVector<double> &x, const QVector<double> &y, double *b, double *m, double *r)
{
    double mm,bb,rr;

    double   sumx = 0.0;                        /* sum of x                      */
    double   sumx2 = 0.0;                       /* sum of x^2                   */
    double   sumxy = 0.0;                       /* sum of x * y                  */
    double   sumy = 0.0;                        /* sum of y                      */
    //double   sumy2 = 0.0;                       /* sum of y^2                   */

    int n = x.size();

   for (int i=0; i<n ;++i)
    {
      sumx  += x.at(i);
      sumx2 += pow((x.at(i)),2);
      sumxy += x.at(i) * y.at(i);
      sumy  += y.at(i);
      //sumy2 += pow(y.at(i),2);
    }

   double   meanx = ((double)sumx)/n;                        /* mean of x                      */
   double   meanx2 = ((double)sumx2)/n;                       /* mean of x^2                   */
   double   meanxy = ((double)sumxy)/n;                       /* mean of x * y                  */
   double   meany = ((double)sumy)/n;                        /* mean of y                      */
   //double   meany2 = sumy2/n;                       /* mean of y^2                   */

   double denom = (pow(meanx,2) - meanx2);
   if (denom == 0) {
       // can't solve the problem.
       *m = 0;
       *b = 0;
       *r = 0;
       return 1;
   }

   mm = ((double)((meanx*meany) - meanxy)) / denom;
   bb = meany - mm*meanx;

   /* compute correlation coeff     */
   double se_y = 0.0; /* squared error in y or total variation in y */
   double se_line = 0.0; /* squared error of the fitted line */
   for(int i=0; i<n; ++i)
   {
       se_y += pow((y.at(i) - meany),2);
       se_line += pow((y.at(i)-(mm*x.at(i)+bb)),2);
   }
   rr = 1 - (((double)se_line)/se_y);

   // results
   *m = mm;
   *b = bb;
   *r = rr;

   return 0;
}

// Q_SLOTS

void PowerLawDialog::on_pushButton_save_clicked()
{
    struct stat st = {0};
    if (stat("results", &st) == -1) {
        mkdir("results", 0700);
    }
    if (stat("results/planning", &st) == -1) {
        mkdir("results/planning", 0700);
    }
    if (stat("results/planning/power_law_2D", &st) == -1) {
        mkdir("results/planning/power_law_2D", 0700);
    }
    QString path("results/planning/power_law_2D/");

    ui->plot_hand_pos->savePdf(path+QString("hand_pos.pdf"),true,0,0,QString(),QString("Hand position"));
    ui->plot_curvature->savePdf(path+QString("radius.pdf"),true,0,0,QString(),QString("Curvature radius"));
    ui->plot_ang_vel->savePdf(path+QString("vel_tan.pdf"),true,0,0,QString(),QString("Tangiental velocity"));
    ui->plot_23->savePdf(path+QString("power_law.pdf"),true,0,0,QString(),QString("Power law"));

    QString pdf_qstr; string pdf_str;
    QString svg_qstr; string svg_str;
    string cmdLine;

    pdf_qstr = path+QString("hand_pos.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("hand_pos.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("radius.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("radius.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("vel_tan.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("vel_tan.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());

    pdf_qstr = path+QString("power_law.pdf"); pdf_str = pdf_qstr.toStdString();
    svg_qstr = path+QString("power_law.svg"); svg_str = svg_qstr.toStdString();
    cmdLine = string("pdftocairo -svg ")+pdf_str+string(" ")+svg_str;
    system(cmdLine.c_str());


}




} // namespace motion_manager
