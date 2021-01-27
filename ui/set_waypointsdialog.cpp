#include "set_waypointsdialog.h"
#include "ui_set_waypointsdialog.h"

set_waypointsdialog::set_waypointsdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::set_waypointsdialog)
{
    ui->setupUi(this);
}

set_waypointsdialog::~set_waypointsdialog()
{
    delete ui;
}
