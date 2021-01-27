#ifndef SET_WAYPOINTSDIALOG_H
#define SET_WAYPOINTSDIALOG_H

#include <QDialog>

namespace Ui {
class set_waypointsdialog;
}

class set_waypointsdialog : public QDialog
{
    Q_OBJECT

public:
    explicit set_waypointsdialog(QWidget *parent = 0);
    ~set_waypointsdialog();

private:
    Ui::set_waypointsdialog *ui;
};

#endif // SET_WAYPOINTSDIALOG_H
