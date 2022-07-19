
#ifndef RVIZ_PLUGIN_KORBANLOG_H
#define RVIZ_PLUGIN_KORBANLOG_H


#include <ros/ros.h>
#include <QString>

#include <QTableView>
#include <QItemDelegate>
#include <QStandardItemModel>

#include <rviz/panel.h>
#include "ui_korbanLog.h"

namespace Ui{
    class korbanLog;
}

class korbanLogPanel: public rviz::Panel{
    Q_OBJECT

public:
    korbanLogPanel(QWidget* parent = 0);

public Q_SLOTS:

protected:
    Ui::korbanLog *ui_;

private:
    QStandardItemModel *model;

    void initUi();
    void initSubscriber();
    void addList(long double lat_, long double long_);
    // ros::Subscriber subs_diagnotsic, subs_gps_vel, subs_local_vel, subs_compass, subs_mavconn;
};

#endif //RVIZ_PLUGIN_KORBANLOG_H
