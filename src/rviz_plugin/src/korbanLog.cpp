#include "korbanLog.h"

korbanLogPanel::korbanLogPanel(QWidget *parent):
    rviz::Panel(parent), ui_(new Ui::korbanLog()){

    ui_->setupUi(this);
    
    initUi();
    initSubscriber();
    
    addList(-7.286889, 112.796051);
    ui_->label_korban_ditemukan->setStyleSheet("background-color: green;\ncolor: white;");
    ui_->label_korban_ditemukan->setText("Korban Last Found at 14:30:22");
}

void korbanLogPanel::initUi(){
    model = new QStandardItemModel(1, 4, this);
    model->setHeaderData(0, Qt::Horizontal, QObject::tr("ID"));
    model->setHeaderData(1, Qt::Horizontal, QObject::tr("Latitude"));
    model->setHeaderData(2, Qt::Horizontal, QObject::tr("Longitude"));
    model->setHeaderData(3, Qt::Horizontal, QObject::tr("Time"));

    ui_->tableView_korban->setModel(model);
    ui_->tableView_korban->setEditTriggers(QAbstractItemView::NoEditTriggers);

}

void korbanLogPanel::initSubscriber(){

}

void korbanLogPanel::addList(long double lat_, long double long_){
    int id_ = 0;
    // Generate data
    for(int row = 0; row < 2; row++)
    {
        QModelIndex index = model->index(row,0,QModelIndex());
        model->setData(index, id_);

        index = model->index(row,1,QModelIndex());
        model->setData(index, QString(std::to_string(lat_).c_str()));

        index = model->index(row,2,QModelIndex());
        model->setData(index, QString(std::to_string(long_).c_str()));

        index = model->index(row,3,QModelIndex());
        model->setData(index, "14:30:22");

        std::cout << "ADDING ROW\n";

    }

    model->appendRow(new QStandardItem(4, 3));
    
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(korbanLogPanel,rviz::Panel )