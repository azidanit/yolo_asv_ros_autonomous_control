#include "pid_plugin.h"
#include "ui_pid.h"

PIDPanel::PIDPanel(QWidget* parent)
  : rviz::Panel(parent), ui(new Ui::Pid()){
    ui->setupUi(this);

    initPublisher();
    initConnection();
}

void PIDPanel::initPublisher(){
  pid_publisher = nh.advertise<rviz_plugin::Pid>("/pids", 1);
}

void PIDPanel::initConnection(){
  connectTab0();
  connectM1();
  connectM2();
  connectM3();
  connectM4();
  connectM5();
  connectM6();
  connectM7();
  


}

void PIDPanel::connectTab0(){
    // Tab 0
  connect(ui->spinBoxSpeedNav, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(0, 's', 0, value);
    }
  );
  connect(ui->spinBoxTrim, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(0, 't', 0, value);
    }
  );
  connect(ui->spinBoxPDistance, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(0, 'p', 0, value);
    }
  );
  connect(ui->spinBoxIDistance, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(0, 'i', 0, value);
    }
  );
  connect(ui->spinBoxDDistance, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(0, 'd', 0, value);
    }
  );
}

void PIDPanel::connectM1(){
  // Tab M.1
  connect(ui->spinBoxSpeedMisi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 's', 0, value);
    }
  );
  connect(ui->spinBoxSudutMisi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'a', 0, value);
    }
  );
  ////////////////
  connect(ui->spinBoxP1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 0, value);
    }
  );
  connect(ui->spinBoxI1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 0, value);
    }
  );
  connect(ui->spinBoxD1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 0, value);
    }
  );
  ////////////////
   connect(ui->spinBoxP2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 1, value);
    }
  );
  connect(ui->spinBoxI2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 1, value);
    }
  );
  connect(ui->spinBoxD2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 1, value);
    }
  );
  ////////////////
   connect(ui->spinBoxP3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 2, value);
    }
  );
  connect(ui->spinBoxI3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 2, value);
    }
  );
  connect(ui->spinBoxD3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 2, value);
    }
  );
}


void PIDPanel::connectM2(){
  
  // Tab M.1
  connect(ui->spinBoxSpeedMisi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 's', 0, value);
    }
  );
  connect(ui->spinBoxSudutMisi_2, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'a', 0, value);
    }
  );
  ////////////////
  connect(ui->spinBoxP1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 0, value);
    }
  );
  connect(ui->spinBoxI1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 0, value);
    }
  );
  connect(ui->spinBoxD1Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 0, value);
    }
  );
  ////////////////
  connect(ui->spinBoxP2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 1, value);
    }
  );
  connect(ui->spinBoxI2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 1, value);
    }
  );
  connect(ui->spinBoxD2Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 1, value);
    }
  );
  ////////////////
  connect(ui->spinBoxP3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'p', 2, value);
    }
  );
  connect(ui->spinBoxI3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'i', 2, value);
    }
  );
  connect(ui->spinBoxD3Misi_1, QOverload<int>::of(&QSpinBox::valueChanged),
    [=](int value){
      publish(1, 'd', 2, value);
    }
  );
}

void PIDPanel::connectM3(){
  
}

void PIDPanel::connectM4(){
  
}

void PIDPanel::connectM5(){
  
}

void PIDPanel::connectM6(){
  
}

void PIDPanel::connectM7(){
  
}

void PIDPanel::publish(const int misi_idx, const char var, const int index, const int value){
  msg.misi_idx = misi_idx;
  msg.var = var;
  msg.index = index;
  msg.value = value;

  pid_publisher.publish(msg);
}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PIDPanel,rviz::Panel )