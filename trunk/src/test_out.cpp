#include "dynamixelexceptions.h"
#include "dynamixel_motor.h"
#include <math.h>
#include <time.h>
#include "eventexceptions.h"
#include "dynamixelserver_ftdi.h"
#include <iostream>

int main(int argc, char *argv[])
{
  std::cout << "Start: " << std::endl;
  CDynamixelServerFTDI *dyn_server=CDynamixelServerFTDI::instance();
  CEventServer *event_server=CEventServer::instance();
  int num_buses=0,baudrate=0,event_id;
  std::list<std::string> events;
  std::vector<int> devices;
  unsigned short int model;
  CDynamixel *dyn_device;
  unsigned int i,num_dev;
  //___________________________________________________________________
  CDynamixelMotor *cont = NULL;
  int device = 6;
  std::string cont_name="";


  num_buses=dyn_server->get_num_buses();
  std::cout << "Num. buses: " << num_buses << std::endl;
  for(int b=0; b<num_buses; b++)
  {
    if(num_buses>0)
    {
      try{
      events.push_back(dyn_server->get_scan_done_event_id());
      events.push_back(dyn_server->get_scan_error_event_id());
      dyn_server->config_bus(0,1000000);
      std::string serial = dyn_server->get_bus_serial();
      std::cout << "For bus with id " << b << ", serial "<< serial << std::endl;
      dyn_server->start_scan();
      event_id=event_server->wait_first(events);
      if(event_id==0)
      {
        num_dev=dyn_server->get_num_devices();
        std::cout << "Num. devices: " << num_dev << std::endl;
        baudrate=dyn_server->get_baudrate();
        std::cout << "Baudrate: " << baudrate << " bps" << std::endl;
        devices=dyn_server->get_device_ids();
        for(i=0;i<num_dev;i++)
        {
          dyn_device=dyn_server->get_device(devices[i]);
          dyn_device->read_word_register(0x00,&model);
          std::cout << "servo " << devices[i] << " model " << (int)model << std::endl;
          dyn_server->free_device(devices[i]);
          delete dyn_device;
        }
      }
      else
        std::cout << "Error while scanning the bus: " << dyn_server->get_scan_error() << std::endl;
      try{
        dyn_server->config_bus(0,1000000);
        dyn_server->start_scan();
        event_server->wait_first(events,2000);
      }catch(CException &e){
        dyn_server->stop_scan();
        std::cout << "Scanning Finished" << std::endl;
      }
      }
      catch(CException &e){
        std::cout << e.what() << std::endl;
      }
    }
  }
  /////////////////////////////////////////////////////////////////////////
  /////MOVE RELATIVE ANGLE
  /////////////////////////////////////////////////////////////////////////
  double desired_speed   = 100.0; //chosen speed when moving angle
  double max_angle_error =   0.5; //max angle error permitted
  double time_interval   =   0.1; //time in secs between checks
  int max_time_sec       =   10.0; //max time to wait until timeout

  double  desired_angle;
  int t;
  double uperiod = time_interval*1000000.0;
  double timeout = max_time_sec/(uperiod/1000000.0);


  for(i=0;i<num_dev;i++)
  {
    std::cout << "-----------------------------------------------------------" <<  std::endl;
    double absolute_angle=100.0;
    double current_abs_angle;
    std::cout << "MOVE ABSOLUTE ANGLE: " << absolute_angle << "\tservo: "<< devices[i] << std::endl;
    cont = new CDynamixelMotor(cont_name, dyn_server, devices[i]);

    current_abs_angle = cont->get_current_angle();
    desired_angle=absolute_angle;
    std::cout << "Desired angle: " << desired_angle << std::endl;
    std::cout << "Current angle: " << current_abs_angle << std::endl;

    cont->move_absolute_angle(absolute_angle, desired_speed);
    std::cout << "Moving..." << std::endl;

    t=0;
    while(fabs(current_abs_angle)>max_angle_error && t<timeout)
    {
      current_abs_angle = cont->get_current_angle();
      usleep(uperiod);
      t++;
    }

    if(t==timeout)
      std::cout << "Reached " << max_time_sec << "sec timeout"<< std::endl;

    std::cout << "Desired angle: " << desired_angle << std::endl;
    std::cout << "Reached angle: " << current_abs_angle << std::endl;
    std::cout << "Error angle: " << current_abs_angle-desired_angle << std::endl;
    std::cout << "Done" << std::endl;
    sleep(1);
  }
}
