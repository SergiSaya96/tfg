#include "eventexceptions.h"
#include "dynamixelserver_ftdi.h"
#include <iostream>

int main(int argc, char *argv[])
{
  CDynamixelServerFTDI *dyn_server=CDynamixelServerFTDI::instance();
  CEventServer *event_server=CEventServer::instance();
  int num_buses=0,baudrate=0,event_id;
  std::list<std::string> events;
  std::vector<int> devices;
  unsigned short int model;
  CDynamixel *dyn_device;
  unsigned int i,num_dev;

  num_buses=dyn_server->get_num_buses();
  std::cout << "Num. buses: " << num_buses << std::endl;
  if(num_buses>0)
  {
    try{
    events.push_back(dyn_server->get_scan_done_event_id());
    events.push_back(dyn_server->get_scan_error_event_id());
    dyn_server->config_bus(0,1000000);
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
      std::cout << "Scanning canceled !!!" << std::endl;
    }
    }
    catch(CException &e){
      std::cout << e.what() << std::endl;
    }
  }
}
