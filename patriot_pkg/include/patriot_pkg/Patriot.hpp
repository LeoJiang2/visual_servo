

#ifndef PATRIOT_HPP
#define PATRIOT_HPP

#include <string.h>
#include <usb.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include "protocol.h"
#include <vector>

// Class for communicating with a Polhemus Patriot sensor
class Patriot {
public:
  Patriot(void);  
  ~Patriot(void);
  std::vector<float> get_pose( int station_num);
  void set_hemisphere(int x, int y, int z);
  std::vector<int> get_hemisphere();
  int nstations;

	
private:
  //#int nstations;
  int x_hs, y_hs, z_hs;
  struct usb_device *dev;
  usb_dev_handle *handle;
  buffer_t buf;
  station_t *stations;
	
  int count_bits(uint16_t v);
  struct usb_device *find_device_by_id(uint16_t vendor, uint16_t product);
  int request_num_of_stations(buffer_t *b);
  void init_buffer(buffer_t *b);
  int patriot_write(usb_dev_handle *handle, char *buf, int size, int timeout);
  int patriot_init(usb_dev_handle *handle);
  int patriot_read(usb_dev_handle *handle, void *buf, int size, int timeout);
  void patriot_clear_input(usb_dev_handle *handle);
  void patriot_reset(usb_dev_handle *handle);
  int patriot_receive(usb_dev_handle *handle, buffer_t *b, void *buf, int size);
  int patriot_send(usb_dev_handle *handle, char *cmd);
};


#endif
