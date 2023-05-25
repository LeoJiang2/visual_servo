
#include "patriot_pkg/Patriot.hpp"
#include "patriot_pkg/protocol.h"
#include <iostream>
//##include <usb.h>

/* Vendor 0x0f44 -> Polhemus
 * Product 0xef20 -> Patriot
 * This is found with the command 'lsusb' whihc returns something like:
 * 'Bus 001 Device 008: ID 0f44:ef20 Polhemus'
 */
#define VENDOR 0xf44
#define PRODUCT 0xef20

/* make control character out of ordinary character */
#define control(c) ((c) & 0x1f)

#define IN 0x88
#define OUT 0x4
#define INTERFACE 0
#define CONFIGURATION 1
#define TIMEOUT 1000
#ifdef DEBUG
#define warn(as...) { fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); fprintf(stderr, as); }
#endif

Patriot::Patriot(){
	usb_init();
	
  dev = find_device_by_id(VENDOR, PRODUCT);
  if (!dev) {
    fprintf(stderr, "Could not find the Polhemus Liberty device.\n");
    abort();
  }

  handle = usb_open(dev);
  if (!handle) {
    fprintf(stderr, "Could not get a handle to the Polhemus Liberty device.\n");
    abort();
  }

  if (!patriot_init(handle)) {
    fprintf(stderr, "Could not initialize the Polhemus Liberty device.\n");
    usb_close(handle);
    abort();
    //#return 1;
  }

  init_buffer(&buf);
  patriot_send(handle, (char *)"f1\r"); // activate binary mode

  nstations = request_num_of_stations(&buf);
  if(nstations == 0){ // Try again.  If not shutdown properly, a second try will often work
      nstations = request_num_of_stations(&buf);
  }
  fprintf(stderr, "Found %d stations.\n\n", nstations);
  if (nstations == 0) {    // If it still equals zero, shutdown
    usb_close(handle);
    abort();
  }
  
  /* Set the output type
   * O* - Apllies to all startions
   * 8 - Timestamp
   * 9 - Frame count
   * 3 - x,y,z Cartisian coords, extended precison
   * 7 - Orientation Quaternion
   */
  patriot_send(handle, (char *)"O*,8,9,3,7\r"); // station_t: quaternions
  x_hs = 0;
  y_hs = 0;
  z_hs = -1;
  set_hemisphere(x_hs, y_hs, z_hs);
  // switch output to centimeters
  patriot_send(handle, (char *)"u1\r");
  
  // Clear all the unwanted data from the line
  patriot_clear_input(handle);
  
  // Allocate space for the readings
  stations = new station_t [2];
  if (!stations)
    abort();
    
   /* enable continuous mode (get data points continously) */
  //#patriot_send(handle, (char *)"c\r");
  
}

Patriot::~Patriot(){
	delete[] stations;
	usb_close(handle);
    //#abort();
	}

/*
 * This counts the number of set bits in an integer
 *  (it is kind of hard to see how it works on first glance...)
 */
int Patriot::count_bits(uint16_t v) {

  int c;
  for (c = 0; v; c++)
    {
      v &= v - 1; // clear the least significant bit set
    }
    
  return c;
}


struct usb_device *Patriot::find_device_by_id(uint16_t vendor, uint16_t product) {
  struct usb_bus *bus;

  usb_find_busses();
  usb_find_devices();

  for (bus = usb_get_busses(); bus; bus = bus->next) {
    struct usb_device *dev;
    for (dev = bus->devices; dev; dev = dev->next) {
      if (dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product)
	return dev;
    }
  }
  return NULL;
}


int Patriot::request_num_of_stations(buffer_t *b) {
  static char cmd[] = { control('u'), '0', '\r', '\0' }; // Not sure why the second \0 is there.
  //#static char cmd[] = { control('u'), '0', '\r' };
  active_station_state_response_t resp;
  patriot_send(handle, cmd);
  patriot_receive(handle, b, &resp, sizeof(resp));
  if (resp.head.init_cmd == 21) { // Not sure why 21
    return count_bits(resp.detected & resp.active);
  }
  else {
    return 0;
  }
}

/* sets the zenith of the hemisphere in direction of vector (x, y, z) */
/*
 * Forward Hemisphere (+X) 1,0,0 
 * Back Hemisphere (-X) -1,0,0 
 * Right Hemisphere (+Y) 0,1,0 
 * Left Hemisphere (-Y) 0,-1,0 
 * Lower Hemisphere (+Z) 1,0,1
 * Upper Hemisphere (-Z) 0,0,-1
 */
void Patriot::set_hemisphere(int x, int y, int z) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "h*,%d,%d,%d\r", x, y, z);
  patriot_send(handle, cmd);
  x_hs = x;
  y_hs = y;
  z_hs = z;
}

std::vector<int> Patriot::get_hemisphere() {
	
	std::vector<int> hemisphere = {x_hs, y_hs, z_hs};
	
	return hemisphere;
}


void Patriot::init_buffer(buffer_t *b)
{
    b->fill = 0;
}


int Patriot::patriot_write(usb_dev_handle *handle, char *buf, int size, int timeout)
{
    return usb_bulk_write(handle, OUT, buf, size, timeout);
}

int Patriot::patriot_init(usb_dev_handle *handle)
{
    if (usb_set_configuration(handle, CONFIGURATION) != 0)
        //#warn("could not set usb configuration to %d\n", CONFIGURATION);
        std::cout << "could not set usb configuration to " << CONFIGURATION << std::endl;

    if (usb_claim_interface(handle, INTERFACE) != 0) {
        //#warn("could not claim usb interface %d\n", INTERFACE);
        std::cout << "could not claim usb interface  " << INTERFACE << std::endl;
        return 0;
    }

    /*
    static char magic[] = { '*', '*', 0xff, 0x16, 0, 0, 0, 0 };
    if (patriot_write(handle, magic, sizeof(magic), 0) != sizeof(magic)) {
        warn("usb bulk write failed\n");
        return 0;
    }*/
    patriot_reset(handle);
    return 1;
}

int Patriot::patriot_send(usb_dev_handle *handle, char *cmd)
{
    int size = strlen(cmd);
    if (patriot_write(handle, cmd, size, TIMEOUT) != size) {
        //#warn("sending cmd `%s' to device failed\n");
        std::cout << "sending cmd " << cmd << " to device failed." << std::endl;
        return 0;
    }
    return 1;
}

int Patriot::patriot_read(usb_dev_handle *handle, void *buf, int size, int timeout)
{
    return usb_bulk_read(handle, IN, (char*)buf, size, timeout);
}

void Patriot::patriot_clear_input(usb_dev_handle *handle)
{
    static char buf[1024];

    while( patriot_read(handle, buf, sizeof(buf), TIMEOUT) > 0); 
}


int Patriot::patriot_receive(usb_dev_handle *handle, buffer_t *b, void *buf, int size)
{
    //#printf("patriot_receive: %u\n",size);
    while (1) {

        while (b->fill < size) {
            int n_read = patriot_read(handle, b->buf + b->fill,
                                  sizeof(b->buf) - b->fill, TIMEOUT);
            //#warn("read %d\n", n_read);
            if (n_read < 0) {
                //#warn("error while reading from device (%d)", n_read);
                std::cout << "error while reading from device " << n_read << std::endl;
                return 0;
            }
            b->fill += n_read;
        }

        #ifdef DEBUG
        fprintf(stderr, "- %c%c\n", b->buf[0], b->buf[1]);
        #endif

        if (b->buf[0] == 'P' && b->buf[1] == 'A') { // P and A are specific to the Patriot
            memcpy(buf, b->buf, size);
            memmove(b->buf, b->buf + size, b->fill - size);
            b->fill -= size;
            return 1;
        } else {
            //#warn("got corrupted data\n");
            std::cout << "got corrupted data" << std::endl;
            b->fill = 0;
        }
    }
}

/** this resets previous `c' commands and puts the device in binary mode
 *
 *  beware: the device can be misconfigured in other ways too, though this will
 *  usually work
 */
void Patriot::patriot_reset(usb_dev_handle *handle)
{
    // reset c, this may produce "invalid command" answers
    patriot_send(handle, (char *)"\rp\r");
    // remove everything from input
    patriot_clear_input(handle);
}


std::vector<float> Patriot::get_pose( int station_num){
	patriot_send(handle, (char *)"p");
	
	if (!patriot_receive(handle, &buf, stations, sizeof(station_t) * nstations)) {
      fprintf(stderr, "Receive failed.\n");
      //#return NULL;
	}
	
	float qw = stations[station_num].quaternion[0];
	float qx = stations[station_num].quaternion[1];
	float qy = stations[station_num].quaternion[2];
	float qz = stations[station_num].quaternion[3];
	
	std::vector<float> pose = {stations[station_num].x, stations[station_num].y, stations[station_num].z, qx,qy,qz,qw};

	return pose;
}



