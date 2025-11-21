#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include <stdint.h>

enum { VENDOR_REQUEST_WEBUSB = 1, VENDOR_REQUEST_MICROSOFT = 2 };

extern const uint8_t desc_ms_os_20[];

// These are arbitrarily chosen.
#define VID 0x8f83
#define PID 0x2309

#define MAX_ENDPOINT0_SIZE 64
#define ENDPOINT_BULK_SIZE 64

#define MANUFACTURER "buddy.boards"
#define PRODUCT "laft"
#define SERIAL "000"

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

// Interfaces for the USB device descriptor
enum { ITF_NUM_VENDOR = 0, ITF_NUM_TOTAL };

#define BULK_IN_ENDPOINT_DIR 0x83
#define BULK_OUT_ENDPOINT_DIR 0x04

#endif /* USB_DESCRIPTORS_H_ */
