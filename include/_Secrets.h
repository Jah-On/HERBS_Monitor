/******************************************************************************

Rename this file to Secrets.h after modifying!

******************************************************************************/

// HERBS Data Types
#include <include/Herbs.h>

#define MONITOR_ID    0x0000000000000000

typedef struct MonitorEncryption {
  uint8_t key[17];
  uint8_t iv[9];
} MonitorEncryption;

const MonitorEncryption encryption = {
  "0000000000000000", 
  "00000000"
};