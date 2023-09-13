#include "c_driver_digimesh_parser.h"

#include <string.h>

/***********************/
/* PRIVATE DEFINITIONS */
/***********************/

/**
 * @brief If a digi module id is all this value then it's not
 * initialized.
 */
#define EMPTY_SERIAL 0xFF

/*****************/
/* PRIVATE TYPES */
/*****************/

/**
 * @brief Structure that holds information about a given digimesh module
 * 
 * @param serial - the serial number of the digi module
 */
struct digi_t{
    uint8_t serial[DIGI_SERIAL_LENGTH];
};

/**
 * @brief Frame structure of a message that can be used to SET a field on a local digi device.
 */
typedef struct{
    uint8_t start_delimiter;    // Indicates the start of an API frame
    uint8_t length[2];          // Number of bytes between length and checksum
    uint8_t frame_type;         // The type of the message (local at command 0x08)
    uint8_t frame_id;           // For linking the current frame with a response. If 0 the device will not emmit a response frame.
    uint8_t at_command[2];      // 2 ascii characters representing the field you want to query/modify. E.g. "ID" or "CH"
    uint8_t value[2];           // The value you want to set the field to. Leave empty if you don't want to set it.
    uint8_t checksum;           // 0xFF minus the 8 bit sum of bytes from offset 3 to this byte (betwen length and checksum)
}digi_at_command_set_t;

/**
 * @brief Frame structure of a message that can be used to GET a field on a local digi device.
 */
typedef struct{
    uint8_t start_delimiter;    // Indicates the start of an API frame
    uint8_t length[2];          // Number of bytes between length and checksum
    uint8_t frame_type;         // The type of the message (local at command 0x08)
    uint8_t frame_id;           // For linking the current frame with a response. If 0 the device will not emmit a response frame.
    uint8_t at_command[2];      // 2 ascii characters representing the field you want to query/modify. E.g. "ID" or "CH"
    uint8_t checksum;           // 0xFF minus the 8 bit sum of bytes from offset 3 to this byte (betwen length and checksum)
}digi_at_command_get_t;

/**
 * @brief Identifies what type of frame you want to build.
 */
typedef enum{
    DIGI_FRAME_LOCAL_AT = 0x08,
    DIGI_FRAME_TRANSMIT_REQUEST = 0x10,
    DIGI_FRAME_END
}digi_frame_t;

/*********************/
/* PRIVATE VARIABLES */
/*********************/

// The local digimodule instance
digi_t digi = {0};

// List of ascii strings representing differenct fields. Can be
// indexed by digi_field_t.
char digi_field_strings[DIGI_FIELD_END][2] = 
{
    {'I','D'}  // The network ID of the digi module
};

/*********************************/
/* PRIVATE FUNCTION DECLARATIONS */
/*********************************/

/********************************/
/* PRIVATE FUNCTION DEFINITIONS */
/********************************/

/*******************************/
/* PUBLIC FUNCTION DEFINITIONS */
/*******************************/

void digi_init(void)
{
    memset(digi.serial, EMPTY_SERIAL, DIGI_SERIAL_LENGTH);

    return;   
}

bool digi_is_initialized()
{
    // Check what the value of the digi state is to see if it's empty.
    // It's deemed empty if its serial is all empty values.
    for(uint8_t idx = 0; idx < DIGI_SERIAL_LENGTH; idx++)
    {
        if(digi.serial[idx] == EMPTY_SERIAL){
            continue;
        }
        else
        {
            // If there's a value in the serial that's not empty then the digi object is initialized.
            return true;
        }
    }
    // All bytes in the digi serial were found to be empty so return false.
    return false;
}

digi_status_t digi_get_serial(digi_serial_t * serial)
{
    memcpy(serial->serial, digi.serial, DIGI_SERIAL_LENGTH);

    return DIGI_OK;
}

digi_status_t digi_register(digi_serial_t * serial)
{
    memcpy(digi.serial, &(serial->serial[0]), DIGI_SERIAL_LENGTH);

    return DIGI_OK;
}