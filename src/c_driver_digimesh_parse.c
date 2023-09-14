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

/**
 * @brief This is the maximum number of bytes that the value field of a
 * at command message can be.
 */
#define MAX_FIELD_SIZE 0x02

/**
 * @brief The maximum length of the ascii representation of an at command type.
 */
#define AT_COMMAND_STRING_LEN 2


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
 * @brief Identifies what type of frame you want to build.
 */
typedef enum{
    DIGI_FRAME_TYPE_LOCAL_AT = 0x08,
    DIGI_FRAME_TYPE_TRANSMIT_REQUEST = 0x10,
    DIGI_FRAME_TYPE_END
}digi_frame_type_t;

/**
 * @brief Frame structure of a message that can be used to SET a field on a local digi device.
 * 
 * @param start_delimiter Indicates the start of an API frame
 * @param length Number of bytes between length and checksum
 * @param frame_type The type of the message (local at command 0x08)
 * @param frame_id For linking the current frame with a response. If 0 the device will not emmit a response frame.
 * @param at_command 2 ascii characters representing the field you want to query/modify. E.g. "ID" or "CH"
 * @param value_length  The number of bytes in the value field. Has to be known as the number of bytes can change depending on the field being set.
 * @param value The value you want to set the field to. Leave empty if you don't want to set it.
 * @param checksum 0xFF minus the 8 bit sum of bytes from offset 3 to this byte (betwen length and checksum)
 */
typedef struct{
    uint8_t start_delimiter;         
    uint16_t length;               
    digi_frame_type_t frame_type;      
    uint8_t frame_id;               
    uint8_t at_command[2];         
    uint8_t value_length;           
    uint8_t value[MAX_FIELD_SIZE]; 
    uint8_t checksum;                
}digi_frame_at_command_t;


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
}digi_frame_get_field_t;


/*********************/
/* PRIVATE VARIABLES */
/*********************/

// The local digimodule instance
digi_t digi = {0};

// List of ascii strings representing differenct fields. Can be
// indexed by digi_field_t.
char digi_at_command_strings[DIGI_AT_END][AT_COMMAND_STRING_LEN] = 
{
    {'I','D'},      // The network identifiying number of the digi module
    {'C','H'}       // The network channel of the digi module
};

/*********************************/
/* PRIVATE FUNCTION DECLARATIONS */
/*********************************/
/**
 * @brief Determine the crc of the message frame according to the digimesh protocol.
 * 
 * @param [out] frame The crc field of the frame is set.
 * 
 * @return status
*/
digi_status_t calculate_crc(digi_frame_at_command_t * frame);

/********************************/
/* PRIVATE FUNCTION DEFINITIONS */
/********************************/
digi_status_t calculate_crc(digi_frame_at_command_t * frame)
{
    // 0xFF minus the 8-bit sum of bytes from offset 3 to this byte (between length and checksum).
    uint8_t crc = 0xFF;
    uint8_t sum = frame->frame_type;    
    sum += frame->frame_id;
    sum += frame->at_command[0];
    sum += frame->at_command[1];
    
    for(uint8_t idx = 0; idx < frame->value_length; idx++)
    {
        sum += frame->value[idx];
    }

    crc -= sum;

    frame->checksum = crc;

    return DIGI_OK;
}

    // memcpy(&bytes[1], &(frame->length), 2);                 // LENGTH
digi_status_t generate_byte_array_from_frame(digi_frame_at_command_t * frame, uint8_t * bytes)
{
    bytes[0] = frame->start_delimiter;                      // START DELIMITER
    bytes[1] = frame->length << 8;                          // LENGTH: MSB
    bytes[2] = frame->length;                               // LENGTH: LSB
    bytes[3] = (frame->frame_type);                         // FRAME TYPE
    bytes[4] = (frame->frame_id);                           // FRAME ID
    memcpy(&bytes[5], frame->at_command,2);                 // AT COMMAND 
    memcpy(&bytes[7], frame->value, frame->value_length);   // PARAMETER VALUE
    bytes[7+frame->value_length] = frame->checksum;         // CRC
   
    return DIGI_OK;
}
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

digi_status_t digi_generate_set_field_message(digi_at_command_t field, uint8_t * value, uint8_t value_length, uint8_t * message)
{
    digi_frame_at_command_t frame = {
        .start_delimiter = 0x7E,
        .length = (1 + 1 + 2 + value_length),       // sizeof(frame_type) + sizeof(frame_id) + sizeof(at_command) + value_length
        .frame_type = DIGI_FRAME_TYPE_LOCAL_AT,
        .frame_id = 0x01,
        .value_length = value_length
    };

    memcpy(frame.at_command, digi_at_command_strings[field], AT_COMMAND_STRING_LEN);
    memcpy(frame.value, value, value_length);

    calculate_crc(&frame);
    generate_byte_array_from_frame(&frame, message);
    

    return DIGI_OK;
}