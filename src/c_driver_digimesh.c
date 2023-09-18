#include "c_driver_digimesh.h"

#include <string.h>
#include <stdio.h>

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
#define MAX_FIELD_SIZE 0xFF

/**
 * @brief The maximum length of the ascii representation of an at command type.
 */
#define AT_COMMAND_STRING_LEN 2

/**
 * @brief This is the maximum size of a payload for a digimesh frame using encryption.
*/
#define MAXIMUM_PAYLOAD_SIZE 65

/** @brief This is the start delimiter of every digimesh message*/
#define START_DELIMITER 0x7E

#define MAX_FRAME_SIZE 128

/*****************/
/* PRIVATE TYPES */
/*****************/

/**
 * @brief Structure that holds information about a given digimesh module
 * 
 * @param serial - the serial number of the digi module
 */
struct digi_t{
    uint8_t serial[DIGIMESH_SERIAL_NUMBER_LENGTH];
};

/**
 * @brief Identifies what type of frame you want to build.
 */
typedef enum{
    DIGIMESH_FRAME_TYPE_LOCAL_AT = 0x08,
    DIGIMESH_FRAME_TYPE_TRANSMIT_REQUEST = 0x10,
    DIGIMESH_FRAME_TYPE_LOCAL_AT_COMMAND_RESPONSE = 0x88,
    DIGIMESH_FRAME_TYPE_RECEIVE_PACKET = 0x90,
    DIGIMESH_FRAME_TYPE_END
}digimesh_frame_type_t;

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
    digimesh_frame_type_t frame_type;      
    uint8_t frame_id;               
    uint8_t at_command[2];         
    uint8_t value_length;           
    uint8_t value[MAX_FIELD_SIZE]; 
    uint8_t checksum;                
}digi_frame_at_command_t;

/**
 * @brief Frame structure of a message that can be used to send a payload to another digi module.
 * 
 * @param start_delimiter Indicates the start of an API frame
 * @param length Number of bytes between length and checksum
 * @param frame_type The type of the message (local at command 0x08)
 * @param frame_id For linking the current frame with a response. If 0 the device will not emmit a response frame.
 * @param address The address of the destination digi device.
 * @param reserved Unused but typically set to 0xFFFE
 * @param broadcast_radius This is the maximum number of used for broadcast transmission.
 * @param transmit_options Set to 0xC0 for DigiMesh transmissions.
 * @param payload_data Set to 0xC0 for DigiMesh transmissions.
 * @param payload_length  The number of bytes in the payload field.
 * @param checksum 0xFF minus the 8 bit sum of bytes from offset 3 to this byte (betwen length and checksum).
 */
typedef struct{
    uint8_t start_delimiter;         
    uint16_t length;               
    digimesh_frame_type_t frame_type;      
    uint8_t frame_id;               
    uint8_t address[DIGIMESH_SERIAL_NUMBER_LENGTH];         
    uint8_t reserved[2];           
    uint8_t broadcast_radius;
    uint8_t transmit_options;
    uint8_t payload_data[MAXIMUM_PAYLOAD_SIZE];
    uint8_t payload_length;
    uint8_t checksum;        
}digi_frame_transmit_request_t;


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
}digimesh_frame_get_field_t;


/*********************/
/* PRIVATE VARIABLES */
/*********************/

// The local digimodule instance
static digi_t digi = {0};

// List of ascii strings representing differenct fields. Can be
// indexed by digi_field_t.
static char digi_at_command_strings[DIGIMESH_AT_END][AT_COMMAND_STRING_LEN] =
{
    {'I','D'},      // The network identifiying number of the digi module
    {'C','H'},      // The network channel of the digi module
    {'N','I'}       // The name of the digi module
};

/*********************************/
/* PRIVATE FUNCTION DECLARATIONS */
/*********************************/
/**
 * @brief CRC calculator for digimesh message.
 * @param frame DigiMesh frame
 * @returns uint8_t the crc for the frame
*/
static uint8_t calculate_crc(uint8_t * frame);

/**
 * @brief Determine the crc of the message frame according to the digimesh protocol.
 * 
 * @param [out] frame The crc field of the frame is set.
 * 
 * @return status
*/
static digimesh_status_t calculate_crc_at_command(digi_frame_at_command_t * frame);

/**
 * @brief Calculates the checksum for a transmit request frame.
 * 
 * @param [in] frame The frame to perform the calculation on.
 * @return digimesh_status_t 
 */
static digimesh_status_t calculate_crc_transmit_request(digi_frame_transmit_request_t * frame);

/**
 * @brief Convert an at command frame object into a byte array.
 * 
 * @param [in] frame This is the frame to be converted to bytes.
 * @param [out] bytes This is the byte array that is written to.
 * @return digimesh_status_t 
 */
static digimesh_status_t generate_byte_array_from_frame_at_command(digi_frame_at_command_t * frame, uint8_t * bytes);

/**
 * @brief Convert a transmit request frame object into a byte array.
 * 
 * @param [in] frame This is the frame to be converted to bytes.
 * @param [out] bytes This is the byte array that is written to.
 * @return digimesh_status_t 
 */
static digimesh_status_t generate_byte_array_from_frame_transmit_request(digi_frame_transmit_request_t * frame, uint8_t * bytes);

/********************************/
/* PRIVATE FUNCTION DEFINITIONS */
/********************************/
static uint8_t calculate_crc(uint8_t * bytes)
{
    uint16_t length = (bytes[1] << 8 & 0xFF) | bytes[2];
    uint8_t crc = 0;

    for(uint8_t idx = 0; idx < length; idx++)
    {
        crc += bytes[3 + idx];
    }

    crc = 0xFF - crc;

    return crc;

}


static digimesh_status_t calculate_crc_at_command(digi_frame_at_command_t * frame)
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

    return DIGIMESH_OK;
}

static digimesh_status_t calculate_crc_transmit_request(digi_frame_transmit_request_t * frame)
{
    // 0xFF minus the 8-bit sum of bytes from offset 3 to this byte (between length and checksum).
    uint8_t crc = 0xFF;
    uint8_t sum = frame->frame_type;    
    sum += frame->frame_id;
    
    for(uint8_t idx = 0; idx < DIGIMESH_SERIAL_NUMBER_LENGTH; idx++)
    {
        sum += frame->address[idx];
    }

    sum += frame->reserved[0];
    sum += frame->reserved[1];
    sum += frame->broadcast_radius;
    sum += frame->transmit_options;


    for(uint8_t idx = 0; idx < frame->payload_length; idx++)
    {
        sum += frame->payload_data[idx];
    }

    crc -= sum;

    frame->checksum = crc;

    return DIGIMESH_OK;
}

static digimesh_status_t generate_byte_array_from_frame_at_command(digi_frame_at_command_t * frame, uint8_t * bytes)
{
    bytes[0] = frame->start_delimiter;                      // START DELIMITER
    bytes[1] = (frame->length >> 8) & 0xFF;                 // LENGTH: MSB
    bytes[2] = frame->length;                               // LENGTH: LSB
    bytes[3] = (frame->frame_type);                         // FRAME TYPE
    bytes[4] = (frame->frame_id);                           // FRAME ID
    memcpy(&bytes[5], frame->at_command,2);                 // AT COMMAND 
    memcpy(&bytes[7], frame->value, frame->value_length);   // PARAMETER VALUE
    bytes[7+frame->value_length] = frame->checksum;         // CRC
   
    return DIGIMESH_OK;
}

static digimesh_status_t generate_byte_array_from_frame_transmit_request(digi_frame_transmit_request_t * frame, uint8_t * bytes)
{
    bytes[0] = frame->start_delimiter;                                  // START DELIMITER
    bytes[1] = (frame->length >> 8) & 0xFF;                             // LENGTH: MSB
    bytes[2] = frame->length;                                           // LENGTH: LSB
    bytes[3] = (frame->frame_type);                                     // FRAME TYPE
    bytes[4] = (frame->frame_id);                                       // FRAME ID
    memcpy(&bytes[5], frame->address, DIGIMESH_SERIAL_NUMBER_LENGTH);   // ADDRESS 
    memcpy(&bytes[13], frame->reserved,2);                              // RESERVED
    bytes[15] = (frame->broadcast_radius);                              // BROADCAST RADIUS
    bytes[16] = (frame->transmit_options);                              // TRANSMIT OPTIONS
    memcpy(&bytes[17], &(frame->payload_data[0]), frame->payload_length);     // PAYLOAD
    bytes[17 + frame->payload_length] = frame->checksum;                // CHECKSUM
    
    return DIGIMESH_OK;
}

bool value_is_valid(digimesh_at_command_t field, uint8_t * value, uint8_t value_length)
{
    if(value_length > DIGIMESH_MAXIMUM_MESSAGE_SIZE)
    {
        return false;
    }

    switch(field)
    {
        case DIGIMESH_AT_ID:
            return !(value_length > 2);
        break;

        case DIGIMESH_AT_CH:
            return !(value_length > 1) && (value[0] <= 0x1A && value[0] >= 0x0B);
        break;

        case DIGIMESH_AT_NI:
            if (!(value_length > 20))
            {
                for(uint8_t idx = 0; idx < value_length; idx++)
                {
                    if(value[idx] < 0 || value[idx] > 127)
                    {
                        return false;
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        break;

        default:
            return false;
        break;
    }
}
/*******************************/
/* PUBLIC FUNCTION DEFINITIONS */
/*******************************/

void digi_init(void)
{
    memset(digi.serial, EMPTY_SERIAL, DIGIMESH_SERIAL_NUMBER_LENGTH);

    return;   
}

bool digi_is_initialized()
{
    // Check what the value of the digi state is to see if it's empty.
    // It's deemed empty if its serial is all empty values.
    for(uint8_t idx = 0; idx < DIGIMESH_SERIAL_NUMBER_LENGTH; idx++)
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

digimesh_status_t digi_get_serial(digimesh_serial_t * serial)
{
    memcpy(serial->serial, digi.serial, DIGIMESH_SERIAL_NUMBER_LENGTH);

    return DIGIMESH_OK;
}

digimesh_status_t digi_register(digimesh_serial_t * serial)
{
    memcpy(digi.serial, &(serial->serial[0]), DIGIMESH_SERIAL_NUMBER_LENGTH);

    return DIGIMESH_OK;
}

digimesh_status_t digimesh_generate_at_command_frame(digimesh_at_command_t field, uint8_t * value, uint8_t value_length, uint8_t * message)
{
    if(!value_is_valid(field, value, value_length))
    {
        return DIGIMESH_ERROR;
    }

    digi_frame_at_command_t frame = {
        .start_delimiter = 0x7E,
        .length = (1 + 1 + 2 + value_length),       // sizeof(frame_type) + sizeof(frame_id) + sizeof(at_command) + value_length
        .frame_type = DIGIMESH_FRAME_TYPE_LOCAL_AT,
        .frame_id = 0x01,
        .value_length = value_length
    };

    memcpy(frame.at_command, digi_at_command_strings[field], AT_COMMAND_STRING_LEN);
    memcpy(frame.value, value, value_length);

    calculate_crc_at_command(&frame);
    generate_byte_array_from_frame_at_command(&frame, message);

    return DIGIMESH_OK;
}

uint8_t digimesh_get_frame_size(uint8_t * frame)
{
  return ((frame[1] << 8 | frame[2]) + 4);
}

digimesh_status_t digimesh_generate_transmit_request_frame(digimesh_serial_t * destination, uint8_t * payload, uint8_t payload_length, uint8_t * generated_frame)
{
    if(payload_length > MAXIMUM_PAYLOAD_SIZE)
    {
        return DIGIMESH_ERROR;
    }

    digi_frame_transmit_request_t frame = {0};

    frame.start_delimiter = 0x7E;                                                   // START DELIM
    frame.length = (1 + 1 + 8 + 2 + 1 + 1 + payload_length);                        // LENGTH              frame_type(1) + frame_id(1) + address(8) + reserved(2) + broadcast_radius(1) + transmit_options(1)
    frame.frame_type =  DIGIMESH_FRAME_TYPE_TRANSMIT_REQUEST;                           // FRAME TYPE
    frame.frame_id = 0x01;                                                          // FRAME ID
    memcpy(frame.address, destination->serial, DIGIMESH_SERIAL_NUMBER_LENGTH);      // ADDRESS
    frame.reserved[0] = 0xFF;                                                       // RESERVED 0
    frame.reserved[1] = 0xFE;                                                       // RESERVED 1
    frame.broadcast_radius = 0x00;                                                  // BROADCAST RADIUS
    frame.transmit_options = 0xC0;                                                  // TRANSMIT OPTIONS
    frame.payload_length = payload_length;                                          // Payload length
    memcpy(frame.payload_data, payload, payload_length);                            // PAYLOAD DATA
    calculate_crc_transmit_request(&frame);                                         // CHECKSUM

    generate_byte_array_from_frame_transmit_request(&frame, generated_frame);

    return DIGIMESH_OK;
}

digimesh_status_t digimesh_parse_bytes(uint8_t * input_buffer, uint16_t input_buffer_size, uint8_t * output_buffer, uint16_t * written_bytes, uint16_t * input_tail)
{
    enum{
        START,
        LEN0,
        LEN1,
        BYTES,
        CRC
    };

    uint8_t state = START;

    uint8_t new_frame[MAX_FRAME_SIZE];
    uint8_t frame_count = 0;
    
    for(uint16_t idx = 0; idx < input_buffer_size; idx++)
    {
        // Check if the current byte is a start delimiter because if it is we must start the process again.
        if(input_buffer[idx] == START_DELIMITER)
        {
            // Increment the input tail to flush out the orphaned bytes.
            (*input_tail) += frame_count;
            state = START;
            frame_count = 0;
        }

        switch(state)
        {
            // Search for the next start delimiter
            case START:
                if(input_buffer[idx] == START_DELIMITER)
                {
                    // Add the start delimiter to the new frame
                    new_frame[frame_count] = input_buffer[idx];
                    // Count up as you add bytes to the frame
                    frame_count++;
                    // Switch to looking for the first byte of length data
                    state = LEN0;
                }
                // If you haven't found the start delimiter then keep searching but also increment the input buffer tail as any bytes
                // before a start delimiter are dead byte and need to be removed from the input buffer
                else
                {
                    (*input_tail)++;
                }
            break;

            // Get the length 0 byte and add it to the new frame
            case LEN0:
                new_frame[frame_count] = input_buffer[idx];
                frame_count++;
                
                // Get the second length byte for the frame
                state = LEN1;
            break;

            // Get the length 1 byte and add it to the new frame
            case LEN1:
                new_frame[frame_count] = input_buffer[idx];
                frame_count++;
                // Transition to getting the rest of the bytes for the frame
                state = BYTES;
            break;

            case BYTES:

                uint16_t payload_len = ((new_frame[1] << 8 & 0xFF) | new_frame[2]);
                // Add bytes until frame_count - 3 as this is where the CRC is and frame_length ends.
                if( (frame_count) < payload_len + 3)
                {
                    new_frame[frame_count] = input_buffer[idx];
                    frame_count++;
                }
                // Swap to working out the CRC as we've collected all of the other frame bytes
                else
                {
                    // Calculate the CRC from the new frame bytes
                    uint8_t crc = calculate_crc(new_frame);

                    // The very next byte in the input buffer should be the new frames CRC so check if they match
                    if(crc == input_buffer[idx])
                    {
                        new_frame[frame_count] = crc;
                        frame_count++;                    

                        // The crc matched so we have a whole digimesh packet which we now copy into the output buffer.
                        memcpy(output_buffer, new_frame, frame_count);
                        (*input_tail) += frame_count;
                        (*written_bytes) += frame_count;
                        
                        // Start again
                        state = START;
                        frame_count = 0;
                    }
                    else
                    {
                        // If the CRC doesn't match then these bytes are malformed and we need to get rid of them by advancing the
                        // input buffer's tail position to the bytes that we're up to
                        frame_count++;
                        (*input_tail) += frame_count;

                        // Reset the state and start looking for bytes again
                        state = START;
                        frame_count = 0;
                    }
                }
            break;

            default:
                return DIGIMESH_ERROR;
            break;
        }
    }

    return DIGIMESH_OK;
}
