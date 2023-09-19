/**
 * @file c_driver_digimesh_parser.h
 * @brief For generating and parsing DigiMesh compliant messages.
 * @version 0.1
 * @date 2023-09-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef DIGIMESH_PARSER_H
#define DIGIMESH_PARSER_H

#include <stdint.h> 
#include <stdbool.h>

/**********************/
/* PUBLIC DEFINITIONS */
/**********************/
 
/**
 * @brief Bytes in a digi module serial number. The serial number is also the address of a digi module when sending a message.
 */
#define DIGIMESH_SERIAL_NUMBER_LENGTH 8

/**
 * @brief The maximum size of a byte array representing a message
 */
#define DIGIMESH_MAXIMUM_MESSAGE_SIZE 128

/**
 * @brief This is the maximum size of a payload for a digimesh frame using encryption.
*/
#define DIGIMESH_MAXIMUM_PAYLOAD_SIZE 65


/****************/
/* PUBLIC TYPES */
/****************/

/**
 * @brief Return status of a digi function execution
 */
typedef enum{
    DIGIMESH_OK,
    DIGIMESH_ERROR
}digimesh_status_t;

/**
 * @brief Type that enforces the correct size for the serial.
 */
typedef struct{
    uint8_t serial[DIGIMESH_SERIAL_NUMBER_LENGTH];
}digimesh_serial_t;

/**
 * @brief Holds state information about a digimodule.
 */
typedef struct digi_t digi_t;

/**
 * @brief For identifying what digi device field you want to set or get.
 */
typedef enum{
    DIGIMESH_AT_ID,
    DIGIMESH_AT_CH,
    DIGIMESH_AT_NI,
    DIGIMESH_AT_END
}digimesh_at_command_t;

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



/********************************/
/* PUBLIC FUNCTION DECLARATIONS */
/********************************/

/**
 * @brief Initialize the state of the digimesh parser driver.
 */
void digi_init(void);

/**
 * @brief Allows users to check if the digi module owned by the code module is initialized or not. A digi module is initialzed if it's
 * serial number isn't empty.
 * 
 * @return true - state is initialized
 * @return false - state is not initialized
 */
bool digi_is_initialized(void);

/**
 * @brief Populates an array with the digimesh serial number
 * 
 * @param [in] serial - pointer to a digi serial object for population.
 * 
 * @return digi_status_t 
 */
digimesh_status_t digi_get_serial(digimesh_serial_t * serial);

/**
 * @brief Stores the state information of a digi module
 * 
 * @param [in] serial - Digi serial number object used to give information to the digi module
 * 
 * @return digi_status_t
 */
digimesh_status_t digi_register(digimesh_serial_t * serial);


/**
 * @brief Create a byte array representing a DigiMesh protocol compliant message that can be used to set
 * the value of a configurable field on a Digi module.
 * 
 * @param [in] field            - Selects what field you want to set.
 * @param [in] value            - Sets the value that you want to set the field to.
 * @param [in] value_length     - The nummber of bytes in the value field
 * @param [out] message         - Is used to store the resulting message.
 * @return digi_status_t 
 */
digimesh_status_t digimesh_generate_at_command_frame(digimesh_at_command_t field, uint8_t * value,  uint8_t value_length, uint8_t * message);


/**
 * @fn uint8_t digimesh_get_frame_size(uint8_t*)
 * @brief Determines the total number of bytes in a frame. This includes start bytes and crc.
 *
 * @param frame - the digimesh frame whose length needs to be determined.
 * @return Total size of the frame in bytes.
 */
uint8_t digimesh_get_frame_size(uint8_t * frame);

/**
 * @brief Creates a bytes array representing a DigiMesh transmit request packet. 
 * 
 * @param [in] destination The address of the recipient digi module.
 * @param [in] payload A byte array of data to be sent in the frame.
 * @param [in] payload_length The number of bytes in the payload.
 * @param [out] generated_frame A byte array the frame is written to. 
 * @return digi_status_t 
 */
digimesh_status_t digimesh_generate_transmit_request_frame(uint8_t * destination, uint8_t * payload, uint8_t payload_length, uint8_t * generated_frame);

/**
 * @brief Take a byte array and then write out complete digimesh packets that were found in the byte array to another byte array.
 * NB: It is assumed that the input is not circular and will not overun and then wrap.
 *
 * @param [out] input           The array of unparsed bytes
 * @param [out] input_head      The number of bytes of bytes to parse in the input buffer
 * @param [out] input_tail      The position that was parsed up to in the input buffer.
 * @param [out] output          The array of complete digimesh packet bytes to be written to
 * @param [out] output_head     The number of bytes written to the output_buffer
 *
 * @return digimesh_status_t 
 */
digimesh_status_t digimesh_parse_bytes(uint8_t * input, uint16_t * input_head, uint16_t * input_tail, uint8_t * output, uint16_t * output_head);

/**
 * @brief Extracts the first digimesh packet from a byte array and stores it in a new array and removes it from
 * the original array.
 * 
 * @param [in] input The array that the digimesh packet is in.
 * @param [in] head The head position of the array.
 * @param [out] tail The tail position of the array.
 * @param [out] new_frame The array to store the digimesh packet in.
 * @return digimesh_status_t 
 */
digimesh_status_t digimesh_extract_first_digimesh_packet(uint8_t * input, uint16_t * head, uint16_t * tail, uint8_t * new_frame);

/**
 * @brief Determines the type of a digimesh frame
 *
 * @param [in] frame The frame to determine the type of
 * @return digmesh_frame_type_t
 */
digimesh_frame_type_t digimesh_get_frame_type(uint8_t * frame);

/**
 * @brief Extracts the payload from a receive packet frame
 *
 * @param [in] frame The frame to extract the payload from.
 * @param [out] payload Byte array to store the payload in.
 * @return uint8_t The length of the payload in bytes
 */
uint8_t digimesh_extract_payload_from_receive_frame(uint8_t * frame, uint8_t * payload);
#endif
