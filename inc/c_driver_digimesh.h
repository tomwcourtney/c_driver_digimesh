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
#define DIGI_SERIAL_LEN 8

/**
 * @brief The maximum size of a byte array representing a message
 */
#define DIGIMESH_MAXIMUM_FRAME_SIZE 128

/**
 * @brief This is the maximum size of a payload for a digimesh frame using encryption.
*/
#define DIGIMESH_MAX_PAYLOAD_SIZE 65

/**
 * @brief Position of frame id in transmit request and local at command.
 */
#define DIGIMESH_POS_FRAME_ID 4

/**
 * @def DIGIMESH_POS_TRANSMIT_STATUS_STATUS
 * @brief Position of the transmit status in a transmit status message.
 */
#define DIGIMESH_POS_EXTENDED_TRANSMIT_STATUS_STATUS 8

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
    uint8_t serial[DIGI_SERIAL_LEN];
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
    DIGIMESH_AT_SM,
    DIGIMESH_AT_SN,
    DIGIMESH_AT_SO,
    DIGIMESH_AT_ST,
    DIGIMESH_AT_SP,
    DIGIMESH_AT_WH,
    DIGIMESH_AT_SH,
    DIGIMESH_AT_SL,
    DIGIMESH_AT_WR,
    DIGIMESH_AT_END
}digimesh_at_command_t;

typedef enum
{
  DIGIMESH_AT_STATUS_OK,
  DIGIMESH_AT_STATUS_ERROR,
  DIGIMESH_AT_STATUS_INVALID_COMMAND,
  DIGIMESH_AT_STATUS_INVALID_PARAMETER,
  DIGIMESH_AT_STATUS_END
}digimesh_at_status_t;

/**
 * @brief Length of the values for each AT command in bytes.
 */
typedef enum{
    DIGIMESH_AT_ID_LEN = 2,
    DIGIMESH_AT_CH_LEN = 1,
    DIGIMESH_AT_NI_LEN = 20,
    DIGIMESH_AT_SM_LEN = 1,
    DIGIMESH_AT_SN_LEN = 2,
    DIGIMESH_AT_SO_LEN = 2,
    DIGIMESH_AT_ST_LEN = 3,
    DIGIMESH_AT_SP_LEN = 3,
    DIGIMESH_AT_WH_LEN = 2,
    DIGIMESH_AT_SH_LEN = 0,
    DIGIMESH_AT_SL_LEN = 0,
    DIGIMESH_AT_WR_LEN = 0,
}digimesh_at_command_value_len_t;

/**
 * @brief Identifies what type of frame you want to build.
 */
typedef enum{
    DIGIMESH_FRAME_TYPE_LOCAL_AT = 0x08,
    DIGIMESH_FRAME_TYPE_TRANSMIT_REQUEST = 0x10,
    DIGIMESH_FRAME_TYPE_LOCAL_AT_COMMAND_RESPONSE = 0x88,
    DIGIMESH_FRAME_TYPE_EXTENDED_TRANSMIT_STATUS = 0x8B,
    DIGIMESH_FRAME_TYPE_RECEIVE_PACKET = 0x90,
    DIGIMESH_FRAME_TYPE_END
}digimesh_frame_type_t;


typedef enum
{
  DIGIMESH_TRANSMIT_STATUS_SUCCESS = 0x00
}digimesh_transmit_status_t;

/**
 * @brief For identifying what digi device field you want to set or get.
 */
typedef enum{
    DIGIMESH_SLEEP_NODE = 8,
    DIGIMESH_SLEEP_SUPPORT = 7,
}digimesh_sleep_mode_t;


/**
 * @brief List of strings representing the different at commands.
 */
extern char digimesh_at_command_strings[DIGIMESH_AT_END][3];

/**
 * @brief Strings representing the different statuses for a local at command response.
 */
extern char digimesh_at_status_strings[DIGIMESH_AT_STATUS_END+1][20];

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
 * @fn uint8_t digimesh_get_locat_at_response_size(uint8_t*)
 * @brief Get the number of bytes in the response field of a local at command response.
 *
 * @param frame The total response frame.
 * @return The number of bytes in the response.
 */
uint8_t digimesh_get_local_at_response_size(uint8_t * frame);

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

/**
 * @fn uint16_t digimesh_required_packets(uint32_t)
 * @brief Calculates how many digimesh packets you need to send a payload.
 *
 * @param payload_len
 * @return uint16_t the number of packets required.
 */
uint16_t digimesh_required_packets(uint32_t payload_len);

/**
 * @fn digimesh_at_command_t digimesh_get_command_from_at_response(uint8_t*)
 * @brief Get the at command that the local at command response is for from the
 * frame.
 *
 * @param frame The response packet.
 * @return The command that the response is for.
 */
digimesh_at_command_t digimesh_get_command_from_at_response(uint8_t * frame);

/**
 * @fn digimesh_at_status_t digimesh_get_status_from_at_response(uint8_t*)
 * @brief Get the status of the response to a local at command.
 *
 * @param frame The response packet.
 * @return The status in the response packet.
 */
digimesh_at_status_t digimesh_get_status_from_at_response(uint8_t * frame);

digimesh_at_status_t digimesh_get_at_command_response_value(uint8_t * frame, uint8_t * value);

/**
 * @fn uint8_t digimesh_get_frame_id(uint8_t*)
 * @brief Extract the frame id from a digimesh frame
 *
 * @param frame Byte array representing a digimesh frame.
 * @return The frame id.
 */
uint8_t digimesh_get_frame_id(uint8_t * frame);

/**
 * @fn uint8_t digi_get_transmit_status(uint8_t*)
 * @brief Extract the transmit status from a transmit status message.
 *
 * @param frame Byte array representing a digimesh frame.
 * @return The transmit status.
 */
uint8_t digi_get_transmit_status(uint8_t * frame);
#endif
