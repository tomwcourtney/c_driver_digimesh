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
 * @brief Bytes in a digi module serial number
 */
#define DIGIMESH_SERIAL_LENGTH 8

/**
 * @brief The maximum size of a byte array representing a message
 */
#define DIGIMESH_MAXIMUM_MESSAGE_SIZE 128


/****************/
/* PUBLIC TYPES */
/****************/

/**
 * @brief Return status of a digi function execution
 */
typedef enum{
    DIGI_OK,
    DIGI_ERROR
}digi_status_t;

/**
 * @brief Type that enforces the correct size for the serial.
 */
typedef struct{
    uint8_t serial[DIGIMESH_SERIAL_LENGTH];
}digi_serial_t;

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
digi_status_t digi_get_serial(digi_serial_t * serial);

/**
 * @brief Stores the state information of a digi module
 * 
 * @param [in] serial - Digi serial number object used to give information to the digi module
 * 
 * @return digi_status_t
 */
digi_status_t digi_register(digi_serial_t * serial);


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
digi_status_t digi_generate_set_field_message(digimesh_at_command_t field, uint8_t * value,  uint8_t value_length, uint8_t * message);


/**
 * @fn uint8_t digimesh_get_frame_size(uint8_t*)
 * @brief Determines the total number of bytes in a frame. This includes start bytes and crc.
 *
 * @param frame - the digimesh frame whose length needs to be determined.
 * @return Total size of the frame in bytes.
 */
uint8_t digimesh_get_frame_size(uint8_t * frame);

#endif
