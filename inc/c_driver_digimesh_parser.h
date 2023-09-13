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
#define DIGI_SERIAL_LENGTH 8

/**
 * @brief The maximum size of a byte array representing a message
 */
#define MAXIMUM_MESSAGE_SIZE 128


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
    uint8_t serial[DIGI_SERIAL_LENGTH];
}digi_serial_t;

/**
 * @brief Holds state information about a digimodule.
 */
typedef struct digi_t digi_t;

/**
 * @brief For identifying what digi device field you want to set or get.
 */
typedef enum{
    DIGI_FIELD_ID,
    DIGI_FIELD_END
}digi_field_t;




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
 * @param serial - pointer to a digi serial object for population.
 * 
 * @return digi_status_t 
 */
digi_status_t digi_get_serial(digi_serial_t * serial);

/**
 * @brief Stores the state information of a digi module
 * 
 * @param serial - Digi serial number object used to give information to the digi module
 * @return digi_status_t
 */
digi_status_t digi_register(digi_serial_t * serial);



#endif