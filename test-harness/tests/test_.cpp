#include "CppUTest/TestHarness.h"

extern "C" 
{
    #include "c_driver_digimesh_parser.h"
}


TEST_GROUP(Test) 
{
    void setup()
    {
        digi_init();
    }

    void teardown()
    {
    }

    digi_serial_t id = {.serial = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
    
    digi_status_t register_digi()
    {
        return digi_register(&id);
    }

    #define IS_DIGI_REGISTERED()\
        CHECK(digi_is_initialized());

    #define NO_DIGI_REGISTERED()\
        CHECK(!digi_is_initialized());

    #define IS_OK(status)\
        CHECK(status == DIGI_OK);

  

};

/********/
/* Zero */
/********/

// Check that the no digi module are registered after initialization
TEST(Test, check_state_is_empty_on_init)
{
    NO_DIGI_REGISTERED();
}

/*******/
/* One */
/*******/

// Initialize the digi module state and then check that it's initialized
TEST(Test, check_state_is_not_empty_after_initialization)
{
    register_digi();
    IS_DIGI_REGISTERED();
}

// Create a message to set the network ID of a digi module
TEST(Test, check_message_to_configure_network_id_is_correct)
{
    // digi_field_t field = DIGI_FIELD_ID;
    // uint16_t value = 0x0A0A0;
    // uint8_t message[128] = {0};
    // IS_OK(digi_generate_set_field_message(DIGI_FIELD_ID, value));
    
    
}

/********/
/* Many */
/********/
