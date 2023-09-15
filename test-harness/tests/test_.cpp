#include "CppUTest/TestHarness.h"

extern "C" 
{
    #include "c_driver_digimesh.h"
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

    digimesh_serial_t id = {.serial = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
    
    digi_status_t register_digi()
    {
        return digi_register(&id);
    }

    void are_two_message_equal(uint8_t * message_a, uint8_t * message_b, uint8_t message_length)
    {
        for( uint8_t idx = 0; idx < message_length; idx++)
        {
            LONGS_EQUAL(message_a[idx], message_b[idx]);
        }
    }

    #define IS_DIGI_REGISTERED()\
        CHECK(digi_is_initialized());

    #define NO_DIGI_REGISTERED()\
        CHECK(!digi_is_initialized());

    #define IS_OK(status)\
        CHECK(status == DIGI_OK);

    #define IS_NOT_OK(status)\
        CHECK(!status == DIGI_OK);

    #define ARE_MESSAGES_EQUAL(message_a, message_b, message_length) \
        CHECK(are_two_message_equal(message_a, message_b, message_length));
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

TEST(Test, check_that_you_cant_make_a_value_larger_than_max_value_len)
{
    uint8_t name[DIGIMESH_MAXIMUM_MESSAGE_SIZE+1] = {0};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};

    IS_OK(!digi_generate_at_command_frame(DIGIMESH_AT_NI, name , sizeof(name)/sizeof(name[0]), &generated_frame[0]));
}

TEST(Test, validate_channel_value)
{
    uint8_t channel[] = {0x0A};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};

    IS_NOT_OK(digi_generate_at_command_frame(DIGIMESH_AT_CH, channel , sizeof(channel)/sizeof(channel[0]), &generated_frame[0]));
}

TEST(Test, check_set_network_id_frame)
{
    uint8_t id[] = {0x0A};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};

    IS_OK(digi_generate_at_command_frame(DIGIMESH_AT_ID, id, sizeof(id)/sizeof(id[0]), generated_frame));
    
    uint8_t expected_message[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x49, 0x44, 0x0A, 0x5F};

    are_two_message_equal(expected_message, generated_frame, digimesh_get_frame_size(expected_message));
}

TEST(Test, check_set_channel_frame)
{
    uint8_t channel[] = {0x0B};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};

    IS_OK(digi_generate_at_command_frame(DIGIMESH_AT_CH, channel, sizeof(channel)/sizeof(channel[0]), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x43, 0x48, 0x0B, 0x60};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

TEST(Test, check_set_name_frame)
{
    uint8_t name[] = {'c','r','u','m','b'};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};

    IS_OK(digi_generate_at_command_frame(DIGIMESH_AT_NI, name , sizeof(name)/sizeof(name[0]), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x09, 0x08, 0x01, 0x4E, 0x49, 0x63, 0x72, 0x75, 0x6D, 0x62, 0x46};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

TEST(Test, validate_transmit_request_message)
{
    digimesh_serial_t destination_address = {.serial = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 , 0x00}};
    uint8_t payload[] = {'b','i','g',' ','s','l','u','g'};
    // uint8_t payload[] = {'a'};
    uint8_t len = sizeof(payload)/sizeof(payload[0]);
    uint8_t generated_frame[DIGIMESH_MAXIMUM_MESSAGE_SIZE] = {0};
    
    uint8_t expected_frame[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0xC0, 0x62, 0x69, 0x67, 0x20, 0x73, 0x6C, 0x75, 0x67, 0x24};
    // uint8_t expected_frame[] = {0x7E, 0x00, 0x0F, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0xC0, 0x61, 0xD0};

    IS_OK(digimesh_generate_transmit_request_frame(&destination_address, (unsigned char*)&payload[0], len, generated_frame));


    LONGS_EQUAL(expected_frame[0], generated_frame[0]);                                             // Start delim
    are_two_message_equal(&expected_frame[1], &generated_frame[1], 2);                              // Length
    LONGS_EQUAL(expected_frame[3], generated_frame[3]);                                             // Frame Type
    LONGS_EQUAL(expected_frame[4], generated_frame[4]);                                             // Frame ID
    are_two_message_equal(&expected_frame[5], &generated_frame[5], 8);                              // Address
    are_two_message_equal(&expected_frame[13], &generated_frame[13], 2);                            // Reserved
    LONGS_EQUAL(expected_frame[15], generated_frame[15]);                                           // Broadcast radius
    LONGS_EQUAL(expected_frame[16],generated_frame[16]);                                            // Transmit options
    are_two_message_equal(&expected_frame[17], &generated_frame[17], len);                          // Payload Data
    LONGS_EQUAL(expected_frame[17+len], generated_frame[17+len]);                                   // CRC
}


/********/
/* Many */
/********/
