#include "CppUTest/TestHarness.h"

extern "C" 
{
    #include "../../../c_driver_digimesh/inc/c_driver_digimesh.h"
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
    
    digimesh_status_t register_digi()
    {
        return digi_register(&id);
    }

    void are_two_message_equal(uint8_t * message_a, uint8_t * message_b, uint8_t message_length)
    {
        for( uint8_t idx = 0; idx < message_length; idx++)
        {
            char str[10] = {0};
            sprintf(str, "idx: %d", idx);
            LONGS_EQUAL_TEXT(message_a[idx], message_b[idx], str);
        }
    }

    #define IS_DIGI_REGISTERED()\
        CHECK(digi_is_initialized());

    #define NO_DIGI_REGISTERED()\
        CHECK(!digi_is_initialized());

    #define IS_OK(status)\
        CHECK(status == DIGIMESH_OK);

    #define IS_NOT_OK(status)\
        CHECK(!status == DIGIMESH_OK);

    #define ARE_MESSAGES_EQUAL(message_a, message_b, message_length) \
        CHECK(are_two_messages_equal(message_a, message_b, message_length));
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
    uint8_t name[DIGIMESH_MAXIMUM_FRAME_SIZE+1] = {0};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(!digimesh_generate_at_command_frame(DIGIMESH_AT_NI, name , sizeof(name)/sizeof(name[0]), &generated_frame[0]));
}

TEST(Test, validate_channel_value)
{
    uint8_t channel[] = {0x0A};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_NOT_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_CH, channel , sizeof(channel)/sizeof(channel[0]), &generated_frame[0]));
}

TEST(Test, check_set_network_id_frame)
{
    uint8_t id[] = {0x0A};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_ID, id, sizeof(id)/sizeof(id[0]), generated_frame));
    
    uint8_t expected_message[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x49, 0x44, 0x0A, 0x5F};

    are_two_message_equal(expected_message, generated_frame, digimesh_get_frame_size(expected_message));
}

TEST(Test, check_set_channel_frame)
{
    uint8_t channel[] = {0x0B};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_CH, channel, sizeof(channel)/sizeof(channel[0]), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x43, 0x48, 0x0B, 0x60};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

TEST(Test, check_set_name_frame)
{
    uint8_t name[] = {'c','r','u','m','b'};
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_NI, name , sizeof(name)/sizeof(name[0]), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x09, 0x08, 0x01, 0x4E, 0x49, 0x63, 0x72, 0x75, 0x6D, 0x62, 0x46};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

TEST(Test, validate_transmit_request_message)
{
    uint8_t destination_address[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 , 0x00};
    uint8_t payload[] = {'b','i','g',' ','s','l','u','g'};
    // uint8_t payload[] = {'a'};
    uint8_t len = sizeof(payload)/sizeof(payload[0]);
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};
    
    uint8_t expected_frame[] = {0x7E, 0x00, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0xC0, 0x62, 0x69, 0x67, 0x20, 0x73, 0x6C, 0x75, 0x67, 0x24};
    // uint8_t expected_frame[] = {0x7E, 0x00, 0x0F, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0xC0, 0x61, 0xD0};

    IS_OK(digimesh_generate_transmit_request_frame(destination_address, (unsigned char*)&payload[0], len, generated_frame));


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

// Packet type 0x88
TEST(Test, parse_digimesh_local_at_response)
{
    // Put received bytes plus a bunch of other bytes either side into a byte array
    uint8_t input[] = {0x01, 0x00, 0x03, 0x99, 0x10, 0x7E, 0x00, 0x05, 0x88, 0x01, 0x4E, 0x49, 0x00, 0xDF, 0x99, 0x23, 0x00, 0xFF};
    uint16_t input_head = sizeof(input)/sizeof(input[0]);
    uint16_t input_tail = 0;

    uint8_t output[100] = {0};
    uint16_t output_head = 0;

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x88, 0x01, 0x4E, 0x49, 0x00, 0xDF};
    uint8_t expected_frame_length = sizeof(expected_frame)/sizeof(expected_frame[0]);

    // Pass byte array to the parse funtion which remove the crap either side and store the polished digimesh packets into another circular buffer
    digimesh_parse_bytes(input, &input_head, &input_tail, output, &output_head);

    // Check that the input tail matches the total size of the input buffer
    LONGS_EQUAL(input_head, input_tail);

    // Check the number of bytes written to the output buffer matches the length of the digimesh packets in the input buffer
    LONGS_EQUAL(9, output_head);

    // Check that the whole digimesh packet arrived in the other buffer
    are_two_message_equal(expected_frame, output, expected_frame_length);
}


// Packet type 0x90
TEST(Test, parse_digimesh_receive_packet)
{
    // Put received bytes plus a bunch of other bytes either side into a byte array
    uint8_t input[] = {0x7E, 0x7E, 0x00, 0x01, 0x7E, 0x00, 0x0F, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x61, 0x62, 0x63, 0x53, 0x7E, 0x55};
    uint16_t input_head = sizeof(input)/sizeof(input[0]);
    uint16_t input_tail = 0;
    
    uint8_t output[100] = {0};
    uint16_t output_head = 0;

    uint8_t expected_frame[] = {0x7E, 0x00, 0x0F, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x61, 0x62, 0x63, 0x53};
    uint8_t expected_frame_length = sizeof(expected_frame)/sizeof(expected_frame[0]);
    uint16_t expected_output_head = 19;

    // Pass byte array to the parse funtion which remove the crap either side and store the polished digimesh packets into another circular buffer
    digimesh_parse_bytes(input, &input_head, &input_tail, output, &output_head);

    // Check that the input tail matches the total size of the input buffer
    LONGS_EQUAL(input_head - 2, input_tail);

    // Check the number of bytes written to the output buffer matches the length of the digimesh packets in the input buffer
    LONGS_EQUAL(expected_output_head, output_head);

    // Check that the whole digimesh packet arrived in the other buffer
    are_two_message_equal(expected_frame, output, expected_frame_length);
}

TEST(Test, extract_a_frame_from_an_array_of_frames)
{
    // The frame of digimesh packets
    uint8_t array[] = {0x7E, 0x00, 0x05, 0x88, 0x01, 0x43, 0x48, 0x00, 0xEB, 0x7E, 0x00, 0x05, 0x88, 0x01, 0x4E, 0x49, 0x00, 0xDF, 0x7E, 0x00, 0x0F, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x61, 0x73, 0x64, 0x41};

    
    uint16_t head = sizeof(array)/sizeof(array[0]);
    uint16_t tail = 0;

    // The empty frame to place it into
    uint8_t new_frame[128] = {0};

    // Extract a frame
    IS_OK(digimesh_extract_first_digimesh_packet(array, &head, &tail, new_frame));

    // Check the head and tail of the array
    LONGS_EQUAL(0, tail);
    LONGS_EQUAL(28, head);

    // Check the value of the new_frame
    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x88, 0x01, 0x43, 0x48, 0x00, 0xEB};
    uint8_t expected_frame_len = sizeof(expected_frame)/sizeof(expected_frame[0]);
    are_two_message_equal(expected_frame, new_frame, expected_frame_len);
    
    // Check the updated value of the array
    uint8_t expected_array[] = {0x7E, 0x00, 0x05, 0x88, 0x01, 0x4E, 0x49, 0x00, 0xDF, 0x7E, 0x00, 0x0F, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x61, 0x73, 0x64, 0x41};
    uint8_t expected_array_len = sizeof(expected_array)/sizeof(expected_array[0]);
    are_two_message_equal(expected_array, array, expected_array_len);

}

TEST(Test, extract_payload_from_receive_packet)
{
    uint8_t frame[] = {0x7E, 0x00, 0x12, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x24};

    uint8_t payload[DIGIMESH_MAX_PAYLOAD_SIZE] = {0};

    uint8_t expected_payload[] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66};
    uint8_t expected_payload_len = sizeof(expected_payload)/sizeof(expected_payload[0]);

    digimesh_extract_payload_from_receive_frame(frame, payload);

    are_two_message_equal(expected_payload, payload, expected_payload_len);
}

// Packet type 0x90
TEST(Test, parse_digimesh_receive_packet2)
{
    // Put received bytes plus a bunch of other bytes either side into a byte array
    uint8_t input[] = {0x7E, 0x00, 0x0E, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x03, 0x44, 0x32};
    uint16_t input_head = sizeof(input)/sizeof(input[0]);
    uint16_t input_tail = 0;
    
    uint8_t output[100] = {0};
    uint16_t output_head = 0;

    uint8_t expected_frame[] = {0x7E, 0x00, 0x0E, 0x90, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0x03, 0x44, 0x32};
    uint8_t expected_frame_head = sizeof(expected_frame)/sizeof(expected_frame[0]);

    // Pass byte array to the parse funtion which remove the crap either side and store the polished digimesh packets into another circular buffer
    digimesh_parse_bytes(input, &input_head, &input_tail, output, &output_head);

    // Check that the whole digimesh packet arrived in the other buffer
    are_two_message_equal(expected_frame, output, expected_frame_head);
}

// SLEEP MODE (SM)
TEST(Test, check_set_sleep_mode_cyclic_sleep)
{
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    uint8_t value = 8; // Cyclic sleep mode
    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SM, &value, sizeof(value), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x53, 0x4D, 0x08, 0x4E};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}


// SLEEP NUMBER (SN)
TEST(Test, set_sleep_number)
{
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    uint8_t value = 1; 

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SN, &value, 1, &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x53, 0x4E, 0x01, 0x54};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

// SLEEP OPTIONS (SO)
TEST(Test, check_set_sleep_options)
{
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    uint8_t value = 0x01;

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SO, &value, sizeof(value), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x53, 0x4F, 0x01, 0x53};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

// WAKE TIME (ST)
TEST(Test, check_set_wake_time)
{
    uint8_t value = 0x7D;
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_ST, &value, sizeof(value), &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x53, 0x54, 0x7D, 0xD2};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}


// SLEEP PERIOD (SP)
TEST(Test, check_set_sleep_period)
{
    uint8_t value = 0xC8;
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SP, &value, 1, &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x53, 0x50, 0xC8, 0x8B};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

// HOST DELAY (WH) 
TEST(Test, check_set_host_delay)
{
    uint8_t value = 0;
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_WH, &value, 1, &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x05, 0x08, 0x01, 0x57, 0x48, 0x00, 0x57};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

// Serial High (SH) 
TEST(Test, get_serial_high)
{
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SH, NULL, 0, &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x53, 0x48, 0x5B};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}

// Serial Low (SL) 
TEST(Test, get_serial_low)
{
    uint8_t generated_frame[DIGIMESH_MAXIMUM_FRAME_SIZE] = {0};

    IS_OK(digimesh_generate_at_command_frame(DIGIMESH_AT_SL, NULL, 0, &generated_frame[0]));

    uint8_t expected_frame[] = {0x7E, 0x00, 0x04, 0x08, 0x01, 0x53, 0x4C, 0x57};

    are_two_message_equal(expected_frame, generated_frame, digimesh_get_frame_size(expected_frame));
}