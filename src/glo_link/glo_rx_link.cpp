// Includes
#include "crc.h"
#include "glo_rx_link.h"
#include "globs.h"
#include <ros/ros.h>

// Constructor
GloRxLink::GloRxLink(serial::Serial * port) :
    port_(port),
    parse_state_(-1),
    num_body_bytes_(0),
    body_start_idx_(0),
    data_idx_(0),
    num_messages_received_(0)
{
    resetParse();
}

//*****************************************************************************
bool GloRxLink::parse(glob_meta_t & meta_data)
{
    const uint8_t msg_start_byte = 0xFE;

    uint8_t in_byte = 0; // new byte read in from port
    bool complete_message_received = false; // true if received new glob
    bool valid_message_received = false; // true if have new complete message with good CRC

    if (port_ == NULL)
    {
        ROS_ERROR("Parse failed due to null port.");
        return false;
    }

    while (port_->read(&in_byte, 1))
    {
        switch (parse_state_)
        {
            case -1:  // looking for start byte
                if (in_byte == msg_start_byte)
                {
                    message_data_[data_idx_++] = in_byte;
                    advanceParse();
                }
                break;
            case 0:  // pull out id (fall through)
            case 1:  // pull out byte one of instance (fall through)
            case 2:  // pull out byte two of instance
                message_data_[data_idx_++] = in_byte;
                advanceParse();
                break;
            case 3:  // pull out length of data
                num_body_bytes_ = in_byte;
                message_data_[data_idx_++] = in_byte;
                body_start_idx_ = data_idx_;
                advanceParse();
                if (num_body_bytes_ == 0)
                {
                    advanceParse(); // skip to getting CRC bytes
                }
                break;
            case 4:  // pull out body
                message_data_[data_idx_++] = in_byte;
                if ((data_idx_ - body_start_idx_) >= num_body_bytes_)
                {
                    advanceParse(); // received all body bytes
                }
                break;
            case 5: // pull out lower byte of CRC
                expected_crc1_ = in_byte;
                advanceParse();
                break;
            case 6: // pull out upper byte of CRC
                expected_crc2_ = in_byte;
                complete_message_received = true;
                // reset parse after handling new message
                break;
            default: // safety reset
                resetParse();
                break;
        }

        if (complete_message_received)
        {
            complete_message_received = false;

            if (verifyCRC())
            {
                meta_data.id = message_data_[1];
                meta_data.instance = message_data_[2] + (uint16_t)(message_data_[3] << 8);
                meta_data.num_bytes = num_body_bytes_;
                meta_data.ptr = message_data_ + body_start_idx_;
                valid_message_received = true;
            }

            resetParse();
        }

        if (valid_message_received)
        {
            break; // so caller has a chance to deal with new message.
        }
    }

    return valid_message_received;
}

//*****************************************************************************
bool GloRxLink::verifyCRC(void)
{
    uint16_t expected_crc = (uint16_t)expected_crc1_ + (uint16_t)(expected_crc2_ << 8);

    uint16_t actual_crc = calculate_crc(message_data_, data_idx_, 0xFFFF);

    bool crc_matches = (actual_crc == expected_crc);

    if (!crc_matches)
    {
        ROS_WARN("Bad CRC. Expected %x actual %x", expected_crc, actual_crc);
    }

    return crc_matches;
}

//*****************************************************************************
void GloRxLink::handleReceivedMessage(void)
{
    uint8_t object_id = message_data_[1];
    // KLM commented out 'instance' to avoid warning since it's not being used yet.
    //uint16_t instance = message_data_[2] + (uint16_t)(message_data_[3] << 8);
    void * glob_data = message_data_ + body_start_idx_;

    switch (object_id)
    {
        case GLO_ID_DRIVING_COMMAND:
            //communications_task.handle(*((glo_driving_command_t *)glob_data));
            break;
        default:
            ROS_WARN("Received unhandled glob with id: %d", object_id);
            break;
    }

    num_messages_received_++;

}

//*****************************************************************************
void GloRxLink::resetParse(void)
{
    body_start_idx_ = 0;
    data_idx_ = 0;
    parse_state_ = -1;
}

//*****************************************************************************
void GloRxLink::advanceParse(void)
{
    parse_state_++;
}
