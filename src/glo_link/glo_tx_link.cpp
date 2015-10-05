// Includes
#include "crc.h"
#include "glo_tx_link.h"
#include "globs.h"
#include <ros/ros.h>

//*****************************************************************************
// Constructor
GloTxLink::GloTxLink(serial::Serial * port) :
    port_(port),
    num_messages_sent_(0),
    num_messages_failed_(0)
{  
}

//*****************************************************************************
bool GloTxLink::send(glob_meta_t const & data)
{
    if (port_ == NULL)
    {
        num_messages_failed_++;
        ROS_ERROR("Send failed due to null port.");
        return false;
    }
    
    if (data.id >= NUM_GLOBS) { return false; }
    
    const uint16_t num_header_bytes = 5;
    send_buffer_[0] = 0xFE; // message start byte
    send_buffer_[1] = data.id;
    send_buffer_[2] = (uint8_t)data.instance;
    send_buffer_[3] = (uint8_t)(data.instance >> 8);
    send_buffer_[4] = data.num_bytes;
    
    memcpy(send_buffer_ + num_header_bytes, (void const *)data.ptr, data.num_bytes);
    
    uint16_t footer_start = data.num_bytes + num_header_bytes;
    
    uint16_t crc = calculate_crc(send_buffer_, footer_start, 0xFFFF);
    
    const uint16_t num_footer_bytes = 2;
    send_buffer_[footer_start]   = (uint8_t)crc; 
    send_buffer_[footer_start+1] = (uint8_t)(crc >> 8);   
       
    uint16_t packet_size = data.num_bytes + num_header_bytes + num_footer_bytes;

    // Block here until there is room in the buffer
    if (packet_size != port_->write(send_buffer_, packet_size))
    {
        num_messages_failed_++;
        return false;
    }

    port_->flushOutput();

    num_messages_sent_++;
    
    return true;

}
