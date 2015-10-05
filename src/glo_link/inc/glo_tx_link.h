#ifndef GLO_TX_LINK_INCLUDED_H
#define GLO_TX_LINK_INCLUDED_H

// Includes
#include <serial/serial.h>
#include <stdint.h>
#include "globs.h"

// Transmit globs over serial port.
class GloTxLink
{
  public: // methods
      
    // Constructor
    explicit GloTxLink(serial::Serial * port);

    bool send(glob_meta_t const & data);

  private: // fields

    // Serial port that data gets sent/received over
    serial::Serial  * port_;

    uint32_t num_messages_sent_;
    uint32_t num_messages_failed_;
      
    // Make larger than necessary to protect against future changes.
    uint8_t send_buffer_[300]; 
    
};

#endif

