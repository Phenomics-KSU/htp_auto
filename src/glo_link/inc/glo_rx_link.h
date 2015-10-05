#ifndef GLO_RX_LINK_INCLUDED_H
#define GLO_RX_LINK_INCLUDED_H

// Includes
#include "globs.h"
#include <serial/serial.h>
#include <stdint.h>

// Receive data off a serial port and parse it into complete globs.
class GloRxLink
{
  public: // methods
      
    // Constructor
    explicit GloRxLink(serial::Serial * port);

    // Look for newly transmitted object information and progresses through 
    // state machine.  Needs be called by a periodic task that runs at least 
    // fast enough to handle data stream rate.  If new message is received
    // then returns false and 'meta_data' will be valid. User need to handle
    // data before calling parse again since the data pointer in meta_data
    // just points to the class's internal receive buffer.
    bool parse(glob_meta_t & meta_data);
    
  private: // methods

      void handleReceivedMessage(void);
      
      // Return true if actual CRC matches expected CRC stored in message.
      bool verifyCRC(void);
      
      void advanceParse(void);

      void resetParse(void);
      
  private: // fields

    // Serial port that data gets received over.
    serial::Serial * port_;

    // Message info that needs to be remembered when parsing.
    int8_t parse_state_;
    uint8_t num_body_bytes_;
    uint8_t body_start_idx_;
    uint8_t expected_crc1_;
    uint8_t expected_crc2_;

    // Data of entire message excluding checksum.
    // Larger than necessary to protect against future changes.
    uint8_t message_data_[300]; 

    // Current index to store next byte in 'message data'.
    uint16_t data_idx_;
    
    // How many complete messages have been parsed from serial port.
    uint32_t num_messages_received_;

};

#endif

