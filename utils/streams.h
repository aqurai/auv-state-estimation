/*
 * streams.h
 *
 */


#ifndef STREAMS_H_
#define STREAMS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ring-buffer.h"

/*class StreamData {
    
}*/

class ByteStream {
public:
    ByteStream() { buffer_.init(); }
    virtual uint8_t get();                  // get a byte from the stream
    virtual int8_t put(uint8_t byte);       // put a byte into the stream. Depending on implementation this may or may not block
    virtual void flush();                   // initiate transmission and wait for it to finish (blocking)
    virtual void start_transmission();      // initiate transmission (non-blocking)
    virtual bool buffer_empty();            // returns true if the stream is empty
    virtual int bytes_available();          // returns how many bytes can be retrieved from the stream
    virtual void clear_stream();            // clears the stream buffer (buffer_empty will return true after this)

protected:
    ByteBuffer buffer_;
};

#endif
