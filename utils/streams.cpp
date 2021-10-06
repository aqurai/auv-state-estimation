/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "streams.h"


uint8_t ByteStream::get() {
    return buffer_.get();
}

int8_t ByteStream::put(uint8_t byte) { 
    return (int8_t)buffer_.put(byte);
}

void ByteStream::flush() {
    
}

void ByteStream::start_transmission() {
    
}

bool ByteStream::buffer_empty() {
    return buffer_.isEmpty();
}

int ByteStream::bytes_available() {
    return buffer_.numElements();
}
void ByteStream::clear_stream() {
    buffer_.clearBuffer();
}