/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file buffer.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Buffer
 *
 ******************************************************************************/


#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "project_config/conf_constants.h"

#define BUFFER_SIZE 1024
#define BUFFER_MASK (BUFFER_SIZE-1)

template<typename T, size_t _size>
class RingBufferT {
public:
    RingBufferT();
    void init();
    uint8_t putLossy(const T& byte);  // Overwrites if buffer is full and there is more new data
    uint8_t put(const T& byte);
    T get();
    void clearBuffer();
    uint32_t numElements() const;
    bool isFull() const;
    bool isEmpty() const;
    size_t getSize() { return size_; }
    
    // TODO(anwar): Implement custom iterator/const iterator for this container
    
private:
    T elements_[_size];
    uint16_t head_idx_;
    uint16_t tail_idx_;
    uint8_t is_full_;
    size_t size_ = _size;
    size_t mask_ = _size-1;
};

#include "ring-buffer-impl.h"

typedef RingBufferT<uint8_t, BUFFER_SIZE> ByteBuffer;

#endif /* BUFFER_H_ */