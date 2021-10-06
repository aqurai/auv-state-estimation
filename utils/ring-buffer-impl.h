/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   buffer-impl.h
 * Author: quraishi
 *
 * Created on 9 March, 2019, 7:36 PM
 */

#ifndef BUFFER_IMPL_H
#define BUFFER_IMPL_H

template<typename T, size_t _size>
RingBufferT<T,_size>::RingBufferT() {
    init();
}

template<typename T, size_t _size>
void RingBufferT<T,_size>::init() {
    head_idx_ = 0;
    tail_idx_ = 0;
    is_full_ = 0;
}

template<typename T, size_t _size>
uint8_t RingBufferT<T,_size>::putLossy(const T& byte) {
    uint16_t tmp;
    tmp = (head_idx_ + 1) & mask_;

    if (tmp == tail_idx_) {
        // overwrite oldest data at the end of the buffer
        tail_idx_ = (tail_idx_ + 1) & mask_;
    }

    // store incoming data in buffer
    elements_[head_idx_] = byte;
    head_idx_ = tmp;

    if (isFull()) {
        is_full_ = 1;
    } else {
        is_full_ = 0;
    }

    return 0;
}

template<typename T, size_t _size>
uint8_t RingBufferT<T,_size>::put(const T& byte) {
    uint16_t tmp;
    tmp = (head_idx_ + 1) & mask_;

    if (tmp == tail_idx_) {
        // buffer full, return 1
        return 1;
    } else {
        // store incoming data in buffer
        elements_[head_idx_] = byte;
        head_idx_ = tmp;

        if (isFull()) {
            is_full_ = 1;
        } else {
            is_full_ = 0;
        }
        return 0;
    }
}

template<typename T, size_t _size>
T RingBufferT<T,_size>::get() {
    T ret = 0;
    if (head_idx_ != tail_idx_) {
        ret = elements_[tail_idx_];
        tail_idx_ = (tail_idx_ + 1) & mask_;
        is_full_ = 0;
    }
    return ret;
}

template<typename T, size_t _size>
void RingBufferT<T,_size>::clearBuffer() {
    head_idx_ = 0;
    tail_idx_ = 0;
    is_full_ = 0;    
}

template<typename T, size_t _size>
uint32_t RingBufferT<T,_size>::numElements() const {
    return (size_ + head_idx_ - tail_idx_)&mask_;
}

template<typename T, size_t _size>
bool RingBufferT<T,_size>::isFull() const {
    return (((head_idx_ + 1)&mask_) == tail_idx_);
}

template<typename T, size_t _size>
bool RingBufferT<T,_size>::isEmpty() const {
    return (head_idx_==tail_idx_);
}

#endif /* BUFFER_IMPL_H */

