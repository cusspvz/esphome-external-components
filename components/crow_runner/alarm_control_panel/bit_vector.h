#pragma once
#include <vector>

class BitVector {
public:
    BitVector(size_t size_in_bits) : bit_size(size_in_bits) {
        byte_size = (size_in_bits + 7) / 8;
        data = std::vector<uint8_t>(byte_size);
        clear();
    }

    bool get_bit(size_t index) const {
        // if (index >= bit_size) throw std::out_of_range("Index out of range");
        size_t byte_index = index / 8;
        size_t bit_index = index % 8;
        return (data[byte_index] >> bit_index) & 1;
    }

    void set_bit(size_t index, bool value) {
        // if (index >= bit_size) throw std::out_of_range("Index out of range");
        size_t byte_index = index / 8;
        size_t bit_index = index % 8;
        if (value) {
            data[byte_index] |= (1 << bit_index);
        } else {
            data[byte_index] &= ~(1 << bit_index);
        }
    }

    uint8_t get_byte(size_t index) const {
        // if (index >= byte_size) throw std::out_of_range("Index out of range");

        return data[index];
    }

    size_t size() const {
        return bit_size;
    }

    // Access the internal data as uint8_t
    const std::vector<uint8_t>& get_data() const {
        return data;
    }

    // Clone method
    BitVector clone(size_t start_bit, size_t end_bit) const {
        // if (start_bit > end_bit || end_bit > bit_size) throw std::out_of_range("Invalid range");
        size_t new_size = end_bit - start_bit;

        BitVector newBitVector(new_size);
        size_t start_byte = start_bit / 8;
        size_t end_byte = (end_bit + 7) / 8;
        size_t start_offset = start_bit % 8;
        size_t end_offset = end_bit % 8;

        if (start_offset == 0) {
            std::copy(data.begin() + start_byte, data.begin() + end_byte, newBitVector.data.begin());
        } else {
            for (size_t i = start_bit; i < end_bit; ++i) {
                newBitVector.set_bit(i - start_bit, this->get_bit(i));
            }
        }

        // // Handle the case where the end bit does not align with the byte boundary
        // if (end_offset != 0) {
        //     newBitVector.data.back() &= (1 << end_offset) - 1;
        // }

        return newBitVector;
    }

    void clear() {
        std::fill(data.begin(), data.end(), 0);
        write_head_bit = 0;
        // read_head_bit = 0;
    }

    void write_bit(bool value) {
        set_bit(write_head_bit, value);
        write_head_bit++;
    }
    bool is_writeable() const {
        return write_head_bit < bit_size;
    }
    const size_t written_bits_so_far() const {
        return write_head_bit;
    }
    const size_t written_bytes_so_far() const {
        return (write_head_bit + 7) / 8;
    }

    // bool is_readable() const {
    //     return read_head_bit < bit_size;
    // }

    // void read_bit(bool value) {
    //     read_bit(read_head_bit, value);
    //     read_head_bit++;
    // }

private:
    std::vector<uint8_t> data;
    size_t byte_size;
    size_t bit_size;

    size_t write_head_bit;
    // size_t read_head_bit;
};
