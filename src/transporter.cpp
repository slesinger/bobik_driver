#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>

#include "transporter.hpp"
#include "protocol_types.h"

size_t header_len = 3;
size_t msghdr_len = 2;

Transporter::Transporter(size_t ring_buffer_size) : ringbuf_(ring_buffer_size)
{
}

Transporter::~Transporter()
{
}

/*
uint16_t Transporter::crc16_byte(uint16_t crc, uint8_t data)
{
    return static_cast<uint8_t>(crc >> 8U) ^ crc16_table[static_cast<uint8_t>(crc ^ data)];
}

uint16_t Transporter::crc16(uint8_t const *buffer, size_t len)
{
    uint16_t crc = 0;

    while ((len--) != 0)
    {
        crc = crc16_byte(crc, *buffer++);
    }

    return crc;
}
*/


ssize_t Transporter::find_and_copy_message(uint8_t *out_buffer)
{
    // Find header
    std::array<uint8_t, 3> headerseq{LOOP_START, LOOP_START, LOOP_START};
    ssize_t offset = ringbuf_.findseq(&headerseq[0], header_len);
    if (offset < 0)
    {
        // We didn't find the sequence, so just return
        return -ENODATA;
    }

    if (offset > 0)
    {
        // There is some garbage at the front, so just throw it away.
        std::vector<uint8_t> garbage(offset);
        if (ringbuf_.memcpy_from(&garbage[0], offset) < 0)  // TODO neni tady buffer overrun?
        {
            throw std::runtime_error("Failed getting garbage data from ring buffer");
        }
        if (ringbuf_.bytes_used() < header_len)
        {
            // Not enough bytes now.
            return -ENODATA;
        }
    }

    // Find footer
    std::array<uint8_t, 3> footerseq{LOOP_END, LOOP_END, LOOP_END};
    ssize_t footer_offset = ringbuf_.findseq(&footerseq[0], footerseq.size());
    if (footer_offset < 0)
    {
        // We didn't find the sequence, so just return
        return -ENODATA;
    }

    ringbuf_.memcpy_from(out_buffer, header_len);
    ssize_t out_len = ringbuf_.memcpy_from(out_buffer, footer_offset - header_len);
    if (out_len < 0)
    {
        // We already checked above, so this should never happen.
        throw std::runtime_error("Unexpected ring buffer failure when reading messages");
    }
    //TODO cti CRC kod
    return out_len - header_len;
}

ssize_t Transporter::read(uint8_t *out_buffer)
{
    if (nullptr == out_buffer || !fds_OK())
    {
        return -1;
    }

    if (ringbuf_.bytes_used() >= header_len)
    {
        ssize_t len = find_and_copy_message(out_buffer);
        if (len >= 0)
        {
            return len;
        }
    }

    ssize_t len = node_read();
    if (len < 0)
    {
        if (errno != 0 && errno != EAGAIN && errno != ETIMEDOUT)
        {
            ::printf("Read fail %d\n", errno);
        }

        return len;
    }
    if (len == 0)
    {
        // No data returned, just return -ENODATA
        return -ENODATA;
    }

    if (ringbuf_.bytes_used() >= header_len)
    {
        ssize_t len = find_and_copy_message(out_buffer);
        if (len >= 0)
        {
            return len;
        }
    }

    return -ENODATA;
}


ssize_t Transporter::write(uint8_t msg_type, uint8_t *buffer, size_t data_length)
{
    if (!fds_OK())
    {
        return -1;
    }

    std::lock_guard<std::mutex> lock(write_mutex_);

    uint8_t header[2] = {MSG_6BYTES, msg_type};
    ssize_t written = node_write(header, 2);
    if (written < 0)
    {
        return written;
    }
    written = node_write(buffer, data_length);
    if (written < 0)
    {
        return written;
    }
    std::lock_guard<std::mutex> unlock(write_mutex_);

    return data_length;
}
