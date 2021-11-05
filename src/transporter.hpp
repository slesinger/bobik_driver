#ifndef ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_
#define ROS2_SERIAL_EXAMPLE__TRANSPORTER_HPP_

#include <cstdint>
#include <mutex>
#include <string>

#include "ring_buffer.hpp"


/**
 * The Transporter class provides an abstract class for transporting data over
 * the serial wire.  The Transporter is responsible for marshalling and
 * unmarshalling data in the proper serial wrapper, then calls the base class
 * implementation of node_write() or node_read() to get data to and from the
 * serial wire, respectively.
 */
class Transporter
{
public:
    /**
     * Do Transporter-specific initialization.
     *
     * Since this is an abstract class, it can't be directly constructed, but
     * this constructor is expected to be called during the derived class
     * constructor to setup the Transporter.
     *
     * @param[in] protocol The backend protocol to use; either 'px4' or 'cobs'.
     * @param[in] ring_buffer_size The number of bytes to allocate to the
     *                             underlying ring buffer that is used to
     *                             accept data from the UDP socket.  Larger
     *                             numbers will allow the transport to accept
     *                             larger packets (or more of them), at the
     *                             expense of memory.  It is recommended to
     *                             start with 8192.
     */
    explicit Transporter(size_t ring_buffer_size);
    virtual ~Transporter();

    Transporter(Transporter const &) = delete;
    Transporter& operator=(Transporter const &) = delete;
    Transporter(Transporter &&) = delete;
    Transporter& operator=(Transporter &&) = delete;

    /**
     * Do some transport-specific initialization.
     *
     * This method is provided so that derived classes can do transport-specific
     * initialization.  If the derived class doesn't need to do any of this, it
     * does not need to be overridden.
     *
     * @returns 0 on success, -1 on error.
     */
    virtual int init() {return 0;}

    /**
     * Do some transport-specific close tasks.
     *
     * This method is provided so that derived classes can do transport-specific
     * shutdown.  If the derived class doesn't need to do any of this, it does
     * not need to be overridden.
     *
     * @returns 0 on success, -1 on error.
     */
    virtual int close() {return 0;}

    /**
     * Read some data from the underlying transport and return the payload in
     * out_buffer.
     *
     * This method will call down into the underlying transport and attempt to
     * get some payload bytes.  If there are no valid payloads available, the
     * method will typically return immediately with a return code of 0, though
     * this depends on how the underlying transport is configured.
     *
     * @param[out] out_buffer The buffer to receive the payload into.
     * @returns The payload size on success, 0 if there are no messages
     *          available, and < 0 if the payload couldn't fit into the given
     *          buffer.
     * @throws std::runtime_error If an internal contract was not fulfilled;
     *         this is typically fatal.
     */
    ssize_t read(uint8_t *out_buffer);

    /**
     * Write data from a buffer out to the underlying transport.
     *
     * This method takes payload data passed to it, adds on the serial protocol
     * configured during construction, then calls down to the underlying
     * node_write() method to actually send the data out to the serial port.
     *
     * @param[in] msg_type The topic ID to add to the message.
     * @param[in] buffer The buffer containing the payload to send.
     * @param[in] length The length of the payload buffer.
     * @returns The payload length written on success, or -1 on error.
     */
    ssize_t write(uint8_t msg_type, uint8_t *buffer, size_t data_length);

    // These methods and members are protected because derived classes need
    // access to them.
protected:
    /**
     * Pure virtual method to read data from the underlying transport.
     *
     * Derived classes should override this method to read some data from the
     * underlying transport and store it into the ring buffer.  How much data
     * to read and how long to wait for the data is implementation specific.
     *
     * @returns The number of bytes read, or -1 on error.
     */
    virtual ssize_t node_read() = 0;

    /**
     * Pure virtual method to write data to the underlying transport.
     *
     * Derived classes should override this method to write some data to the
     * underlying transport.  This method should not return until either all
     * of the data has been written or an error occurs.
     *
     * @params[in] buffer The buffer containing the data to write.
     * @params[in] len The number of bytes in the buffer to write.
     * @returns The number of bytes written on success (which must be equal to
     *          len), or -1 on error.
     */
    virtual ssize_t node_write(uint8_t *buffer, size_t len) = 0;

    /**
     * Pure virtual method to detect whether the file descriptors are ready.
     *
     * Derived classes should override this method to determine whether the
     * underlying transport is ready to send and receive data.
     *
     * @returns true if the underlying transport is ready to send and receive
     *          data, false otherwise.
     */
    virtual bool fds_OK() = 0;

    /**
     * Method to add the CRC16 of one additional byte.
     *
     * Given an existing CRC16 and a new byte, update the CRC16 to include the
     * data from the new byte.
     *
     * @param[in] crc The existing CRC16.
     * @param[in] data The new byte to add to the CRC16.
     * @returns The new CRC16.
     */
    uint16_t crc16_byte(uint16_t crc, uint8_t data);

    /**
     * Method to calculate the CRC16 of an entire buffer.
     *
     * @param[in] buffer The buffer to calculate the CRC16 over.
     * @param[in] len The length of the buffer to calculate.
     * @returns The CRC16 of the entire buffer.
     */
    uint16_t crc16(uint8_t const *buffer, size_t len);

    impl::RingBuffer ringbuf_;

    // These methods and members are protected because the tests need access
    // to them.
protected:

    /**
     * Internal method to find a valid serial message in the ring buffer.
     *
     * This method goes looking through the ring buffer for a valid serial
     * message (what constitues a valid serial message depends on the serial
     * protocol currently in use).  If it finds a valid message, it unpacks it,
     * stuffs the corresponding topic_ID, and fills in the output buffer with
     * the payload.
     *
     * @param[out] out_buffer The buffer to receive the payload into.
     * @returns The payload length on success (which may be 0, and < 0 if a
     *          valid message could not be returned.
     * @throws std::runtime_error If an internal contract was not fulfilled;
     *         this is typically fatal.
     */
    ssize_t find_and_copy_message(uint8_t *out_buffer);

private:
    std::mutex write_mutex_;
};

#endif
