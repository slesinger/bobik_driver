/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Based on work of: Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *********************************************************************/
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace xv_11_driver
{
    class XV11Laser
    {
    public:
        uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an XV11 packet
        /**
         * @brief Constructs a new XV11Laser attached to the given serial port
         * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
         * @param baud_rate The baud rate to open the serial port at.
         * @param io Boost ASIO IO Service to use when creating the serial port object
         */
        XV11Laser(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io);

        /**
         * @brief Default destructor
         */
        ~XV11Laser(){};

        /**
         * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
         * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
         */
        void poll(uint16_t *ranges, uint32_t *time_increment);

        /**
         * @brief Close the driver down and prevent the polling loop from advancing
         */
        void close() { shutting_down_ = true; };

    private:
        std::string port_;   ///< @brief The serial port the driver is attached to
        uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
        uint32_t firmware_;  ///< @brief The firmware version to check.  Currently supports two different versions: 1 and 2.

        bool shutting_down_;              ///< @brief Flag for whether the driver is supposed to be shutting down or not
        boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the XV11 Laser Scanner
        uint16_t motor_speed_;            ///< @brief current motor speed as reported by the XV11.
    };
}
