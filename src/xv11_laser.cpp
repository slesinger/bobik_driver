/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Based on work: Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *********************************************************************/

#include "xv11_laser.h"

namespace xv_11_driver
{

    XV11Laser::XV11Laser(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io) : port_(port),
                                                                                                                        baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
    {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    }

    void XV11Laser::poll(uint16_t *ranges, uint32_t *time_increment)
    {
        //uint8_t temp_char;
        uint8_t start_count = 0;
        bool got_scan = false;

        // This is for the newer driver that outputs packets 4 pings at a time
        boost::array<uint8_t, 1980> raw_bytes;
        uint8_t good_sets = 0;
        uint32_t motor_speed = 0;
        rpms = 0;
        int index;
        while (!shutting_down_ && !got_scan)
        {
            // Wait until first data sync of frame: 0xFA, 0xA0
            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
            if (start_count == 0)
            {
                if (raw_bytes[start_count] == 0xFA)
                {
                    start_count = 1;
                }
            }
            else if (start_count == 1)
            {
                if (raw_bytes[start_count] == 0xA0)
                {
                    start_count = 0;

                    // Now that entire start sequence has been found, read in the rest of the message
                    got_scan = true;

                    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 1978));

                    // read data in sets of 4
                    for (uint16_t i = 0; i < raw_bytes.size(); i = i + 22)
                    {
                        if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22))
                        { //&& CRC check
                            good_sets++;
                            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; // accumulate count for avg. time increment
                            rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 64;

                            for (uint16_t j = i + 4; j < i + 20; j = j + 4)
                            {
                                index = (4 * i) / 22 + (j - 4 - i) / 4;
                                // Four bytes per reading
                                uint8_t byte0 = raw_bytes[j];
                                uint8_t byte1 = raw_bytes[j + 1];
                                //uint8_t byte2 = raw_bytes[j + 2];
                                //uint8_t byte3 = raw_bytes[j + 3];
                                // First two bits of byte1 are status flags
                                // uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
                                // uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                                // Remaining bits are the range in mm
                                uint16_t range = ((byte1 & 0x3F) << 8) + byte0;

                                ranges[index] = range;
                            }
                        }
                    }

                    *time_increment = motor_speed / good_sets;
                }
            }
        }
    }
}
