/**
 * @file   serial_glove.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May  5 15:30:17 2011
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief Communicate via the serial port with the Cyberglove.
 *
 */

#include "cyberglove/serial_glove.hpp"

#include <iostream>
#include <cstdio>

namespace cyberglove_freq
{
  const std::string CybergloveFreq::fastest = "t 1152 0\r"; //fastest speed, just for testing
  const std::string CybergloveFreq::hundred_hz = "t 1152 1\r"; //100Hz
  const std::string CybergloveFreq::fourtyfive_hz = "t 2560 1\r"; //45Hz
  const std::string CybergloveFreq::ten_hz = "t 11520 1\r"; //10Hz
  const std::string CybergloveFreq::one_hz = "t 57600 2\r"; //1Hz
}

namespace cyberglove
{
  const unsigned short CybergloveSerial::glove_size = 22;
  const unsigned short CybergloveSerial::timestamp_size = 14;

  CybergloveSerial::CybergloveSerial(std::string serial_port, std::string cyberglove_version, std::string streaming_protocol, boost::function<void(std::vector<float>, bool)> callback) :
    nb_msgs_received(0), glove_pos_index(0), timestamp_bytes_(0), byte_index_(0), current_value(0), sensor_value_(0), light_on(true), button_on(true), no_errors(true),
    cyberglove_version_(cyberglove_version), reception_state_(INITIAL), streaming_protocol_(streaming_protocol)
  {
    //initialize the vector of positions with 0s
    for (int i = 0; i < glove_size; ++i)
    {
      glove_positions.push_back(0);
    }

    //open the serial port
    cereal_port = boost::shared_ptr<cereal::CerealPort>(new cereal::CerealPort());
    cereal_port->open(serial_port.c_str());

    //set the callback function
    callback_function = callback;
  }

  CybergloveSerial::~CybergloveSerial()
  {
    cereal_port->stopStream();
    //stop the cyberglove transmission
    cereal_port->write("^c", 2);
  }

  int CybergloveSerial::set_filtering(bool value)
  {
    char aux[30];
    aux[0] = 'F';
    if( value ) //Filtering will be on
    {
      aux[1] = 0x01;
      cereal_port->write(aux, 2);
      std::cout << " - Data filtered" << std::endl;
    }
    else // Filtering off
    {
      aux[1] = 0x00;
      cereal_port->write(aux, 2);
      std::cout << " - Data not filtered" << std::endl;
    }
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);

    return 0;
  }

  int CybergloveSerial::set_transmit_info(bool value)
  {
    if( value ) //transmit info will be on
    {
      cereal_port->write("u 1\r", 4);
      std::cout << " - Additional info transmitted" << std::endl;
    }
    else // transmit info off
    {
      cereal_port->write("u 0\r", 4);
      std::cout << " - Additional info not transmitted" << std::endl;
    }
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);

    return 0;
  }

  int CybergloveSerial::set_frequency(std::string frequency)
  {
    cereal_port->write(frequency.c_str(), frequency.size());
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);
    return 0;
  }

  int CybergloveSerial::start_stream()
  {
    std::cout << "starting stream"<<std::endl;

    cereal_port->startReadStream(boost::bind(&CybergloveSerial::stream_callback, this, _1, _2));

    if((cyberglove_version_ == "3") && (streaming_protocol_ == "16bit"))
    {
      // enable USB streaming
      cereal_port->write("1eu", 3);
      cereal_port->flush();
      // start streaming by writing 1S to the serial port
      cereal_port->write("1S", 2);
      cereal_port->flush();
    }
    else
    {
      //start streaming by writing S to the serial port
      cereal_port->write("S", 1);
      cereal_port->flush();
    }

    return 0;
  }

  void CybergloveSerial::stream_callback(char* world, int length)
  {
    //read each received char.
    for (int i = 0; i < length; ++i)
    {
      current_value = (unsigned int)(unsigned char)world[i];

      if((cyberglove_version_ == "3") && (streaming_protocol_ == "16bit"))
      {
        switch(reception_state_)
        {
          case reception_16bit::SYNCHRONIZATION_1:
            switch( current_value )
            {
              //the data set starts after 0xd 0xa 0x0, it starts with the time + sample index in the format
              // HH:MM:SS:ss:n'S' where ss is a number from 1 to 30 indicating the index of the sample (if the sampling frequency is 30 Hz)
              // the n is an index referring to the multiplier index (0-2 if the multiplier is 3)
              // This is followed by the sensors values (2 bytes per sensor)
              case 0x0D:
                reception_state_ = reception_16bit::SYNCHRONIZATION_2;
                break;
            }
            break;
          case reception_16bit::SYNCHRONIZATION_2:
            switch( current_value )
            {
              case 0x0A:
                reception_state_ = reception_16bit::SYNCHRONIZATION_3;
                break;
              default:
                reception_state_ = reception_16bit::SYNCHRONIZATION_1;
                break;
            }
            break;
          case reception_16bit::SYNCHRONIZATION_3:
            switch( current_value )
            {
              case 0x00:
                timestamp_bytes_ = 0;
                reception_state_ = reception_16bit::TIMESTAMP;
                break;
              default:
                reception_state_ = reception_16bit::SYNCHRONIZATION_1;
                break;
            }
            break;
          case reception_16bit::TIMESTAMP:
            timestamp_bytes_++;
            // special case observed: sometimes after D A 0 sequence we get n'S' instead of directly the time
            if ((timestamp_bytes_ == 2) && (current_value == 'S'))
            {
              timestamp_bytes_ = 0;
            }
            if (timestamp_bytes_ == timestamp_size)
            {
              //the line starts with S, followed by the sensors values
              ++nb_msgs_received;
              //reset the index to 0
              glove_pos_index = 0;
              byte_index_ = 0;
              //reset no_errors to true for the new message
              no_errors = true;
              reception_state_ = reception_16bit::RECEIVING_FRAME;
            }
            break;
          case reception_16bit::RECEIVING_FRAME:
            if (byte_index_)
            {
              sensor_value_ += current_value;
              // the values sent by the glove are in the range [1;4094] (12 bit ADC)
              //   -> we convert them to float in the range [0;1]
//              char aux[30];
//              sprintf(aux, "%u", sensor_value_);
//              std::cout << aux << std::endl;

              glove_positions[glove_pos_index] = (((float)sensor_value_) - 1.0f) / (float)(0x0FFF - 1);
              ++glove_pos_index;
              byte_index_ = 0;
            }
            else
            {
              sensor_value_ = current_value << 8;
              byte_index_ = 1;
            }
            //this is a joint data from the glove
            //the value in the message should never be 0.
            if((byte_index_ == 0) && (sensor_value_ == 0))
            {
              no_errors = false;
              //std::cout << "error detected" << std::endl;
            }

            if (glove_pos_index == glove_size)
            {
              if(no_errors)
                callback_function(glove_positions, true);
              reception_state_ = reception_16bit::SYNCHRONIZATION_1;
            }
            break;
        }
      }
      else
      {
        switch(reception_state_)
        {
          case INITIAL:
            switch( current_value )
            {
            case 'S':
              //the line starts with S, followed by the sensors values
              ++nb_msgs_received;
              //reset the index to 0
              glove_pos_index = 0;
              //reset no_errors to true for the new message
              no_errors = true;
              reception_state_ = RECEIVING_FRAME;
              break;
            }
            break;
          case RECEIVING_FRAME:
            //this is a glove sensor value, a status byte or a "message end"
            switch( glove_pos_index )
            {
            case glove_size:
              if(cyberglove_version_ == "2")
              {
                //the last char of the msg is the status byte

                //the status bit 1 corresponds to the button
                if(current_value & 2)
                  button_on = true;
                else
                  button_on = false;
                //the status bit 2 corresponds to the light
                if(current_value & 4)
                  light_on = true;
                else
                  light_on = false;
              }
              else
              {
                //the last char of the line should be 0
                //if it is 0, then the full message has been received,
                //and we call the callback function.
                if( current_value == 0 && no_errors)
                  callback_function(glove_positions, light_on);
                if( current_value != 0)
                  std::cout << "Last char is not 0: " << current_value << std::endl;

                reception_state_ = INITIAL;
              }
              break;

            case glove_size + 1:
              //the last char of the line should be 0
              //if it is 0, then the full message has been received,
              //and we call the callback function.
              if( current_value == 0 && no_errors)
                callback_function(glove_positions, light_on);
              if( current_value != 0)
                std::cout << "Last char is not 0: " << current_value << std::endl;
              reception_state_ = INITIAL;
              break;

            default:
              //this is a joint data from the glove
              //the value in the message should never be 0.
              if( current_value == 0)
              {
                no_errors = false;
              }
              // the values sent by the glove are in the range [1;254]
              //   -> we convert them to float in the range [0;1]
              glove_positions[glove_pos_index] = (((float)current_value) - 1.0f) / 254.0f;
              break;
            }
            ++glove_pos_index;
            break;
        }
      }
    }
  }

  int CybergloveSerial::get_nb_msgs_received()
  {
    return nb_msgs_received;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
