/**
* @file   shadowhand_publisher.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>
* @date   12/11/2014
*
*
* Copyright 2014 Shadow Robot Company Ltd.
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
* @brief gets joint positions from the cyberglove via serial port, and calibrates them,
* remaps them to Shadow hand joints, and publishes them as joint trajectories in a single node.
*
*/

//ROS include
#include <ros/ros.h>

//generic C/C++ include
#include <string>
#include <sstream>

#include "cyberglove_trajectory/cyberglove_trajectory_publisher.h"
#include <boost/assign.hpp>
#include <math.h>

using namespace ros;
using namespace xml_calibration_parser;

namespace cyberglove{

const std::vector<std::string> CybergloveTrajectoryPublisher::joint_name_vector_ = boost::assign::list_of
                                                                                  ("THJ1")
                                                                                  ("THJ2")
                                                                                  ("THJ3")
                                                                                  ("THJ4")
                                                                                  ("THJ5")
                                                                                  ("FFJ1")
                                                                                  ("FFJ2")
                                                                                  ("FFJ3")
                                                                                  ("FFJ4")
                                                                                  ("MFJ1")
                                                                                  ("MFJ2")
                                                                                  ("MFJ3")
                                                                                  ("MFJ4")
                                                                                  ("RFJ1")
                                                                                  ("RFJ2")
                                                                                  ("RFJ3")
                                                                                  ("RFJ4")
                                                                                  ("LFJ1")
                                                                                  ("LFJ2")
                                                                                  ("LFJ3")
                                                                                  ("LFJ4")
                                                                                  ("LFJ5")
                                                                                  ("WRJ1")
                                                                                  ("WRJ2");

const std::vector<std::string> CybergloveTrajectoryPublisher::joint_mapping_vector_ = boost::assign::list_of
                                                                                  ("THJ1")
                                                                                  ("THJ2")
                                                                                  ("THJ3")
                                                                                  ("THJ4")
                                                                                  ("THJ5")
                                                                                  ("FFJ0")
                                                                                  ("FFJ3")
                                                                                  ("FFJ4")
                                                                                  ("MFJ0")
                                                                                  ("MFJ3")
                                                                                  ("MFJ4")
                                                                                  ("RFJ0")
                                                                                  ("RFJ3")
                                                                                  ("RFJ4")
                                                                                  ("LFJ0")
                                                                                  ("LFJ3")
                                                                                  ("LFJ4")
                                                                                  ("LFJ5")
                                                                                  ("WRJ1")
                                                                                  ("WRJ2");





//initialises joint names (the order is important)
const std::vector<std::string> CybergloveTrajectoryPublisher::glove_sensors_vector_ = boost::assign::list_of
                                                                                    ("G_ThumbRotate")
                                                                                    ("G_ThumbMPJ")
                                                                                    ("G_ThumbIJ")
                                                                                    ("G_ThumbAb")
                                                                                    ("G_IndexMPJ")
                                                                                    ("G_IndexPIJ")
                                                                                    ("G_IndexDIJ")
                                                                                    ("G_MiddleMPJ")
                                                                                    ("G_MiddlePIJ")
                                                                                    ("G_MiddleDIJ")
                                                                                    ("G_MiddleIndexAb")
                                                                                    ("G_RingMPJ")
                                                                                    ("G_RingPIJ")
                                                                                    ("G_RingDIJ")
                                                                                    ("G_RingMiddleAb")
                                                                                    ("G_PinkieMPJ")
                                                                                    ("G_PinkiePIJ")
                                                                                    ("G_PinkieDIJ")
                                                                                    ("G_PinkieRingAb")
                                                                                    ("G_PalmArch")
                                                                                    ("G_WristPitch")
                                                                                    ("G_WristYaw");
  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  CybergloveTrajectoryPublisher::CybergloveTrajectoryPublisher()
    : n_tilde("~"), publish_counter_max(0), publish_counter_index(0),
      path_to_glove("/dev/ttyS0"), publishing(true)
  {
    std::string param;
    std::string path;
    n_tilde.searchParam("cyberglove_mapping_path", param);
    n_tilde.param(param, path, std::string());
    map_calibration_parser.reset(new CalibrationParser(path));
    ROS_INFO("Mapping file loaded for the Cyberglove: %s", path.c_str());


    std::string path_to_calibration;
    n_tilde.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
    ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

    initialize_calibration(path_to_calibration);

    std::string searched_param;
    std::string joint_prefix;
    searched_param = "joint_prefix";
    n_tilde.param(searched_param, joint_prefix, std::string());
    std::string action_server_name = "trajectory_controller/follow_joint_trajectory";
    action_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(joint_prefix + action_server_name, true));

    for (size_t i = 0; i < joint_name_vector_.size(); i++)
    {
      trajectory_goal_.trajectory.joint_names.push_back(joint_prefix + joint_name_vector_[i]);
    }

    //set sampling frequency
    double sampling_freq;
    n_tilde.param("sampling_frequency", sampling_freq, 100.0);

    // set publish_counter: the number of data we'll average
    // before publishing.
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_counter_max = (int)(sampling_freq / publish_freq);

    ROS_INFO_STREAM("Sampling at " << sampling_freq << "Hz ; Publishing at "
                    << publish_freq << "Hz ; Publish counter: "<< publish_counter_max);

    // set path to glove
    n_tilde.param("path_to_glove", path_to_glove, std::string("/dev/ttyS0"));
    ROS_INFO("Opening glove on port: %s", path_to_glove.c_str());

    //initialize the connection with the cyberglove and binds the callback function
    serial_glove = boost::shared_ptr<CybergloveSerial>(new CybergloveSerial(path_to_glove, boost::bind(&CybergloveTrajectoryPublisher::glove_callback, this, _1, _2)));

    int res = -1;
    cyberglove_freq::CybergloveFreq frequency;

    switch( (int)sampling_freq)
    {
    case 100:
      res = serial_glove->set_frequency(frequency.hundred_hz);
      break;
    case 45:
      res = serial_glove->set_frequency(frequency.fourtyfive_hz);
      break;
    case 10:
      res = serial_glove->set_frequency(frequency.ten_hz);
      break;
    case 1:
      res = serial_glove->set_frequency(frequency.one_hz);
      break;
    default:
      res = serial_glove->set_frequency(frequency.hundred_hz);
      break;
    }
    //No filtering: we're oversampling the data, we want a fast poling rate
    res = serial_glove->set_filtering(false);
    //We want the glove to transmit the status (light on/off)
    res = serial_glove->set_transmit_info(true);
    //start reading the data.
    res = serial_glove->start_stream();
  }

  CybergloveTrajectoryPublisher::~CybergloveTrajectoryPublisher()
  {
  }

  void CybergloveTrajectoryPublisher::initialize_calibration(std::string path_to_calibration)
  {
    calibration_parser = XmlCalibrationParser(path_to_calibration);
  }

  bool CybergloveTrajectoryPublisher::isPublishing()
  {
    if (publishing)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void CybergloveTrajectoryPublisher::setPublishing(bool value)
  {
    publishing = value;
  }

  /////////////////////////////////
  //       CALLBACK METHOD       //
  /////////////////////////////////
  void CybergloveTrajectoryPublisher::glove_callback(std::vector<float> glove_pos, bool light_on)
  {
    //if the light is off, we don't publish any data.
    if( !light_on )
    {
      publishing = false;
      ROS_DEBUG("The glove button is off, no data will be read / sent");
      ros::spinOnce();
      return;
    }
    publishing = true;

    //appends the current position to the vector of position
    glove_positions.push_back( glove_pos );

    publish_counter_index += 1;

    //if we've enough samples, publish the data:
    if( publish_counter_index == publish_counter_max )
    {
      std::vector<double> glove_calibrated_positions, hand_positions, hand_positions_no_J0;

      //fill the joint_state msg with the averaged glove data
      for(unsigned int index_joint = 0; index_joint < CybergloveSerial::glove_size; ++index_joint)
      {
        //compute the average over the samples for the current joint
        float averaged_value = 0.0f;
        for (unsigned int index_sample = 0; index_sample < publish_counter_max; ++index_sample)
        {
          averaged_value += glove_positions[index_sample][index_joint];
        }
        averaged_value /= publish_counter_max;


        float calibration_value = calibration_parser.get_calibration_value(averaged_value, glove_sensors_vector_[index_joint]);
        glove_calibrated_positions.push_back(calibration_value);
      }

      publish_counter_index = 0;
      glove_positions.clear();


      applyJointMapping(glove_calibrated_positions, hand_positions);
      processJointZeros(hand_positions, hand_positions_no_J0);

      //Build and send the goal

      trajectory_goal_.trajectory.points.clear();

      trajectory_msgs::JointTrajectoryPoint trajectory_point = trajectory_msgs::JointTrajectoryPoint();
      trajectory_point.positions = hand_positions_no_J0;
      trajectory_point.velocities = std::vector<double>(trajectory_goal_.trajectory.joint_names.size(), 0.0);
      // We set the time from start to 10 ms, to allow some time to get there
      trajectory_point.time_from_start = ros::Duration (0.010);

      for (size_t i=0; i < trajectory_point.positions.size(); i++)
      {
        if(isnan(trajectory_point.positions[i]))
          return;
      }

      trajectory_goal_.trajectory.points.push_back(trajectory_point);

      action_client_->sendGoal(trajectory_goal_);
    }
    
    ros::spinOnce();
  }


  void CybergloveTrajectoryPublisher::applyJointMapping(const std::vector<double>& glove_postions, std::vector<double>& hand_positions )
  {
      //Do conversion
      std::vector<double> vect = map_calibration_parser->get_remapped_vector(glove_postions);

      //Process J4's
      getAbductionJoints(glove_postions, vect);

      for (size_t i=0; i < vect.size(); i++)
      {
        vect[i] = vect[i] * 0.017453292519943295; //convert degrees to radians
      }

      hand_positions = vect;
  }

  void CybergloveTrajectoryPublisher::processJointZeros(const std::vector<double>& postions_with_J0, std::vector<double>& postions_without_J0 )
  {
    for(unsigned int i = 0; i < postions_with_J0.size(); ++i )
    {
      if (joint_mapping_vector_[i][joint_mapping_vector_[i].size()-1] == '0')
      {
        postions_without_J0.push_back(postions_with_J0[i] / 2);
        postions_without_J0.push_back(postions_with_J0[i] / 2);
      }
      else
      {
        postions_without_J0.push_back(postions_with_J0[i]);
      }
    }
  }

  void CybergloveTrajectoryPublisher::getAbductionJoints(const std::vector<double>& glove_postions, std::vector<double>& hand_positions)
  {
    //Add the 3 abduction angles to have an idea of where the centre lies
    double ab_total = glove_postions[10] + glove_postions[14] +  glove_postions[18];

    //When trying to understand this code bear in mind that the abduction sign convention
    // in the shadow hand is the opposite for ff and mf than for rf and lf.
    if (ab_total/2 < glove_postions[10]) // If the centre lies between ff and mf
    {
      //FFJ4
      hand_positions[7] = -ab_total/2;
      //MFJ4
      hand_positions[10] = glove_postions[10] - ab_total/2;
      //RFJ4
      hand_positions[13] = -(glove_postions[14] + hand_positions[10]);
      //LFJ4
      hand_positions[16] = -glove_postions[18] + hand_positions[13];
    }
    else if (ab_total/2 < glove_postions[10] + glove_postions[14]) // If the centre lies between mf and rf
    {
      //MFJ4
      hand_positions[10] = -(ab_total/2 - glove_postions[10]);
      //FFJ4
      hand_positions[7] = -glove_postions[10] + hand_positions[10];
      //RFJ4
      hand_positions[13] = -(glove_postions[14] + hand_positions[10]);
      //LFJ4
      hand_positions[16] = -glove_postions[18] + hand_positions[13];
    }
    else // If the centre lies between rf and lf
    {
      //LFJ4
      hand_positions[16] = -ab_total/2;
      //RFJ4
      hand_positions[13] = glove_postions[18] + hand_positions[16];
      //MFJ4
      hand_positions[10] = -(glove_postions[14] + hand_positions[13]);
      //FFJ4
      hand_positions[7] = -glove_postions[10] + hand_positions[10];
    }
  }
}// end namespace



/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
