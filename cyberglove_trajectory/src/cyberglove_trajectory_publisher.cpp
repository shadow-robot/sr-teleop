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
#include <sr_utilities/sr_math_utils.hpp>
#include <std_srvs/Empty.h>

using namespace ros;

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

    calibration_map.reset(new CalibrationMap(read_joint_calibration()));

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

    cyberglove_raw_pub = n_tilde.advertise<sensor_msgs::JointState>("raw/joint_states", 2);
    cyberglove_cal_pub = n_tilde.advertise<sensor_msgs::JointState>("calibrated/joint_states", 2);

    reload_calibration_service = n_tilde.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
      "reload_calibration", boost::bind(&CybergloveTrajectoryPublisher::reload_calibration, this, _1, _2));

    //initialises joint names (the order is important)
    raw_jointstate_msg.name.push_back("G_ThumbRotate");
    raw_jointstate_msg.name.push_back("G_ThumbMPJ");
    raw_jointstate_msg.name.push_back("G_ThumbIJ");
    raw_jointstate_msg.name.push_back("G_ThumbAb");
    raw_jointstate_msg.name.push_back("G_IndexMPJ");
    raw_jointstate_msg.name.push_back("G_IndexPIJ");
    raw_jointstate_msg.name.push_back("G_IndexDIJ");
    raw_jointstate_msg.name.push_back("G_MiddleMPJ");
    raw_jointstate_msg.name.push_back("G_MiddlePIJ");
    raw_jointstate_msg.name.push_back("G_MiddleDIJ");
    raw_jointstate_msg.name.push_back("G_MiddleIndexAb");
    raw_jointstate_msg.name.push_back("G_RingMPJ");
    raw_jointstate_msg.name.push_back("G_RingPIJ");
    raw_jointstate_msg.name.push_back("G_RingDIJ");
    raw_jointstate_msg.name.push_back("G_RingMiddleAb");
    raw_jointstate_msg.name.push_back("G_PinkieMPJ");
    raw_jointstate_msg.name.push_back("G_PinkiePIJ");
    raw_jointstate_msg.name.push_back("G_PinkieDIJ");
    raw_jointstate_msg.name.push_back("G_PinkieRingAb");
    raw_jointstate_msg.name.push_back("G_PalmArch");
    raw_jointstate_msg.name.push_back("G_WristPitch");
    raw_jointstate_msg.name.push_back("G_WristYaw");

    cal_jointstate_msg.name.push_back("G_ThumbRotate");
    cal_jointstate_msg.name.push_back("G_ThumbMPJ");
    cal_jointstate_msg.name.push_back("G_ThumbIJ");
    cal_jointstate_msg.name.push_back("G_ThumbAb");
    cal_jointstate_msg.name.push_back("G_IndexMPJ");
    cal_jointstate_msg.name.push_back("G_IndexPIJ");
    cal_jointstate_msg.name.push_back("G_IndexDIJ");
    cal_jointstate_msg.name.push_back("G_MiddleMPJ");
    cal_jointstate_msg.name.push_back("G_MiddlePIJ");
    cal_jointstate_msg.name.push_back("G_MiddleDIJ");
    cal_jointstate_msg.name.push_back("G_MiddleIndexAb");
    cal_jointstate_msg.name.push_back("G_RingMPJ");
    cal_jointstate_msg.name.push_back("G_RingPIJ");
    cal_jointstate_msg.name.push_back("G_RingDIJ");
    cal_jointstate_msg.name.push_back("G_RingMiddleAb");
    cal_jointstate_msg.name.push_back("G_PinkieMPJ");
    cal_jointstate_msg.name.push_back("G_PinkiePIJ");
    cal_jointstate_msg.name.push_back("G_PinkieDIJ");
    cal_jointstate_msg.name.push_back("G_PinkieRingAb");
    cal_jointstate_msg.name.push_back("G_PalmArch");
    cal_jointstate_msg.name.push_back("G_WristPitch");
    cal_jointstate_msg.name.push_back("G_WristYaw");


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

    //Get the cyberglove version '2' or '3'
    n_tilde.param("cyberglove_version", cyberglove_version_, std::string("2"));
    ROS_INFO("Cyberglove version: %s", cyberglove_version_.c_str());

    //Get the cyberglove streaming protocol '8bit' or '16bit'
    n_tilde.param("streaming_protocol", streaming_protocol_, std::string("8bit"));
    ROS_INFO("Streaming protocol: %s", streaming_protocol_.c_str());

    // set path to glove
    n_tilde.param("path_to_glove", path_to_glove, std::string("/dev/ttyS0"));
    ROS_INFO("Opening glove on port: %s", path_to_glove.c_str());

    //set trajectory tx delay: the delay it takes to get to the trajectory controller.
    // it is used to set the timestamp of the trajectory goal. 10ms default
    // (the trajectory point might be discarded if the trajectory arrives later)
    double delay;
    n_tilde.param("trajectory_tx_delay", delay, 0.01);
    trajectory_tx_delay_ = ros::Duration(delay);

    //set trajectory delay: the delay from the trajectory beginning to the trajectory point.
    // it is used to set the time_from start of the single trajectory point. 2ms default
    // (it can be very small, but not zero, or the point will be discarded as past).
    n_tilde.param("trajectory_delay", delay, 0.002);
    trajectory_delay_ = ros::Duration(delay);

    //initialize the connection with the cyberglove and binds the callback function
    serial_glove = boost::shared_ptr<CybergloveSerial>(new CybergloveSerial(path_to_glove, cyberglove_version_, streaming_protocol_, boost::bind(&CybergloveTrajectoryPublisher::glove_callback, this, _1, _2)));

    int res = -1;
    if(cyberglove_version_ == "2")
    {
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

      //We want the glove to transmit the status (light on/off)
      res = serial_glove->set_transmit_info(true);
    }

    // Should the glove filter the data? (it leads to less smooth movements, but quieter behaviour on the motors)
    bool filtering;
    n_tilde.param("filter", filtering, false);
    std::string filt_msg(filtering?"ON":"OFF");
    ROS_INFO("Filtering: %s", filt_msg.c_str());
    res = serial_glove->set_filtering(filtering);

    //start reading the data.
    res = serial_glove->start_stream();
  }

  CybergloveTrajectoryPublisher::~CybergloveTrajectoryPublisher()
  {
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

  bool CybergloveTrajectoryPublisher::reload_calibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    calibration_map.reset(new CalibrationMap(read_joint_calibration()));
    return true;
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

      raw_jointstate_msg.position.clear();
      raw_jointstate_msg.header.stamp = ros::Time::now();
      cal_jointstate_msg.position.clear();
      cal_jointstate_msg.header.stamp = ros::Time::now();


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

	calibration_tmp = calibration_map->find(glove_sensors_vector_[index_joint]);
	double calibration_value = calibration_tmp->compute(static_cast<double> (averaged_value));

        raw_jointstate_msg.position.push_back(averaged_value);
        cal_jointstate_msg.position.push_back(calibration_value);
        glove_calibrated_positions.push_back(calibration_value);
      }
      cyberglove_raw_pub.publish(raw_jointstate_msg);
      cyberglove_cal_pub.publish(cal_jointstate_msg);


      publish_counter_index = 0;
      glove_positions.clear();

      applyJointMapping(glove_calibrated_positions, hand_positions);
      processJointZeros(hand_positions, hand_positions_no_J0);

      //Build and send the goal

      trajectory_goal_.trajectory.points.clear();
      //WARNING if this node runs on a different machine from the trajectory controller, both machines will need to be synchronized
      // chrony (sudo apt-get install crony) has been used successfully to achieve that
      // The extra 10ms will allow time for the trajectory to get to the trajectory controller
      trajectory_goal_.trajectory.header.stamp = ros::Time::now() + trajectory_tx_delay_;

      trajectory_msgs::JointTrajectoryPoint trajectory_point = trajectory_msgs::JointTrajectoryPoint();
      trajectory_point.positions = hand_positions_no_J0;
      // We set the time from start to 10 ms, to allow some time for the hand to get there
      trajectory_point.time_from_start = trajectory_delay_;

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
    double middleIndexAb = glove_postions[10];
    double ringMiddleAb = glove_postions[14];
    double pinkieRingAb = glove_postions[18];

    // if the abduction sensors are less than 0, it is an artifact of the calibration (we don't want to consider anything smaller than 0 for these sensors)
    if (middleIndexAb < 0.0)
      middleIndexAb = 0.0;
    if (ringMiddleAb < 0.0)
      ringMiddleAb = 0.0;
    if (pinkieRingAb < 0.0)
      pinkieRingAb = 0.0;
    //Add the 3 abduction angles to have an idea of where the centre lies
    double ab_total = middleIndexAb + ringMiddleAb +  pinkieRingAb;

    //When trying to understand this code bear in mind that the abduction sign convention
    // in the shadow hand is the opposite for ff and mf than for rf and lf.
    if (ab_total/2 < middleIndexAb) // If the centre lies between ff and mf
    {
      //FFJ4
      hand_positions[7] = -ab_total/2;
      //MFJ4
      hand_positions[10] = middleIndexAb - ab_total/2;
      //RFJ4
      hand_positions[13] = -(ringMiddleAb + hand_positions[10]);
      //LFJ4
      hand_positions[16] = -pinkieRingAb + hand_positions[13];
    }
    else if (ab_total/2 < middleIndexAb + ringMiddleAb) // If the centre lies between mf and rf
    {
      //MFJ4
      hand_positions[10] = -(ab_total/2 - middleIndexAb);
      //FFJ4
      hand_positions[7] = -middleIndexAb + hand_positions[10];
      //RFJ4
      hand_positions[13] = -(ringMiddleAb + hand_positions[10]);
      //LFJ4
      hand_positions[16] = -pinkieRingAb + hand_positions[13];
    }
    else // If the centre lies between rf and lf
    {
      //LFJ4
      hand_positions[16] = -ab_total/2;
      //RFJ4
      hand_positions[13] = pinkieRingAb + hand_positions[16];
      //MFJ4
      hand_positions[10] = -(ringMiddleAb + hand_positions[13]);
      //FFJ4
      hand_positions[7] = -middleIndexAb + hand_positions[10];
    }
  }

CybergloveTrajectoryPublisher::CalibrationMap CybergloveTrajectoryPublisher::read_joint_calibration()
{
  CalibrationMap joint_calibration;

  XmlRpc::XmlRpcValue calib;
  n_tilde.getParam("cyberglove_calibration", calib);
  ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //iterate on all the joints
  for (int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
  {
    //check the calibration is well formatted:
    // first joint name, then calibration table
    ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

    string joint_name = static_cast<string> (calib[index_cal][0]);
    vector<joint_calibration::Point> calib_table_tmp;

    //now iterates on the calibration table for the current joint
    for (int32_t index_table = 0; index_table < calib[index_cal][1].size(); ++index_table)
    {
      ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
      //only 2 values per calibration point: raw and calibrated (doubles)
      ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
      ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


      joint_calibration::Point point_tmp;
      point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
      point_tmp.calibrated_value = sr_math_utils::to_rad(static_cast<double> (calib[index_cal][1][index_table][1]));
      calib_table_tmp.push_back(point_tmp);
    }

    joint_calibration.insert(joint_name, boost::shared_ptr<shadow_robot::JointCalibration>(new shadow_robot::JointCalibration(calib_table_tmp)));
  }

  return joint_calibration;
} //end read_joint_calibration

}// end namespace



/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
