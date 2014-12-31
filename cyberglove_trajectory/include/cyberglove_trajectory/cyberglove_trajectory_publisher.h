/**
 * @file   cyberglove_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
 *
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
 * @brief The goal of this ROS publisher is to publish raw and calibrated
 * joint positions from the cyberglove at a regular time interval. We're
 * oversampling to get a better accuracy on our data.
 * To publish those data, just call the publish()
 * function.
 *
 *
 */

#ifndef   	CYBERGLOVE_TRAJECTORY_PUBLISHER_H_
# define   	CYBERGLOVE_TRAJECTORY_PUBLISHER_H_

#include <ros/ros.h>
#include <vector>
#include <boost/smart_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "cyberglove/serial_glove.hpp"

//messages
#include <sensor_msgs/JointState.h>
#include <sr_utilities/calibration.hpp>
#include <sr_utilities/thread_safe_map.hpp>
#include "sr_remappers/calibration_parser.h"

using namespace ros;

namespace cyberglove{
  class CybergloveTrajectoryPublisher
  {
  public:
    /// Constructor
    CybergloveTrajectoryPublisher();

    /// Destructor
    ~CybergloveTrajectoryPublisher();

    Publisher cyberglove_pub;

    typedef threadsafe::Map<boost::shared_ptr<shadow_robot::JointCalibration> > CalibrationMap;


    /**
     * Reads the calibration from the parameter server.
     *
     *
     * @return a calibration map
     */
    CalibrationMap read_joint_calibration();

    bool isPublishing();
    void setPublishing(bool value);
  private:
    /////////////////
    //  CALLBACKS  //
    /////////////////

    //ros node handle
    NodeHandle node, n_tilde;
    unsigned int publish_counter_max, publish_counter_index;

    ///the actual connection with the cyberglove is done here.
    boost::shared_ptr<CybergloveSerial> serial_glove;

    /**
     * The callback function: called each time a full message
     * is received. This function is bound to the serial_glove
     * object using boost::bind.
     *
     * @param glove_pos A vector containing the current raw joints positions.
     * @param light_on true if the light is on, false otherwise.
     */
    void glove_callback(std::vector<float> glove_pos, bool light_on);

    std::string path_to_glove;
    bool publishing;

    /// The map used to calibrate each joint.
    boost::shared_ptr<CalibrationMap> calibration_map;
    /// A temporary calibration for a given joint.
    boost::shared_ptr<shadow_robot::JointCalibration> calibration_tmp;
    ///the calibration parser containing the mapping matrix
    boost::scoped_ptr<CalibrationParser> map_calibration_parser;

    Publisher cyberglove_raw_pub;
    sensor_msgs::JointState jointstate_msg;


    std::vector<float> calibration_values;

    std::vector<std::vector<float> > glove_positions;


    void applyJointMapping(const std::vector<double>& glove_postions, std::vector<double>& hand_positions );
    void processJointZeros(const std::vector<double>& postions_with_J0, std::vector<double>& postions_without_J0 );

    /**
     * process the joint_states callback for the finger abductions: processes the message from the cyberglove node, remap it to the Dextrous hand J4s
     * It overwrites whatever was written for the J4s by the calibration parser get_remapped_vector
     *
     * @param glove_postions The positions that come from the glove sensors (usually calibrated)
     * @param hand_positions the vector where the result is written (only J4s are written)
     */
    void getAbductionJoints( const std::vector<double>& glove_postions, std::vector<double>& hand_positions);

    static const std::vector<std::string> joint_name_vector_;
    static const std::vector<std::string> joint_mapping_vector_;
    static const std::vector<std::string> glove_sensors_vector_;

    boost::scoped_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > action_client_;
    control_msgs::FollowJointTrajectoryGoal trajectory_goal_;

    std::string cyberglove_version_;

    ros::Duration trajectory_delay_;
  }; // end class CybergloveTrajectoryPublisher

} // end namespace
#endif 	    /* !CYBERGLOVE_PUBLISHER_H_ */

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
