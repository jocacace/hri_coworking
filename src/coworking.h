/*
 * Copyright (C) 2017, Jonathan Cacace and Prisma Lab
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "helper/helper.h"
#include "TooN/TooN.h"
#include "TooN/LU.h"
#include "boost/thread.hpp"
#include "iiwa_kinematic/LBRiiwa.h"
#include "iiwa_msgs/JointPosition.h"
#include "ctrl_msgs/desired_command.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Wrench.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "human_contact_classification/classify.h"
#include "std_msgs/String.h"

using namespace std;
using namespace TooN;

class coworking {
public:
	coworking();
	void run();
	void cartesian_position_cb( geometry_msgs::PoseStamped pos );
	void cooperation();
	void fts_cb( geometry_msgs::Wrench wrench );
	void pdir_cb( geometry_msgs::Vector3 pdir_ );
	void hdir_cb( geometry_msgs::Vector3 hdir_ );
	void traj_dist( std_msgs::Float64 dist_ );
	void classification_result( std_msgs::String c_ );
	void trajectory_streaming_cb( std_msgs::Bool streaming_ );
	int choose_wp( vector< Vector<3> > eps );
	//void choose_wp( vector< Vector<3> > eps );

private:

	ros::NodeHandle _nh;
	ros::Subscriber _cartesian_pose_sub;
	ros::Subscriber _pdir_sub;
	ros::Subscriber _hdir_sub;
	ros::Subscriber _dist_sub;
	ros::Subscriber _fts_sub;
	ros::Subscriber _trajectory_streaming_sub;
	ros::Subscriber _classification_sub;
	ros::Publisher _marker_hdir_pub;
	ros::Publisher _hip_line_pub;
	ros::Publisher _hip_pts_pub;	
	ros::Publisher _marker_pdir_pub;

	ros::Publisher _pdir_pub;
	ros::Publisher _hdir_pub;
	ros::Publisher _dist_pub;
	ros::Publisher _fts_pub;
	ros::Publisher _wp_pub;
	ros::Publisher _shared_behaviour_pub;

	vector< Vector<3> > _epoints; //Endpoints
	
	Vector<3> _pdir;
	Vector<3> _hdir;
	Vector<3> _force;

	bool _tstreaming;
	bool _first_cpose;
	Vector<3> _mis_p;
	Vector<4> _mis_o;
	int _pindex;
	int _apindex;
	float _tdist;
  ros::ServiceClient _hri_classification_client;
  string _class;

  int _last_wp_index;
};