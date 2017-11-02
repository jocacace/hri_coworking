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
#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/package.h>

using namespace std;
using namespace TooN;

class hrc_validation {
public:
	hrc_validation();
	void run();
	void Init();


	float emg_norm_d;
	Vector<9> dcmd;
	Vector<3> force;
	Vector<3> apoint;
	Vector<3> cmd_pose;
	float dtraj;
	bool enable_stiff;
	float angle;
	Vector<3> mis_p;
	Vector<3> xd_data;
	Vector<3> xc_data;
	string behaviour_d;
	Vector<3> target_d;
	bool streaming_data;

	void emg_norm_cb(std_msgs::Float64 emg_norm_ ); 			// /MYO/myo_emg_norm
	void carteisan_cmd_cb(ctrl_msgs::desired_command dcmd_);  // /cartesian_commad
	void fts_data_cb(geometry_msgs::Wrench w_); //fts_data
	void attraction_point_cb(geometry_msgs::Point); //iiwa/attraction_point	
	//void c_point_cb, //iiwa/c_point
	void cartesian_cmdpose_cb(geometry_msgs::PoseStamped c_pose_);	 //iiwa/command/CartesianPose
	void dist_from_traj_cb(std_msgs::Float64 dtraj_); //iiwa/dist_from_traj
	void eef_dx_cb(geometry_msgs::Twist dx_); //iiwa/eef_dx
	void eef_h_direction_cb(geometry_msgs::Vector3 hdir_); //iiwa/eef_h_direction
	void eef_p_direction_cb(geometry_msgs::Vector3 pdir_); //iiwa/eef_p_direction
	void enable_stiffness_cb(std_msgs::Bool enbale_stiff_); //iiwa/enable_stiffness
	//void hdir_cb, //iiwa/hdir
	//void pdir_cb, //iiwa/pdir
	void shared_angle_cb(std_msgs::Float64 angle_); //iiwa/shared/angle
	void state_cartesian_pose_cb(geometry_msgs::PoseStamped c_pose_); //iiwa/state/CartesianPose
	void state_joint_position_cb(iiwa_msgs::JointPosition jpos_); //iiwa/state/JointPosition
	//void way_point_cb, //iiwa/way_point
	void xc_cb(geometry_msgs::Pose xc_); //iiwa/xc
	void xd_cb(geometry_msgs::Pose xd_); //iiwa/xd
	void shared_baheviour_cb(std_msgs::String behaviour_); //shared/baheviour
	void trajectory_planner_target_cb(geometry_msgs::Pose target_); //trajectory_planner/target
	void trajectory_streaming_cb(std_msgs::Bool streaming_); //trajectory_planner/trajectory_streaming

	void write_data();

private:
	ros::NodeHandle _nh;

	ros::Subscriber emg_norm_sub, 			// /MYO/myo_emg_norm
									carteisan_cmd_sub,  // /cartesian_commad
									fts_data_sub, //fts_data
									attraction_point_sub, //iiwa/attraction_point	
									c_point_sub, //iiwa/c_point
									cartesian_cmdpose_sub, //iiwa/command/CartesianPose
									dist_from_traj_sub, //iiwa/dist_from_traj
									eef_dx_sub, //iiwa/eef_dx
									eef_h_direction_sub, //iiwa/eef_h_direction
									eef_p_direction_sub, //iiwa/eef_p_direction
									enable_stiffness_sub, //iiwa/enable_stiffness
									hdir_sub, //iiwa/hdir
									pdir_sub, //iiwa/pdir
									shared_angle_sub, //iiwa/shared/angle
									state_cartesian_pose_sub, //iiwa/state/CartesianPose
									state_joint_position_sub, //iiwa/state/JointPosition
									way_point_sub, //iiwa/way_point
									xc_sub, //iiwa/xc
									xd_sub, //iiwa/xd
									shared_baheviour_sub, //shared/baheviour
									trajectory_planner_target_sub, //trajectory_planner/target
									trajectory_streaming_sub; //trajectory_planner/trajectory_streaming

  ofstream 				emg_norm_file, 			// /MYO/myo_emg_norm
									carteisan_cmd_file,  // /cartesian_commad
									fts_data_file, //fts_data
									attraction_point_file, //iiwa/attraction_point	
									c_point_file, //iiwa/c_point
									cartesian_cmdpose_file, //iiwa/command/CartesianPose
									dist_from_traj_file, //iiwa/dist_from_traj
									eef_dx_file, //iiwa/eef_dx
									eef_h_direction_file, //iiwa/eef_h_direction
									eef_p_direction_file, //iiwa/eef_p_direction
									enable_stiffness_file, //iiwa/enable_stiffness
									hdir_file, //iiwa/hdir
									pdir_file, //iiwa/pdir
									shared_angle_file, //iiwa/shared/angle
									state_cartesian_pose_file, //iiwa/state/CartesianPose
									state_joint_position_file, //iiwa/state/JointPosition
									way_point_file, //iiwa/way_point
									xc_file, //iiwa/xc
									xd_file, //iiwa/xd
									shared_baheviour_file, //shared/baheviour
									trajectory_planner_target_file, //trajectory_planner/target
									trajectory_streaming_file; //trajectory_planner/trajectory_streaming



									
};


hrc_validation::hrc_validation() {


	emg_norm_sub = _nh.subscribe("/MYO/myo_emg_norm", 0, &hrc_validation::emg_norm_cb, this); 			// 
	carteisan_cmd_sub = _nh.subscribe("/cartesian_commad", 0, &hrc_validation::carteisan_cmd_cb, this);  // 
	fts_data_sub = _nh.subscribe("/fts_data", 0, &hrc_validation::fts_data_cb, this); //
	attraction_point_sub = _nh.subscribe("/iiwa/attraction_point", 0, &hrc_validation::attraction_point_cb, this); //	
	cartesian_cmdpose_sub = _nh.subscribe("/iiwa/command/CartesianPose", 0, &hrc_validation::cartesian_cmdpose_cb, this); //
	dist_from_traj_sub = _nh.subscribe("/iiwa/dist_from_traj", 0, &hrc_validation::dist_from_traj_cb, this); //
	eef_dx_sub = _nh.subscribe("/iiwa/eef_dx", 0, &hrc_validation::eef_dx_cb, this); //
	eef_h_direction_sub = _nh.subscribe("/iiwa/eef_h_direction", 0, &hrc_validation::eef_h_direction_cb, this); //
	eef_p_direction_sub = _nh.subscribe("/iiwa/eef_p_direction", 0, &hrc_validation::eef_p_direction_cb, this); //
	enable_stiffness_sub = _nh.subscribe("/iiwa/enable_stiffness", 0, &hrc_validation::enable_stiffness_cb	, this); //
	shared_angle_sub = _nh.subscribe("/iiwa/shared/angle", 0, &hrc_validation::shared_angle_cb, this); //
	state_cartesian_pose_sub = _nh.subscribe("iiwa/state/CartesianPose", 0, &hrc_validation::state_cartesian_pose_cb, this); //
	state_joint_position_sub = _nh.subscribe("iiwa/state/JointPosition", 0, &hrc_validation::state_joint_position_cb, this); //	
	xc_sub = _nh.subscribe("/iiwa/xc", 0, &hrc_validation::xc_cb, this); //
	xd_sub = _nh.subscribe("/iiwa/xd", 0, &hrc_validation::xd_cb, this); //
	shared_baheviour_sub = _nh.subscribe("/shared/baheviour", 0, &hrc_validation::shared_baheviour_cb, this); //
	trajectory_planner_target_sub = _nh.subscribe("/trajectory_planner/target", 0, &hrc_validation::trajectory_planner_target_cb, this); //
	trajectory_streaming_sub = _nh.subscribe( "trajectory_planner/trajectory_streaming", 0, &hrc_validation::trajectory_streaming_cb, this); //



}

void hrc_validation::Init() {

	string line;
	cout << "Insert session name!" << endl;
	getline( cin, line);

	string hri_pkg_path = ros::package::getPath("hri_coworking");
	hri_pkg_path += "/script/" + line;


	boost::filesystem::path dir(helper::string2char( hri_pkg_path) );

	if(boost::filesystem::create_directory(dir)) {
		
	}
	

	emg_norm_file.open( helper::string2char( hri_pkg_path + "/emg_norm") );
	carteisan_cmd_file.open( helper::string2char( hri_pkg_path + "/carteisan_cmd") );
	fts_data_file.open( helper::string2char( hri_pkg_path + "/fts_data") );
	attraction_point_file.open( helper::string2char( hri_pkg_path + "/attraction_point") );
	
	cartesian_cmdpose_file.open( helper::string2char( hri_pkg_path + "/cartesian_cmdpose") );
	dist_from_traj_file.open( helper::string2char( hri_pkg_path + "/dist_from_traj") );
	eef_dx_file.open( helper::string2char( hri_pkg_path + "/eef_dx") );
	eef_h_direction_file.open( helper::string2char( hri_pkg_path + "/eef_h_direction") );
	eef_p_direction_file.open( helper::string2char( hri_pkg_path + "/eef_p_direction") );
	enable_stiffness_file.open( helper::string2char( hri_pkg_path + "/enable_stiffness") );
	
	shared_angle_file.open( helper::string2char( hri_pkg_path + "/shared_angle") );
	state_cartesian_pose_file.open( helper::string2char( hri_pkg_path + "/state_cartesian_pose") );
	state_joint_position_file.open( helper::string2char( hri_pkg_path + "/state_joint_position") );
	way_point_file.open( helper::string2char( hri_pkg_path + "/way_point") );
	xc_file.open( helper::string2char( hri_pkg_path + "/xc") );
	xd_file.open( helper::string2char( hri_pkg_path + "/xd") );
	shared_baheviour_file.open( helper::string2char( hri_pkg_path + "/shared_baheviour") );
	trajectory_planner_target_file.open( helper::string2char( hri_pkg_path + "/trajectory_planner_target") );
	trajectory_streaming_file.open( helper::string2char( hri_pkg_path + "/trajectory_streaming") );
	
}



void hrc_validation::emg_norm_cb(std_msgs::Float64 emg_norm_ ) {

	emg_norm_d = emg_norm_.data;
	

}
void hrc_validation::carteisan_cmd_cb(ctrl_msgs::desired_command dcmd_) {
	


	dcmd = makeVector( dcmd_.xd.position.x,  dcmd_.xd.position.y,  dcmd_.xd.position.z, 
										 dcmd_.dxd.linear.x,  dcmd_.dxd.linear.y,  dcmd_.dxd.linear.z, 
										 dcmd_.ddxd.linear.x,  dcmd_.ddxd.linear.y,  dcmd_.ddxd.linear.z );


}
void hrc_validation::fts_data_cb(geometry_msgs::Wrench w_) {
	
	force = makeVector( w_.force.x, w_.force.y, w_.force.z );
	
	
}
void hrc_validation::attraction_point_cb(geometry_msgs::Point p_) {
	

	apoint = makeVector( p_.x, p_.y, p_.z );
	
}

void hrc_validation::cartesian_cmdpose_cb(geometry_msgs::PoseStamped c_pose_) {
	
	cmd_pose = makeVector(c_pose_.pose.position.x,  c_pose_.pose.position.y, c_pose_.pose.position.z );
	
}
void hrc_validation::dist_from_traj_cb(std_msgs::Float64 dtraj_) {
	
	dtraj = dtraj_.data;

	

}
void hrc_validation::eef_dx_cb(geometry_msgs::Twist dx_) {
	

}
void hrc_validation::eef_h_direction_cb(geometry_msgs::Vector3 hdir_) {
	
}
void hrc_validation::eef_p_direction_cb(geometry_msgs::Vector3 pdir_) {
	
}
void hrc_validation::enable_stiffness_cb(std_msgs::Bool enable_stiff_) {
	enable_stiff = enable_stiff_.data;
}


void hrc_validation::shared_angle_cb(std_msgs::Float64 angle_) {
	
	angle = angle_.data;
	
}
void hrc_validation::state_cartesian_pose_cb(geometry_msgs::PoseStamped c_pose_) {
	mis_p = makeVector( c_pose_.pose.position.x, c_pose_.pose.position.y, c_pose_.pose.position.z );

}
void hrc_validation::state_joint_position_cb(iiwa_msgs::JointPosition jpos_) {
	
}

void hrc_validation::xc_cb(geometry_msgs::Pose xc_) {
		
	xc_data = makeVector( xc_.position.x, xc_.position.y, xc_.position.z );
	
}
void hrc_validation::xd_cb(geometry_msgs::Pose xd_) {
	xd_data = makeVector( xd_.position.x, xd_.position.y, xd_.position.z );
}
void hrc_validation::shared_baheviour_cb(std_msgs::String behaviour_) {
	behaviour_d = behaviour_.data;
}
void hrc_validation::trajectory_planner_target_cb(geometry_msgs::Pose target_) {
	
	target_d = makeVector( target_.position.x, target_.position.y, target_.position.z );
	
}
void hrc_validation::trajectory_streaming_cb(std_msgs::Bool streaming_) {
	streaming_data = streaming_.data;
}



void hrc_validation::write_data( ) {


	ros::Rate r(100);

	//Wait data
	while( norm(force) == 0.0 ) 
		usleep(0.1*1e6);

	float tc = 1.0/100.0;
	float time = 0;

	while( ros::ok() ) {

		emg_norm_file  << time << " "			<< emg_norm_d << endl;
		carteisan_cmd_file  << time << " " <<dcmd << endl;
		fts_data_file  			<< time << " " <<force << endl;
		attraction_point_file << time << " " <<apoint << endl;
		
		cartesian_cmdpose_file << time << " " << cmd_pose << endl;
		dist_from_traj_file << time << " " <<dtraj << endl;
		enable_stiffness_file << time << " " <<enable_stiff << endl;
		
		shared_angle_file << time << " " <<	angle << endl;
		state_cartesian_pose_file << time << " " << mis_p << endl;
		
		xc_file << time << " " <<xc_data << endl;
		xd_file << time << " " << xd_data << endl;

		if( behaviour_d == "" || behaviour_d == "no_contact" ) {
			shared_baheviour_file << time << " " << 0 << endl;
		}
		else if (behaviour_d == "opposite") {
			shared_baheviour_file << time << " " << 2 << endl;
		}
		else if (behaviour_d == "concorde") {
			shared_baheviour_file << time << " " << 1 << endl;
		}
		else if (behaviour_d == "deviation_o") {
			shared_baheviour_file << time << " " << 4 << endl;
		}
		else if (behaviour_d == "deviation_c") {
			shared_baheviour_file << time << " " << 3 << endl;
		}
		
		trajectory_planner_target_file << time << " " <<target_d << endl;
		trajectory_streaming_file <<time << " " <<streaming_data << endl;

		time += tc;

		r.sleep();
	}

}

void hrc_validation::run() {


	Init();


	boost::thread write_data_t(&hrc_validation::write_data, this );
	ros::spin();

}

int main(int argc, char** argv) {
	ros::init( argc, argv, "hrc_validation");

	hrc_validation validation;	
	validation.run();

	return 0;
}
