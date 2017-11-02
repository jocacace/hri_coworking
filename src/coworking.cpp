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

#include "coworking.h"


visualization_msgs::Marker create_marker(Vector<3> pi, Vector<3> pf, float rgba[4] ) {


	visualization_msgs::Marker arrow;
	arrow.header.frame_id =  "/world";
	arrow.ns = "eef_path";
	arrow.action = visualization_msgs::Marker::ADD;
	

	arrow.id = 2;

	arrow.type = visualization_msgs::Marker::ARROW;

	arrow.scale.x = 0.01;
	arrow.scale.y = 0.01;
	arrow.scale.z = 0.05;
	
	arrow.color.r = rgba[0];
	arrow.color.g = rgba[1];
	arrow.color.b = rgba[2];
	arrow.color.a = rgba[3];

	geometry_msgs::Point p;
	p.x = pi[0];
	p.y = pi[1];
	p.z = pi[2];
	arrow.points.push_back(p);
	
	p.x = pf[0];
	p.y = pf[1];
	p.z = pf[2];
	arrow.points.push_back(p);
	
	return arrow;
	 

}

void create_marker(Vector<3> pi, Vector<3> pf, float rgba[4], int id, visualization_msgs::Marker & line, visualization_msgs::Marker & point ) {


	
	line.header.frame_id = point.header.frame_id = "/world";
	line.ns = point.ns = "eef_path";
	point.action = visualization_msgs::Marker::ADD;
	line.action = visualization_msgs::Marker::ADD;
	line.pose.orientation.w = point.pose.orientation.w = 1.0;

	point.id = 2+id;
	line.id = 2+id+1;

	point.type = visualization_msgs::Marker::POINTS;
	line.type = visualization_msgs::Marker::LINE_STRIP;

	line.lifetime = ros::Duration(1/10.0);
	point.lifetime = ros::Duration(1/10.0);

	point.scale.x = 0.05;
	point.scale.y = 0.05;
	point.scale.z = 0.05;
	point.pose.orientation.w = 1.0;
	line.scale.x = 0.01;
	

	point.color.b = 1.0f;
	point.color.a = 1.0;

	line.color.r = rgba[0];
	line.color.g = rgba[1]; 
	line.color.b = rgba[2];
	line.color.a = rgba[3];


	geometry_msgs::Point p;
	p.x = pi[0];
	p.y = pi[1];
	p.z = pi[2];
	line.points.push_back(p);
	point.points.push_back(p);

	p.x = pf[0];
	p.y = pf[1];
	p.z = pf[2];
	line.points.push_back(p);
	point.points.push_back(p);
	
	//_marker_line_pub.publish(point);
	//_marker_line_pub.publish(line);
	
	
}

//Select the correct behaviour for an effective human robot collaboration
coworking::coworking() {

	_cartesian_pose_sub 				= _nh.subscribe("/iiwa/state/CartesianPose", 0, &coworking::cartesian_position_cb, this);
	_pdir_sub 									= _nh.subscribe("/iiwa/eef_p_direction", 0, &coworking::pdir_cb, this);
	_hdir_sub 									= _nh.subscribe("/iiwa/eef_h_direction", 0, &coworking::hdir_cb, this);
	_fts_sub 										= _nh.subscribe("/fts_data", 0, &coworking::fts_cb, this);	
	_dist_sub 									= _nh.subscribe("/iiwa/dist_from_traj", 0, &coworking::traj_dist, this);
	_classification_sub					= _nh.subscribe("/nn_classification/class", 0, &coworking::classification_result, this);
	_trajectory_streaming_sub 	= _nh.subscribe("/trajectory_planner/trajectory_streaming", 0, &coworking::trajectory_streaming_cb, this);
	_pdir_pub 									= _nh.advertise<geometry_msgs::Vector3>("/pdir", 0);
	_hdir_pub 									= _nh.advertise<geometry_msgs::Vector3>("/hdir", 0);
	_fts_pub 										= _nh.advertise<geometry_msgs::Wrench>("/human_force", 0);
	_dist_pub 									= _nh.advertise<std_msgs::Float64>("/tdist", 0);
	_wp_pub 										= _nh.advertise<geometry_msgs::Pose>("/trajectory_planner/target", 0);
	_shared_behaviour_pub 			= _nh.advertise<std_msgs::String>("/shared/baheviour", 0 );

	_hip_pts_pub								= _nh.advertise<visualization_msgs::Marker>("/coworking/hip_pts", 0);
	_hip_line_pub								= _nh.advertise<visualization_msgs::Marker>("/coworking/hip_line", 0);

	_marker_hdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/hdir", 0);
	_marker_pdir_pub    				= _nh.advertise<visualization_msgs::Marker>("/iiwa/pdir", 0);

	_hri_classification_client  = _nh.serviceClient<human_contact_classification::classify>("/human_contact_classification/classify");
	
	Vector<3> p;
	p[0] =  0.63;
	p[1] = -0.25;
	p[2] =  0.45;
	_epoints.push_back( p );

	p[0] =  0.63;
	p[1] = 	0.25;
	p[2] =  0.45;
	_epoints.push_back( p );


	/*
	Vector<3> p;
	p[0] =  0.38;
	p[1] = -0.25;
	p[2] =  0.45;
	_epoints.push_back( p );

	p[0] =  0.68;
	p[1] = -0.25;
	p[2] =  0.45;
	_epoints.push_back( p );

	p[0] =  0.40;
	p[1] =  0.30;
	p[2] =  0.45;
	_epoints.push_back( p );

	
	p[0] =  0.64;
	p[1] =  0.15;
	p[2] =  0.45;
	_epoints.push_back( p );
	*/

	_first_cpose = false;
	_mis_p = Zeros;
	_mis_o = Zeros;

	_tstreaming = false;
	_last_wp_index = -1;
	_pindex = -1;
	_apindex = -1;

}

void coworking::trajectory_streaming_cb( std_msgs::Bool streaming_ ) {
	_tstreaming = streaming_.data;
}

void coworking::classification_result( std_msgs::String c_ ) {
	_class = c_.data;
}

void coworking::fts_cb( geometry_msgs::Wrench wrench ) {
	_force = makeVector( wrench.force.x, wrench.force.y, wrench.force.z );
}

void coworking::pdir_cb( geometry_msgs::Vector3 pdir_ ) {
	_pdir = makeVector( pdir_.x, pdir_.y, pdir_.z );

	float rgba[4];
	rgba[0] = 1;
	rgba[1] = 0;
	rgba[2] = 0;
	rgba[3] = 1;
	
	Vector<3> pdir_th = Zeros;
	pdir_th = _pdir * 2.0;	
	if ( norm ( _pdir ) > 0.0 ) {
		//publish pdir marker
		visualization_msgs::Marker parrow = create_marker( _mis_p, pdir_th + _mis_p, rgba);
		_marker_pdir_pub.publish( parrow );
	}
}

void coworking::traj_dist( std_msgs::Float64 dist_ ) {
	_tdist = dist_.data;

}

void coworking::hdir_cb( geometry_msgs::Vector3 hdir_ ) {
	_hdir = makeVector( hdir_.x, hdir_.y, hdir_.z );

	float rgba[4];
	rgba[0] = 0;
	rgba[1] = 1;
	rgba[2] = 0;
	rgba[3] = 1;
	
	Vector<3> hdir_th = Zeros;
	hdir_th = _hdir * 0.2;	
	
	if ( norm ( _hdir ) > 0.0 ) {
		//publish pdir marker
		visualization_msgs::Marker harrow = create_marker( _mis_p, hdir_th + _mis_p, rgba);
		_marker_hdir_pub.publish( harrow );
	}
}

void coworking::cartesian_position_cb( geometry_msgs::PoseStamped pos ) {
	_mis_p = makeVector( pos.pose.position.x, pos.pose.position.y, pos.pose.position.z );
	_mis_o = makeVector( pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z );	
	_first_cpose = true;
}

int coworking::choose_wp( vector< Vector<3> >  epoints ) {
//void coworking::choose_wp( vector< Vector<3> >  epoints ) {

	float rgba[4];
	
	rgba[0] = 1;
	rgba[1] = 1;
	rgba[2] = 1;
	rgba[3] = 1;

	Vector<3> starting_point;
	human_contact_classification::classify srv;
	int pindex = -1;
	
	//check the most probable trajectory
	//_hip_line_pub
	ros::Rate r(50);
	bool find = false;


	starting_point = _mis_p;

	for(int i=0; i<c_itr.size(); i++ ) 
		c_itr[i] = 0;

	while( !find ) {
		//starting_point = _mis_p;

		//while( ros::ok() ) {

		//Simualted pdri (considering the next wp)
		Vector<3> s_pdir;
		Vector<3> direction;
		
		float check_sspace = 0.01;
	

		for(int i=0; i<epoints.size();i++) {
				/*
				if ( i == _last_wp_index ) {
					c_results[i] == "C_TYPE_NO_CLASSIFICATION";
					c_itr[i] = 100;
				}
				else {		
				*/
					float dist = 0;
					float pdist = -1000;
					int step = 1;
					Vector<3> cpoint = starting_point;
					float min_dist = 1000;
					Vector<3> c_point_p = Zeros;
					bool maj = false;

					Vector<3> target_point = epoints[i];

					s_pdir = epoints[i] - starting_point;
					s_pdir /= norm( s_pdir );

					std::vector<float > vdist;
					std::vector< Vector<3> > vpts;
					
					direction = ( target_point - _mis_p	);
					direction /= norm( direction );

					while( norm( cpoint - starting_point) <= norm( target_point - starting_point ) ) {						
						dist = norm( cpoint - _mis_p );				
						vdist.push_back( dist );
						vpts.push_back( cpoint );
						cpoint += direction*check_sspace*(step++);										
					}

						
					
					vector<float>::iterator it = min_element(vdist.begin(),vdist.end());			
					int pos = std::distance(vdist.begin(), it);
					
					if( it != vdist.end() ) {
						dist = (*it);
					}
					//
					srv.request.p_dir.x = s_pdir[0]; 
					srv.request.p_dir.y = s_pdir[1];
					srv.request.p_dir.z = s_pdir[2];
					srv.request.h_dir.x = _hdir[0]; 
					srv.request.h_dir.y = _hdir[1];
					srv.request.h_dir.z = _hdir[2];
					srv.request.wrench.force.x = _force[0];
					srv.request.wrench.force.y = _force[1];
					srv.request.wrench.force.z = _force[2];
					srv.request.tdist.data = dist;
					_hri_classification_client.call( srv );
					

					if(c_results[i] == srv.response.classification ) {
						c_itr[i]++;	
					}
					else 
						c_itr[i] = 0;
					c_results[i] = srv.response.classification;

				//}


				if( c_results[i] == "C_TYPE_NO_CONTACT" ) {
					rgba[0] = 1.0/119.0;
					rgba[1] = 1.0/136.0;
					rgba[2] = 1.0/153.0;
				}
				else if( c_results[i] == "C_TYPE_CONTACT_CONCORDE" ) {
					rgba[0] = 1;
					rgba[1] = 0;
					rgba[2] = 0;
				}
				else if( c_results[i] == "C_TYPE_CONTACT_OPPOSITE" ) {
					rgba[0] = 1.0/119.0;
					rgba[1] = 1.0/136.0;
					rgba[2] = 1.0/153.0;
				}
				else if( c_results[i] == "C_TYPE_CONTACT_DEVIATION_C" ) {
					rgba[0] = 0.5;	
					rgba[1] = 0;
					rgba[2] = 0;
				}
				else if( c_results[i] == "C_TYPE_CONTACT_DEVIATION_O" ) {
					rgba[0] = 1.0/119.0;
					rgba[1] = 1.0/136.0;
					rgba[2] = 1.0/153.0;
				}

				visualization_msgs::Marker pts;
				visualization_msgs::Marker ln;
				create_marker( starting_point, epoints[i], rgba, i, ln, pts   );
				_hip_line_pub.publish( ln );
				_hip_pts_pub.publish( pts );			


				//cout << i << ": " << c_results[i] << endl;	
		}		

		int onlyone = 0;
		for( int cl=0; cl<c_results.size();cl++) {
			if( c_results[cl] == "C_TYPE_CONTACT_CONCORDE" ) {
				onlyone++;
				pindex = cl;


			} 

		}
		if( onlyone == 1 && c_itr[pindex] > 10 ) {
			starting_point = _mis_p;
			find = true;			
		}
		else {
			pindex = -1;
		}

		//cout << "POINT: " << pindex << endl;
		_pindex = pindex;
		r.sleep();
	}


	return pindex;
	

}

void coworking::cooperation() {

	while(!_first_cpose)
		usleep(0.1*1e6);

	ros::Rate r(100);

	geometry_msgs::Vector3 pdir;
	geometry_msgs::Vector3 hdir;
	geometry_msgs::Wrench  force;
	std_msgs::Float64      traj_dist;


	geometry_msgs::Pose p;
	int i=0;

	std_msgs::String b_data;

	string s;

	//boost::thread choose_wp_t( &coworking::choose_wp, this, _epoints );

	while(ros::ok()) {

		//cout << (_pindex!=-1) <<  " " << (_pindex != _apindex) << endl;
		/*
		if(_pindex!=-1 && _pindex != _apindex ) { //New wp, different from last one
			_apindex = _pindex;
			
			p.position.x = _epoints[_apindex][0];
			p.position.y = _epoints[_apindex][1];
			p.position.z = _epoints[_apindex][2];						
			//cout <<  "PUBBLICO!" << endl;
			_wp_pub.publish( p );
		}
		*/

		//i = choose_wp( _epoints );
		_last_wp_index = i;
		//cout << "WP chose: " << i << endl;

	
		cout << "Wp: " << i  << " / " << _epoints.size() << endl;
		p.position.x = _epoints[i][0];
		p.position.y = _epoints[i][1];
		p.position.z = _epoints[i][2];
		_wp_pub.publish( p );
		i++;
		
		while( !_tstreaming ) {
			usleep(0.1*1e6);
		} //wait to start

		human_contact_classification::classify srv;
		while ( _tstreaming ) {
		
			srv.request.p_dir.x = _pdir[0]; 
			srv.request.p_dir.y = _pdir[1];
			srv.request.p_dir.z = _pdir[2];
			srv.request.h_dir.x = _hdir[0]; 
			srv.request.h_dir.y = _hdir[1];
			srv.request.h_dir.z = _hdir[2];
			srv.request.wrench.force.x = _force[0];
			srv.request.wrench.force.y = _force[1];
			srv.request.wrench.force.z = _force[2];
			srv.request.tdist.data = _tdist;
			_hri_classification_client.call( srv );

			//cout << "Respnse: " << srv.response.classification  << endl;
			if( srv.response.classification == "C_TYPE_NO_CONTACT") {
				b_data.data = "no_contact";
			}
			else if( srv.response.classification == "C_TYPE_CONTACT_OPPOSITE") {
				b_data.data = "opposite";
			} 
			else if( srv.response.classification == "C_TYPE_CONTACT_DEVIATION_C") {
				b_data.data = "deviation_c";		
			} 
			else if( srv.response.classification == "C_TYPE_CONTACT_DEVIATION_O") {
					b_data.data = "deviation_o";		
			} 
			else if( srv.response.classification == "C_TYPE_CONTACT_CONCORDE") {
				b_data.data = "concorde";		
			}
			else {
				b_data.data = "no_contact";	
			}

			_shared_behaviour_pub.publish( b_data );			
			r.sleep();
		}


		r.sleep();










		/*
		if(  norm( _epoints[i] - _mis_p ) < 0.2 ) {
			i++;
		}
		else {
			
			i = choose_wp( _epoints );
			_last_wp_index = i;
			//cout << "WP chose: " << i << endl;

			cout << "Wp: " << i  << " / " << _epoints.size() << endl;
			p.position.x = _epoints[i][0];
			p.position.y = _epoints[i][1];
			p.position.z = _epoints[i][2];						
			i++;
				
			
			//Decide to stream

			_wp_pub.publish( p );
			while( !_tstreaming ) {
				usleep(0.1*1e6);
			} //wait to start

			human_contact_classification::classify srv;
			while( _tstreaming ) {
		

				srv.request.p_dir.x = _pdir[0]; 
				srv.request.p_dir.y = _pdir[1];
				srv.request.p_dir.z = _pdir[2];
				srv.request.h_dir.x = _hdir[0]; 
				srv.request.h_dir.y = _hdir[1];
				srv.request.h_dir.z = _hdir[2];
				srv.request.wrench.force.x = _force[0];
				srv.request.wrench.force.y = _force[1];
				srv.request.wrench.force.z = _force[2];
				srv.request.tdist.data = _tdist;
				_hri_classification_client.call( srv );

				//cout << "Respnse: " << srv.response.classification  << endl;
				if( srv.response.classification == "C_TYPE_NO_CONTACT") {
					b_data.data = "no_contact";
				}
				else if( srv.response.classification == "C_TYPE_CONTACT_OPPOSITE") {
					b_data.data = "opposite";
				} 
				else if( srv.response.classification == "C_TYPE_CONTACT_DEVIATION_C") {
					b_data.data = "deviation_c";		
				} 
				else if( srv.response.classification == "C_TYPE_CONTACT_DEVIATION_O") {
 					b_data.data = "deviation_o";		
				} 
				else if( srv.response.classification == "C_TYPE_CONTACT_CONCORDE") {
					b_data.data = "concorde";		
				}
				else {
					b_data.data = "no_contact";	
				}

				_shared_behaviour_pub.publish( b_data );

				
				
				usleep(0.1*1e6);
			} //wait to finish
		}
			//getline(cin, s);

		if( i>=_epoints.size() ) i=0;

		//_epoints
		r.sleep();
		*/
		if( i>=_epoints.size() ) i=0;


		
	}
}

void coworking::run() {

	boost::thread classification_request_t( &coworking::cooperation, this  );
	ros::spin();
}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "coworking_node");
	coworking cow;
	cow.run();
	return 0;	
}
