// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <math.h>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/mmp_panda.urdf";

#define INITIALIZING        0
#define MOVE_AND_SWING      1
#define RETURN_AND_POSE     2
          
#define G 9.81
#define HITZ 1.0
#define BASE_HIT_OFF_X      0.8
#define BASE_HIT_OFF_Y      0.0

double swing_arm_length = BASE_HIT_OFF_X;

int state = INITIALIZING;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string OBJ_POSITION_KEY;
std::string OBJ_VELOCITIES_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

void hitting_spot(Vector3d ball_p, Vector3d ball_v, double hit_z, pair<double, double> r_p, pair<double, double> land, double z_at_net, double* hit_param){
	// given the ball position and velocity, and the height of hit position, 
	// returning the x and y position where the ball will be at (where the robot needs to go to)

	// Implementation: calculate 1-3 potential hit positions and select the one which requires smallest speed to get to
	double restitution_vertical = 0.75;
	double restitution_horizontal = 0.6;

	vector<pair<pair<double, double>,double>> potential_hit_spots;
	vector<double> required_speeds;

	double delta = sqrt(pow(ball_v(2,0),2)+2*G*ball_p(2,0));
	
	// two time points when the ball hit the ground
	double t1 = (ball_v(2)-delta)/G;
	double t2 = (ball_v(2)+delta)/G;

	double h_x, h_y;

	// first see if it has already bounced once in its half court
	if(ball_p(1)+t1*ball_v(1)>0){
		// not yet bounced in its half court
		delta = sqrt(pow(ball_v(2),2)+2*G*(ball_p(2)-hit_z)); // TO DO: what if delta is imaginary
		// time point when the ball is at hit_z
		double t = (ball_v(2)+delta)/G;
		
		if(t>0){
			h_x = ball_v(0)*t+ball_p(0);
			h_y = ball_v(1)*t+ball_p(1);
			potential_hit_spots.push_back({{h_x, h_y},t});
			required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/t);
		}
		// then add two more points after bouncing:
		// using the point where it hits the ground:
		Vector3d ball_p_after, ball_v_after;
		ball_p_after << ball_p(0)+ball_v(0)*t2, ball_p(1)+ball_v(1)*t2, 0;
		ball_v_after << restitution_horizontal*ball_v(0), restitution_horizontal*ball_v(1), restitution_vertical*(-ball_v(2)+G*t2);
		double delta_after = sqrt(pow(ball_v_after(2),2)+2*G*(ball_p_after(2)-hit_z));
		double t1_after = (ball_v_after(2)-delta_after)/G;
		double t2_after = (ball_v_after(2)+delta_after)/G;

		h_x = ball_v_after(0)*t1_after+ball_p_after(0);
		h_y = ball_v_after(1)*t1_after+ball_p_after(1);
		potential_hit_spots.push_back({{h_x, h_y},t2+t1_after});
		required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/(t2+t1_after));

		h_x = ball_v_after(0)*t2_after+ball_p_after(0);
		h_y = ball_v_after(1)*t2_after+ball_p_after(1);
		potential_hit_spots.push_back({{h_x, h_y}, t2+t2_after});
		required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/(t2+t2_after));
	} else{
		// already bounced once in its half court
		delta = sqrt(pow(ball_v(2),2)+2*G*(ball_p(2)-hit_z));
		t1 = (ball_v(2)-delta)/G;
		t2 = (ball_v(2)+delta)/G;

		if(t1 > 0){
			h_x = ball_v(0)*t1+ball_p(0);
			h_y = ball_v(1)*t1+ball_p(1);
			potential_hit_spots.push_back({{h_x, h_y}, t1});
			required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/t1);
		}

		if(t2 > 0){
			h_x = ball_v(0)*t2+ball_p(0);
			h_y = ball_v(1)*t2+ball_p(1);
			potential_hit_spots.push_back({{h_x, h_y}, t2});
			required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/t2);
		}
	}
	int index=-1;
	int speed_min = 100000;
	for(int i=0; i<required_speeds.size();i++){
		if(required_speeds[i]<speed_min){
			index = i;
			speed_min = required_speeds[i];
		}
	}
	if(index == -1){
		// no hitable point
		hit_param[0] = r_p.first+BASE_HIT_OFF_X;
		hit_param[1] = r_p.second+BASE_HIT_OFF_Y;
		hit_param[2] = -1;
		hit_param[3] = -1;
		hit_param[4] = -1;
		hit_param[5] = -1;
		return;
	} else{
		hit_param[0] = potential_hit_spots[index].first.first;
		hit_param[1] = potential_hit_spots[index].first.second;
	    hit_param[5] = potential_hit_spots[index].second;
	}

	// Now calculate parameters related to swing speed and orientation.
	// t_hit_land: the time it takes from hit to land
	double y0 = potential_hit_spots[index].first.second;
	double yd = land.second;
	double t_hit_land = sqrt(2*(yd-y0)*(yd*hit_z-(yd-y0)*z_at_net)/(G*y0*yd));
	double vox = (land.first - potential_hit_spots[index].first.first)/t_hit_land; // v out x
	double voy = (land.second- potential_hit_spots[index].first.second)/t_hit_land;// v out y
	double voz = 0.5*G*t_hit_land-hit_z/t_hit_land;										   // v out z

	double approx_alpha_square = 0.5*0.5; // first parameter to tune, value should be 0.7^2 to 0.8^2 : in the racket frame, assume |v_out| = approx_alpha * |v_in|
	double a = 1 - approx_alpha_square;
	double b = 2*approx_alpha_square*ball_v(1)-2*voy;
	double c = vox*vox+voy*voy+voz*voz-approx_alpha_square*(ball_v(0)*ball_v(0)+ball_v(1)*ball_v(1)+ball_v(2)*ball_v(2));
	double sqrt_delta = sqrt(b*b-4*a*c);
	if(b*b-4*a*c<0){
		cout << "ERROR: NO swing speed" << endl;
	}
	double swing_speed = (-b - sqrt_delta)/(2*a);
	if(swing_speed < 0){
		swing_speed = (-b + sqrt_delta)/(2*a);
	}
	hit_param[2] = swing_speed;

	double ratio = 0.5; // second parameter to tune, hard to explain, value should be near 0.5 (0.4-0.6 maybe)

	// normal vector
	double nx = vox*(1-ratio)-ratio*ball_v(0);
	double ny = (voy-swing_speed)*(1-ratio)-ratio*(ball_v(1)-swing_speed);
	double nz = voz*(1-ratio)-ratio*ball_v(2);

	hit_param[3] = atan(nx/ny);
	hit_param[4] = atan(nz/sqrt(nx*nx+ny*ny));	
	return;
}

int main() {
	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
	OBJ_POSITION_KEY  = "cs225a::robot::ball::sensors::q";
	OBJ_VELOCITIES_KEY = "cs225a::robot::ball::sensors::dq";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	VectorXd initial_dq = robot->_dq;
	robot->updateModel();

	// position and velocity of the ball (without creating a robot object)
	VectorXd ball_p(6);
	VectorXd ball_v(6);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "racquetlink";
	const Vector3d control_point = Vector3d(0.0,0.45,0.0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 70.0;
	joint_task->_kv = 20.0;

	VectorXd q_init_desired = initial_q;
	VectorXd dq_init_desired = initial_dq;
	q_init_desired << 0.0,0.0,0.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	dq_init_desired <<  0,  0,  0,     0,     0,     0,      0,   0,    0,    0;
	//q_init_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;
	joint_task->_desired_velocity = dq_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// Testing code
	bool test = true;
	Vector3d robot1_inWorld = Vector3d(0, 1.5, 0.5);
	int count = 0;
	// pair<double,double> hit_point;
	double hit_param[6]; // x,y,speed, theta1, theta2, time
	while (runloop) {
		

		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;


		// wait for next scheduled loop
		count += 1;
		if(count % 2 == 0) // collect the ball position and velocity every 5 ms
		{
			ball_p = redis_client.getEigenMatrixJSON(OBJ_POSITION_KEY);
			ball_p(1) += 8.0;
			ball_p(2) += 0.7;
			ball_v = redis_client.getEigenMatrixJSON(OBJ_VELOCITIES_KEY);
		}


		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();
	
		// based on ball condition determine the robot state
		if (state != INITIALIZING){
			if(ball_v(1)<0 && ball_p(1)<6 && ball_p(1) > robot->_q(1)-7.0){
				state = MOVE_AND_SWING;
			} else state = RETURN_AND_POSE;
		}


		switch(state) {
			case INITIALIZING: {
				// cout << "Initilizing\n\r";
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);

				joint_task->_desired_position = q_init_desired;
				posori_task->_desired_position = Vector3d(0.75,0.0,0.5);
				posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix() * AngleAxisd(0, Vector3d::UnitY()).toRotationMatrix();	

				N_prec.setIdentity();
				posori_task->updateTaskModel(N_prec);
				N_prec = posori_task->_N;
				joint_task->updateTaskModel(N_prec);



				joint_task->computeTorques(joint_task_torques);

				posori_task->computeTorques(posori_task_torques);

				command_torques = joint_task_torques + posori_task_torques;
				
				if( posori_task->goalOrientationReached(0.1,false) )
				{
					joint_task->reInitializeTask();
					N_prec.setIdentity();
					joint_task->updateTaskModel(N_prec);

					// joint_task->_kp = 200.0;
					//q_init_desired(0) = -0.5;
					//q_init_desired(1) = 0;
					joint_task->_desired_position(0) = -0.5;
					joint_task->_desired_position(1) = 0.0;

					// cout << joint_task->_desired_position << "\n\r";
					
					VectorXd maxVelocities = VectorXd::Zero(dof);
					maxVelocities << 2.0,2.0,M_PI/3,5*M_PI/3,M_PI/3,M_PI/3,M_PI/3,M_PI/3,M_PI/3,M_PI/3;
					joint_task->_otg->setMaxVelocity(maxVelocities);
					joint_task->_otg->setMaxAcceleration(2*M_PI);

					state = RETURN_AND_POSE;
				}
			}
			break;

			case MOVE_AND_SWING: {
				// cout<<"MOVE_AND_SWING\n\r";
				hitting_spot(ball_p.head(3), ball_v.head(3), HITZ, {robot->_q(0),robot->_q(1)-6.0}, {5.0, 5.0}, 2.3, hit_param);
				// cout << "x: " << hit_param[0] << "y: " << hit_param[1] << " swing_speed: " << hit_param[2] << " theta1: " << hit_param[3] << " theta2: " << hit_param[4] << " time: " << hit_param[5];

				joint_task->_desired_position(0) = hit_param[0] - BASE_HIT_OFF_X;
				joint_task->_desired_position(1) = hit_param[1] + 6.0 - BASE_HIT_OFF_Y;
				joint_task->_desired_position(7) = -2.50965+hit_param[4];

				if(hit_param[5] > 0 && hit_param[5] < 0.18+(0.3-hit_param[3])/hit_param[2]){
					// joint_task->_use_interpolation_flag = true;
					// cout << "swing speed!" << hit_param[2]<< " ";
					joint_task->_desired_position(3) = 1;

					joint_task->_kp = 2000.0;
					joint_task->_kv = 50.0;
					VectorXd maxVelocities = VectorXd::Zero(dof);
					maxVelocities << 2.0, 2.0, M_PI/3, hit_param[2]/0.75, M_PI/3, M_PI/3,M_PI/3,9*M_PI/3,M_PI/3,M_PI/3;
					joint_task->_otg->setMaxVelocity(maxVelocities);
					joint_task->_otg->setMaxAcceleration(20*M_PI);
					// joint_task->_desired_velocity(3) = hit_param[2]/swing_arm_length;
				}

				joint_task->updateTaskModel(N_prec);

				joint_task->computeTorques(joint_task_torques);

				// cout << "joint_task_torques.size()" << joint_task_torques.size() << endl;
				// cout << joint_task_torques(0) << " " << joint_task_torques(1) << " " << joint_task_torques(2) << " " << joint_task_torques(3) <<endl;
				command_torques = joint_task_torques;
			}
			break;

			case RETURN_AND_POSE: {
				// cout<<"RETURN_AND_POSE\n\r";
				// update task model and set hierarchy
				N_prec.setIdentity();
				//posori_task->updateTaskModel(N_prec);
				//N_prec = posori_task->_N;
				joint_task->_desired_position(0) = -0.5;
				joint_task->_desired_position(1) = 0.0;
				joint_task->_desired_position(3) = -1.5;
				joint_task->_desired_position(7) = -2.50965+M_PI/6;
				joint_task->updateTaskModel(N_prec);

				// compute torques
				joint_task->computeTorques(joint_task_torques);
				//posori_task->computeTorques(posori_task_torques);
				// cout << "joint(7): " << robot->_q(7);
				command_torques = joint_task_torques;// + posori_task_torques;
			}
			break;
		
		}

		// send to redis
		//cout << "Command torques   :\n\r" << command_torques << "\n\r\n\r";
		if (controller_counter %10 == 0) {
			redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
			// enforcedCommand = false;
		}
		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
