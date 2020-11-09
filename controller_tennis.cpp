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

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1
#define SWING_ORIENT          2
#define SWING_MOTION          3
#define HITTING 	      4
#define RETURNING 	      5
#define G 9.81
#define HITZ 0.5

int swing_state = SWING_ORIENT;
int state = HITTING;

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

pair<double, double> hitting_spot(Vector3d ball_p, Vector3d ball_v, double hit_z, pair<double, double> r_p){
	// given the ball position and velocity, and the height of hit position, 
	// returning the x and y position where the ball will be at (where the robot needs to go to)

	// Implementation: calculate 1-3 potential hit positions and select the one which requires smallest speed to get to
	double restitution = 0.759;

	vector<pair<double, double>> potential_hit_spots;
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
			potential_hit_spots.push_back({h_x, h_y});
			required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/t);
		}
		// then add two more points after bouncing:
		// using the point where it hits the ground:
		Vector3d ball_p_after, ball_v_after;
		ball_p_after << ball_p(0)+ball_v(0)*t2, ball_p(1)+ball_v(1)*t2, 0;
		ball_v_after << restitution*ball_v(0), restitution*ball_v(1), restitution*(-ball_v(2)+G*t2);
		double delta_after = sqrt(pow(ball_v_after(2),2)+2*G*(ball_p_after(2)-hit_z));
		double t1_after = (ball_v_after(2)-delta_after)/G;
		double t2_after = (ball_v_after(2)+delta_after)/G;

		h_x = ball_v_after(0)*t1_after+ball_p_after(0);
		h_y = ball_v_after(1)*t1_after+ball_p_after(1);
		potential_hit_spots.push_back({h_x, h_y});
		required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/(t+t1_after));

		h_x = ball_v_after(0)*t2_after+ball_p_after(0);
		h_y = ball_v_after(1)*t2_after+ball_p_after(1);
		potential_hit_spots.push_back({h_x, h_y});
		required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/(t+t2_after));
	} else{
		// already bounced once in its half court
		delta = sqrt(pow(ball_v(2),2)+2*G*(ball_p(2)-hit_z));
		t1 = (ball_v(2)-delta)/G;
		t2 = (ball_v(2)+delta)/G;

		if(t1 > 0){
			h_x = ball_v(0)*t1+ball_p(0);
			h_y = ball_v(1)*t1+ball_p(1);
			potential_hit_spots.push_back({h_x, h_y});
			required_speeds.push_back((pow(r_p.first-h_x,2)+pow(r_p.second-h_y,2))/t1);
		}

		if(t2 > 0){
			h_x = ball_v(0)*t2+ball_p(0);
			h_y = ball_v(1)*t2+ball_p(1);
			potential_hit_spots.push_back({h_x, h_y});
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
		return r_p;
	} else{
		return potential_hit_spots[index];
	}
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
	robot->updateModel();

	// position and velocity of the ball (without creating a robot object)
	VectorXd ball_p(6);
	VectorXd ball_v(6);
	ball_p;
	ball_v;


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
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	q_init_desired << 0.0,0.0,0.0, -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	//q_init_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

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
	pair<double,double> hit_point;
	while (runloop) {
		// wait for next scheduled loop
		count += 1;
		if(count % 2 == 0) // collect the ball position and velocity every 5 ms
		{
			ball_p = redis_client.getEigenMatrixJSON(OBJ_POSITION_KEY);
			ball_p(1) += 5.0;
			ball_p(2) += 3.0;
			ball_v = redis_client.getEigenMatrixJSON(OBJ_VELOCITIES_KEY);
		}

		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();
	
		// based on ball condition determine the robot state
		if(ball_v(1)<0 && ball_p(1)>-8) {
			state = HITTING;
		} else state = RETURNING;

		if(state == HITTING)
		{
			// cout << "HITTING\n";
			joint_task->_kp = 250.0;
			hit_point = hitting_spot(ball_p.head(3), ball_v.head(3), HITZ, {robot->_q(0),robot->_q(1)-5.0});
			q_init_desired(0) = hit_point.first-0.5;
			q_init_desired(1) = hit_point.second+5.0;
			if(count % 200 == 0){
				cout << "ball_p.head(3) " << ball_p.head(3) << "\nball_v.head(3) " << ball_v.head(3) << "\nrobot->_q " << robot->_q(0) << " " << robot->_q(1)-5.0 << endl;
				cout << " q_init_desired: " << q_init_desired(0) << ", " << q_init_desired(1)<<endl;
			}
			joint_task->_desired_position = q_init_desired;
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_init_desired).norm() < 0.15 )
			{
				posori_task->reInitializeTask();
				// Just adjust orientation at the end
				// Acceleration of the base to hit the base
				// Use joint 4
				posori_task->_desired_position = Vector3d(0.75,0.0,0.5);
				//Vector3d Position = Vector3d(0,0.75,1) - robot1_inWorld;
				//posori_task->_otg->setGoalPositionAndLinearVelocity(Position, Vector3d(1.0,1.0,1.0));
				
				//posori_task->_desired_velocity = Vector3d(1,1,1);
				
				posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * posori_task->_desired_orientation;
				
				posori_task->_desired_orientation = AngleAxisd(-M_PI/10, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				

				joint_task->reInitializeTask();
				

				

				state = POSORI_CONTROLLER;
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
			// cout << "POSORI\n";
			joint_task->_kp = 10;
			//cout << "Bye\n\r";			
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			
			if (posori_task->goalOrientationReached(0.15,false) && test) {
				//posori_task->reInitializeTask();
				joint_task->reInitializeTask();
				joint_task->_kp = 40;
				//q_init_desired << 0.0,0.0,90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				//q_init_desired << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				q_init_desired *= M_PI/180.0;
				joint_task->_desired_position[3] = -180*M_PI/180.0;
				//joint_task->_desired_position[1] = -.5;
				state = SWING_ORIENT;

			}
			
		} 
		else if(state == SWING_ORIENT) {
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;
		} else if (state == RETURNING){
			// cout << "RETURNING\n";
			joint_task->_kp = 250.0;
			q_init_desired(0) = -0.5;
			q_init_desired(1) = 0;
			joint_task->_desired_position = q_init_desired;
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques

			command_torques = joint_task_torques;
		}

		// send to redis
		//cout << "Command torques   :\n\r" << command_torques << "\n\r\n\r";
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
