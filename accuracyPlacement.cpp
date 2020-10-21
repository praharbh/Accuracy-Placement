#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <string>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>

/*
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
*/

#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/Kinematic.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/NloptInverseKinematics.h>

#include <nlopt.hpp>

#include "Manipulator.h"
#include "Data.h"
#include "Read.h"
#include "Constraints.h"
#include "Optimization.h"

using namespace std;
using namespace webots;
//using namespace KDL;
using namespace rl;

int main(int argc, char **argv) {
	
	// Initialize robot or supervisor
	// Supervisor* robot = new Supervisor();
	Robot *robot = new Robot();
	int timeStep = (int)robot->getBasicTimeStep();

	// Get joint motors
	vector<string> jointNames = Data::JointNames(robot->getName());
	vector<Motor*> motors = Manipulator::JointMotors(robot, jointNames);
	
	// Get joint sensors
	vector<string> sensorNames = Data::SensorNames(robot->getName());
	vector<PositionSensor*> sensors = Manipulator::JointSensors
		(robot, sensorNames, timeStep);

	cout << "Turning on controller for " + robot->getName() << endl;

	// Get URDF
	mdl::UrdfFactory factory;
	mdl::Kinematic* kinematics = dynamic_cast<mdl::Kinematic*>
		(factory.create("C:/Bhatt_Work_Drive_Advanced/ICRA 2021/URDF/IRB4600.xml"));
	
	// Obtain path
	vector<math::Transform> frames = 
		Read::CSV("C:/Bhatt_Work_Drive_Advanced/ICRA 2021/CSV/vase630.csv");

	// Testing part placement constraint at robot home location
	math::Vector solution(kinematics->getDof());
	solution << 0, 0, 0, 0, 90, 0;
	solution *= math::DEG2RAD;
	math::Transform home = Manipulator::FKRL(kinematics, solution);
	math::Transform location = math::Transform();
	location.setIdentity();
	location.translation() = home.translation();
	vector<math::Transform> reFrames = Read::Relocate(location, frames);
	vector<math::Vector> solutions = Constraints::Reachability(kinematics, reFrames);
	cout << Constraints::MaxXe(kinematics, reFrames, solutions, 0.020) << " m" << endl;

	// Optimizing part placement 
	Optimization optimize;
	location.translation() = 
		Manipulator::FKRL(kinematics, optimize.AMPlace(kinematics, frames, motors)).translation();
	cout << "optimized" << endl;
	reFrames = Read::Relocate(location, frames);
	solutions = Constraints::Reachability(kinematics, reFrames);
	cout << Constraints::MaxXe(kinematics, reFrames, solutions, 0.010) << " m" << endl;

	// Simulating the first point
	int i = 0;
	Manipulator::Simulate(motors, solution);


	// Simulating the entire path 
	while (robot->step(timeStep*2) != -1) {
	
		if (i < solutions.size()) {
			cout << i << endl;
			Manipulator::Simulate(motors, solutions[i]);
			i++;
		}
		
	}

	// Exit and cleanup
	cout << "Execution ended." << endl;
	delete robot;
	return 0;
}