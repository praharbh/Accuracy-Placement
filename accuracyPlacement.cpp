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

using namespace std;
using namespace webots;
//using namespace KDL;
using namespace rl;

int main(int argc, char **argv) {
	
	//Supervisor* robot = new Supervisor();
	Robot *robot = new Robot();
	int timeStep = (int)robot->getBasicTimeStep();

	vector<string> jointNames = Data::JointNames(robot->getName());
	vector<Motor*> motors = Manipulator::JointMotors(robot, jointNames);
	
	vector<string> sensorNames = Data::SensorNames(robot->getName());
	vector<PositionSensor*> sensors = Manipulator::JointSensors
		(robot, sensorNames, timeStep);

	cout << "Turning on controller for " + robot->getName() << endl;

	mdl::UrdfFactory factory;
	mdl::Kinematic* kinematics = dynamic_cast<mdl::Kinematic*>(factory.create("C:/Bhatt_Work_Drive_Advanced/ICRA 2021/URDF/IRB4600.xml"));
	
	vector<math::Transform> frames = Read::CSV("C:/Bhatt_Work_Drive_Advanced/ICRA 2021/CSV/vase630.csv");

	math::Vector solution(6);
	solution << 0, 0, 0, 0, 90, 0;
	solution *= math::DEG2RAD;
	math::Transform home = Manipulator::FKRL(kinematics, solution);
	
	math::Transform location = math::Transform();
	location.setIdentity();
	location.translation() = home.translation();
	vector<math::Transform> reFrames = Read::Relocate(location, frames);
	cout << frames.size() << endl;
	//math::Vector solution = Manipulator::IKRL(kinematics, t);
	int i = 0;
	Manipulator::Simulate(motors, solution);

	vector<math::Vector> solutions = Constraints::Reachability(kinematics, reFrames);
	cout << solutions.size() << endl;

	cout << Constraints::Speed(reFrames, solutions, 0.050).transpose() * 180 / PI << endl;
	cout << Constraints::Position(kinematics, solutions) << endl;

	while (robot->step(timeStep*2) != -1) {
		continue;
	
		//cout << Manipulator::Poll(sensors) << endl << endl;
		//cout << solution << endl;
		if (i < solutions.size()) {
			cout << i << endl;
			//frames[i].translation() += t.translation();
			/*if (Manipulator::IKRL(kinematics, reFrames[i])) {
				solution = kinematics->getPosition();
				Manipulator::Simulate(motors, solution);
			}*/
			Manipulator::Simulate(motors, solutions[i]);
			i++;
		}
		
	}

	cout << "Execution ended." << endl;

	// Enter here exit cleanup code.
	delete robot;
	return 0;
}