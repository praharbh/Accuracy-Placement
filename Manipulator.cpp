#include "Manipulator.h"

vector<Motor*> Manipulator::JointMotors(Robot *robot, vector<string> jointNames)
{

	vector<Motor*> motors = vector<Motor*>();
	
	for (int i = 0; i < jointNames.size(); i++) {
	
		motors.push_back(robot->getMotor(jointNames[i]));
	
	}

	return motors;

}

vector<PositionSensor*> Manipulator::JointSensors(Robot *robot, 
	vector<string> sensorNames, int timeStep)
{
	vector<PositionSensor*> sensors = vector<PositionSensor*>();

	for (int i = 0; i < sensorNames.size(); i++) {

		sensors.push_back(robot->getPositionSensor(sensorNames[i]));
		sensors.back()->enable(timeStep);
	}

	return sensors;
}

void Manipulator::Simulate(vector<Motor*> motors, math::Vector jointPositions)
{
	for (int i = 0; i < motors.size(); i++) {

		motors[i]->setPosition(jointPositions[i]);

	}
}

math::Transform Manipulator::FKRL(mdl::Kinematic* kinematics, math::Vector angles)
{
	kinematics->setPosition(angles);
	
	kinematics->forwardPosition();
	math::Transform frame = kinematics->getOperationalPosition(0);
	
	return frame;
}

bool Manipulator::IKRL(mdl::Kinematic* kinematics, math::Transform frame)
{
	math::Vector angles = kinematics->getPosition();
	mdl::NloptInverseKinematics ikSolver(kinematics);
	ikSolver.duration = std::chrono::seconds(1);
	ikSolver.goals.push_back(make_pair(frame, 0));
	bool result = ikSolver.solve();
	//angles = kinematics->getPosition();
	return result;
} 

math::Vector Manipulator::Poll(vector<PositionSensor*> sensors)
{
	math::Vector angles = math::Vector(sensors.size());
	for (int i = 0; i < sensors.size(); i++) {
		angles(i) = round(sensors[i]->getValue() * 10000)/10000;
	}
	return angles;
}

math::Vector Manipulator::Random(vector<Motor*> motors)
{
	math::Vector randSol;
	randSol.setRandom(motors.size());

	for (int i = 0; i < randSol.size(); i++) {
		if (randSol(i) < 0) {
			randSol(i) = abs(randSol(i)) * motors[i]->getMinPosition();
		}
		else {
			randSol(i) = randSol(i) * motors[i]->getMaxPosition();
		}
	}
	return randSol;
}

/*
Frame Manipulator::FKKDL(Chain robotChain, JntArray jointPosition){

	vector<Frame> cartPosition = vector<Frame>(robotChain.getNrOfSegments());//Frame::Identity();
	int fkStatus = 0;

	ChainFkSolverPos_recursive fkSolver = ChainFkSolverPos_recursive(robotChain);
	fkStatus = fkSolver.JntToCart(jointPosition, cartPosition);
	cout << "FK: ";
	cout << fkStatus << endl;

	for (int i = 0; i < robotChain.getNrOfSegments(); i++) {
		cout << i << endl;
		cout << cartPosition[i] << endl;
	}

	return cartPosition.back();

}

JntArray Manipulator::IKKDL(JntArray inputPosition, Chain robotChain, Frame cartPosition)
{
	JntArray outputPosition = JntArray(robotChain.getNrOfJoints());
	ChainFkSolverPos_recursive fkSolver = ChainFkSolverPos_recursive(robotChain);
	ChainIkSolverVel_pinv ivSolver = ChainIkSolverVel_pinv(robotChain);
	ChainIkSolverPos_NR ikSolver = 
		ChainIkSolverPos_NR(robotChain, fkSolver, ivSolver);
	int errorID = ikSolver.CartToJnt(inputPosition, cartPosition, outputPosition);
	cout << "IK";
	cout<< ikSolver.strError(errorID) << endl;
	
	return outputPosition;
}
*/
