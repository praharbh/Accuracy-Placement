#include "Constraints.h"

// Function to test the reachability of the path
vector<math::Vector> Constraints::Reachability(mdl::Kinematic* kinematics, vector<math::Transform> frames)
{
	vector<math::Vector> solutions = vector<math::Vector>();

	for (int i = 0; i < frames.size(); i++) {
		if (Manipulator::IKRL(kinematics, frames[i])) {
			solutions.push_back(kinematics->getPosition());
		}
		else {
			break;
		}
	}

	return solutions;
}

// Function to obtain the max joint speed on the path
double Constraints::Speed(vector<math::Transform> reFrames,
	vector<math::Vector> solutions, double velocity)
{
	math::Vector jointVelocity = math::Vector();
	double maxVelocity = 0;

	double distance = 0;
	double time = 0;

	for (int i = 0; i < solutions.size()-1; i++) {
		distance = (reFrames[i + 1].translation() -
			reFrames[i].translation()).norm();
		time = distance / velocity;
		jointVelocity = (solutions[i + 1] - solutions[i])/time;

		for (int j = 0; j < jointVelocity.size(); j++) {
			if (jointVelocity(j) > maxVelocity) {
				maxVelocity = jointVelocity(j);
			}
		}	
	}

	return maxVelocity * 180/M_PI;
}

// Function to obatin the max position error on path
double Constraints::Position(mdl::Kinematic* kinematics, 
	vector<math::Vector> solutions)
{
	double maxDeviation = 0;
	math::Matrix jacobian = math::Matrix();
	math::Vector theta = math::Vector(kinematics->getDof());
	theta.setOnes();
	math::Vector deviation = math::Vector(6);

	for (int i = 0; i < solutions.size(); i++) {

		Manipulator::FKRL(kinematics, solutions[i]);
		kinematics->calculateJacobian();
		jacobian = kinematics->getJacobian();
		deviation = jacobian * theta;

		for (int j = 0; j < 3; j++) {
			if (maxDeviation < abs(deviation(j))) {
				maxDeviation = abs(deviation(j));
			}
		}
	}

	return maxDeviation;
}

// Function to calculate the max Xe error
double Constraints::MaxXe(mdl::Kinematic* kinematics, vector<math::Transform> reFrames,
	vector<math::Vector> solutions, double velocity)
{
	math::Matrix jacobian = math::Matrix();
	math::Vector ka = math::Vector(kinematics->getDof());
	math::Vector kc = math::Vector(kinematics->getDof());
	math::Vector thetae = math::Vector(kinematics->getDof());
	double distance = 0;
	double time = 0;
	math::Vector thetaDot = math::Vector(kinematics->getDof());
	math::Vector Xe = math::Vector(6);
	math::Vector XeTrans = math::Vector(3);
	double normXeTrans = 0;
	double maxXe = 0;
	
	ka << 1.0, 5.0 / 6.0, 4.0 / 6.0, 3.0 / 6.0, 2.0 / 6.0, 1.0 / 6.0;
	kc << 1.0, 5.0 / 6.0, 4.0 / 6.0, 3.0 / 6.0, 2.0 / 6.0, 1.0 / 6.0;

	for (int i = 1; i < solutions.size(); i++) {

		Manipulator::FKRL(kinematics, solutions[i]);
		kinematics->calculateJacobian();
		jacobian = kinematics->getJacobian();
		distance = (reFrames[i].translation() -
			reFrames[i - 1].translation()).norm();
		time = distance / velocity;
		thetaDot = (solutions[i] - solutions[i - 1]) / time;

		for (int j = 0; j < thetaDot.size(); j++) {

			thetae[j] = (ka[j] * M_PI / 180) + (kc[j] * thetaDot[j]);

		}

		Xe = jacobian * thetae;
		XeTrans = math::Vector(3);
		XeTrans << Xe[0], Xe[1], Xe[2];
		normXeTrans = XeTrans.norm();

		if (normXeTrans > maxXe) {
			maxXe = normXeTrans;
		}

	}

	return maxXe;
}
