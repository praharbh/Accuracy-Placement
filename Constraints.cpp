#include "Constraints.h"

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

math::Vector Constraints::Speed(vector<math::Transform> reFrames,
	vector<math::Vector> solutions, double velocity)
{
	math::Vector jointVelocity = math::Vector();
	math::Vector maxVelocity = math::Vector();

	double distance = 0;
	double time = 0;

	for (int i = 0; i < solutions.size()-1; i++) {
		distance = (reFrames[i + 1].translation() -
			reFrames[i].translation()).norm();
		time = distance / velocity;
		jointVelocity = (solutions[i + 1] - solutions[i])/time;

		if (i == 0) {
			maxVelocity = jointVelocity;
		}
		else {
			for (int j = 0; j < jointVelocity.size(); j++) {
				if (jointVelocity(j) > maxVelocity(j)) {
					maxVelocity(j) = jointVelocity(j);
				}
			}
		}	
	}

	return maxVelocity;
}

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
		jacobian = kinematics->getJacobian();
		deviation = jacobian * theta;
		if (maxDeviation < deviation.maxCoeff()) {
			cout << deviation.transpose() << endl;
			maxDeviation = deviation.maxCoeff();
			cout << maxDeviation;
		}
	}

	return maxDeviation;
}
