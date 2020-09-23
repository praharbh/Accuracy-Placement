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

math::Vector Constraints::Position(vector<math::Vector> solutions)
{
	math::Vector maxAngles = math::Vector(solutions[0].size());
	maxAngles.setZero();

	for (int i = 0; i < solutions.size(); i++) {
		for (int j = 0; j < maxAngles.size(); j++) {
			if (abs(solutions[i](j)) > maxAngles(j)) {
				maxAngles(j) = abs(solutions[i](j));
			}
		}
			
	}
	return maxAngles;
}
