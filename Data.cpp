#define _USE_MATH_DEFINES

#include <math.h>

#include "Data.h"

vector<string> Data::JointNames(string robotName)
{
    vector<string> names = vector<string>();

    if (robotName == "UR5e") {

        names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
        
        return names;
    }
	else if (robotName == "IRB 4600/40") {
		
		names = { "A motor", "B motor", "C motor",
			"D motor", "E motor", "F motor" };
		
		return names;
	}
    else {
        return names;
    }
    
}

vector<string> Data::SensorNames(string robotName)
{
	vector<string> names = vector<string>();

	if (robotName == "UR5e") {

		names = { "shoulder_pan_joint_sensor", "shoulder_lift_joint_sensor", 
			"elbow_joint_sensor","wrist_1_joint_sensor", 
			"wrist_2_joint_sensor", "wrist_3_joint_sensor" };

		return names;
	}
	else if (robotName == "IRB 4600/40") {

		names = { "A sensor", "B sensor", "C sensor",
			"D sensor", "E sensor", "F sensor" };

		return names;
	}
	else {
		return names;
	}

}

/*
Chain Data::Kinematics(string robotName)
{
	Chain robotChain = Chain();

	if (robotName == "UR5e") {
		//robotChain.addSegment(Segment(Joint(Joint::None),
			//Frame(Rotation::EulerZYX(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 0.0))));
		robotChain.addSegment(Segment(Joint(Joint::RotZ),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 0.163))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, M_PI_2, 0.0)) * Frame(Vector(0.0, 0.138, 0.0))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, -0.131, 0.425))));
		
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, M_PI_2, 0.0)) * Frame(Vector(0.0, 0.0, 0.392))));
		robotChain.addSegment(Segment(Joint(Joint::RotZ),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.127, 0.0))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 0.1))));

		robotChain.addSegment(Segment(Joint(Joint::None),
			Frame(Rotation::RPY(0.0, 0.0, M_PI_2)) * Frame(Vector(0.0, 0.1, 0.0))));
	}
	else if (robotName == "IRB 4600/40") {

		robotChain.addSegment(Segment(Joint(Joint::None),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 0.0))));

		robotChain.addSegment(Segment(Joint(Joint::RotZ),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 0.495))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.175, 0.0, 0.0))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.0, 0.0, 1.095))));

		robotChain.addSegment(Segment(Joint(Joint::RotX),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.33, 0.0, 0.175))));
		robotChain.addSegment(Segment(Joint(Joint::RotY),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.960, 0.0, 0.0))));
		robotChain.addSegment(Segment(Joint(Joint::RotX),
			Frame(Rotation::RPY(0.0, 0.0, 0.0)) * Frame(Vector(0.135, 0.0, 0.0))));

		robotChain.addSegment(Segment(Joint(Joint::None),
			Frame(Rotation::RPY(0.0, M_PI_2, 0.0)) * Frame(Vector(0.0, 0.0, 0.0))));
	}

	return robotChain;
	
}

JntArray* Data::JointLimits(string robotName)
{
	JntArray limits[2];

	if (robotName == "UR5e") {
		limits[0] = JntArray(6);
		for (int i = 0; i < 6; i++) {
			limits[0](i) = -PI;
		}
		limits[1] = JntArray(6);
		for (int i = 0; i < 6; i++) {
			limits[1](i) = PI;
		}
	}

	return limits;
}
*/