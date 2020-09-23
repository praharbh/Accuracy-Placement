#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include<vector>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

/*
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
*/

#include <rl/mdl/Model.h>
#include <rl/mdl/Kinematic.h>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/NloptInverseKinematics.h>

using namespace std;
using namespace webots;
//using namespace KDL;
using namespace rl;

static class Manipulator
{
public:
	static vector<Motor*> JointMotors(Robot*, vector<string>);
	static vector<PositionSensor*> JointSensors(Robot*, vector<string>, int);
	static void Simulate(vector<Motor*>, math::Vector);
	static math::Transform FKRL(mdl::Kinematic*, math::Vector);
	static bool IKRL(mdl::Kinematic*, math::Transform);
	static math::Vector Poll(vector<PositionSensor*>);
	static math::Vector Random(vector<Motor*>);

	//static Frame FKKDL(Chain, JntArray);
	//static JntArray IKKDL(JntArray, Chain, Frame);
};

