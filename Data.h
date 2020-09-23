#pragma once
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>

using namespace std;
using namespace KDL;

static class Data{
public:
	static vector<string> JointNames(string);
	static vector<string> SensorNames(string);
	//static Chain Kinematics(string);
	//static JntArray* JointLimits(string);
};

