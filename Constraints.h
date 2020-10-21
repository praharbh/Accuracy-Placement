#pragma once

#define _USE_MATH_DEFINES

#include <rl/mdl/Model.h>
#include <rl/mdl/Kinematic.h>
#include <rl/math/Transform.h>

#include "Manipulator.h"

using namespace std;

static class Constraints
{
public:
	static vector<math::Vector> Reachability(mdl::Kinematic*, 
		vector<math::Transform>);
	static double Speed(vector<math::Transform>, vector<math::Vector>,
		double);
	static double Position(mdl::Kinematic*, vector<math::Vector>);
	static double MaxXe(mdl::Kinematic*, vector<math::Transform>,
		vector<math::Vector>, double);
};

