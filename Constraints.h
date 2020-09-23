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
	static math::Vector Speed(vector<math::Transform>, vector<math::Vector>,
		double);
	static math::Vector Position(vector<math::Vector>);
};

