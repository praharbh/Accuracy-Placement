#pragma once

#define _USE_MATH_DEFINES

#include <rl/mdl/Model.h>
#include <rl/mdl/Kinematic.h>
#include <rl/math/Transform.h>

#include <nlopt.hpp>

#include "Manipulator.h"
#include "Read.h"
#include "Constraints.h"

using namespace std;

class Optimization
{
public:
	vector<math::Vector> AMPlace(mdl::Kinematic*,
		vector<math::Transform>, vector<Motor*>);
	//double myfunc(unsigned, const double*, double*, void*);
};

