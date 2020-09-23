#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <sstream>

//#include <kdl/frames_io.hpp>

#include <rl/math/Transform.h>

using namespace std;
//using namespace KDL;
using namespace rl;

static class Read
{
public:
	static vector<math::Transform> CSV(string);
	static vector<math::Transform> Relocate(math::Transform, 
		vector<math::Transform>);
};

