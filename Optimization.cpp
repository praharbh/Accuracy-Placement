#include "Optimization.h"

int itCounter = 0;

// Defining the structure for the optimizer data
typedef struct {
    mdl::Kinematic* kinematics;
    vector<math::Transform> frames;
} optData;

// Function for the error
double errFunc(unsigned n, const double* x, double* grad, void* data)
{
    optData* d = (optData*)data;
    mdl::Kinematic* kinematics = d->kinematics;
    vector<math::Transform> frames = d->frames;

    math::Vector angles = math::Vector(kinematics->getDof());
    for (int i = 0; i < kinematics->getDof(); i++) {
        angles[i] = x[i];
    }

    math::Transform location = math::Transform();
    location.setIdentity();
    location.translation() = Manipulator::FKRL(kinematics, angles).translation();
    vector<math::Transform> reFrames = Read::Relocate(location, frames);

    vector<math::Vector> solutions =
        Constraints::Reachability(kinematics, reFrames);

    double error = Constraints::MaxXe(kinematics, reFrames, solutions, 0.010);

    itCounter++;
    cout << "Iteration: " << itCounter << " Error: " << error;
    
    return error;
}

// Function of the reach constraint
double reachConstraint(unsigned n, const double* x, double* grad, void* data)
{
    optData* d = (optData*)data;
    mdl::Kinematic* kinematics = d->kinematics;
    vector<math::Transform> frames = d->frames;

    math::Vector angles = math::Vector(kinematics->getDof());
    for (int i = 0; i < kinematics->getDof(); i++) {
        angles[i] = x[i];
    }

    math::Transform location = math::Transform();
    location.setIdentity();
    location.translation() = Manipulator::FKRL(kinematics, angles).translation();
    vector<math::Transform> reFrames = Read::Relocate(location, frames);

    vector<math::Vector> solutions =
        Constraints::Reachability(kinematics, reFrames);

    if (solutions.size() - frames.size() != 0) {
        cout << ", Not reachable." << endl;
    }
    else {
        cout << "." << endl;
    }

    return solutions.size()-frames.size();
}

// Method to optimize the part placement
math::Vector Optimization::AMPlace(mdl::Kinematic* kinematics,
    vector<math::Transform> frames, vector<Motor*> motors)
{
    int dof = kinematics->getDof();

    math::Vector angles = math::Vector(dof);
    angles.setZero();

    if (kinematics->getDof() != motors.size()) {
        cout << "optimizer DOF mismatch!" << endl;
        return angles;
    }

    nlopt::opt opt(nlopt::LN_COBYLA, dof);

    vector<double> lb(dof);
    vector<double> ub(dof);
    for (int i = 0; i < dof; i++) {
        lb[i] = motors[i]->getMinPosition();
        ub[i] = motors[i]->getMaxPosition();
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    optData taskData = { kinematics, frames };

    opt.set_min_objective(errFunc, &taskData);
  
    opt.add_equality_constraint(reachConstraint, &taskData, 0);

    opt.set_xtol_rel(1e-3);
    std::vector<double> x(dof);
    for (int j = 0; j < dof; j++) {
        x[j] = 0;
    }

    double minf;

    try {
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << "," <<
            x[2] << "," << x[3] << "," << x[4] << "," << x[5]  << ") = "
            << minf << std::endl;
    }
    catch (std::exception& e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    for (int k = 0; k < dof; k++) {
        angles(k) = x[k];
    }
    return angles;
}

