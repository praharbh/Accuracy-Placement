#include "Optimization.h"

typedef struct {
    mdl::Kinematic* kinematics;
    vector<math::Transform> frames;
} my_func_data;

double myfunc(unsigned n, const double* x, double* grad, void* data)
{
    my_func_data* d = (my_func_data*)data;
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

    double error = 0;

    error += Constraints::Speed(reFrames, solutions, 0.020).norm();
    error += Constraints::Position(kinematics, solutions);

    return error;
}

vector<math::Vector> Optimization::AMPlace(mdl::Kinematic* kinematics,
    vector<math::Transform> frames, vector<Motor*> motors)
{
    if (kinematics->getDof() != motors.size()) {
        cout << "optimizer DOF mismatch!" << endl;
        return vector<math::Vector>();
    }

    int dof = kinematics->getDof();

    nlopt::opt opt(nlopt::LD_MMA, dof);

    vector<double> lb(dof);
    vector<double> ub(dof);
    for (int i = 0; i < dof; i++) {
        lb[i] = motors[i]->getMinPosition();
        ub[i] = motors[i]->getMaxPosition();
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    opt.set_min_objective(myfunc, NULL);
    
    /*my_constraint_data data[2] = { {2,0}, {-1,1} };
    opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
    opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);
    opt.set_xtol_rel(1e-4);
    std::vector<double> x(2);
    x[0] = 1.234; x[1] = 5.678;
    double minf;

    try {
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
            << std::setprecision(10) << minf << std::endl;
    }
    catch (std::exception& e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
    */
    return vector<math::Vector>();
}

