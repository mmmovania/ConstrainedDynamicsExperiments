#ifndef INTEGRATION_HPP
#define INTEGRATION_HPP

#include <Eigen/Dense>
using namespace Eigen;

class PhysicalSystem {
    // parent class for physical systems that support implicit integration
public:
    // return number of *positional* degrees of freedom (not velocity)
    virtual int getDOFs() = 0;
    // write position and velocity into vectors
    virtual void getState(VectorXd &x, VectorXd &v) = 0;
    // read position and velocity from vectors
    virtual void setState(const VectorXd &x, const VectorXd &v) = 0;
    // write mass matrix
    virtual void getInertia(MatrixXd &M) = 0;
    // write forces
    virtual void getForces(VectorXd &f) = 0;
    // write Jacobians
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv) = 0;
};

// perform a forward Euler step of length dt
void forwardEulerStep(PhysicalSystem *system, double dt);

// perform a backward Euler step of length dt
void backwardEulerStep(PhysicalSystem *system, double dt);

#endif
