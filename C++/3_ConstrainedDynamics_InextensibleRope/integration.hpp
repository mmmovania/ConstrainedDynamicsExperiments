#ifndef INTEGRATION_HPP
#define INTEGRATION_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
using namespace Eigen;

typedef SparseVector<double> SparseVectorXd;
typedef SparseMatrix<double> SparseMatrixXd;

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
    // write inverse mass matrix
    virtual void getInverseInertia(MatrixXd &Mi) = 0;
    // write forces
    virtual void getForces(VectorXd &f) = 0;
    // write Jacobians
    virtual void getJacobians(MatrixXd &Jx, MatrixXd &Jv) = 0;
};

class ConstrainedSystem: public PhysicalSystem {
    // parent class for physical systems with constraints
public:
    // return number of constraints
    virtual int getConstraints() = 0;
    // write constraint values into vector
    virtual void getConstraintValues(VectorXd &c) = 0;
    // write constraint gradients into matrix
    virtual void getConstraintJacobian(MatrixXd &J) = 0;
};

// perform a forward Euler step of length dt
void forwardEulerStep(PhysicalSystem *system, double dt);

// perform a backward Euler step of length dt
void backwardEulerStep(PhysicalSystem *system, double dt);

// perform a constrained forward Euler step of length dt
void constrainedForwardEulerStep(ConstrainedSystem *system, double dt);

// perform a constrained backward Euler step of length dt
void constrainedBackwardEulerStep(ConstrainedSystem *system, double dt);

#endif
