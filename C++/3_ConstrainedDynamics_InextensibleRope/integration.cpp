#include "integration.hpp"
#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
using namespace std;

void forwardEulerStep(PhysicalSystem *system, double dt) {
    int n = system->getDOFs();
    static VectorXd x0(n), v0(n);
    system->getState(x0, v0); 
    static MatrixXd Mi(n,n);
    system->getInverseInertia(Mi);
    static VectorXd f0(n);
    system->getForces(f0);
    static VectorXd a0(n); // acceleration
    a0 = Mi*f0;
    x0 = x0 + v0*dt;
    v0 = v0 + a0*dt;
    system->setState(x0, v0);
}

VectorXd solve(const MatrixXd &A, const VectorXd &b) {
    SparseMatrix<double> spA = A.sparseView();
    ConjugateGradient< SparseMatrix<double> > solver;
    return solver.compute(spA).solve(b);
}

void backwardEulerStep(PhysicalSystem *system, double dt) {
    int n = system->getDOFs();
    static VectorXd x0(n), v0(n);
    system->getState(x0, v0);
    static MatrixXd M(n,n);
    system->getInertia(M);
    static VectorXd f(n);
    static MatrixXd Jx(n,n), Jv(n,n);
    system->getForces(f);
    system->getJacobians(Jx, Jv);
    MatrixXd A = M - Jx*dt*dt - Jv*dt;
    VectorXd b = (f + Jx*v0*dt)*dt;
    VectorXd dv = solve(A, b);
    VectorXd v1 = v0 + dv;
    system->setState(x0 + v1*dt, v1);
}

void projectPositions(ConstrainedSystem *system) {
    int n = system->getDOFs(),	m = system->getConstraints();
	static VectorXd c(m);			system->getConstraintValues(c);
    static MatrixXd J(m,n);			system->getConstraintJacobian(J);
	static MatrixXd Mi(n, n);		system->getInverseInertia(Mi);
	//printf("Rows: %d Cols: %d", c.rows(), c.cols());
    MatrixXd A = J*Mi*J.transpose();

	//printf("Rows: %d Cols: %d", A.rows(), A.cols());

    VectorXd lambda = solve(A, -c); 
    static VectorXd dx(n);		    dx = Mi*J.transpose()*lambda;
    static VectorXd x(n), v(n);		system->getState(x, v);
    system->setState(x + dx, v);
}

void projectVelocities(ConstrainedSystem *system) {
    int n = system->getDOFs(), m = system->getConstraints();
    static VectorXd x(n), v(n);
    system->getState(x, v);
	v = v * 0.995; //global dampening
    static MatrixXd J(m,n);
    system->getConstraintJacobian(J);
    static MatrixXd Mi(n,n);
    system->getInverseInertia(Mi);
    MatrixXd A = J*Mi*J.transpose();
    VectorXd b = -J*v;
    VectorXd lambda = solve(A, b);
    static VectorXd dv(n);
    dv = Mi*J.transpose()*lambda;
    system->setState(x, v + dv);
}

void constrainedForwardEulerStep(ConstrainedSystem *system, double dt) {
    forwardEulerStep(system, dt);
    projectPositions(system);
    projectVelocities(system);
}

void constrainedBackwardEulerStep(ConstrainedSystem *system, double dt) {
    // Let's not bother with this one.
	backwardEulerStep(system, dt);
	projectPositions(system);
	projectVelocities(system);
}
