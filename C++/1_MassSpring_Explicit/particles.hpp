#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#include "integration.hpp"
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

class Particle;
class Force;

class ParticleSystem: public PhysicalSystem {
public:
    std::vector<Particle*> particles;
    std::vector<Force*> forces;
    void init();          // initialize the system
    void step(double dt); // perform a time step of length dt
    void draw();          // draw everything
    // PhysicalSystem functions, see integration.hpp
    int getDOFs();
    void getState(VectorXd &x, VectorXd &v);
    void setState(const VectorXd &x, const VectorXd &v);
    void getInertia(MatrixXd &M);
    void getForces(VectorXd &f);
    void getJacobians(MatrixXd &Jx, MatrixXd &Jv);
};

class Particle {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int i;      // index
    double m;   // mass
    Vector2d x; // position
    Vector2d v; // velocity
    Particle(int i, double m, const Vector2d& x, const Vector2d& v): i(i), m(m), x(x), v(v) {}
    void draw();
};

class Force {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void addForces(VectorXd &f) = 0;
    virtual void addJacobians(MatrixXd &Jx, MatrixXd &Jv) = 0;
    virtual void draw() = 0;
};

class DragForce: public Force {
public:
    ParticleSystem *ps; // apply drag to all particles
    double kd;          // drag coefficient
    DragForce(ParticleSystem *ps, double kd): ps(ps), kd(kd) {}
    void addForces(VectorXd &f);
    void addJacobians(MatrixXd &Jx, MatrixXd &Jv);
    void draw();
};

class SpringForce: public Force {
    // connects two particles by a spring
public:
    Particle *p0, *p1; // particles
    double ks, kd;     // spring constant, damping coefficient
    double l0;         // rest length
    SpringForce(Particle *p0, Particle *p1, double ks, double kd, double l0):
        p0(p0), p1(p1), ks(ks), kd(kd), l0(l0) {}
    void addForces(VectorXd &f);
    void addJacobians(MatrixXd &Jx, MatrixXd &Jv);
    void draw();
};

class AnchorForce: public Force {
    // attaches a particle to a fixed point by a spring
public:
	
    Particle *p;   // particle
    Vector2d x;    // point to anchor it to
    double ks, kd; // spring constant, damping coefficient
    AnchorForce(Particle *p, const Vector2d& x, double ks, double kd):
        p(p), x(x), ks(ks), kd(kd) {}
    void addForces(VectorXd &f);
    void addJacobians(MatrixXd &Jx, MatrixXd &Jv);
    void draw();
};
 
#endif
