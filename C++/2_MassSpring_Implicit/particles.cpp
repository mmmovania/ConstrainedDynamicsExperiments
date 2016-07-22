#include "particles.hpp"
#include "opengl.hpp"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

void makeGrid(ParticleSystem *psys);
void makeRope(ParticleSystem* psys);

void ParticleSystem::init() {
    makeGrid(this); // also creates spring forces
	//makeRope(this);
	forces.push_back(new DragForce(this, 0.01));// 5));
}

void ParticleSystem::step(double dt) {
    bool implicit = true;
    if (implicit) {
        backwardEulerStep(this, dt);
    } else {
		int substeps = 50;
        for (int i = 0; i < substeps; i++)
            forwardEulerStep(this, dt/substeps);
    }
}

void ParticleSystem::draw() {
	glLineWidth(2);
	//for cloth draw forces and particles
	
    for (size_t p = 0; p < particles.size(); p++)
        particles[p]->draw();
    for (size_t f = 0; f < forces.size(); f++)
        forces[f]->draw();
	
	
	/*
	//for rope, draw particles
	glBegin(GL_LINES);
	for (size_t p = 0; p < particles.size() - 1; p++) {
		Vector2d p1 = particles[p]->x;
		Vector2d p2 = particles[p+1]->x;
		glVertex2d(p1.x(), p1.y());
		glVertex2d(p2.x(), p2.y());
	}
	glEnd();

	for (size_t p = 0; p < particles.size(); p++)
		particles[p]->draw();
		*/
}

int ParticleSystem::getDOFs() {
    return 2*particles.size();
}

void ParticleSystem::getState(VectorXd &x, VectorXd &v) {
    for (size_t p = 0; p < particles.size(); p++) {
        Particle *particle = particles[p];
        x.segment(p*2, 2) = particle->x;
        v.segment(p*2, 2) = particle->v;
    }
}

void ParticleSystem::setState(const VectorXd &x, const VectorXd &v) {
    for (size_t p = 0; p < particles.size(); p++) {
        Particle *particle = particles[p];
        particle->x = x.segment(p*2, 2);
        particle->v = v.segment(p*2, 2);
    }
}

void ParticleSystem::getInertia(MatrixXd &M) {
    M.setZero();
    for (size_t p = 0; p < particles.size(); p++)
        M.block(p*2,p*2, 2,2) = particles[p]->m*MatrixXd::Identity(2,2);
}

void ParticleSystem::getForces(VectorXd &f) {
    f.setZero();
    for (size_t i = 0; i < forces.size(); i++)
        forces[i]->addForces(f);
}

void ParticleSystem::getJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    Jx.setZero();
    Jv.setZero();
    for (size_t i = 0; i < forces.size(); i++)
        forces[i]->addJacobians(Jx, Jv);
}

void Particle::draw() {
    glPointSize(5);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    glVertex2d(x(0), x(1));
    glEnd();
}

void DragForce::addForces(VectorXd &f) {
    for (size_t p = 0; p < ps->particles.size(); p++) {
        Particle *particle = ps->particles[p];
        f.segment(2*p, 2) += -kd*particle->v;
    }
}

void DragForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    for (size_t p = 0; p < ps->particles.size(); p++)
        Jv.block(2*p,2*p, 2,2) += -kd*Matrix2d::Identity();
}

void DragForce::draw() {
}


void Gravity::addForces(VectorXd &f) { 
	Vector2d g(0.0, amount);
	for (size_t p = 0; p < ps->particles.size(); p++) {
		Particle *particle = ps->particles[p];
		f.segment(2 * p, 2) += g;
	}
}

void Gravity::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
}

void Gravity::draw() {
}

void GroundForce::addForces(VectorXd &f) {
	Vector2d g(0.0, 1);
	for (size_t p = 0; p < ps->particles.size(); p++) {
		Particle *particle = ps->particles[p];
		if (particle->x.y() < yLow) {
			f.segment(2 * p, 2) += g;
		}
	}
}

void GroundForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
}

void GroundForce::draw() {
}

void BoundsForce::addForces(VectorXd &f) {
	Vector2d g(0.0, 1);
	for (size_t p = 0; p < ps->particles.size(); p++) {
		Particle *particle = ps->particles[p];
		if (particle->x.y() < yMin) {
			f.segment(2 * p, 2) += Vector2d(0,1);
		}
		if (particle->x.y() > yMax) {
			f.segment(2 * p, 2) += Vector2d(0, -1);
		}
		if (particle->x.x() < xMin) {
			f.segment(2 * p, 2) += Vector2d(1, 0);
		}
		if (particle->x.x() > xMax) {
			f.segment(2 * p, 2) += Vector2d(-1, 0);
		}
	}
}

void BoundsForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
}

void BoundsForce::draw() {
}

void SpringForce::addForces(VectorXd &f) {
    Vector2d x0 = p0->x, x1 = p1->x, dx = x1 - x0;
    Vector2d v0 = p0->v, v1 = p1->v, dv = v1 - v0;
    double l = (p1->x - p0->x).norm();
    Vector2d dxhat = dx/l;
    Vector2d force = -ks*(l - l0)*dxhat - kd*dv.dot(dxhat)*dxhat;
    f.segment(2*p0->i, 2) -= force;
    f.segment(2*p1->i, 2) += force;
}

void SpringForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    int i0 = p0->i, i1 = p1->i;
    Vector2d x0 = p0->x, x1 = p1->x, dx = x1 - x0;
    double l = (p1->x - p0->x).norm();
    Vector2d dxhat = dx/l;
    Matrix2d jx = -ks*(max(1 - l0/l, 0.0)*(Matrix2d::Identity() - dxhat*dxhat.transpose()) + dxhat*dxhat.transpose());
    Matrix2d jv = -kd*dxhat*dxhat.transpose();
    Jx.block(2*i0,2*i0, 2,2) += jx;    Jx.block(2*i0,2*i1, 2,2) -= jx;
    Jx.block(2*i1,2*i0, 2,2) -= jx;    Jx.block(2*i1,2*i1, 2,2) += jx;
    Jv.block(2*i0,2*i0, 2,2) += jv;    Jv.block(2*i0,2*i1, 2,2) -= jv;
    Jv.block(2*i1,2*i0, 2,2) -= jv;    Jv.block(2*i1,2*i1, 2,2) += jv;
}

void SpringForce::draw() {
	 
	glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
	    glVertex2d(p0->x(0), p0->x(1));
		glVertex2d(p1->x(0), p1->x(1));
    glEnd();
	
}

void AnchorForce::addForces(VectorXd &f) {
    if (p == NULL)
        return;
    Vector2d dx = p->x - x;
    f.segment(2*p->i, 2) += -ks*dx - kd*p->v;
}

void AnchorForce::addJacobians(MatrixXd &Jx, MatrixXd &Jv) {
    if (p == NULL)
        return;
    Jx.block(2*p->i,2*p->i, 2,2) += -ks*Matrix2d::Identity();
    Jv.block(2*p->i,2*p->i, 2,2) += -kd*Matrix2d::Identity();
}

void AnchorForce::draw() {
    if (p == NULL)
        return;
    glLineWidth(2);
    glColor3f(0.8f,0.8f,0.8f);
    glBegin(GL_LINES);
    glVertex2d(x(0), x(1));
    glVertex2d(p->x(0), p->x(1));
    glEnd();
}

void makeGrid(ParticleSystem *psys) {
    const int m = 10, n = 10;                      // resolution of system
    double mass = 1;                               // total mass
    double pmass = mass/((m + 1)*(n + 1));         // mass per particle
    double x0 = 0.3, x1 = 0.7, y0 = 0.3, y1 = 0.7; // extent of rectangle
    double dx = (x1 - x0)/m, dy = (y1 - y0)/n;     // lengths of springs
    double dd = sqrt(dx*dx + dy*dy);
    double ks = 200, kd = 0.1;                     // spring constant, damping
    double ks_diag = 50;
    // create particles
    Particle *particles[m+1][n+1];
    for (int i = 0; i <= m; i++) {
        for (int j = 0; j <= n; j++) {
            Vector2d x(x0+dx*i, y0+dy*j);
            Vector2d v(0, 0);
            int index = psys->particles.size();
            particles[i][j] = new Particle(index, pmass, x, v);
            psys->particles.push_back(particles[i][j]);
        }
    }
    // create springs
    for (int i = 0; i <= m; i++) {
        for (int j = 0; j <= n; j++) {
            Particle *p0 = particles[i][j];
			 
			//structural springs
			if (i < m) {
				Particle *p1 = particles[i + 1][j];
				psys->forces.push_back(new SpringForce(p0, p1, ks, kd, dx));
			}
			if (j < n) {
				Particle *p1 = particles[i][j + 1];
				psys->forces.push_back(new SpringForce(p0, p1, ks, kd, dy));
			}
			//shear spring
			if (i < m && j < n) {
				Particle *p1 = particles[i + 1][j + 1];
				psys->forces.push_back(new SpringForce(p0, p1, ks_diag, kd, dd));
			}
			if (i > 0 && j < n) {
				Particle *p1 = particles[i - 1][j + 1];
				psys->forces.push_back(new SpringForce(p0, p1, ks_diag, kd, dd));
			}
        }
    }
}

void makeRope(ParticleSystem *psys) {
	const int m = 10		;                   // resolution of system
	double mass = 1;                            // total mass
	double pmass = mass / ((m + 1));			// mass per particle

	//for vertical rope
	//double x0 = 0.5, y0 = 0.3, y1 = 0.7;			// extent of rope	
	//double dx = 0, dy = ((y1 - y0) / m);			// lengths of springs
	//double dy2 = dy * 2;

	//for horizontal rope
	double x0 = 0.5, x1=0.9, y0 = 0.7;			// extent of rope	
	double dx = ((x1-x0)/m), dy = 0;			// lengths of springs
	double dy2 = dx * 2;
		
	double ks = 200, kd = 1;                  // spring constant, damping 
	double ks_bend = 1000;

	// create particles
	Particle *particles[m + 1];

	for (int j = 0; j <= m; j++) {
		//Vector2d x(x0, y0 + dy*j); //for vertical rope
		Vector2d x(x0 + dx*j, y0);	//for horizontal rope
		Vector2d v(0, 0);
		int index = psys->particles.size();
		particles[j] = new Particle(index, pmass, x, v);
		psys->particles.push_back(particles[j]);
	}
 
	// create springs
	for (int j = 0; j <= m; j++) {
		Particle *p0 = particles[j]; 
		if (j < m) {
			Particle *p1 = particles[j + 1]; 
			psys->forces.push_back(new SpringForce(p0, p1, ks, kd, dy));
		} 
	}
	 
	//create bend springs
	for (int j = 0; j <= m; j++) {
		Particle *p0 = particles[j];
		if ((j + 2) <= m) {
			
			Particle *p2 = particles[j + 2]; 
			psys->forces.push_back(new SpringForce(p0, p2, ks_bend, kd, dy2));
		}
	}
}

