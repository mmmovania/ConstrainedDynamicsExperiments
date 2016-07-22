#include "particles.hpp"
#include "opengl.hpp"

int w = 600, h = 600;                          // size of window in pixels
float xmin = 0, xmax = 1, ymin = 0, ymax = 1; // range of coordinates drawn
float dt = 0.01f;                              // time step

ParticleSystem psys;
AnchorForce mouseForce(NULL, Vector2d(0, 0), 500, 1);

void display() { 
    glClear(GL_COLOR_BUFFER_BIT);  
    psys.draw();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    ::w = w;
    ::h = h;
    glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(::xmin, ::xmax, ::ymin, ::ymax);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle() {
    psys.step(::dt);
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            ::mouseForce.x = Vector2d(xmin + (double)x/w * (xmax - xmin),
                                      ymax - (double)y/h * (ymax - ymin));
            // find nearest particle
            double dmin = 0.2; // ignore particles farther than 0.2
            for (size_t i = 0; i < psys.particles.size(); i++) {
                Particle *p = psys.particles[i];
                double d = (p->x - ::mouseForce.x).norm();
                if (d < dmin) {
                    ::mouseForce.p = p;
                    dmin = d;
                }
            }
        } else if (state == GLUT_UP) {
            ::mouseForce.p = NULL;
        }
    }
}

void motion(int x, int y) {
    ::mouseForce.x = Vector2d(xmin + (double)x/w * (xmax - xmin),
                              ymax - (double)y/h * (ymax - ymin));
}

void init() {
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClearColor(1, 1, 1, 1);
}

int main(int argc, char **argv) {
    psys.init();
    psys.forces.push_back(&mouseForce); 
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_MULTISAMPLE);
    glutInitWindowSize(::w, ::h);
    glutCreateWindow("Animation");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

	init();
    glutMainLoop();
}
