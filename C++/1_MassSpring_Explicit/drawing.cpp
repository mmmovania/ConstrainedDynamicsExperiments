#include "drawing.hpp"

#include <GL/glut.h>

void point(const Vector2d& x) {
    glPointSize(5);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    glVertex2d(x(0), x(1));
    glEnd();
}

void line(const Vector2d& x0, const Vector2d& x1) {
    glLineWidth(2);
    glColor3f(0,0,0);
    glBegin(GL_LINES);
    glVertex2d(x0(0), x0(1));
    glVertex2d(x1(0), x1(1));
    glEnd();
}
