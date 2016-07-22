#ifndef OPENGL_HPP
#define OPENGL_HPP

#if __APPLE__ & __MACH__
#include <GLUT/glut.h>
#elif defined(_WIN32)
#include <Windows.h>
#include <GL/freeglut.h>
#else
#include <GL/freeglut.h>
#endif

#endif
