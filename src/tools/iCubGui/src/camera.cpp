/*
 * camera.cpp
 */

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#endif
#include <GL/glu.h>
#endif

#include "camera.h"

Camera::Camera()
{
  reset();
}

void Camera::rotate(float x, float y)
{
  rotX += x;
  if (rotX < 2) rotX = 2;
  if (rotX > 80) rotX = 80;
  rotY += y;
  if (rotY < 0) rotY += 360;
  if (rotY > 360) rotY -= 360;
}

void Camera::pan(float x, float y, float z)
{
  panX -= x/2;
  if (panX < -500) panX = -500;
  if (panX > 500) panX = 500;
  panY += y/2;
  if (panY < 5) panY = 5;
  if (panY > 500) panY = 500;
  panZ += z;
  if (panZ < 10) panZ = 10;
  if (panZ > 1000) panZ = 1000;
}

void Camera::setModelView()
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(panX, panY, panZ, panX, panY, 0, 0, 1, 0);
  glRotatef(rotX, 1, 0, 0);
  glRotatef(rotY, 0, 1, 0);
}

void Camera::reset()
{
  rotX = 10;
  rotY = 0;
  panX = 0;
  panY = 40;
  panZ = 100;
}

float Camera::xRotation() const
{
  return rotX;
}

float Camera::yRotation() const
{
  return rotY;
}
