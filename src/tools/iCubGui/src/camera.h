/*
 * camera.h
 */

#ifndef CAMERA_H
#define CAMERA_H

class Camera
{
  public:
    Camera();

    void rotate(float x, float y);
    void pan(float x, float y, float z);
    void setModelView();
    void reset();

    float xRotation() const;
    float yRotation() const;

  private:
    float rotX, rotY;
    float panX, panY, panZ;
};

#endif
