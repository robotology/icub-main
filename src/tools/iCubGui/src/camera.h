/*
 * camera.h
 */

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
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


