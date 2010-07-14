/*
 * settings.h
 */

#ifndef SETTINGS_H
#define SETTINGS_H

/**
	@author Zi Ree <Zi Ree @ Second Life>
*/

class Settings
{
  public:
    Settings();
    ~Settings();

    static void setFog(bool on);
    static bool fog();

    static void setFloorTranslucency(int value);
    static int  floorTranslucency();

    static void setEaseIn(bool on);
    static bool easeIn();
    static void setEaseOut(bool on);
    static bool easeOut();
};

#endif
