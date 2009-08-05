#ifndef __VIDEO_TEXTURE__
#define __VIDEO_TEXTURE__

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file VideoTexture.h
 * \brief Header file for the video stream texture
 * \author Paul Fitzpatrick
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/


#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
//#include <yarp/sig/ImageFile.h>
//#include <yarp/sig/ImageDraw.h>

#include <vector>

class TextureInput {
private:
    int textureIndex;
    double lastData;
    int t;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > port;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> cache, input;
	
public:
    TextureInput() {
        textureIndex = -1;
        lastData = -1000;
        t = 0;
    }
	~TextureInput(){
		port.close();
	} 

    bool open(yarp::os::Searchable& config);
    void apply(unsigned int *textures);
};


class VideoTexture {
public:
    std::vector<TextureInput *> inputs;

    ~VideoTexture();

    bool add(const char *port, int textureIndex) {
        yarp::os::Property config;
        config.put("port",port);
        config.put("textureIndex",textureIndex);
        return add(config);
    }

    bool add(yarp::os::Searchable& config);

    void apply(unsigned int *textures);
};


#endif
