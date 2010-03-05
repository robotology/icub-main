#include <iCub/YarpImage2Pixbuf.h>

#include <string.h>

bool yarpImage2Pixbuf(yarp::sig::ImageOf<yarp::sig::PixelRgb> *sourceImg, 
                      GdkPixbuf* destPixbuf)
{
	// il pixbuf deve essere già allocato e di dimensioni opportune
	guchar *dst_data;
	char *src_data;
	unsigned int rowstride;
	guchar *p_dst;
	char *p_src;
	unsigned int width, height;
	unsigned int n_channels;
	yarp::sig::PixelRgb srcPixel;
	unsigned int dst_size_in_memory;
	unsigned int src_line_size;

	dst_data = gdk_pixbuf_get_pixels(destPixbuf);
	width = sourceImg->width();
	printf("width: %d \n",width);
	height = sourceImg->height();
	printf("height: %d \n",height);
	rowstride = gdk_pixbuf_get_rowstride (destPixbuf);
	printf("rowstride: %d \n",rowstride);
	n_channels = gdk_pixbuf_get_n_channels (destPixbuf);
	printf("n_channels: %d \n",n_channels);
	dst_size_in_memory = rowstride * height;
	printf("destination size in memory: %d \n",dst_size_in_memory);
	src_line_size = sourceImg->getRowSize(); //GetAllocatedLineSize();
	src_data = (char *) sourceImg->getRawImage(); //GetRawBuffer();
	printf("got raw image from sourceImage \n");
	if ( src_line_size == rowstride)
        {
            memcpy(dst_data, src_data, dst_size_in_memory);
			printf("just copy src_data into dst_data \n");
        }
	else
        {
            for (int i=0; i < (int)height; i++)
                {
                    p_dst = dst_data + i * rowstride;
                    p_src = src_data + i * src_line_size;
                    memcpy(p_dst, p_src, (n_channels*width));
                }
        }
	printf("end of Image2PixBuffer \n");
	return true;
}

