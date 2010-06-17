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
	height = sourceImg->height();
	rowstride = gdk_pixbuf_get_rowstride (destPixbuf);
	n_channels = gdk_pixbuf_get_n_channels (destPixbuf);
	dst_size_in_memory = rowstride * height;
	src_line_size = sourceImg->getRowSize(); //GetAllocatedLineSize();
	src_data = (char *) sourceImg->getRawImage(); //GetRawBuffer();

	if ( src_line_size == rowstride)
        {
            memcpy(dst_data, src_data, dst_size_in_memory);
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

	return true;
}

bool yarpImage2Pixbuf(yarp::sig::ImageOf<yarp::sig::PixelMono> *sourceImg, 
                      GdkPixbuf* destPixbuf)
{
	// il pixbuf should already been allocated with the right dimensions
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
	printf("width=%d /n",width);
	height = sourceImg->height();
	printf("height=%d /n",height);
	rowstride = gdk_pixbuf_get_rowstride (destPixbuf);
	printf("rowstride=%d /n",rowstride);
	n_channels = gdk_pixbuf_get_n_channels (destPixbuf);
	printf("n_channels=%d /n",n_channels);
	n_channels=1;
	dst_size_in_memory = rowstride * height; //960*240
	dst_size_in_memory = width * height; //960*240
	src_line_size = sourceImg->getRowSize(); //GetAllocatedLineSize();
	printf("src_line_size=%d /n",src_line_size);
	src_data = (char *) sourceImg->getRawImage(); //GetRawBuffer();
	if ( src_line_size == rowstride)
        {
            memcpy(dst_data, src_data, dst_size_in_memory);
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

	return true;
}
