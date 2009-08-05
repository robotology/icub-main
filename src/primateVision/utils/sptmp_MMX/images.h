#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>
#define RED 0
#define GREEN 1
#define BLUE 2

static const int r_mask = 0xff0000; 
static const int g_mask = 0x00ff00; 
static const int b_mask = 0x0000ff; 


void pack_rgb (unsigned long * long_img, unsigned char * rgb_img, int size_x, int size_y);

void rgb_to_greyscale (unsigned char * rgb_img, unsigned char * grey_img, int size_x, int size_y);

void greyscale_to_rgb (unsigned char * grey_img, unsigned char * rgb_img, int size_x, int size_y);

void rank_trans (unsigned char * source_img, unsigned char * result_img, int size_x, int size_y, int w);

void pack_long ( unsigned char *  rgb_img, unsigned long * long_img, int size_x, int size_y);

void get_image_region_32int (unsigned long * source, unsigned long * dest,
           int swidth, int sheight, int startx, int starty,
           int dwidth, int dheight);

void get_image_region_rgb (unsigned char * source, unsigned char * dest,
           int swidth, int sheight, int startx, int starty,
           int dwidth, int dheight);

void get_image_region_grey (unsigned char * source, unsigned char * dest,
           int swidth, int sheight, int startx, int starty,
           int dwidth, int dheight);

void get_image_region_rgb_addr (unsigned char * source, unsigned char * dest,
           int swidth, int sheight, int addr,
           int dwidth, int dheight);

void load_image_ppm(char filename[], unsigned long * imagebuf, 
        int image_width, int image_height);

void save_image_ppm (char * filename, unsigned char * image,
         int image_width, int image_height);


void load_image_32int(char filename[], unsigned long * imagebuf, 
          int image_width, int image_height);

void save_image_32int(char * filename, unsigned long * image, 
                           int image_width, int image_height);
