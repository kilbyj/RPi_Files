/**********
	Harris Corner Detector in C
	Authors: Jonathan Kilby <jonathan.kilby1991@gmail.com>
		 Shawn Weaver <sweaver699@gmail.com>
                 Aadel Ragaban <aadel.ragaban@gmail.com>
	Date: 	 2014/3/11

	We are writing this Harris Corner in C so that we can later run it in Vivado HLS to convert it to hardware.
	Therefore, there will be lines of code that are specific to the HLS tool that will be noted.

	We are basing the way that we do this implementation from matlab code that was written by Peter Kovesi.
	The code can be found here. <http://www.cs.illinois.edu/~slazebni/spring14/harris.m>
*/

/*****Includes*****/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <string.h>

/*****Defines*****/
#define MAX_HEIGHT  480
#define MAX_WIDTH   640
#define PI_APR      3.14159
#define win_size    3
#define K           .04
#define THRESHOLD   100000

int main()
{

int fd;
long int screensize = 0;
long int row_size = 0;
long int col_size = 0;

fd = open("/dev/fb0", O_RDWR);

if (fd == -1)
{
perror("Error: cannot open framebuffer device");
exit(-1);
}
struct fb_fix_screeninfo finfo;
struct fb_var_screeninfo vinfo;
if (ioctl(fd,FBIOGET_VSCREENINFO, &vinfo) == -1){
 perror("Error reading variable information");
exit(-1);
}
if(ioctl(fd,FBIOGET_FSCREENINFO,&finfo) == -1){
 perror("Error reading fixed information");
 exit(-1);
}
screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
row_size = vinfo.xres * vinfo.bits_per_pixel / 8;
col_size = vinfo.yres * vinfo.bits_per_pixel / 8;
//printf("ss = %d \n",screensize);
//printf("rs = %d \n",row_size);
//printf("cs = %d \n",col_size);

char *fbp;

fbp = (char *)mmap(0,screensize,PROT_READ | PROT_WRITE, MAP_SHARED, fd,0);
if ((int)fbp == -1) {
perror ("Error: failed to map framebuffer device to memory");
exit(-1);
}

int i = 0;
int j = 0, location = 0;
int blue = 255;
int red = 0;
int green = 255;

for (i=0; i< 480; i++)
{
for (j=0; j< 640; j++)
{
//(*((unsigned short *) (fbp + i*(row_size) + j*2)) = (red << 11) | (green << 5) | blue;
location = (j+vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (i+vinfo.yoffset) * (finfo.line_length);
*((unsigned short *) (fbp + location)) = (red << 11) | (green <<5) | blue;
}
}
//webcam
int fd_cap,ret,fd1,y0,u,y1,v,r[2],g[2],b[2],offset;
struct v4l2_format fmt;
struct v4l2_requestbuffers req;
struct v4l2_buffer buf;
fd_set fds;
struct timeval tv = {0};
tv.tv_sec = 2;
void *buffer;

fd_cap = open("/dev/video0",O_RDWR);
if (fd_cap == -1) {
perror("opening video device");
return 0;
}

memset (&fmt,0,sizeof(fmt));
fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
fmt.fmt.pix.width = 640;
fmt.fmt.pix.height = 480;
fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
fmt.fmt.pix.field = V4L2_FIELD_NONE;

if (ioctl(fd_cap, VIDIOC_S_FMT,&fmt)==-1) 
{
perror("setting pixel format");
return 0;
}

if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
fprintf(stderr,"capture device didn't accept RGB24 format\n");
return 0;
}

if ((fmt.fmt.pix.width != 640) || (fmt.fmt.pix.height != 480)) {
fprintf(stderr,"driver is sending image at %d x %d\n",fmt.fmt.pix.width, fmt.fmt.pix.height);
return 0;
}

memset (&req,0,sizeof(req));
req.count = 1;
req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
req.memory = V4L2_MEMORY_MMAP;
if (ioctl(fd_cap,VIDIOC_REQBUFS,&req)==-1) {
perror("requesting buffer");
return 0;
}

memset (&buf,0,sizeof(buf));
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
buf.memory = V4L2_MEMORY_MMAP;
buf.index = 0;

if (ioctl(fd_cap, VIDIOC_QUERYBUF,&buf)==-1) {
perror("querying buffer");
return 0;
}

buffer = mmap(0,buf.length,PROT_READ | PROT_WRITE,MAP_SHARED,fd_cap,buf.m.offset);

if (ioctl(fd_cap, VIDIOC_QBUF, &buf)==-1) {
perror("queing buffer");
return 0;
}

if (ioctl(fd_cap,VIDIOC_STREAMON,&buf.type)==-1) {
perror("start capture");
return 0;
}

FD_ZERO(&fds);
FD_SET(fd_cap, &fds);
unsigned short gray_image[480][640];
while (1) {


ret = select(fd_cap+1,&fds, NULL, NULL, &tv);
if (ret == -1) {
perror("waiting for frame");
return 1;
}

if(ioctl(fd_cap, VIDIOC_DQBUF, &buf) == -1) {
perror("retrieving frame");
return 1;
}
if (ioctl(fd_cap,VIDIOC_QBUF,&buf) == -1) {
perror("queuing buffer");
return 0;
}


for (i = 0; i < 480; i++)
for (j=0; j< 640; j++) {
offset = (j+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (i+vinfo.yoffset)*finfo.line_length;

y0 = *((unsigned char *) (buffer+i*640*2 +j*2 + 0));
u = *((unsigned char *) (buffer+i*640*2  +j*2 + 1)) - 128;

v = *((unsigned char *) (buffer+i*640*2  +j*2 + 3)) - 128;

r[0] = y0 +((int)(1.13983 *(v))>>16);

g[0] = y0 - ((int)(.39465 * (u)- .58060 * (v))>>16);

b[0] = y0 + ((int)(2.03211 * (u))>>16);

gray_image[i][j] = ((((r[0]>>3))<<11) | (((g[0]>>2))<<5) | ((b[0]>>3)));


}


	//Testing for now display just image when we check for maxima draw image unless past threshold draw RED
	for(i=0;i<480;i++){
		for(j=0;j<640;j++){
			offset = (j+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (i+vinfo.yoffset)*finfo.line_length;\
		/*	if(harris_measure[i][j] == gray_image[i][j])
			{
				if (harris_measure[i][j] >= THRESHOLD){
					*((unsigned short*)(fbp + offset)) = (255 << 11) | (0<<5) | 0;
				}

			}*/
			*((unsigned short*)(fbp + offset)) = gray_image[i][j];
		}
	}
        


}

munmap(fbp, screensize);
close(fd);

}
