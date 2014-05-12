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

/*****Functions*****/

/*****
 *  Function to apply a 2-D Convolution to an image
 *  Input:  image - the image matrix that we are going running the convolution on
 *          con_mat - this is the matrix that will be used to convolude the image
 *          con_image - this is the matrix that will be "returned" and contain the convoluded image
 **/
 void convolude2D(const unsigned short image[MAX_HEIGHT][MAX_WIDTH], const double con_mat[win_size][win_size], double con_image[MAX_HEIGHT][MAX_WIDTH]){
    //var result = newArray(image.length - filter.length + 1);
    int i,j,w,z;
    for (i = 0; i < MAX_HEIGHT; i++) 
    {
        for (j = 0; j < MAX_WIDTH; j++) 
        {
            double sum = 0;
            for (w = 0; w < win_size; w++) 
            {
                if(MAX_HEIGHT - i < win_size) break;

                for (z = 0; z < win_size; z++) 
                {
                    if(MAX_WIDTH - j < win_size) break;
                    sum += (image[w + i][z + j]) * (con_mat[w][z]);
                }
            }

            if(i < MAX_HEIGHT && j < MAX_WIDTH)
                con_image[i][j] = sum;
        }   
    }
}

/*****
 *  Function to create a Gaussian Filter. Found on website <www.programming-techniques.com/2013/02/gaussian-filter-generation-using-cc.html>
 *  Author: Bibek Subedi
 *  Edited: Jonathan Kilby, Shawn Weaver
 *  Input:  gKernel - the matrix that is going to contain your gaussian filter
 * */
 void createFilter(double gKernel[win_size][win_size])
 {
       // set standard deviation to 1.0
    double sigma = 1.0;
    double r, s = 2.0 * sigma * sigma;
 
    // sum is for normalization
    double sum = 0.0;
 
    // generate 3x3 kernel
    int x,y;
    for (x = -1; x <= 1; x++)
    {
        for(y = -1; y <= 1; y++)
        {
            r = sqrt((double)(x*x + y*y));
            gKernel[x + 1][y + 1] = (exp((double)(-(r*r)/s)))/(PI_APR * s);
            sum += gKernel[x + 1][y + 1];
        }
    }
 
    // normalize the Kernel
    int i,j;
    for(i = 0; i < win_size; ++i)
        for(j = 0; j < win_size; ++j)
            gKernel[i][j] /= sum;
}

/*****
 * Function to square a single matrix.
 * Input:   input_matrix - this is the matrix that will be squared
 *          output_matrix - this is the matrix that will be used to store the values
 * */
 void squareMatrix(const double input_matrix[MAX_HEIGHT][MAX_WIDTH], double output_matrix[MAX_HEIGHT][MAX_WIDTH])
 {
     int i,j;
     for(i = 0; i < MAX_HEIGHT; i++)
    {
        for(j = 0; j< MAX_WIDTH; j++)
        {
            output_matrix[i][j] = (input_matrix[i][j]*input_matrix[i][j]);
        }
    }
 }
 
/*****
 * Function to multiply two matrices piece by piece.
 * Input:   A - this is the first matrix
 *          B - this is the second matrix
 *          output_matrix - this is the matrix where the values will be stored
 * */
 void multMatrix(const double A[MAX_HEIGHT][MAX_WIDTH], const double B[MAX_HEIGHT][MAX_WIDTH], double output_matrix[MAX_HEIGHT][MAX_WIDTH])
 {
     
     int i,j;
     for(i = 0; i < MAX_HEIGHT; i++)
    {
        for(j = 0; j< MAX_WIDTH; j++)
        {
            output_matrix[i][j] = (A[i][j]*B[i][j]);
        }
    }
     
 }

/*****
 * Function to subtract two matrices piece by piece. A - B
 * Input:   A - this is the first matrix
 *          B - this is the second matrix
 *          output_matrix - this is the matrix where the values will be stored
 * */
 void subtractMatrix(const double A[MAX_HEIGHT][MAX_WIDTH], const double B[MAX_HEIGHT][MAX_WIDTH], double output_matrix[MAX_HEIGHT][MAX_WIDTH])
 {
     
     int i,j;
     for(i = 0; i < MAX_HEIGHT; i++)
    {
        for(j = 0; j< MAX_WIDTH; j++)
        {
            output_matrix[i][j] = (A[i][j]-B[i][j]);
        }
    }
     
 }
 
 
/*****
 * Function to add two matrices piece by piece. A + B
 * Input:   A - this is the first matrix
 *          B - this is the second matrix
 *          output_matrix - this is the matrix where the values will be stored
 * */
 void addMatrix(const double A[MAX_HEIGHT][MAX_WIDTH], const double B[MAX_HEIGHT][MAX_WIDTH], double output_matrix[MAX_HEIGHT][MAX_WIDTH])
 {
     
     int i,j;
     for(i = 0; i < MAX_HEIGHT; i++)
    {
        for(j = 0; j< MAX_WIDTH; j++)
        {
            output_matrix[i][j] = (A[i][j]+B[i][j]);
        }
    }
     
 }


/*****
 * Function to multiply a matrix by a scalar. c * B
 * Input:   c - this is the scalar factor
 *          B - this is the matrix
 *          output_matrix - this is the matrix where the values will be stored
 * */
 void scalarMatrix(double scalar, const double B[MAX_HEIGHT][MAX_WIDTH], double output_matrix[MAX_HEIGHT][MAX_WIDTH])
 {
     
     int i,j;
     for(i = 0; i < MAX_HEIGHT; i++)
    {
        for(j = 0; j< MAX_WIDTH; j++)
        {
            output_matrix[i][j] = (scalar*B[i][j]);
        }
    }
     
 }


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
printf("ss = %d \n",screensize);
printf("rs = %d \n",row_size);
printf("cs = %d \n",col_size);

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
unsigned short rgb_image[480][640];
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

//offset = ;
for (i = 0; i < 480; i++)
for (j=0; j< 640; j++) {
offset = (j+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (i+vinfo.yoffset)*finfo.line_length;

y0 = *((unsigned char *) (buffer+i*640*2 +j*2 + 0));
u = *((unsigned char *) (buffer+i*640*2  +j*2 + 1)) - 128;
//y1 = *((unsigned char *) (buffer+i*640*2 +j*2 + 2));
v = *((unsigned char *) (buffer+i*640*2  +j*2 + 3)) - 128;

r[0] = y0 +((int)(1.13983 *(v))>>16);
//r[1] = y1 + ((int)(1.13983 * (v))>>16);

g[0] = y0 - ((int)(.39465 * (u)- .58060 * (v))>>16);
//g[1] = y1 - ((int)(.39465 * (u)- .58060 *(v))>>16);

b[0] = y0 + ((int)(2.03211 * (u))>>16);
//b[1] = y1 + ((int)(2.03211 * (u))>>16);

rgb_image[i][j] = ((((r[0]>>3))<<11) | (((g[0]>>2))<<5) | ((b[0]>>3)));
//rgb_image[i+2][j+2] = ((((r[1]>>3))<<11) | (((g[1]>>2))<<5) | ((b[1]>>3)));

//*((unsigned short*)(fbp + offset)) = rgb_image[i][j];/*((((r[0]>>3))<<11) | (((g[0]>>2))<<5) | ((b[0]>>3)));*/

//*((unsigned short*)(fbp + offset + 2)) = rgb_image[i+2][j+2];/*((((r[1]>>3))<<11) | (((g[1]>>2))<<5) | ((b[1]>>3)));*/
}
/*
int offset2 = 0;
for (i=0; i< 480; i++)
{
for (j=0; j< 640; j++)
{
//(*((unsigned short *) (fbp + i*(row_size) + j*2)) = (red << 11) | (green << 5) | blue;
offset2 = (j+vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (i+vinfo.yoffset) * (finfo.line_length);
*((unsigned short *) (fbp + offset2)) = rgb_image[i][j];
}
}*/
//Do the Harris Corner
	//The matrices for the derivative in X and Y
	double dx[win_size][win_size] = {{-1,0,1},{-1,0,1},{-1,0,1}};
	double dy[win_size][win_size] = {{-1,-1,-1},{0,0,0},{1,1,1}};
    
    	//The matrix to hold our gaussian filter
	double gKernel[5][5];
    
    	//The matrices that will be the image derivatives
	double Ix[MAX_HEIGHT][MAX_WIDTH];
	double Iy[MAX_HEIGHT][MAX_WIDTH];
	
	//The matrices that will be our squares
	double Ix2_t[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double Iy2_t[MAX_HEIGHT][MAX_WIDTH]; //temp
	double Ix2y2_t[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double Ixy_t[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double Ixy2_t[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double first_half_matrix[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double second_half_matrix[MAX_HEIGHT][MAX_WIDTH]; //temp
    	double Ix2[MAX_HEIGHT][MAX_WIDTH];
    	double Iy2[MAX_HEIGHT][MAX_WIDTH];
    	double Ixy[MAX_HEIGHT][MAX_WIDTH];
    	double harris_measure[MAX_HEIGHT][MAX_WIDTH];
    
   
    	//We need to apply the 2-D convolution
	convolude2D(gray_image,dx,Ix);
	convolude2D(gray_image,dy,Iy); 
        
    	//We need to generate a Gaussian Filter to use
    	createFilter(gKernel);
    
    	//Square the image derivative matrices and multiply them together
    	squareMatrix(Ix,Ix2_t);
    	squareMatrix(Iy,Iy2_t);
    	multMatrix(Ix,Iy,Ixy_t);
    
    	//Next we smooth the image squares
    	convolude2D(Ix2_t,gKernel, Ix2);
    	convolude2D(Iy2_t,gKernel, Iy2);
    	convolude2D(Ixy_t,gKernel, Ixy);
    
    	//First we need to get some temporary values again so we can do operations to get the Harris Corner Measure
    	multMatrix(Ix2, Iy2, Ix2y2_t);                      // Ix2 * Iy2
    	squareMatrix(Ixy, Ixy2_t);                          // Ixy ^ 2
    	subtractMatrix(Ix2y2_t, Ixy2_t, first_half_matrix); // (Ix2 * Iy2 - Ixy ^ 2)
    	addMatrix(Ix2, Iy2, Ix2y2_t);                        // (Ix2 + Iy2)
    	squareMatrix(Ix2y2_t,Ixy2_t);                        // (Ix2 + Iy2)^2
    	scalarMatrix(K, Ixy2_t, second_half_matrix);         // k*(Ix2 + Iy2)^2
    
    	//Now we need to calculate the Harris corner measure
    	subtractMatrix(first_half_matrix, second_half_matrix, harris_measure); // (Ix2 * Iy2 - Ixy ^ 2) - k * (Ix2 + Iy2) ^ 2;
    
    	//Compare the harris_measure to the grayscale image and find where they match (maxima) and make sure that they are greater than our threshold and store the locations of the points that match so that we can draw things in those places or around them
	
	//Testing for now display just image when we check for maxima draw image unless past threshold draw RED
	for(i=0;i<480;i++){
		for(j=0;j<640;j++){
			offset = (j+vinfo.xoffset)*(vinfo.bits_per_pixel/8) + (i+vinfo.yoffset)*finfo.line_length;
			*((unsigned short*)(fbp + offset)) = gray_image[i][j];
		}
	}
        


}

munmap(fbp, screensize);
close(fd);

}
