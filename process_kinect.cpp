
/*
This is a program that reads the depth and color
image from the kinect sensor and track
the person's head using Tracker based on
Kalman filter estimates and hangarian assignments.
It counts the number of people going in and out
of the door
-----------------------------------------------

Below is the distance calculation to find the 
3D coordinate using depth

//distance = 0.1236 * tan(rawDisparity / 2842.5 + 1.1863)
//x = (i - w / 2) * (z + minDistance) * scaleFactor
//y = (j - h / 2) * (z + minDistance) * scaleFactor
//z = z
//Where
//minDistance = -10
//scaleFactor = .0021.
-----------------------------------------------
The Program uses, OpenNI2, with freekinect library. Uses 
opencv to handle the matrix operations. It also uses
code obtained from cpp implementation of 2D K tracker
from Андрей Смородов

written by Engr. Waqar S. Qureshi

*/

#include <stdio.h>
#include <OpenNI.h>
#include<iostream>
#include "OniSampleUtilities.h"

/*--------------------------
Open CV header files here*/
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "depthColor.h"
//--------------------------------------

using namespace cv;
using namespace std;
int keyboard; //input from keyboard
//--------------------------
void help()
{
    cout
    << "--------------------------------------------------------------------------" << endl
    << "This program uses kinnect for depth map or a pre-recorded depth maps     "  << endl
    << "         You can process both videos (-vid) or kinnect via in real time. "  << endl
                                                                                    << endl
    << "Usage:"                                                                     << endl
    << "./bs {-vid <video filename>|-img <image filename>}"                         << endl
    << "for example: ./bs -vid video.avi"                                           << endl
    << "or: ./bs                          "                                         << endl
    << "--------------------------------------------------------------------------" << endl
    << endl;
}

//--------------------------
int main(int argc, char** argv)
{
	//This is an object of class status to check if the status is ok
	openni::Status rc = openni::STATUS_OK;
//------------------------------------------------------------------------------
	//Device and video stream intialization
	openni::Device device;
	openni::VideoStream depth, color;
	openni::VideoFrameRef frame_depth;
	openni::VideoFrameRef frame_color;
	openni::VideoStream* p_depth = &depth ;
	openni::VideoStream* p_color = &color ;
	openni::VideoStream**		multi_streams;
	multi_streams = new openni::VideoStream*[2]; // pointer to pointer to multiple video streams
	multi_streams[0] = &depth;
	multi_streams[1] = &color;
	int depth_cols;
	int depth_rows;
	cv::Mat depthImage; 
	cv::Mat showDepth;
	cv::Mat depthColorShow(480,640,CV_8UC3);;
	cv::Mat colorImage;
	int color_cols;
	int color_rows;
//------------------------------------------------
// opencv video capture object creationCV_FOURC
	VideoWriter depthVideo, colorVideo;
	depthVideo.open("./depth.avi", CV_FOURCC('M','J','P','G') , 25, Size(640,480), true); // Size(1280,720)
	colorVideo.open("./color.avi", CV_FOURCC('M','J','P','G') ,25, Size(640,480), true); // Size(1280,720)

//------------------------------------------------

	const char* deviceURI = openni::ANY_DEVICE;
//--------------------------------------------------------------------------------
	// check if you have given device URI or path to oni file
	if (argc > 1)
	{
		deviceURI = argv[1];
	}
	// intialize the OPENNI library
	rc = openni::OpenNI::initialize();
	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	// Check if the device to be used is available for use
	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("Test: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}
	device.setDepthColorSyncEnabled( true );

	// check if the registration mode is available and then set the image registration mode
	// Image registration mode is available for kinnect and for details look into libfreenect in OpenNI2 driver DerviceDriver.cpp
	// It is supported under device property not as a device property.
	//check if you can create depth sensor Video stream
	if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			rc = depth.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Test: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				depth.destroy();
			}
			else
			{
				const openni::SensorInfo& si = depth.getSensorInfo();
				cout << "Depth supported res " << endl;
				for (int i=0; i<si.getSupportedVideoModes().getSize(); i++)
				{
					cout << i << ": " << si.getSupportedVideoModes()[i].getResolutionX() << "x" << si.getSupportedVideoModes()[i].getResolutionY()<< ": " << PixelFormatToStr(si.getSupportedVideoModes()[i].getPixelFormat())  <<" Fps:"<<si.getSupportedVideoModes()[i].getFps()<< endl;
				}
				depth.setVideoMode(si.getSupportedVideoModes()[0]);
				device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
				openni::VideoMode vm_depth = depth.getVideoMode();
				depth_cols=vm_depth.getResolutionX();
				depth_rows=vm_depth.getResolutionY();
				depthImage.create(depth_rows, depth_cols, CV_16UC1);//opencv depth image
			}
		}
		else
		{
			printf("Test: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		}

	
	}

	// check of color sensor stream
	if (device.getSensorInfo(openni::SENSOR_COLOR) != NULL)
	{
		rc = color.create(device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			rc = color.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Test: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
				color.destroy();
			}
			else
			{
				const openni::SensorInfo& si = color.getSensorInfo();
				cout<<"Color supported res"<<endl;
				for (int i=0; i<si.getSupportedVideoModes().getSize(); i++)
				{
				  cout << i << ": " << si.getSupportedVideoModes()[i].getResolutionX() << "x" << si.getSupportedVideoModes()[i].getResolutionY()<< ": " << PixelFormatToStr(si.getSupportedVideoModes()[i].getPixelFormat()) <<" Fps:"<<si.getSupportedVideoModes()[i].getFps()<< endl;//getFps
				}
				color.setVideoMode(si.getSupportedVideoModes()[0]);
				openni::VideoMode vm_color = color.getVideoMode();
				color_cols=vm_color.getResolutionX();
				color_rows=vm_color.getResolutionY();
				colorImage.create(color_rows,color_cols,CV_8UC3);// opencv color image creation
			}
		}
		else
		{
			printf("Test: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		}
	}

// Now check if still the depth color  stream are not working then shut down the OPENNI

		if (!depth.isValid() || !color.isValid())
	{
		printf("Test: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}



	// now reading the video stream and then converting it to opencv
	while ((char)keyboard != 'q' && (char)keyboard != 27 )
	{
	  int changedIndex;
	  openni::DepthPixel* dData;
	  openni::RGB888Pixel* cData;
	  p_depth->readFrame(&frame_depth); 
	  p_color->readFrame(&frame_color); 
	  if (frame_depth.isValid()) 
	  {
	    dData = (openni::DepthPixel*)frame_depth.getData();
	    memcpy(depthImage.data,dData, depth_rows*depth_cols*sizeof(uint16_t) );
	    depthImage.convertTo(showDepth,CV_8U,255/4096.0,0); //
			cvtColor(showDepth,depthColorShow,COLOR_GRAY2RGB);
			
		}
	  if (frame_color.isValid())
	  {
	      cData = (openni::RGB888Pixel*)frame_color.getData();
	      const int dim = 3;
	      memcpy( colorImage.data,cData, dim*color_rows*color_cols*sizeof(uint8_t) );
	      cvtColor(colorImage, colorImage, COLOR_BGR2RGB);
	  }
	  
//---------------------------------------

//---------------------------------------
//convert the depth image to color for show
	  cv::imshow("depth",showDepth);
	  //cv::imshow("depth2",depthColorShow);
	  cv::imshow("Color",colorImage);
	  depthVideo.write(depthColorShow);
	  cv::imwrite("my_bitmap.bmp", showDepth);
	  colorVideo.write(colorImage);
	  keyboard = waitKey( 30 ); //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	  usleep(2*1000);

	}	

	depth.stop();
	color.stop();
	depth.destroy();
	color.destroy();
	device.close();
	openni::OpenNI::shutdown();
	destroyAllWindows();
	
	return EXIT_SUCCESS;;

}
/*
//image.at<cv::Vec3b>(y,x); gives you the RGB (it might be ordered as BGR) vector of type cv::Vec3b
			//image.at<cv::Vec3b>(y,x)[0] = newval[0];
			//image.at<cv::Vec3b>(y,x)[1] = newval[1];
			//image.at<cv::Vec3b>(y,x)[2] = newval[2]
*/
