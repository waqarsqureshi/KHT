
/*
 * This is a program that reads the depth and color
 * image from the kinect sensor and track
 * the person's head using Tracker based on
 * Kalman filter estimates and hangarian assignments.
 * It counts the number of people going in and out
 * of the door

********Compiled and Edited by Waqar S. Qureshi********
 * Parts of the code takes from
 * 1 - OpenNi sample program ( opening kinect)
 * 2 - OpenCV sample program ( background subtraction and )
 * 3 - Андрей Смородов cpp implementation of Kalman tracker ( 2D tracker)
 * 4 - Hangarian assignments implementation ( cost minimization for find best match of estimation
 *     and detection.
 * 5 - Detection class is written by Waqar and based on opencv BKMOD2
 
 */

/*--------------------------
Open CV header files here*/
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include "opencv2/highgui/highgui_c.h"

//--------------------------------------
//C
#include <stdio.h>
#include "Timer.h"
#include <signal.h>
//OpenNI2 library
#include <OpenNI.h>
//C++
#include<iostream>
#include <vector>
// custom
#include "OniSampleUtilities.h"
#include "depthColor.h"
#include "Ctracker.h"
#include "Detector.h"
// defining namespace
using namespace cv;
using namespace std;

extern double tick1;
extern double tick2;
//global variables
float X=0,Y=0;
float Xmeasured=0,Ymeasured=0;
RNG rng; //random number generator
int keyboard; //input from keyboard
Mat frame; Mat color_frame; 
Mat depthColorShow;
//--------------------------
void help()
{
    cout
    << "--------------------------------------------------------------------------" << endl
    << "This program uses kinnect for depth map or a pre-recorded depth maps     "  << endl
    << "         You can process both videos (-vid) or kinnect via in real time. "  << endl
                                                                                    << endl
    << "Usage:"                                                                     << endl
    << "./bs {-vid <video filename> <color filename>}"                              << endl
    << "for example: ./bs -vid depth.avi color.avi"                                 << endl
    << "or: ./bs                          "                                         << endl
    << "--------------------------------------------------------------------------" << endl
    << endl;
}

// defining custom function
int run_process_video(char* videoFilename, char* videoFilenameColor); 
int run_kinect_process();

int main(int argc, char** argv)
{
 
  if(strcmp(argv[1], "-vid") == 0) 
  {
    if(argc != 4)
    {
      cerr <<"Incorret input list" << endl;
      help();
      return EXIT_FAILURE;
    }
    //input data coming from a video
    run_process_video(argv[2], argv[3]);
  }
  else if(strcmp(argv[1], "-kinect") == 0) 
  {
    //input data coming from a sequence of images
   run_kinect_process();
        cerr <<"Exiting..." << endl;

    return EXIT_FAILURE;
  }
  else 
  {
    //error in reading input parameters
    help();
    cerr <<"Exiting..." << endl;

    return EXIT_FAILURE;
  }
    //destroy GUI windows
  destroyAllWindows();
  return EXIT_SUCCESS;
}


int run_process_video(char* videoFilename, char* videoFilenameColor) 
{
    int counter_out = 0; int counter_in = 0; int counter_no_detection = 0;
    Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};
    VideoCapture capture(videoFilename);
    VideoCapture captureColor(videoFilenameColor);
    VideoWriter colorVideo;
    
    colorVideo.open("./segmentation.avi", CV_FOURCC('D','I','V','X') ,15, Size(640,480), true); // Size(1280,720)
    if(!capture.isOpened()||!captureColor.isOpened())
    {
      
        //error in opening the video input
        cerr << "Unable to open video file: " << videoFilename << endl;
        exit(EXIT_FAILURE);
    }
    //read input data. ESC or 'q' for quitting
    int history = 50; double varThreshold = 9.0;
    //CTracker tracker(0.2,0.5,60.0,10,10);
    CTracker tracker(0.2,0.7,70.0,5,50);//(0.3,0.7,70.0,5,15);
    CDetector* detector=new CDetector(history,varThreshold);
    vector<Point2d> centers;
    
    int frame_num = 0;
    while( (char)keyboard != 'q' && (char)keyboard != 27 )
    {
        //read the current frame
        if(!capture.read(frame)||!captureColor.read(color_frame)) {
            cerr << "Unable to read next frame." << endl;
            cerr << "Exiting..." << endl;
            exit(EXIT_FAILURE);
        }
        if(frame.empty())
	{
	capture.set(CAP_PROP_POS_FRAMES,0);
	continue;
	}
	//-------------------
	centers = detector->Detect(frame,color_frame);
	//-------------------
	rectangle(color_frame, cv::Point(10, 2), cv::Point(100,20),cv::Scalar(255,255,255), -1);
	rectangle(color_frame, cv::Point(10,460), cv::Point(100,480),cv::Scalar(255,255,255), -1);
	stringstream ss, f_no;
	ss << (centers.size());
	f_no << frame_num;
	frame_num++;
	string frameNumberString = f_no.str();
	string counter = ss.str();
	putText(color_frame, counter.c_str(), cv::Point(15,15),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
	putText(color_frame, frameNumberString.c_str(), cv::Point(15, 470),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
	line(color_frame,cv::Point(0,150),cv::Point(639,150),Scalar(255,255,0),2,CV_AA);
	line(color_frame,cv::Point(0,370),cv::Point(639,370),Scalar(0,255,0),2,CV_AA);

	for(unsigned int i=0; i<centers.size(); i++)
	{
	circle(color_frame,centers[i],3,Scalar(0,255,0),1,CV_AA);
	}
	
	if(centers.size()>0)
	{
		tracker.Update(centers,counter_in,counter_out);
		
		//cout << tracker.tracks.size()  << endl;

		for(unsigned int i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(unsigned j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					line(color_frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
					//cout<<"A = "<<tracker.tracks[i]->trace[j]<<", B = "<<tracker.tracks[i]->trace[j+1]<<endl;
					
				}
			}
		}
	}
	else
	{

	  if(counter_no_detection>5)
	  {
	    //cout<<"Track size"<<tracker.tracks.size()<<endl;
	    counter_no_detection = 0;
	    for(unsigned int i=0;i<tracker.tracks.size();i++)
	    {
	      if(tracker.tracks[i]->trace.size()>10)
	      {
		cout<<"yFront = "<<tracker.tracks[i]->trace.front().y<<endl;
		cout<<"yBack = "<<tracker.tracks[i]->trace.back().y<<endl;
		if(tracker.tracks[i]->trace.back().y<150 && tracker.tracks[i]->trace.front().y>370)
		 counter_out++;
		if(tracker.tracks[i]->trace.back().y>370 && tracker.tracks[i]->trace.front().y<150)
		 counter_in++;
	      }
	    tracker.tracks.clear();
	    }
	  }
	  else
	  {
	    counter_no_detection++;
	  }
	  
	}
	
	stringstream countIn, countOut;
	countIn << (counter_in);
	countOut<< (counter_out);
	string countInString = countIn.str();
	string countOutString = countOut.str();
	putText(color_frame, countInString.c_str(), cv::Point(620,15),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,255,0));
	putText(color_frame, countOutString.c_str(), cv::Point(620,460),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
	imshow("video",color_frame);
	colorVideo.write(color_frame);
	//get the input from the keyboard
        keyboard = waitKey( 30 );

    }
    //delete capture object
    capture.release();
    delete detector;
    return EXIT_SUCCESS;
    
}
//----------------------------------------------------------------

int run_kinect_process()
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
  int depth_cols;
  int depth_rows; 
  cv::Mat showDepth;
  namedWindow( "People Counting System",CV_GUI_EXPANDED);
  depthColorShow.create(480,640,CV_8UC3);;
  int color_cols;
  int color_rows;
  int counter_out = 0; int counter_in = 0; int counter_no_detection = 0;
  Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};
//	------------------------------------------------
// opencv video capture object creationCV_FOURC
  VideoWriter depthVideo, colorVideo, resultVideo;
  depthVideo.open("./depth.avi", CV_FOURCC('M','J','P','G') , 10, Size(640,480), true); // Size(1280,720)
  colorVideo.open("./color.avi", CV_FOURCC('M','J','P','G') ,10, Size(640,480), true); // Size(1280,720)
  resultVideo.open("./result.avi", CV_FOURCC('M','J','P','G') ,10, Size(640,480), true); // Size(1280,720)	
//------------------------------------------------
  const char* deviceURI = openni::ANY_DEVICE;
//--------------------------------------------------------------------------------  
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
  device.setDepthColorSyncEnabled(true);
  
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
	frame.create(depth_rows, depth_cols, CV_16UC1);//opencv depth image
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
	color_frame.create(color_rows,color_cols,CV_8UC3);// opencv color image creation
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

    int history = 100; double varThreshold = 9.0;
    //CTracker tracker(0.2,0.5,60.0,10,10);
    CTracker tracker(0.3,0.7,70.0,5,50);//(0.3,0.7,70.0,5,15);
    CDetector* detector=new CDetector(history,varThreshold);
    vector<Point2d> centers;
    int frame_num = 0;
    
// now reading the video stream and then converting it to opencv   
  while ((char)keyboard != 'q' && (char)keyboard != 27 )
  {    
    int changedIndex;
    openni::DepthPixel* dData;
    openni::RGB888Pixel* cData;

    Timer *time=new Timer();
    int ms;
    time->start();
    
    p_depth->readFrame(&frame_depth); 
    p_color->readFrame(&frame_color); 
    
    if (frame_depth.isValid()) 
    {
      dData = (openni::DepthPixel*)frame_depth.getData();
      memcpy(frame.data,dData, depth_rows*depth_cols*sizeof(uint16_t) );
      frame.convertTo(showDepth,CV_8U,255/4096.0,0); //
      cvtColor(showDepth,depthColorShow,COLOR_GRAY2RGB);
    }
    if (frame_color.isValid())
    {
      cData = (openni::RGB888Pixel*)frame_color.getData();
      const int dim = 3;
      memcpy( color_frame.data,cData, dim*color_rows*color_cols*sizeof(uint8_t) );
      cvtColor(color_frame, color_frame, COLOR_BGR2RGB);
    }
    //colorVideo.write(color_frame);
    
     centers = detector->Detect(depthColorShow,color_frame);//centers = detector->Detect(depthColorShow,color_frame);
	//-------------------

    rectangle(color_frame, cv::Point(10, 2), cv::Point(100,20),cv::Scalar(255,255,255), -1);
    rectangle(color_frame, cv::Point(10,460), cv::Point(100,480),cv::Scalar(255,255,255), -1);
    stringstream ss, f_no;
    ss << (centers.size());
    f_no << frame_num;
    frame_num++;
    string frameNumberString = f_no.str();
    string counter = ss.str();
    putText(color_frame, counter.c_str(), cv::Point(15,15),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
    putText(color_frame, frameNumberString.c_str(), cv::Point(15, 470),FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
    line(color_frame,cv::Point(0,200),cv::Point(639,200),Scalar(0,0,255),.2,CV_AA);
    line(color_frame,cv::Point(0,230),cv::Point(639,230),Scalar(255,0,0),.2,CV_AA);
    /*
    for(unsigned int i=0; i<centers.size(); i++)
    {
	circle(color_frame,centers[i],3,Scalar(0,255,0),1,CV_AA);
    }
   */
    if(centers.size()>0)
    {
      tracker.Update(centers,counter_in,counter_out);
      //cout << tracker.tracks.size()  << endl;
      for(unsigned int i=0;i<tracker.tracks.size();i++)
      {
	if(tracker.tracks[i]->trace.size()>1)
	{
	  for(unsigned int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
	  {
	    line(color_frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
	    //cout<<"A = "<<tracker.tracks[i]->trace[j]<<", B = "<<tracker.tracks[i]->trace[j+1]<<endl;
	  }
	}
      }
    }
    else
    {
      if(counter_no_detection>5)
      {
	//cout<<"Track size"<<tracker.tracks.size()<<endl;
	counter_no_detection = 0;
	for(unsigned int i=0;i<tracker.tracks.size();i++)
	{
	  if(tracker.tracks[i]->trace.size()>15)
	  {
	    //cout<<"yFront = "<<tracker.tracks[i]->trace.front().y<<endl;
	    //cout<<"yBack = "<<tracker.tracks[i]->trace.back().y<<endl;
	    if(tracker.tracks[i]->trace.back().y<200 && tracker.tracks[i]->trace.front().y>230)
	      counter_out++;
	    if(tracker.tracks[i]->trace.back().y>230 && tracker.tracks[i]->trace.front().y<200)
	      counter_in++;
	  }
	  tracker.tracks.clear();
	}
      }
      else
      {
	counter_no_detection++;
      }
	  
    }
    stringstream countIn, countOut;
    countIn << "OUT = " <<(counter_in);
    countOut<< "IN = "<<(counter_out);
    string countInString = countIn.str();
    string countOutString = countOut.str();
    putText(color_frame, countInString.c_str(), cv::Point(550,15),FONT_HERSHEY_SIMPLEX, 0.6 , cv::Scalar(0,0,255));
    putText(color_frame, countOutString.c_str(), cv::Point(550,460),FONT_HERSHEY_SIMPLEX, 0.6 , cv::Scalar(255,0,0));
    //cv::imshow("depth",showDepth);

    cv::imshow("People Counting System",color_frame);
    //depthVideo.write(depthColorShow);
    resultVideo.write(color_frame);
    keyboard = waitKey(1); //If 'esc' key is pressed, break loop
    ms=time->getElapsedTimeInMilliSec();
    cout<<"Frame/Sec = "<<(1000/ms)<<endl;
    time->stop();
    
  }

  depth.stop();
  color.stop();
  depth.destroy();
  color.destroy();
  device.close();
  delete detector;
  openni::OpenNI::shutdown();
  destroyAllWindows();
  return EXIT_SUCCESS;
  
   
}// end of run_kinect_process function
