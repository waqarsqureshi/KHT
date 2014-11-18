//////////////////////////////////////////////////////////////////////////////
// Timer.cpp
// =========
// High Resolution Timer.
// This timer is able to measure the elapsed time with 1 micro-second accuracy
// in both Windows, Linux and Unix system 
//
// AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2003-01-13
// UPDATED: 2006-01-13
// Copyright (c) 2003 Song Ho Ahn
//
// Updated by Ramesh Marikhu (marikhu@gmail.com)
// UPDATED: 2010-05-09
// Copyright (c) 2011 Ramesh Marikhu
//////////////////////////////////////////////////////////////////////////////

#include "Timer.h"
#include <cstdlib>

Timer::Timer()
{
#ifdef WIN32
    QueryPerformanceFrequency(&frequency);
    startCount.QuadPart = 0;
    endCount.QuadPart = 0;
#else
    startCount.tv_sec = startCount.tv_usec = 0;
    endCount.tv_sec = endCount.tv_usec = 0;
#endif

    stopped = 0;
    startTimeInMicroSec = 0;
    endTimeInMicroSec = 0;
}


Timer::~Timer(){}


void Timer::start()
{
	// start timer.
	// startCount will be set at this point.
    stopped = 0; // reset stop flag
#ifdef WIN32
    QueryPerformanceCounter(&startCount);
#else
    gettimeofday(&startCount, NULL);
#endif
}


void Timer::stop()
{	
	// stop the timer.
	// endCount will be set at this point.
    stopped = 1; // set timer stopped flag

#ifdef WIN32
    QueryPerformanceCounter(&endCount);
#else
    gettimeofday(&endCount, NULL);
#endif
}


double Timer::getElapsedTimeInMicroSec()
{
	// compute elapsed time in micro-second resolution.
	// other getElapsedTime will call this first, then convert to correspond resolution.
#ifdef WIN32
    if(!stopped) QueryPerformanceCounter(&endCount);
    startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
    endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
    if(!stopped) gettimeofday(&endCount, NULL);
    startTimeInMicroSec = (startCount.tv_sec * 1000000.0) + startCount.tv_usec;
    endTimeInMicroSec = (endCount.tv_sec * 1000000.0) + endCount.tv_usec;
#endif
    return endTimeInMicroSec - startTimeInMicroSec;
}


double Timer::getElapsedTimeInMilliSec()
{
	// divide elapsedTimeInMicroSec by 1000
    return this->getElapsedTimeInMicroSec() * 0.001;
}


double Timer::getElapsedTimeInSec()
{
	// divide elapsedTimeInMicroSec by 1000000
    return this->getElapsedTimeInMicroSec() * 0.000001;
}


#ifdef WIN32

long Timer::getCurrentTimeInMilliSec()
{
	LARGE_INTEGER lCurrentTime = getCurrentTime();
	long lCurrentTimeInMicroSec = (long)lCurrentTime.QuadPart * (1000000.0 / frequency.QuadPart);
	return lCurrentTimeInMicroSec;
}


LARGE_INTEGER Timer::getCurrentTime()
{	
	LARGE_INTEGER currentTime;
   	currentTime.QuadPart = 0;
 	QueryPerformanceCounter( &currentTime );
 	return currentTime;
}


double Timer::getElapsedTimeInMicroSec( LARGE_INTEGER startTime, LARGE_INTEGER endTime )
{
	startTimeInMicroSec = startTime.QuadPart * (1000000.0 / frequency.QuadPart);
    endTimeInMicroSec = endTime.QuadPart * (1000000.0 / frequency.QuadPart);
    return endTimeInMicroSec - startTimeInMicroSec;
}


double Timer::getElapsedTimeInMilliSec( LARGE_INTEGER startTime, LARGE_INTEGER endTime )
{
	return this->getElapsedTimeInMicroSec( startTime, endTime ) * 0.001;
}

#else

timeval Timer::getCurrentTime()
{
	timeval currentTime;
	currentTime.tv_sec = currentTime.tv_usec = 0;
	gettimeofday( &currentTime, NULL );
	return currentTime;
}


double Timer::getElapsedTimeInMicroSec( timeval startTime, timeval endTime )
{
 	startTimeInMicroSec = (startTime.tv_sec * 1000000.0) + startTime.tv_usec;
    endTimeInMicroSec = (endTime.tv_sec * 1000000.0) + endTime.tv_usec;
    return endTimeInMicroSec - startTimeInMicroSec;
}          


double Timer::getElapsedTimeInMilliSec( timeval startTime, timeval endTime )
{
	return this->getElapsedTimeInMicroSec( startTime, endTime ) * 0.001;
}          

#endif

