/*
	CADET - Center for Advances in Digital Entertainment Technologies
	Copyright 2012 Fachhochschule Salzburg GmbH

		http://www.cadet.at

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "Publisher.h"
#include "Subscriber.h"
#include "BlobTracker.h"
#include "ProjectorImageMerger.h"


#include "../HydraNILib_Common/Common.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <conio.h>

#include <boost/timer/timer.hpp>




using namespace hydraNI;


namespace hydraNI
{
	struct DepthFrameStruct
	{
		DepthFrameStruct( const std::string &name, const std::string &serial, int width, int height ) :
			name( name ),
			serial( serial ),
			width( width ),
			height( height ),
			image( cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 ) )
		{}

		~DepthFrameStruct()
		{
			if( this->image )
			{
				cvReleaseImage( &( this->image ) );
				this->image = NULL;
			}
		}

		const std::string name;
		const std::string serial;

		int width;
		int height;

		IplImage *image;
	};
}



std::mutex frameMutex;

bool update = false;
bool sendDataFlag = true;


BlobTracker *blobTracker = NULL;
ProjectorImageMerger *projectorImageMerger = NULL;

std::map<std::string, DepthFrameStruct*>	depthFrames;


unsigned int receiveCntr = 0;
unsigned int dataReceived = 0;

IplImage *tempImg = NULL;


void transform2DTo3D( size_t size, const glm::vec3 *in, glm::vec3 *out, unsigned int cx, unsigned int cy, float f, bool switchHandedness )
{
	float s = 0.001f / f;
	float a = -( cx * s );
	float b = -( cy * s );
	float zs = 0.001 * ( switchHandedness?-1.0f:1.0f );

	while( size-- )
	{
		out->x = -( in->x * s + a ) * in->z;
		out->y = ( in->y * s + b ) * in->z;
		out->z = in->z * zs;

		in++;
		out++;
	}
}


void depthFrameCallback( _2Real::CustomDataItem const &c )
{
	std::string name = c.getValue<std::string>( "CameraName" );
	std::string serial = c.getValue<std::string>( "CameraSerial" );
	int32_t frameNr = c.getValue<int32_t>( "FrameNr" );
	time_t timeStamp = c.getValue<uint64_t>( "TimeStamp" );
	uint32_t width = c.getValue<uint32_t>( "Width" );
	uint32_t height = c.getValue<uint32_t>( "Height" );
	const std::vector<uint16_t> &data = c.getValue<std::vector<uint16_t> >( "DepthData" );

	char tempStr[256];

	makeDateTimeString( tempStr, timeStamp );

	{
		std::lock_guard<std::mutex> lock( frameMutex );

		//std::cout << "camera " << serial << " sent frame " << frameNr << " with dimensions " << width << " x " << height << " at " << tempStr << std::endl;

		auto it = depthFrames.find( serial );
		if( it == depthFrames.end() )
		{
			DepthFrameStruct *dfs = new DepthFrameStruct( name, serial, width, height );
			it = depthFrames.insert( std::make_pair( serial, dfs ) ).first;

			projectorImageMerger->add( it->second->image );
		}

		DepthFrameStruct *dfs = it->second;

		if( dfs->image->width != width || dfs->image->height != height || dfs->image->depth != 16 )
		{
			std::cerr << "dimensions don't match (" << serial << ")" << std::endl;
			return;
		}

		if( dfs->image->imageSize != data.size() * sizeof( uint16_t ) )
			std::cerr << "data size doesn't match. struggling on..." << std::endl;
		memcpy( dfs->image->imageData, (void*)&( data[0] ), hydraNI::min<size_t>( data.size() * sizeof( uint16_t ), dfs->image->imageSize ) );

		dataReceived += data.size() * sizeof( uint16_t );
		receiveCntr++;

		update = true;
	}
}


static void cleanup()
{
	if( tempImg )
	{
		cvReleaseImage( &tempImg );
		tempImg = NULL;
	}

	cvDestroyAllWindows();

	{
		std::lock_guard<std::mutex> lock( frameMutex );

		for( auto it = depthFrames.begin(); it != depthFrames.end(); ++it )
			delete( it->second );
		depthFrames.clear();
	}

	safeDelete( blobTracker );
	safeDelete( projectorImageMerger );
}

void __cdecl onExit()
{
	std::cerr << "on exit called" << std::endl;

	cleanup();
}

void __cdecl onTerminate()
{
	std::cerr << "process terminated" << std::endl;
	abort();
}

void __cdecl onUnexpected()
{
	std::cerr << "an unexpected error occured" << std::endl;
}

int main( int argc, char *argv[] )
{
	atexit( onExit );
	std::set_terminate( onTerminate );
	std::set_unexpected( onUnexpected );

	int projectorWidth = 640;
	int projectorHeight = 400;
	float projectorFocalLength = 517.087f;

	try
	{
		projectorImageMerger = new ProjectorImageMerger();

		blobTracker = new BlobTracker( projectorWidth, projectorHeight );
		tempImg = cvCreateImage( cvSize( projectorWidth, projectorHeight ), IPL_DEPTH_8U, 3 );

		blobTracker->setMinSize( 1000 );
		blobTracker->start();

		_2Real::app::Engine engine;

		auto threadpool = engine.createThreadpool( _2Real::ThreadpoolPolicy::FIFO );

		// additional bundle loaded b/c of custom type
		auto customTypeBundle = engine.loadBundle( "HydraNIBundle_CustomTypes" );
		auto blobInfo = customTypeBundle.second.getExportedType( "blobFrame" );

		std::shared_ptr< _2Real::network::Publisher > publisher = _2Real::network::Publisher::create( "tcp://*:5557", engine, threadpool );

		// each topic may only publish a single type of data.
		// this restriction makes things a lot easier
		auto topic_custom = publisher->addTopic( "blobFrame", "blobs" );

		_2Real::CustomDataItem blobData = blobInfo.makeCustomData();

		auto asyncSubscriber_depth = _2Real::network::AsyncSubscriber_T< _2Real::CustomDataItem >::create( "tcp://localhost:5556", "frames", "depthFrame", depthFrameCallback, engine, threadpool );

		static boost::timer::cpu_timer timer;

		bool log = false;
		double runningTime = 0.0f;
		int sendCntr = 0;
		int dataAccu = 0;

		cvNamedWindow( projectorImageMerger->getName(), CV_WINDOW_AUTOSIZE );
		
		while( 1 )
		{
			double dt = timer.elapsed().wall * 1e-9f;
			timer.start();

			int lastSecond = (int) runningTime;
			runningTime += dt;
			log = ( lastSecond != (int) runningTime );

			if( kbhit() )
				if( getch() == hydraNI::HNI_KEY_ESC )
					break;

			if( update )
			{
				{
					std::lock_guard<std::mutex> lock( frameMutex );

					const IplImage *mergedImage = projectorImageMerger->process();
					cvShowImage( projectorImageMerger->getName(), mergedImage );

					update = blobTracker->updateFrame( mergedImage, dt );

					for( auto it = depthFrames.begin(); it != depthFrames.end(); ++it )
					{
						cvNamedWindow( it->second->serial.c_str(), CV_WINDOW_AUTOSIZE );
						cvShowImage( it->second->serial.c_str(), it->second->image );
					}

					blobTracker->getDebugView( (unsigned char*) tempImg->imageData, tempImg->width, true );
					cvShowImage( blobTracker->getName(), tempImg );

					//redraw cv windows
					cvWaitKey( 1 );
				}

				if( update )
				{
					if( sendDataFlag )
					{
						sendCntr++;

						const std::vector<Contour> &contours = blobTracker->getContours();
						std::vector<float> &contourData = blobData.getValue<std::vector<float> >( "Contour" );

						if( contours.size() )
						{
							//current limitation: only publishing first contour
							size_t size = contours[0].size() * 2;	//two-component vector

							if( contourData.size() != size )
								contourData.resize( size );

							memcpy( &( contourData[0] ), &( contours[0][0] ), size * sizeof( float ) );

							dataAccu += size * sizeof( float );
						}
						else
							contourData.clear();

						const std::vector<glm::vec3> &comList2D = blobTracker->getCOMList();
						std::vector<float> &comData = blobData.getValue<std::vector<float> >( "COM" );

						if( comList2D.size() )
						{
							std::vector<glm::vec3> comList3D( comList2D.size() );

							transform2DTo3D(
								comList2D.size(),
								&( comList2D[0] ),
								&( comList3D[0] ),
								projectorWidth / 2,
								projectorHeight / 2,
								projectorFocalLength,
								false );

							//current limitation: only publishing first COM
							//NOTE: this are projector space coordinates
							glm::vec2 groundPosition( -( comList3D[0] ).x, -( comList3D[0] ).z );

							size_t size = 2;

							if( comData.size() != size )
								comData.resize( size );

							memcpy( &( comData[0] ), &( groundPosition ), size * sizeof( float ) );

							dataAccu += size * sizeof( float );
						}
						else
							comData.clear();

						topic_custom.publish( blobData );
					}
				}
			}
			else
				std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );

			if( log )
			{
				std::cout << "received " << dataReceived / 1024 << " kb/s from " << receiveCntr << " packages" << std::endl;
				std::cout << "sent " << dataAccu / 1024 << " kb/s at " << sendCntr << " Hz" << std::endl;

				dataAccu = 0;
				sendCntr = 0;

				dataReceived = 0;
				receiveCntr = 0;
			}
		}
		publisher.reset();		// <---- absolutely vital! the 'high level' publisher attempts to manipulate ( singlestep ) a framework block
								// in a separate thread; clearing the engine in this thread while a block is still in use is a very bad idea
		engine.clear();
	}
	catch ( _2Real::Exception &e )
	{
		std::cout << "-------------exception caught in main------------" << std::endl;
		std::cout << e.what() << " " << e.message() << std::endl;
		std::cout << "-------------exception caught in main------------" << std::endl;

#ifdef _DEBUG
		std::cout << "PRESS ANY KEY TO CONTINUE" << std::endl;
		getch();
#endif
	}
	catch ( std::exception const& e )
	{
		std::cout << "-------------exception caught in main------------" << std::endl;
		std::cout << e.what() << std::endl;
		std::cout << "-------------exception caught in main------------" << std::endl;

#ifdef _DEBUG
		std::cout << "PRESS ANY KEY TO CONTINUE" << std::endl;
		getch();
#endif
	}

	return 0;
}