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

#define HNI_GL_WINDOW

#include "Publisher.h"

#include "NUICamera.h"
#include "NUICameraContext.h"

#include "FrameProcessor.h"
#include "IntrinsicsCalibration.h"
#include "ExtrinsicsCalibration.h"

#include "Line.h"
#include "Plane.h"
#include "Trackball.h"
#include "CameraTransform.h"

#include "../HydraNILib_Common/Common.h"
#include "../HydraNILib_Common/CommonGL.h"

#include <conio.h>

#include <GL/glew.h>
#include <GL/glut.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#include <boost/timer/timer.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace hydraNI
{
	struct CamStruct
	{
		CamStruct( const std::string &name, const std::string &serial, double offlineTimeOut ) :
			name( name ),
			serial( serial ),
			offlineTimeOut( offlineTimeOut ),
			nuiCam( NULL ),
			intrinsicsCalibration( NULL ),
			extrinsicsCalibration( NULL ),
			fpc( NULL )
		{}

		//this is redundant, but we need name and serial somewhere for NUICamera recovery
		std::string		name;
		std::string		serial;
		double			offlineTimeOut;

		NUICamera				*nuiCam;
		IntrinsicsCalibration	*intrinsicsCalibration;
		ExtrinsicsCalibration	*extrinsicsCalibration;

		_2Real::CustomDataItem	frameData;

		CameraTransform		transform;

		glm::mat4			glCamToProj;
		glm::mat4			glCamToWorld;

		hydraNI::FrameProcessorChain	*fpc;
	};

}

using namespace hydraNI;

bool debugView = false;

const bool isColorRGB = true;
const bool isIntrinsicsFromRGB = true;

const float cylinderClippingCenterX = 0.0f;
const float cylinderClippingCenterZ = 0.0f;
const float cylinderClippingRadius = 3.0f;

CamStruct projector( "projector", "", 0.0f );
std::vector<CamStruct>		cameras;
hydraNI::NUICameraContext	*camContext	=	NULL;


#define GL_WIN_POS_X	100
#define GL_WIN_POS_Y	100

#define MAX_USER_LABEL_LEN	50
#define MAX_ID_LABEL_LEN	120

const double CLIP_NEAR = 0.05;
const double CLIP_FAR = 500.0;

int windowID = -1;

const int initialWindowWidth = 1280;
const int initialWindowHeight = 960;

int windowWidth = initialWindowWidth;
int windowHeight = initialWindowHeight;

double fov = 60.0;
double ar = (double) windowWidth / windowHeight;

float pointSize = 1.0f;
glm::vec3	clearColor;

unsigned int mouseButtonFlags = 0;
unsigned int oldMouseButtonFlags = 0;

int mouseX = 0;
int mouseY = 0;

int oldMouseX = 0;
int oldMouseY = 0;

const float translationSpeed = 5.0;
const float rotationSpeed = 180.0;

Trackball mouseTrackball( glm::vec3( 0.0f, 0.0f, -25.0f ), glm::vec3( 30.0f, -30.0f, 0.0f ), "mouse trackball" );

IplImage *tempImg = NULL;

_2Real::network::Publisher::Topic_T< _2Real::CustomDataItem > topic_custom;

enum DisplayMode
{
	DM_NONE,
	DM_FRUSTUM,
	DM_CLIPPING,

	DM_COUNT
};

DisplayMode displayMode = DM_FRUSTUM;



NUICamera *createNUICamera( const std::string &name, const std::string &serial, double offlineTimeOut )
{
	NUICamera *cam = NULL;

	if( isColorRGB )
	{
#ifdef linux
		//NOTE: linux kinect drivers seem to have problems with simultaneously streaming RGB and DEPTH data
		cam = camContext->createNUICamera( name, serial, (StreamType) ( ST_DEPTH ) );
#else
		cam = camContext->createNUICamera( name, serial, (StreamType) ( ST_RGB | ST_DEPTH ) );
#endif
	}
	else
		cam = camContext->createNUICamera( name, serial, ST_IR );

	if( cam )
	{
		cam->setOfflineTimeOut( offlineTimeOut );
		if( isIntrinsicsFromRGB )
			cam->setViewpointColor();
	}

	return cam;
}

NUICamera *createNUICamera( const std::string &name, double offlineTimeOut )
{
	return createNUICamera( name, "", offlineTimeOut );
}


bool tryNUICamRecovery()
{
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		safeDelete( it->nuiCam );

	camContext->rescan();

	bool success = true;

	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
	{
		CameraSource *cs = it->fpc->getLink<CameraSource>();

		it->nuiCam = createNUICamera( it->name, it->serial, it->offlineTimeOut );
		cs->setNUICamera( it->nuiCam );

		std::cout << "successfully recovered camera " << it->name << std::endl;

		if( it->nuiCam )
		{
			std::cout << "restarting camera " << it->name << std::endl;
			it->nuiCam->start();
		}
		else
		{
			std::cout << "recovering camera " << it->name << " failed" << std::endl;
			success = false;
		}
	}

	return success;
}

std::string toResourcePath( const std::string &fileName )
{
	std::string ret( "../data/" );
	return ret.append( fileName );
}

void recalcC2PTransform( CamStruct &cam )
{
	//NOTE: extrinsicCalibration content is no longer valid (unless called by extrinsicsUpdated of course) -- CHANGES WILL NOT BE SAVED!!

	cam.glCamToProj = projector.transform.getExtrinsicsInv() * cam.transform.getExtrinsics();
	cam.glCamToWorld = cam.transform.getExtrinsics();

	if( cam.fpc )
	{
		Reprojector *rp = cam.fpc->getLink<Reprojector>();
		if( rp )
		{
			rp->setCamToProjector( cam.glCamToProj );
			rp->setCamToWorld( cam.glCamToWorld );
		}
	}
}

void recalcC2PTransforms()
{
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		recalcC2PTransform( *it );
}

void intrinsicsUpdated( CamStruct &cam )
{
	const glm::mat3 &intrinsics = cam.intrinsicsCalibration->getIntrinsics();

	cam.transform.setIntrinsicParams(
		intrinsics[0][0] * cam.transform.getPixelSizeX(),
		intrinsics[2][0],
		intrinsics[2][1],
		intrinsics[1][0] );
	cam.extrinsicsCalibration->setIntrinsics(
		intrinsics[0][0],
		intrinsics[1][1],
		intrinsics[2][0],
		intrinsics[2][1],
		intrinsics[1][0] );
	cam.extrinsicsCalibration->setDistortionCoeffs( cam.intrinsicsCalibration->getDistortionCoeffs() );

	if( cam.fpc )
	{
		Reprojector *rp = cam.fpc->getLink<Reprojector>();
		if( rp )
		{
			rp->setFocalLengthCamera( cam.transform.getFocalLength() / cam.transform.getPixelSizeX() );
			rp->setCenterXCamera( cam.transform.getCenterX() );
			rp->setCenterYCamera( cam.transform.getCenterY() );
			std::cout << "set reprojector focal length to " << rp->getFocalLengthProjector() << " for camera \"" << cam.name << "\"" << std::endl;
		}
	}
}

void extrinsicsUpdated( CamStruct &cam )
{
	cam.transform.setExtrinsics(
		cam.extrinsicsCalibration->getExtrinsics(),
		cam.extrinsicsCalibration->getExtrinsicsInv() );

	recalcC2PTransform( cam );
}

bool loadIntrinsics( CamStruct &cam )
{
	//if there is no nuiCam, it seems to be the projector -- just use it's name
	std::string strInt( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );
	strInt.append( "/" );
	strInt.append( ( isIntrinsicsFromRGB?"sensorRGB":"sensorIR" ) );

	std::string strDist( strInt );
	strInt.append( ".int" );
	strDist.append( ".dist" );

	CvMat *tempMat = NULL;

	//load intrinsics
	if( !( tempMat = (CvMat*) cvLoad( strInt.c_str() ) ) )
	{
		std::cerr << "file " << strInt << " not found" << std::endl;
		return false;
	}
	if( tempMat->rows != 3 || tempMat->cols != 3 )
	{
		std::cerr << "invalid matrix dimensions in file " << strInt << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &tempMat );

		return false;
	}
	std::cout << "loaded intrinsic matrix from " << strInt << std::endl;

	CvMat *i = cvCreateMat( 3, 3, CV_32FC1 );
	cvCopy( tempMat, i );

	cvReleaseMat( &tempMat );


	//load distortion coefficients
	if( !( tempMat = (CvMat*) cvLoad( strDist.c_str() ) ) )
	{
		std::cerr << "file " << strDist << " not found" << std::endl;
		cvReleaseMat( &i );
		return false;
	}
	if( tempMat->rows != 5 || tempMat->cols != 1 )
	{
		std::cerr << "invalid matrix dimensions in file " << strDist << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &i );
		cvReleaseMat( &tempMat );

		return false;
	}
	std::cout << "loaded distortion coefficients from " << strDist << std::endl;

	CvMat *dist = cvCreateMat( 5, 1, CV_32FC1 );
	cvCopy( tempMat, dist );

	cvReleaseMat( &tempMat );

	cam.intrinsicsCalibration->setIntrinsics(
		cvmGet( i, 0, 0 ),
		cvmGet( i, 1, 1 ),
		cvmGet( i, 0, 2 ),
		cvmGet( i, 1, 2 ),
		cvmGet( i, 0, 1 ) );
	cam.intrinsicsCalibration->setDistortionCoeffs( dist->data.fl );

	cvReleaseMat( &i );
	cvReleaseMat( &dist );

	intrinsicsUpdated( cam );

	return true;
}

bool loadIntrinsics()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= loadIntrinsics( *it );

	return succeeded;
}

bool loadExtrinsics( CamStruct &cam )
{
	//if there is no nuiCam, it seems to be the projector -- just use it's name
	std::string strExt( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );
	strExt.append( "/" );
	strExt.append( ( isIntrinsicsFromRGB?"sensorRGB":"sensorIR" ) );

	std::string strExtInv( strExt );
	strExt.append( ".ext" );
	strExtInv.append( ".extInv" );

	CvMat *tempMat = NULL;

	glm::mat4 extT;
	glm::mat4 extInvT;

	//load extrinsics
	if( !( tempMat = (CvMat*) cvLoad( strExt.c_str() ) ) )
	{
		std::cerr << "file " << strExt << " not found" << std::endl;
		return false;
	}
	if( tempMat->rows != 4 || tempMat->cols != 4 )
	{
		std::cerr << "invalid matrix dimensions in file " << strExt << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &tempMat );

		return false;
	}

	extT = glm::make_mat4x4( tempMat->data.fl );

	cvReleaseMat( &tempMat );


	//load distortion coefficients
	if( !( tempMat = (CvMat*) cvLoad( strExtInv.c_str() ) ) )
	{
		std::cerr << "file " << strExtInv << " not found" << std::endl;
		return false;
	}
	if( tempMat->rows != 4 || tempMat->cols != 4 )
	{
		std::cerr << "invalid matrix dimensions in file " << strExtInv << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &tempMat );

		return false;
	}

	extInvT = glm::make_mat4x4( tempMat->data.fl );

	cvReleaseMat( &tempMat );

	std::cout << "loaded extrinsics matrix from " << strExt << std::endl;
	std::cout << "loaded inverse extrinsics coefficients from " << strExtInv << std::endl;

	extT = glm::transpose( extT );
	extInvT = glm::transpose( extInvT );

	cam.extrinsicsCalibration->setExtrinsics( extT, extInvT );

	extrinsicsUpdated( cam );

	return true;
}

bool loadExtrinsics()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= loadExtrinsics( *it );

	return succeeded;
}

bool loadBackgroundImages()
{
	bool succeeded = true;

	int cntr = 0;
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it, cntr++ )
	{
		BackgroundSubtracter *bgs = it->fpc->getLink<BackgroundSubtracter>();
		if( bgs )
		{
			char tempStr[256];
			sprintf( tempStr, "%s/background.tiff", it->serial.c_str() );	//TODO: serial characters sometimes are not file system compliant

			succeeded &= bgs->loadBackground( toResourcePath( tempStr ) );
		}
	}

	return succeeded;
}


bool loadClippingParams( CamStruct &cam )
{
	if( !cam.fpc )
	{
		std::cerr << "loading clipping params for camera " << cam.name << " failed -- no frameprocessorchain found" << std::endl;
		return false;
	}

	Reprojector *rp = cam.fpc->getLink<Reprojector>();
	if( !rp )
	{
		std::cerr << "loading clipping params for camera " << cam.name << " failed -- no reprojector found" << std::endl;
		return false;
	}

	//if there is no nuiCam, it seems to be the projector -- just use it's name
	std::string strClipParamsNear( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );

	std::string strClipParamsFar( strClipParamsNear );
	strClipParamsNear.append( "/paramsNear.clip" );
	strClipParamsFar.append( "/paramsFar.clip" );

	CvMat *tempMat = NULL;

	//load near clipping params
	if( !( tempMat = (CvMat*) cvLoad( strClipParamsNear.c_str() ) ) )
	{
		std::cerr << "file " << strClipParamsNear << " not found" << std::endl;
		return false;
	}
	if( tempMat->rows != 3 || tempMat->cols != 1 )
	{
		std::cerr << "invalid matrix dimensions in file " << strClipParamsNear << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &tempMat );

		return false;
	}
	std::cout << "loaded near clipping params from " << strClipParamsNear << std::endl;

	CvMat *clipParamsNear = cvCreateMat( 3, 1, CV_32FC1 );
	cvCopy( tempMat, clipParamsNear );

	cvReleaseMat( &tempMat );


	//load far clipping params
	if( !( tempMat = (CvMat*) cvLoad( strClipParamsFar.c_str() ) ) )
	{
		std::cerr << "file " << strClipParamsFar << " not found" << std::endl;
		cvReleaseMat( &clipParamsNear );
		return false;
	}
	if( tempMat->rows != 3 || tempMat->cols != 1 )
	{
		std::cerr << "invalid matrix dimensions in file " << strClipParamsFar << ": " << tempMat->rows << "x" << tempMat->cols << std::endl;
		cvReleaseMat( &clipParamsNear );
		cvReleaseMat( &tempMat );

		return false;
	}
	std::cout << "loaded distortion coefficients from " << strClipParamsFar << std::endl;

	CvMat *clipParamsFar = cvCreateMat( 3, 1, CV_32FC1 );
	cvCopy( tempMat, clipParamsFar );

	cvReleaseMat( &tempMat );

	rp->setClipNear( clipParamsNear->data.fl[0] );
	rp->setClipNearPlaneRotationX( clipParamsNear->data.fl[1] );
	rp->setClipNearPlaneRotationY( clipParamsNear->data.fl[2] );

	rp->setClipFar( clipParamsFar->data.fl[0] );
	rp->setClipFarPlaneRotationX( clipParamsFar->data.fl[1] );
	rp->setClipFarPlaneRotationY( clipParamsFar->data.fl[2] );

	cvReleaseMat( &clipParamsNear );
	cvReleaseMat( &clipParamsFar );

	intrinsicsUpdated( cam );

	return true;
}

bool loadClippingParams()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= loadClippingParams( *it );

	return succeeded;
}

bool saveIntrinsics( const CamStruct &cam )
{
	std::string strInt( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );

	if( !boost::filesystem::exists( strInt ) )
		boost::filesystem::create_directories( strInt );

	strInt.append( "/" );
	strInt.append( ( isIntrinsicsFromRGB?"sensorRGB":"sensorIR" ) );

	std::string strDist( strInt );
	strInt.append( ".int" );
	strDist.append( ".dist" );

	glm::mat3 intT = glm::transpose( cam.intrinsicsCalibration->getIntrinsics() );

	CvMat cvInt = cvMat( 3, 3, CV_32FC1, glm::value_ptr( intT ) );
	CvMat cvDist = cvMat( 5, 1, CV_32FC1, (void*) ( cam.intrinsicsCalibration->getDistortionCoeffs() ) );

	cvSave( strInt.c_str(), &cvInt );
	std::cout << "saved intrinsic matrix to " << strInt << std::endl;

	cvSave( strDist.c_str(), &cvDist );
	std::cout << "saved distortion coefficients to " << strDist << std::endl;

	return true;
}

bool saveIntrinsics()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::const_iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= saveIntrinsics( *it );

	return succeeded;
}

bool saveExtrinsics( const CamStruct &cam )
{
	std::string strExt( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );

	if( !boost::filesystem::exists( strExt ) )
		boost::filesystem::create_directories( strExt );

	strExt.append( "/" );
	strExt.append( ( isIntrinsicsFromRGB?"sensorRGB":"sensorIR" ) );

	std::string strExtInv( strExt );
	strExt.append( ".ext" );
	strExtInv.append( ".extInv" );

	//CV uses different order in memory. could just save as is, since I load into glm matrices again, but just to keep consistent I transpose first.
	glm::mat4 extT = glm::transpose( cam.transform.getExtrinsics() );
	glm::mat4 extInvT = glm::transpose( cam.transform.getExtrinsicsInv() );

	CvMat cvExt = cvMat( 4, 4, CV_32FC1, glm::value_ptr( extT ) );
	CvMat cvExtInv = cvMat( 4, 4, CV_32FC1, glm::value_ptr( extInvT ) );

	cvSave( strExt.c_str(), &cvExt );
	std::cout << "saved extrinsics matrix to " << strExt << std::endl;

	cvSave( strExtInv.c_str(), &cvExtInv );
	std::cout << "saved inverse extrinsics matrix to " << std::endl;

	return true;
}

bool saveExtrinsics()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::const_iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= saveExtrinsics( *it );

	return succeeded;
}

bool saveBackgroundImages()
{
	bool succeeded = true;

	int cntr = 0;
	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it, cntr++ )
	{
		BackgroundSubtracter *bgs = it->fpc->getLink<BackgroundSubtracter>();
		if( bgs )
		{
			char tempStr[256];
			sprintf( tempStr, "%s/background.tiff", it->serial.c_str() );

			succeeded &= bgs->saveBackground( toResourcePath( tempStr ) );
		}
	}

	return succeeded;
}

bool saveClippingParams( const CamStruct &cam )
{
	std::string strClipParamsNear( toResourcePath( cam.nuiCam?cam.nuiCam->getSerial():cam.name ) );

	if( !boost::filesystem::exists( strClipParamsNear ) )
		boost::filesystem::create_directories( strClipParamsNear );

	std::string strClipParamsFar( strClipParamsNear );
	strClipParamsNear.append( "/paramsNear.clip" );
	strClipParamsFar.append( "/paramsFar.clip" );

	float clipParamsNear[3];
	float clipParamsFar[3];

	if( !cam.fpc )
	{
		std::cerr << "saving clipping params failed -- no frameprocessing chain found for camera " << cam.name << std::endl;
		return false;
	}

	const Reprojector *rp = cam.fpc->getLink<Reprojector>();

	if( !rp )
	{
		std::cerr << "saving clipping params failed -- no reprojector found for camera " << cam.name << std::endl;
		return false;
	}

	clipParamsNear[0] = rp->getClipNear();
	clipParamsNear[1] = rp->getClipNearPlaneRotationX();
	clipParamsNear[2] = rp->getClipNearPlaneRotationY();

	clipParamsFar[0] = rp->getClipFar();
	clipParamsFar[1] = rp->getClipFarPlaneRotationX();
	clipParamsFar[2] = rp->getClipFarPlaneRotationY();

	CvMat cvClipNear = cvMat( 3, 1, CV_32FC1, (void*) ( clipParamsNear ) );
	CvMat cvClipFar = cvMat( 3, 1, CV_32FC1, (void*) ( clipParamsFar ) );

	cvSave( strClipParamsNear.c_str(), &cvClipNear );
	std::cout << "saved near clipping params to " << strClipParamsNear << std::endl;

	cvSave( strClipParamsFar.c_str(), &cvClipFar );
	std::cout << "saved far clipping params to " << strClipParamsFar << std::endl;

	return true;
}

bool saveClippingParams()
{
	bool succeeded = true;
	for( std::vector<CamStruct>::const_iterator it = cameras.begin(); it != cameras.end(); ++it )
		succeeded &= saveClippingParams( *it );

	return succeeded;
}


void step()
{
	static boost::timer::cpu_timer timer;
	static double runningTime = 0.0f;
	static int dataAccu = 0;
	static int sendCntr = 0;

	char timeStr[256];
	time_t t;

	double dt = timer.elapsed().wall * 1e-9f;
	timer.start();

	int lastSecond = (int) runningTime;
	runningTime += dt;
	bool log = ( lastSecond != (int) runningTime );

	camContext->update();
	time( &t );
	makeDateTimeString( timeStr, t );

	for( auto it = cameras.begin(); it != cameras.end(); ++it )
	{
		it->nuiCam->update( dt );

		for( int i = 0; i < it->fpc->getLinkCount(); i++ )
		{
			FrameProcessor *fp = it->fpc->getLink( i );

			std::string windowName = it->serial + std::string( " " ) + std::string( fp->getName() );
			cvNamedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );
			if( debugView )
			{
				fp->getDebugView( (unsigned char*) tempImg->imageData, tempImg->width, true );
				cvShowImage( windowName.c_str(), tempImg );
			}
			else
				cvShowImage( windowName.c_str(), fp->getImage() );
		}

		//redraw cv windows
		cvWaitKey( 1 );

		if( it->nuiCam->isDepthDataNew() )
		{
			it->fpc->update();

			Reprojector *rp = it->fpc->getLink<Reprojector>();

			if( rp )
			{
				const IplImage *img = rp->getImage();

				uint32_t width = img->width;
				uint32_t height = img->height;

				//just to be sure...
				if( img->depth != 16 )
				{
					std::cerr << "wait a minute.... image should have depth of 16 bits. i'll better skip this." << std::endl;
					continue;
				}

				it->frameData.set( "FrameNr", (int32_t) it->nuiCam->getDepthFrameNr() );
				it->frameData.set( "TimeStamp", (uint64_t) t );

				it->frameData.set( "CameraName", it->nuiCam->getName() );
				it->frameData.set( "CameraSerial", it->nuiCam->getSerial() );

				it->frameData.set( "Width", (uint32_t) width );
				it->frameData.set( "Height", (uint32_t) height );

				size_t size = width * height;
				std::vector<uint16_t> &data = it->frameData.getValue<std::vector<uint16_t> >( "DepthData" );
				if( data.size() != size )
				{
					std::cout << "resizing depth data buffer from " << data.size() << " to " << size << " (" << width << " x " << height << ")" << std::endl;
					data.resize( size );
				}
				memcpy( &( data[0] ), img->imageData, size * sizeof( uint16_t ) );

				dataAccu += size * sizeof( uint16_t );
				sendCntr++;

				topic_custom.publish( it->frameData );
			}
			else
				std::cerr << "camera " << it->name << " has no reprojector associated" << std::endl;
		}
	}

	if( log )
	{
		std::cout << "published " << sendCntr << " packets (" << ( cameras.size()?sendCntr / cameras.size():0 ) << "Hz) at " << dataAccu / 1024 << " kb/s from " << cameras.size() << " cameras: " << std::endl;

		int cntr = 0;
		for( auto it = cameras.begin(); it != cameras.end(); ++it, cntr++ )
			std::cout << "\tcam# " << cntr << ": " << it->name << ", S/N: " << it->serial << " @ " << it->nuiCam->getDepthFPS() << " fps" << std::endl;

		dataAccu = 0;
		sendCntr = 0;
	}
}

#ifdef HNI_GL_WINDOW
void drawCameraClippingZone( double sensorWidth, double sensorHeight, double focalLength, double clipNear, const glm::vec3 &nearPlaneNormal, double clipFar, const glm::vec3 &farPlaneNormal, const float *color )
{
	Plane nearPlane( glm::vec3( 0.0, 0.0, -clipNear ), nearPlaneNormal );
	Plane farPlane( glm::vec3( 0.0, 0.0, -clipFar ), farPlaneNormal );

	float dx = sensorWidth * 0.5f / focalLength;
	float dy = sensorHeight * 0.5f / focalLength;

	Line rays[4];

	rays[0] = Line( zero(), glm::vec3( -dx, dy, -1.0 ) );	//up left
	rays[1] = Line( zero(), glm::vec3( -dx, -dy, -1.0 ) );	//down left
	rays[2] = Line( zero(), glm::vec3( dx, -dy, -1.0 ) );	//down right
	rays[3] = Line( zero(), glm::vec3( dx, dy, -1.0 ) );	//up right

	GLfloat vertsSide[2 * 5 * 3];
	GLfloat vertsClip[2 * 4 * 3];

	for( int i = 0; i < 4; i++ )
		for( int j = 0; j < 2; j++ )
		{
		Plane &p = ( j?farPlane:nearPlane );

		float dist = 0.0f;
		glm::vec3 v;

		if( !p.getIntersection( rays[i], v, dist ) )
			return;

		for( int k = 0; k < 3; k++ )
			vertsSide[i * 2 * 3 + j * 3 + k] = v[k];
		}

	for( int j = 0; j < 2; j++ )
		for( int k = 0; k < 3; k++ )
			vertsSide[4 * 2 * 3 + j * 3 + k] = vertsSide[j * 3 + k];

	for( int i = 0; i < 4; i++ )
		for( int k = 0; k < 3; k++ )
		{
			vertsClip[0 * 4 * 3 + i * 3 + k] = vertsSide[i * 2 * 3 + 0 * 3 + k];
			vertsClip[1 * 4 * 3 + i * 3 + k] = vertsSide[( 3 - i ) * 2 * 3 + 1 * 3 + k];
		}

	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );
		glEnable( GL_BLEND );

		if( color )
			glColor4f( color[0], color[1], color[2], 0.25f );

		glVertexPointer( 3, GL_FLOAT, 0, vertsSide );
		glDrawArrays( GL_QUAD_STRIP, 0, 10 );

		glVertexPointer( 3, GL_FLOAT, 0, vertsClip );
		glDrawArrays( GL_QUADS, 0, 8 );
	}
	glPopAttrib();
}

void displayBlob( const VertexList &vertices, const ColorList &colors, const glm::mat4 &m = glm::mat4( 1.0f ) )
{
	if( vertices.size() )
	{
		glPushAttrib( GL_ALL_ATTRIB_BITS );
		glPushMatrix();

		if( colors.size() )
			glEnableClientState( GL_COLOR_ARRAY );

		glVertexPointer( 3, GL_FLOAT, 0, &( vertices[0] ) );
		if( colors.size() )
			glColorPointer( 3, GL_FLOAT, 0, &( colors[0] ) );
		glDrawArrays( GL_POINTS, 0, vertices.size() );

		if( colors.size() )
			glDisableClientState( GL_COLOR_ARRAY );

		glPopMatrix();
		glPopAttrib();
	}
}

void printText( const char *string, float x, float y, void *font = GLUT_BITMAP_8_BY_13 )
{
	glRasterPos3f( x, y, 0.0 );

	while( *string )
		glutBitmapCharacter( font, *( string++ ) );
}

void glutDisplay()
{
	try
	{
		glClearColor( clearColor[0], clearColor[1], clearColor[2], 1.0 );

		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		glEnable( GL_DEPTH_TEST );
		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		gluPerspective( fov, ar, CLIP_NEAR, CLIP_FAR );

		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();

		mouseTrackball.apply();

		float scale = 5.0f;
		glPushMatrix();
		glPushAttrib( GL_ENABLE_BIT );
		{
			glDisable( GL_LIGHTING );
			glDisable( GL_TEXTURE_2D );
			glDisable( GL_COLOR_MATERIAL );

			glScalef( scale, scale, scale );

			glPushAttrib( GL_ALL_ATTRIB_BITS );
			{
				glLineWidth( 3.0f );
				drawAxes( 1.0f );
			}
			glPopAttrib();

			glPushMatrix();
			{
				glRotatef( 90.0f, 1.0f, 0.0f, 0.0f );
				drawGrid( glm::value_ptr( grey() ), 10.0f, 9 );

				drawCircle( cylinderClippingCenterX, cylinderClippingCenterZ, 0.0, cylinderClippingRadius, glm::value_ptr( yellow() ), false, 60 );
			}
			glPopMatrix();

			glm::vec3 color( yellow() );

			glColor3fv( glm::value_ptr( color ) );

			glPushMatrix();
			{
				glm::vec3 worldPos( projector.transform.getWorldPosition() );

				glBegin( GL_LINES );
				{
					glVertex3f( worldPos[0], 0.0f, worldPos[2] );
					glVertex3f( worldPos[0], worldPos[1], worldPos[2] );
				}
				glEnd();

				glTranslatef( worldPos[0], 0.0f, worldPos[2] );
				drawLocator( glm::value_ptr( color ), 0.1f );
			}
			glPopMatrix();


			//render projector coord system
			glPushMatrix();
			{
				glMultMatrixf( glm::value_ptr( projector.transform.getExtrinsics() ) );

				switch( displayMode )
				{
				case DM_NONE:
					drawAxes( 0.5, 0.0 );
					break;
				case DM_FRUSTUM:
				{
					drawCameraCone(
						projector.transform.getSensorWidth(),
						projector.transform.getSensorHeight(),
						projector.transform.getFocalLength(),
						0.02f, 1.0f,
						glm::value_ptr( color ),
						projector.transform.getCenterX(),
						projector.transform.getCenterY() );
				}
					break;
				case DM_CLIPPING:
				{
					drawCameraCone(
						projector.transform.getSensorWidth(),
						projector.transform.getSensorHeight(),
						projector.transform.getFocalLength(),
						0.02f, 1.0f,
						glm::value_ptr( color ) );
				}
					break;
				}

				glColor3fv( glm::value_ptr( color ) );

				glPushAttrib( GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT );
				{
					glDisable( GL_DEPTH_TEST );
					glEnable( GL_COLOR_MATERIAL );

					printText( "projector", 0, 0 );

					glm::vec3 worldPos( projector.transform.getWorldPosition() );

					char tempStr[128];
					sprintf( tempStr, "[%.02f %.02f %.02f]",
						worldPos[0],
						worldPos[1],
						worldPos[2] );

					printText( tempStr, 0, -0.1 );
				}
				glPopAttrib();

				unsigned int camIndex = 0;
				for( std::vector<CamStruct>::const_iterator it = cameras.begin(); it != cameras.end(); ++it, camIndex++ )
				{
					Reprojector *rp = it->fpc->getLink<Reprojector>();
					if( rp )
					{
						displayBlob( rp->getPrjVertices(), rp->getPrjColors() );
					}
				}
			}
			glPopMatrix();


			//render cameras coord system
			int camIndex = 0;
			for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it, camIndex++ )
			{
				glm::vec3 color( red() );

				glPushMatrix();
				{
					glColor3fv( glm::value_ptr( color ) );

					glMultMatrixf( glm::value_ptr( it->transform.getExtrinsics() ) );

					for( unsigned int i = 0; i < it->fpc->getLinkCount(); i++ )
					{
						const Reprojector *rp = dynamic_cast<const Reprojector*>( it->fpc->getLink( i ) );
						if( rp )
							displayBlob( rp->getCamVertices(), rp->getCamColors() );
					}
				}
				glPopMatrix();
			}

			camIndex = 0;
			for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it, camIndex++ )
			{
				glm::vec3 color( red() );

				glColor3fv( glm::value_ptr( color ) );

				glPushMatrix();
				{
					glm::vec3 worldPos( it->transform.getWorldPosition() );

					glBegin( GL_LINES );
					{
						glVertex3f( worldPos[0], 0.0f, worldPos[2] );
						glVertex3f( worldPos[0], worldPos[1], worldPos[2] );
					}
					glEnd();

					glTranslatef( worldPos[0], 0.0f, worldPos[2] );
					drawLocator( glm::value_ptr( color ), 0.1f );
				}
				glPopMatrix();

				glPushMatrix();
				{
					glMultMatrixf( glm::value_ptr( it->transform.getExtrinsics() ) );

					glPushAttrib( GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT );
					{
						glDisable( GL_DEPTH_TEST );
						glEnable( GL_COLOR_MATERIAL );

						char tempStr[128];
						sprintf( tempStr, "camera %02d", camIndex );

						printText( tempStr, 0, 0 );
						printText( it->name.c_str(), 0, -0.1 );
						printText( it->serial.c_str(), 0, -0.2 );

						glm::vec3 worldPos( it->transform.getWorldPosition() );

						sprintf( tempStr, "[%.02f %.02f %.02f]",
							worldPos[0],
							worldPos[1],
							worldPos[2] );

						printText( tempStr, 0, -0.3 );
					}
					glPopAttrib();

					switch( displayMode )
					{
					case DM_NONE:
						drawAxes( 0.5, 0.0 );
						break;
					case DM_FRUSTUM:
					{
						drawCameraCone(
							it->transform.getSensorWidth(),
							it->transform.getSensorHeight(),
							it->transform.getFocalLength(),
							0.02f, 1.0f,
							glm::value_ptr( it->intrinsicsCalibration->getEnabled()?white():color ) );
					}
						break;
					case DM_CLIPPING:
					{
						Reprojector *rp = it->fpc->getLink<Reprojector>();
						if( rp )
						{
							drawCameraClippingZone(
								it->transform.getSensorWidth(),
								it->transform.getSensorHeight(),
								it->transform.getFocalLength(),
								rp->getClipNear() * 0.001,
								rp->getClipNearPlaneNormal(),
								rp->getClipFar() * 0.001,
								rp->getClipFarPlaneNormal(),
								glm::value_ptr( it->intrinsicsCalibration->getEnabled()?white():color ) );
						}
					}
						break;
					}
				}
				glPopMatrix();
			}
		}
		glPopAttrib();
		glPopMatrix();

		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		glOrtho( 0, windowWidth, windowHeight, 0, -1.0, 1.0 );
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();

		glFlush();

		glPopAttrib();

		glutSwapBuffers();

		checkForGLError();
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutDisplay: " << e.what() << std::endl;
	}
}

void glutIdle()
{
	static bool first = true;

	static boost::timer::cpu_timer timer;
	static double timeAccu = 0.0f;
	static double timeAccuSend = 0.0f;

	try
	{
		step();
		
		if( !first )
		{
			if( ::mouseButtonFlags != ::oldMouseButtonFlags || ::mouseX != ::oldMouseX || ::mouseY != ::oldMouseY )
			{
				mouseTrackball.setRotationSpeed( ::rotationSpeed / ::windowWidth );
				mouseTrackball.setTranslationSpeed( hydraNI::max( glm::length( mouseTrackball.getTranslation() ), ::translationSpeed ) / ::windowWidth );

				mouseTrackball.onMouseRel( ::mouseX - ::oldMouseX, ::mouseY - ::oldMouseY, ::mouseButtonFlags );
			}
		}

		first = false;

		::oldMouseX = ::mouseX;
		::oldMouseY = ::mouseY;
		::oldMouseButtonFlags = ::mouseButtonFlags;

		// Display the frame
		glutPostRedisplay();
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutIdle: " << e.what() << std::endl;
	}
}


void glutMouse( int button, int state, int x, int y )
{
	try
	{
		::mouseX = x;
		::mouseY = y;

		if( state )
			::mouseButtonFlags &= ~0x01 << button;
		else
			::mouseButtonFlags |= 0x01 << button;
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutMouse: " << e.what() << std::endl;
	}
}

void glutMotion( int x, int y )
{
	try
	{
		::mouseX = x;
		::mouseY = y;
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutMotion: " << e.what() << std::endl;
	}
}

void glutPassiveMotion( int x, int y )
{
	try
	{
		::mouseX = x;
		::mouseY = y;
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutPassiveMotion: " << e.what() << std::endl;
	}
}

void glutReshape( int width, int height )
{
	try
	{
		::windowWidth = width;
		::windowHeight = height;

		std::cout << "reshape: " << width << "/" << height << std::endl;

		::ar = ( double )::windowWidth / ::windowHeight;

		glViewport( 0, 0, width, height );
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutReshape: " << e.what() << std::endl;
	}
}

void glutKeyboard( unsigned char key, int x, int y )
{
	try
	{
		switch( key )
		{
		case HNI_KEY_ESC:
			//publisher.reset();		// <---- absolutely vital! the 'high level' publisher attempts to manipulate ( singlestep ) a framework block
			//						// in a separate thread; clearing the engine in this thread while a block is still in use is a very bad idea
			//engine.clear();

			exit( 0 );
			break;
		}
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutKeyboard: " << e.what() << std::endl;
	}
}

void glutKeyboardUp( unsigned char key, int x, int y )
{
	try
	{
		switch( key )
		{
		}
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutKeyboardUp: " << e.what() << std::endl;
	}
}

void glutSpecial( int key, int x, int y )
{
	try
	{
		switch( key )
		{
		case GLUT_KEY_F1:
			break;
		case GLUT_KEY_F2:
			//saveAll();
			break;
		case GLUT_KEY_F3:
			//loadAll();
			break;
		case GLUT_KEY_F4:
			break;
		case GLUT_KEY_F5:
			break;
		case GLUT_KEY_F6:
			break;
		case GLUT_KEY_F7:
			break;
		case GLUT_KEY_F8:
			break;
		case GLUT_KEY_F9:
			//toggleAutoRecovery();
			break;
		case GLUT_KEY_F12:
			break;
		case GLUT_KEY_LEFT:
			break;
		case GLUT_KEY_RIGHT:
			break;
		case GLUT_KEY_UP:
			break;
		case GLUT_KEY_DOWN:
			break;
		case GLUT_KEY_PAGE_UP:
			break;
		case GLUT_KEY_PAGE_DOWN:
			break;
		}
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutSpecial: " << e.what() << std::endl;
	}
}

void glutSpecialUp( int key, int x, int y )
{
	try
	{
	}
	catch( std::exception &e )
	{
		std::cerr << "caught exception in glutSpecialUp: " << e.what() << std::endl;
	}

}

void glInit( int *argc, char **argv )
{
	clearColor = glm::vec3( 0 );

	glutInit( argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	glutInitWindowPosition( GL_WIN_POS_X, GL_WIN_POS_Y );
	glutInitWindowSize( ::windowWidth, ::windowHeight );
	windowID = glutCreateWindow( "Track-the-Performer-Module" );

	glutKeyboardFunc( glutKeyboard );
	glutKeyboardUpFunc( glutKeyboardUp );
	glutSpecialFunc( glutSpecial );
	glutSpecialUpFunc( glutSpecialUp );
	glutDisplayFunc( glutDisplay );
	glutReshapeFunc( glutReshape );
	glutIdleFunc( glutIdle );
	glutMouseFunc( glutMouse );
	glutMotionFunc( glutMotion );
	glutPassiveMotionFunc( glutPassiveMotion );

	GLenum glErr = glewInit();
	if( glErr != GLEW_OK )
	{
		std::stringstream sstr;
		sstr << "Error initializing glew: \"" << glewGetErrorString( glErr ) << "\"";
		throw std::runtime_error( sstr.str() );
	}

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_TEXTURE_2D );

	glEnableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_COLOR_ARRAY );

	glPointSize( pointSize );
}
#endif

void cleanupCamStruct( CamStruct &cam )
{
	safeDelete( cam.nuiCam );
	safeDelete( cam.intrinsicsCalibration );
	safeDelete( cam.extrinsicsCalibration );
	safeDelete( cam.fpc );
}

static void cleanup()
{
	if( tempImg )
	{
		cvReleaseImage( &tempImg );
		tempImg = NULL;
	}

	for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it )
		cleanupCamStruct( *it );
	cameras.clear();

	safeDelete( camContext );

	cvDestroyAllWindows();
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

	try
	{
		camContext = new NUICameraContext();

		NUICamera *nuiCam = createNUICamera( "NUICamera", 2.0 );
		if( !nuiCam )
			std::cerr << "couldn't create camera" << std::endl;
		else
		{
			CamStruct cam( nuiCam->getName(), nuiCam->getSerial(), nuiCam->getOfflineTimeOut() );
			cam.nuiCam = nuiCam;

			XnUInt16 resX = 0;
			XnUInt16 resY = 0;

			float f = 0.0f;
			float sx = 0.0f;
			float sy = 0.0f;

			if( !cam.nuiCam->getDepthRes( resX, resY ) )
			{
				std::cerr << "getting camera depth resolution failed!" << std::endl;

				resX = 640;
				resY = 480;
			}
			if( !cam.nuiCam->getDepthIntrinsics( f, sx, sy ) )
			{
				std::cerr << "getting camera depth intrinsics failed!" << std::endl;

				f = 59.429428f;

				//hardcoded for now -- get vendor/product id and decide from that
				sx = 0.10420000f; //Kinect
				sy = 0.10420000f; //Kinect

				//sx = 0.10520000; //Asus Xtion
				//sy = 0.10520000; //Asus Xtion
			}
			std::cout << "cam " << cam.nuiCam->getName() << ": f=" << f << "; sx=" << sx << "; sy = " << sy << std::endl;

			cam.transform.setResolution( resX, resY );
			cam.transform.setPixelSize( sx, sy );

			if( !cam.nuiCam->getColorRes( resX, resY ) )
			{
				std::cerr << "getting camera color resolution failed!" << std::endl;

				resX = 640;
				resY = 480;
			}
			cam.intrinsicsCalibration = new IntrinsicsCalibration(
				resX, resY,
				8, 7,
				0.032f, 0.032f,
				10, 5.0f );
			cam.intrinsicsCalibration->setIntrinsics(
				f / sx,
				f / sy,
				resX * 0.5f,
				resY * 0.5f,
				0.0f );
			cam.extrinsicsCalibration = new ExtrinsicsCalibration(
				resX, resY,
				8, 7,
				0.032f, 0.032f,
				cam.transform.getFocalLength() / cam.transform.getPixelSizeX(),
				cam.transform.getFocalLength() / cam.transform.getPixelSizeY(),
				cam.transform.getCenterX(),
				cam.transform.getCenterY(),
				cam.transform.getSkew() );

			intrinsicsUpdated( cam );

			cameras.push_back( cam );
		}

		glm::vec3 projectorWorldPosition( 0.0, 1.29, 6.0 );
		glm::vec3 projectorWorldRotation( 0.0, 0.0, 0.0 );
		float projectorFocalLength = 517.087f;
		int projectorWidth = 640;
		int projectorHeight = 400;

		projector.transform.setResolution( projectorWidth, projectorHeight );
		projector.transform.transRight( projectorWorldPosition );
		projector.transform.rotRight( projectorWorldRotation );
		projector.transform.setIntrinsicParams(
			projectorFocalLength,
			projectorWidth / 2,
			projectorHeight / 2,
			0.0 );

		recalcC2PTransforms();

		loadIntrinsics();
		loadExtrinsics();

		for( auto it = cameras.begin(); it != cameras.end(); ++it )
		{
			unsigned int cntr = 0;
			for( std::vector<CamStruct>::iterator it = cameras.begin(); it != cameras.end(); ++it, cntr++ )
			{
				it->fpc = new FrameProcessorChain();

				CameraSource *cs = new CameraSource( it->nuiCam );
				BackgroundSubtracter *bgs = new BackgroundSubtracter( "", 350 );
				MorphologicCloser *mc = new MorphologicCloser( 1 );
				Reprojector *rp = new Reprojector(
					it->glCamToProj,
					it->glCamToWorld,
					2,
					projectorWidth,
					projectorHeight,
					projectorFocalLength,
					projectorWidth / 2, projectorHeight / 2,
					it->transform.getFocalLength() / it->transform.getPixelSizeX(),
					it->transform.getCenterX(), it->transform.getCenterY(),
					0.1,
					10.0
					);

				rp->setCylinderClippingOn( false );
				rp->setFrustumClippingOn( false );

				rp->setClipNearPlaneRotationX( 0.0f );
				rp->setClipNearPlaneRotationY( 0.0f );
				rp->setClipFarPlaneRotationX( 0.0f );
				rp->setClipFarPlaneRotationY( 0.0f );
				rp->setCylinderClippingCenterX( cylinderClippingCenterX );
				rp->setCylinderClippingCenterZ( cylinderClippingCenterZ );
				rp->setCylinderClippingRadius( cylinderClippingRadius );

				it->fpc->add( cs );
				it->fpc->add( bgs );
				it->fpc->add( mc );
				it->fpc->add( rp );

				XnUInt16 width = 0;
				XnUInt16 height = 0;

				if( !it->nuiCam->getDepthRes( width, height ) )
				{
					std::cerr << "getting camera depth resolution failed!" << std::endl;

					width = 640;
					height = 480;
				}

				it->fpc->resize( width, height );	//trigger resize of all processing links
			}
		}

		loadBackgroundImages();
		loadClippingParams();

		_2Real::app::Engine engine;

		_2Real::app::ThreadpoolHandle threadpool = engine.createThreadpool( _2Real::ThreadpoolPolicy::FIFO );

		// additional bundle loaded b/c of custom type
		auto customTypeBundle = engine.loadBundle( "HydraNIBundle_CustomTypes" );
		auto depthInfo = customTypeBundle.second.getExportedType( "depthFrame" );

		std::shared_ptr< _2Real::network::Publisher > publisher = _2Real::network::Publisher::create( "tcp://*:5556", engine, threadpool );

		// each topic may only publish a single type of data.
		// this restriction makes things a lot easier
		topic_custom = publisher->addTopic( "depthFrame", "frames" );

		for( auto it = cameras.begin(); it != cameras.end(); ++it )
		{
			it->frameData = depthInfo.makeCustomData();
			it->nuiCam->start();
		}

		tempImg = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 3 );

#ifdef HNI_GL_WINDOW
		glInit( &argc, argv );
		glutMainLoop();
#else
		bool leave = false;
		while( !leave )
		{
			if( kbhit() )
			{
				char c = getch();
				switch( c )
				{
				case hydraNI::HNI_KEY_ESC:
					leave = true;
					break;
				case ' ':
					debugView = true;
					break;
				}
			}

			step();
		}
		publisher.reset();		// <---- absolutely vital! the 'high level' publisher attempts to manipulate ( singlestep ) a framework block
								// in a separate thread; clearing the engine in this thread while a block is still in use is a very bad idea
		engine.clear();
#endif
	}
	catch ( _2Real::Exception &e )
	{
		std::cout << "-------------exception caught in main------------" << std::endl;
		std::cout << e.what() << " " << e.message() << std::endl;
		std::cout << formatLastWinError() << std::endl;
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