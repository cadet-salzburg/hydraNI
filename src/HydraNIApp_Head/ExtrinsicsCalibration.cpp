#include "ExtrinsicsCalibration.h"

#include "../HydraNILib_Common/Common.h"

#include <opencv/highgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <boost/filesystem.hpp>

using namespace hydraNI;


ExtrinsicsCalibration::ExtrinsicsCalibration( unsigned int width, unsigned int height, unsigned int boardSizeX, unsigned int boardSizeY, double fieldWidth, double fieldHeight, double fx, double fy, double cx, double cy, double skew ) :
	enabled( false ),
	useAdaptiveThreshold( false ),
	adaptiveThresholdGaussian( false ),
	adaptiveThresholdBlockSize( 71 ),
	adaptiveThresholdOffset( 15 ),
	rgbImage( NULL ),
	gsImage( NULL ),
	threshImage( NULL ),
	boardSize( cvSize( boardSizeX, boardSizeY ) ),
	objectPointsMat( cvCreateMat( 1, boardSizeX * boardSizeY, CV_32FC3 ) ),
	objectPoints( ( CvPoint3D32f* )objectPointsMat->data.fl ),
	imagePointsMatCam( cvCreateMat( 1, boardSizeX * boardSizeY, CV_32FC2 ) ),
	imagePointsCam( ( CvPoint2D32f* )imagePointsMatCam->data.fl ),
	intrinsicMatrix( cvCreateMat( 3, 3, CV_32FC1 ) ),
	distortionCoeffs( cvCreateMat( 5, 1, CV_32FC1 ) ),
	glExtrinsics( 1.0f ),
	glExtrinsicsInv( 1.0f )
{
	this->checkSize( width, height );

	int k = 0;
	for( int j = 0; j < boardSizeY; j++ )
		for( int i = 0; i < boardSizeX; i++, k++ )
		{
			objectPoints[k].x = ( i - ( boardSizeX - 1 ) / 2.0 ) * fieldWidth;
			objectPoints[k].y = ( j - ( boardSizeY - 1 ) / 2.0 ) * fieldHeight;
			objectPoints[k].z = 0.0;
		}

	this->setIntrinsics( fx, fy, cx, cy, skew );

	cvSetZero( this->distortionCoeffs );
}

ExtrinsicsCalibration::~ExtrinsicsCalibration()
{
	if( this->rgbImage )
	{
		cvReleaseImage( &( this->rgbImage ) );
		this->rgbImage = NULL;
	}
	if( this->gsImage )
	{
		cvReleaseImage( &( this->gsImage ) );
		this->gsImage = NULL;
	}
	if( this->threshImage )
	{
		cvReleaseImage( &( this->threshImage ) );
		this->threshImage = NULL;
	}

	if( this->imagePointsMatCam )
	{
		cvReleaseMat( &( this->imagePointsMatCam ) );
		this->imagePointsMatCam = NULL;
		this->imagePointsCam = NULL;
	}

	if( this->objectPointsMat )
	{
		cvReleaseMat( &( this->objectPointsMat ) );
		this->objectPointsMat = NULL;
		this->objectPoints = NULL;
	}

	if( this->intrinsicMatrix )
	{
		cvReleaseMat( &( this->intrinsicMatrix ) );
		this->intrinsicMatrix = NULL;
	}

	if( this->distortionCoeffs )
	{
		cvReleaseMat( &( this->distortionCoeffs ) );
		distortionCoeffs = NULL;
	}
}

void ExtrinsicsCalibration::checkSize( unsigned int width, unsigned int height )
{
	if( this->gsImage && ( this->gsImage->width != width || this->gsImage->height != height ) )
	{
		cvReleaseImage( &( this->gsImage ) );
		this->gsImage = NULL;
	}
	if( this->threshImage && ( this->threshImage->width != width || this->threshImage->height != height ) )
	{
		cvReleaseImage( &( this->threshImage ) );
		this->threshImage = NULL;
	}
	if( this->rgbImage && ( this->rgbImage->width != width || this->rgbImage->height != height ) )
	{
		cvReleaseImage( &( this->rgbImage ) );
		this->rgbImage = NULL;
	}

	if( !this->gsImage )
		this->gsImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	if( !this->threshImage )
		this->threshImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	if( !this->rgbImage )
		this->rgbImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 3 );
}

bool ExtrinsicsCalibration::update( const unsigned char *rgbData, unsigned int width, unsigned int height, size_t pixelSize, size_t layers )
{
	this->checkSize( width, height );

	if( pixelSize == 3 )
		memcpy( this->rgbImage->imageData, rgbData, width * height * pixelSize );
	else
	{
		unsigned int depth = ( pixelSize / layers ) << 3;

		if( depth == this->rgbImage->depth )
		{
			IplImage *temp = cvCreateImageHeader( cvSize( width, height ), depth, layers );
			temp->imageData = (char*)( rgbData );	//stupid openCV doesn't care much about constant data

			cvCvtColor( temp, this->rgbImage, CV_GRAY2RGB );

			cvReleaseImageHeader( &temp );
		}
		else //assuming 2 byte depth
		{
			unsigned char val = 0;

			unsigned short *irData = (unsigned short*)( rgbData );
			unsigned char *imgPtr = ( unsigned char *)( this->rgbImage->imageData );
			for( int i = 0; i < 640; i++ )
				for( int j = 0; j < 480; j++ )
				{
					val = irData[0] >> 2;

					//IR data seems to be 10 bit in OpenNI. why is there no documentation on that? *sigh*
					imgPtr[0] = val;
					imgPtr[1] = val;
					imgPtr[2] = val;

					imgPtr += 3;
					irData++;
				}
		}
	}

	if( this->enabled )
	{
		cvCvtColor( this->rgbImage, this->gsImage, CV_RGB2GRAY );

		if( this->useAdaptiveThreshold )
		{
			cvEqualizeHist( this->gsImage, this->gsImage );
			cvAdaptiveThreshold(
				this->gsImage, this->threshImage, 255,
				( this->adaptiveThresholdGaussian ? CV_ADAPTIVE_THRESH_GAUSSIAN_C : CV_ADAPTIVE_THRESH_MEAN_C ),
				CV_THRESH_BINARY,
				this->adaptiveThresholdBlockSize,
				this->adaptiveThresholdOffset );
			cvCvtColor( this->threshImage, this->rgbImage, CV_GRAY2RGB );
		}
		else
		{
			cvEqualizeHist( this->gsImage, this->gsImage );
			cvCvtColor( this->gsImage, this->rgbImage, CV_GRAY2RGB );
		}

		int cornerCount = 0;

		int found = cvFindChessboardCorners( this->gsImage, this->boardSize, this->imagePointsCam,
			&cornerCount, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

		if( found )
		{
			// Get subpixel accuracy on those corners
			cvFindCornerSubPix( this->gsImage, this->imagePointsCam, cornerCount, cvSize( 11, 11 ),
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );

			// Draw it
			cvDrawChessboardCorners( this->rgbImage, this->boardSize, imagePointsCam, cornerCount, found );

			float rv[3];
			float r[9];
			float t[3];
			CvMat RV = cvMat( 3, 1, CV_32F, rv );
			CvMat R = cvMat( 3, 3, CV_32F, r );
			CvMat T = cvMat( 3, 1, CV_32F, t );

			cvFindExtrinsicCameraParams2( this->objectPointsMat, this->imagePointsMatCam, this->intrinsicMatrix, this->distortionCoeffs, &RV, &T );

			cvRodrigues2( &RV, &R );

			glm::mat4 rot( 1.0f );
			rot[0][0] = r[0];
			rot[1][0] = r[1];
			rot[2][0] = r[2];

			rot[0][1] = r[3];
			rot[1][1] = r[4];
			rot[2][1] = r[5];

			rot[0][2] = r[6];
			rot[1][2] = r[7];
			rot[2][2] = r[8];

			float dummy = 180;

			this->glExtrinsicsInv = glm::mat4( 1.0f );
			this->glExtrinsicsInv = glm::rotate( this->glExtrinsicsInv, dummy, glm::vec3( 1.0, 0.0, 0.0 ) );	//no idea why i have to do this -- openCV maybe uses weird coord systems?
			this->glExtrinsicsInv = glm::translate( this->glExtrinsicsInv, glm::vec3( t[0], t[1], t[2] ) );
			this->glExtrinsicsInv *= rot;
			this->glExtrinsicsInv = glm::rotate( this->glExtrinsicsInv, dummy, glm::vec3( 1.0, 0.0, 0.0 ) );	//no idea why i have to do this -- openCV maybe uses weird coord systems?

			//assuming an orthonormal rotation matrix we can simply invert the rotation by transposing the matrix
			glm::mat4 rotInv = glm::transpose( rot );

			this->glExtrinsics = glm::mat4( 1.0f );
			this->glExtrinsics = glm::rotate( this->glExtrinsics, dummy, glm::vec3( 1.0, 0.0, 0.0 ) );	//no idea why i have to do this
			this->glExtrinsics *= rotInv;
			this->glExtrinsics = glm::translate( this->glExtrinsics, glm::vec3( -t[0], -t[1], -t[2] ) );
			this->glExtrinsics = glm::rotate( this->glExtrinsics, dummy, glm::vec3( 1.0, 0.0, 0.0 ) );	//no idea why i have to do this

			return true;
		}
		else
		{
			std::cout << "camera chessboard not found" << std::endl;

			return false;
		}
	}

	return false;
}

void ExtrinsicsCalibration::getDebugView( unsigned char *rgbData, size_t stride )
{
	if( !this->rgbImage )
		throw std::runtime_error( "chessboard calibration not initialized!" );

	if( stride < this->rgbImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned char *rgbImagePtr = (const unsigned char*)( this->rgbImage->imageData );

	for( int j = 0; j < this->rgbImage->height; j++ )
	{
		for( int i = 0; i < this->rgbImage->width; i++ )
		{
			rgbData[0] = rgbImagePtr[0];
			rgbData[1] = rgbImagePtr[1];
			rgbData[2] = rgbImagePtr[2];

			rgbData += 3;
			rgbImagePtr += 3;
		}

		rgbData += ( stride - this->rgbImage->width ) * 3;
	}
}

void ExtrinsicsCalibration::setIntrinsics( double fx, double fy, double cx, double cy, double skew )
{
	cvSetIdentity( this->intrinsicMatrix );

	cvmSet( this->intrinsicMatrix, 0, 1, skew );

	cvmSet( this->intrinsicMatrix, 0, 2, cx );
	cvmSet( this->intrinsicMatrix, 1, 2, cy );

	cvmSet( this->intrinsicMatrix, 0, 0, fx );
	cvmSet( this->intrinsicMatrix, 1, 1, fy );
}

void ExtrinsicsCalibration::setDistortionCoeffs( const float *c )
{
	CvMat cm = cvMat( 5, 1, CV_32FC1, const_cast<float*>( c ) );
	cvCopy( &cm, this->distortionCoeffs );
}

void ExtrinsicsCalibration::setExtrinsics( const glm::mat4 &e, const glm::mat4 &eInv )
{
	this->glExtrinsics = e;
	this->glExtrinsicsInv = eInv;
}
