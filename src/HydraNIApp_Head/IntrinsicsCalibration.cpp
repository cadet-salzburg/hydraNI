#include "IntrinsicsCalibration.h"

#include "../HydraNILib_Common/Common.h"
#include "../HydraNILib_Common/CommonNI.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <boost/filesystem.hpp>

using namespace hydraNI;


IntrinsicsCalibration::IntrinsicsCalibration( unsigned int width, unsigned int height, unsigned int boardSizeX, unsigned int boardSizeY, double fieldWidth, double fieldHeight, unsigned int requiredSteps, double timeBetweenSteps ) :
	enabled( false ),
	refinementMode( false ),
	foundChessboardInFrame( false ),
	chessboardFrames( 0 ),
	chessboardCornerCount( boardSizeX * boardSizeY ),
	requiredSteps( requiredSteps ),
	timeUntilNextStep( timeBetweenSteps ),
	timeBetweenSteps( timeBetweenSteps ),
	inImage( NULL ),
	rgbImage( NULL ),
	boardSize( cvSize( boardSizeX, boardSizeY ) ),
	objectPointsMat( cvCreateMat( 1, boardSizeX * boardSizeY, CV_32FC3 ) ),
	objectPoints( ( CvPoint3D32f* )objectPointsMat->data.fl ),
	imagePointsMatCam( cvCreateMat( 1, boardSizeX * boardSizeY, CV_32FC2 ) ),
	imagePointsCam( ( CvPoint2D32f* )imagePointsMatCam->data.fl ),
	imagePointsOverall( cvCreateMat( requiredSteps * chessboardCornerCount, 2, CV_32FC1 ) ),
	objectPointsOverall( cvCreateMat( requiredSteps * chessboardCornerCount, 3, CV_32FC1 ) ),
	pointCountsCamera( cvCreateMat( requiredSteps, 1, CV_32SC1 ) ),
	intrinsicMatrix( cvCreateMat( 3, 3, CV_32FC1 ) ),
	distortionCoeffs( cvCreateMat( 5, 1, CV_32FC1 ) ),
	glIntrinsics( 1.0f )
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

	cvSetIdentity( this->intrinsicMatrix );

	std::fill( distortionCoeffs->data.fl, distortionCoeffs->data.fl + distortionCoeffs->rows, 0.0f );
}

IntrinsicsCalibration::~IntrinsicsCalibration()
{
	if( this->inImage )
	{
		cvReleaseImage( &( this->inImage ) );
		this->inImage = NULL;
	}
	if( this->rgbImage )
	{
		cvReleaseImage( &( this->rgbImage ) );
		this->rgbImage = NULL;
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

	if( this->imagePointsOverall )
	{
		cvReleaseMat( &( this->imagePointsOverall ) );
		imagePointsOverall = NULL;
	}

	if( this->objectPointsOverall )
	{
		cvReleaseMat( &( this->objectPointsOverall ) );
		objectPointsOverall = NULL;
	}

	if( this->pointCountsCamera )
	{
		cvReleaseMat( &( this->pointCountsCamera ) );
		pointCountsCamera = NULL;
	}
}

void IntrinsicsCalibration::checkSize( unsigned int width, unsigned int height )
{
	if( this->inImage && ( this->inImage->width != width || this->inImage->height != height ) )
	{
		cvReleaseImage( &( this->inImage ) );
		this->inImage = NULL;
	}
	if( this->rgbImage && ( this->rgbImage->width != width || this->rgbImage->height != height ) )
	{
		cvReleaseImage( &( this->rgbImage ) );
		this->rgbImage = NULL;
	}


	if( !this->inImage )
		this->inImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	if( !this->rgbImage )
		this->rgbImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 3 );
}

void IntrinsicsCalibration::start()
{
	if( this->enabled )
		this->abort();

	this->enabled = true;
	this->chessboardFrames = 0;
	this->timeUntilNextStep = this->timeBetweenSteps;
}

void IntrinsicsCalibration::abort()
{
	this->enabled = false;
}

bool IntrinsicsCalibration::update( double dt, const unsigned char *rgbData, const unsigned short *depthData, unsigned int width, unsigned int height, size_t pixelSize, size_t layers )
{
	this->checkSize( width, height );

	this->foundChessboardInFrame = false;

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

	this->timeUntilNextStep -= dt;

	if( this->enabled )
	{
		cvCvtColor( this->rgbImage, this->inImage, CV_RGB2GRAY );

		cvEqualizeHist( this->inImage, this->inImage );

		cvCvtColor( this->inImage, this->rgbImage, CV_GRAY2RGB );

		if( depthData )
		{
			char *rgbPtr = this->rgbImage->imageData;
			const unsigned short *depthPtr = depthData;
			for( int j = 0; j < height; j++ )
				for( int i = 0; i < width; i++, rgbPtr += 3, depthPtr++ )
					if( *depthPtr == 0 )
					{
						rgbPtr[0] = 255;
						rgbPtr[1] = 0;
						rgbPtr[2] = 0;
					}
		}

		if( this->timeUntilNextStep <= 0.0 )
		{
			this->timeUntilNextStep = this->timeBetweenSteps;

			int cornerCount = 0;

			this->foundChessboardInFrame = cvFindChessboardCorners( this->inImage, this->boardSize, this->imagePointsCam,
				&cornerCount, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

			if( this->foundChessboardInFrame )
			{
				cvFindCornerSubPix( this->inImage, this->imagePointsCam, cornerCount, cvSize( 11, 11 ),
					cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

				if( cornerCount == this->chessboardCornerCount )
				{
					std::cout << "adding camera chessboard frame #" << chessboardFrames << std::endl;

					float cx = cvmGet( this->intrinsicMatrix, 0, 2 );
					float cy = cvmGet( this->intrinsicMatrix, 1, 2 );

					float fx = cvmGet( this->intrinsicMatrix, 0, 0 );

					unsigned int offset = this->chessboardFrames * this->chessboardCornerCount;
					for( unsigned int i = offset, j = 0; j < this->chessboardCornerCount; ++i, ++j )
					{
						CV_MAT_ELEM( *( this->imagePointsOverall ), float, i, 0 ) = imagePointsCam[j].x;
						CV_MAT_ELEM( *( this->imagePointsOverall ), float, i, 1 ) = imagePointsCam[j].y;

						if( depthData )
						{
							unsigned short depthValue = *( depthData + width * (int)imagePointsCam[j].y + (int)imagePointsCam[j].x );
							if( depthValue == NI_CAMERA_NO_VALUE )
							{
								std::cerr << "depth at " << CV_MAT_ELEM( *( this->imagePointsOverall ), float, i, 0 ) << " / " << CV_MAT_ELEM( *( this->imagePointsOverall ), float, i, 1 ) << " is NO_VALUE (offset = " << width * (int) imagePointsCam[j].y + (int) imagePointsCam[j].x << ")" << std::endl;
								std::cerr << "not all depth values found in frame -- aborting..." << std::endl;

								return false;
							}
							XnVector3D v;
							v.X = imagePointsCam[j].x;
							v.Y = imagePointsCam[j].y;
							v.Z = -depthValue;

							transform2DTo3D( 1, &v, &v, cx, cy, fx, false );

							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 0 ) = v.X;
							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 1 ) = v.Y;
							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 2 ) = v.Z;
						}
						else
						{
							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 0 ) = objectPoints[j].x;
							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 1 ) = objectPoints[j].y;
							CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 2 ) = 0.0f;
						}

						std::cout << "point " << j << " on " << CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 0 ) << " / " << CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 1 ) << " / " << CV_MAT_ELEM( *( this->objectPointsOverall ), float, i, 2 ) << std::endl;
					}
					std::cout << "SUCCESSFULLY RECORDED FRAME" << std::endl;

					CV_MAT_ELEM( *( this->pointCountsCamera ), int, this->chessboardFrames, 0 ) = cornerCount;
					this->chessboardFrames++;

					cvNot( this->rgbImage, this->rgbImage );
					cvDrawChessboardCorners( this->rgbImage, this->boardSize, this->imagePointsCam, cornerCount, this->foundChessboardInFrame );

					if( this->chessboardFrames == this->requiredSteps )
					{
						int flags = 
							CV_CALIB_FIX_ASPECT_RATIO | 
							CV_CALIB_FIX_K1 |
							CV_CALIB_FIX_K2 |
							CV_CALIB_FIX_K3 |
							CV_CALIB_ZERO_TANGENT_DIST
							;

						if( !this->refinementMode )
						{
							std::cout << "calculating camera calibration from scratch" << std::endl;
							cvSetIdentity( this->intrinsicMatrix );
						}
						else
						{
							std::cout << "refining camera calibration from previous parameters" << std::endl;
							flags |= CV_CALIB_USE_INTRINSIC_GUESS;
						}

						// Calibrate the camera
						cvCalibrateCamera2( this->objectPointsOverall, this->imagePointsOverall, this->pointCountsCamera, cvGetSize( this->inImage ),
							this->intrinsicMatrix, this->distortionCoeffs, NULL, NULL, flags );

						this->updateGLMatrices();
						this->refinementMode = true;

						for( int j = 0; j < 3; j++ )
						{
							for( int i = 0; i < 3; i++ )
							{
								std::cout << this->intrinsicMatrix->data.fl[i + j * 3] << "\t";
							}
							std::cout << std::endl;
						}
						std::cout << std::endl;

						this->chessboardFrames = 0;
						this->enabled = false;

						return true;
					}
				}
			}
			else
			{
				std::cout << "camera chessboard not found" << std::endl;
			}
		}
	}

	return false;
}

void IntrinsicsCalibration::getDebugView( unsigned char *rgbData, size_t stride )
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

void IntrinsicsCalibration::updateGLMatrices()
{
	this->glIntrinsics = glm::mat3( 1.0 );

	this->glIntrinsics[0][0] = cvmGet( this->intrinsicMatrix, 0, 0 );	//fx
	this->glIntrinsics[1][1] = cvmGet( this->intrinsicMatrix, 1, 1 );	//fy

	this->glIntrinsics[1][0] = cvmGet( this->intrinsicMatrix, 0, 1 );	//skew

	this->glIntrinsics[2][0] = cvmGet( this->intrinsicMatrix, 0, 2 );	//cx
	this->glIntrinsics[2][1] = cvmGet( this->intrinsicMatrix, 1, 2 );	//cy
}

void IntrinsicsCalibration::setIntrinsics( double fx, double fy, double cx, double cy, double skew )
{
	cvSetIdentity( this->intrinsicMatrix );

	cvmSet( this->intrinsicMatrix, 0, 1, skew );

	cvmSet( this->intrinsicMatrix, 0, 2, cx );
	cvmSet( this->intrinsicMatrix, 1, 2, cy );

	cvmSet( this->intrinsicMatrix, 0, 0, fx );
	cvmSet( this->intrinsicMatrix, 1, 1, fy );

	this->updateGLMatrices();

	this->refinementMode = true;
}

void IntrinsicsCalibration::setDistortionCoeffs( const float *c )
{
	CvMat cm = cvMat( 5, 1, CV_32FC1, const_cast<float*>( c ) );
	cvCopy( &cm, this->distortionCoeffs );
}
