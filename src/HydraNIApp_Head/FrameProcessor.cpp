#include "FrameProcessor.h"
#include "NUICamera.h"

#include "../HydraNILib_Common/Common.h"

#include <opencv/highgui.h>

#include <glm/gtx/transform.hpp>


using namespace hydraNI;



FrameProcessor::FrameProcessor() :
	isEnabled( true ),
	colorImage( NULL ),
	outImage( NULL )
{
	this->desc[0] = 0;
}

FrameProcessor::~FrameProcessor()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}
}

void FrameProcessor::checkSize( unsigned int width, unsigned int height )
{
	if( !width || !height )
		throw std::runtime_error( "image size has to be > 0" );

	if( !this->outImage || width != this->outImage->width || height != this->outImage->height )
		this->resize( width, height );
}




CameraSource::CameraSource( const NUICamera *nuiCamera ) :
	FrameProcessor(),
	nuiCamera( nuiCamera ),
	colorSource( NULL )
{
	if( !nuiCamera )
		throw std::runtime_error( "NUICamera must not be NULL!" );
}

CameraSource::~CameraSource()
{
	this->release();
}

const IplImage *CameraSource::process( const IplImage *image, const IplImage *colorImage )
{
	this->colorImage = colorImage;

	if( !nuiCamera )
	{
		std::cerr << "NUICamera not set!" << std::endl;
		return NULL;
	}

	XnUInt16 width = 0;
	XnUInt16 height = 0;

	this->nuiCamera->getDepthRes( width, height );

	this->checkSize( width, height );

	if( this->isEnabled )
	{
		const XnDepthPixel *depthData = this->nuiCamera->getDepthData();
		memcpy( this->outImage->imageData, depthData, sizeof( XnDepthPixel ) * width * height );
	}
	else
	{}

#ifdef WIN32
	sprintf_s( this->desc,
		"[%s] {%dx%dx%dx%d}",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"[%s] {%dx%dx%dx%d}",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth );
#endif // WIN32

	return this->outImage;
}

void CameraSource::resize( unsigned int width, unsigned int height )
{
	if( this->outImage && width == this->outImage->width && height == this->outImage->height )
		return;

	this->release();

	this->outImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
	this->colorSource = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 3 );
}

void CameraSource::getDebugView( unsigned char *rgbData, size_t stride, bool switchRB )
{
	if( !this->outImage )
		throw std::runtime_error( "camera source not initialized!" );

	if( stride < this->outImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned short *outImagePtr = (const unsigned short*)( this->outImage->imageData );
	const unsigned char *colorImagePtr = ( this->colorImage ? (const unsigned char*)( this->colorImage->imageData ) : NULL );

	float r = 1.0f;
	float g = 1.0f;
	float b = 1.0f;

	for( int j = 0; j < this->outImage->height; j++ )
	{
		for( int i = 0; i < this->outImage->width; i++ )
		{
			unsigned char val = 255 - (unsigned char)( (int)( *outImagePtr ) * 255.0f / 7000.0f );

			if( colorImagePtr )
			{
				r = colorImagePtr[0] / 255.0f;
				g = colorImagePtr[1] / 255.0f;
				b = colorImagePtr[2] / 255.0f;
				colorImagePtr += 3;
			}

			if( switchRB )
			{
				rgbData[2] = (unsigned char) ( val * r );
				rgbData[1] = (unsigned char) ( val * g );
				rgbData[0] = (unsigned char) ( val * b );
			}
			else
			{
				rgbData[0] = (unsigned char) ( val * r );
				rgbData[1] = (unsigned char) ( val * g );
				rgbData[2] = (unsigned char) ( val * b );
			}

			rgbData += 3;

			outImagePtr++;
		}

		rgbData += ( stride - this->outImage->width ) * 3;
	}
}

void CameraSource::release()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}

	if( this->colorSource )
	{
		cvReleaseImage( &( this->colorSource  ) );
		this->colorSource = NULL;
	}
}

IplImage *CameraSource::getColorImage()
{
	if( !nuiCamera )
	{
		std::cerr << "NUICamera not set!" << std::endl;
		return NULL;
	}
	if( !nuiCamera->hasColor() )
	{
		std::cerr << "NUICamera was not initialized with color stream!" << std::endl;
		return NULL;
	}

	XnUInt16 widthDepth = 0;
	XnUInt16 heightDepth = 0;
	XnUInt16 widthColor = 0;
	XnUInt16 heightColor = 0;

	this->nuiCamera->getDepthRes( widthDepth, heightDepth );
	this->nuiCamera->getDepthRes( widthColor, heightColor );

	if( widthDepth != widthColor || heightDepth != heightColor )
	{
		std::cerr << "NUICamera depth and color streams must have same dimensions!" << std::endl;
		return NULL;
	}

	if( this->isEnabled )
	{
		const XnRGB24Pixel *rgbData = this->nuiCamera->getColorData();
		memcpy( this->colorSource->imageData, rgbData, sizeof( XnRGB24Pixel ) * widthColor * heightColor );
	}

	return this->colorSource;
}


BackgroundSubtracter::BackgroundSubtracter( const std::string &fileName, double tolerance  ) :
	FrameProcessor(),
	doRecordBackground( false ),
	showBgAvrg( false ),
	doResetNoDataAreas( true ),
	resetNoDataThreshold( 10 ),
	tolerance( tolerance ),
	avrgSpeed( 0.1 ),
	bgAvrg( NULL ),
	bgImage( NULL ),
	mask( NULL ),
	noDataMask( NULL )
{
	if( fileName.size() )
		this->loadBackground( fileName );
}

BackgroundSubtracter::~BackgroundSubtracter()
{
	this->release();

	if( this->bgAvrg )
	{
		cvReleaseImage( &( this->bgAvrg ) );
		this->bgAvrg = NULL;
	}

	if( this->bgImage )
	{
		cvReleaseImage( &( this->bgImage ) );
		this->bgImage = NULL;
	}
}

const IplImage *BackgroundSubtracter::process( const IplImage *image, const IplImage *colorImage )
{
	this->colorImage = colorImage;

	this->checkSize( image->width, image->height );

	if( this->isEnabled && ( this->bgImage || this->doRecordBackground ) )	//TODO: avrg should be calculated at all times. this condition is wrong
	{
		if( !this->bgAvrg )
		{
			this->bgAvrg = cvCreateImage( cvSize( image->width, image->height ), IPL_DEPTH_16U, 1 );
			cvSet( this->bgAvrg, cvScalar( std::numeric_limits<unsigned short>::max() ) );
		}

		const unsigned short *imgPtr = (const unsigned short*)image->imageData;
		unsigned short *avrgPtr = (unsigned short*)this->bgAvrg->imageData;
		unsigned short *noDataPtr = (unsigned short*)this->noDataMask->imageData;
		for( int j = 0; j < image->width; j++ )
			for( int i = 0; i < image->height; i++ )
			{
				if( *imgPtr )
				{
					( *noDataPtr ) = 0; //reset counter

					if( *avrgPtr != std::numeric_limits<unsigned short>::max() )
						*avrgPtr = (unsigned short)( *avrgPtr * ( 1.0f - this->avrgSpeed ) + *imgPtr * this->avrgSpeed );
					else
						*avrgPtr = *imgPtr;
				}
				else if( this->doResetNoDataAreas )
				{
					( *noDataPtr )++;   //increase counter
					if( *noDataPtr >= this->resetNoDataThreshold )
					{
						//if counter exceeded threshold, then reset value in background mask to NOVALUE
						*avrgPtr = std::numeric_limits<unsigned short>::max();
					}
				}

				imgPtr++;
				avrgPtr++;
				noDataPtr++;
			}

		if( this->doRecordBackground )
		{
			if( this->bgImage && ( this->bgImage->width != image->width || this->bgImage->height != image->height ) )
			{
				cvReleaseImage( &( this->bgImage ) );
				this->bgImage = NULL;
			}
			if( !this->bgImage )
			{
				this->bgImage = cvCreateImage( cvSize( image->width, image->height ), IPL_DEPTH_16U, 1 );
				cvSet( this->bgImage, cvScalar( std::numeric_limits<unsigned short>::max() ) );
			}

			cvCopy( this->bgAvrg, this->bgImage );

			//set all OOR-Values to maxvalue
			unsigned short *tempPtr = (unsigned short*)( this->bgImage->imageData );
			for( int j = 0; j < this->bgImage->height; j++ )
				for( int i = 0; i < this->bgImage->width; i++, tempPtr++ )
					if( !( *tempPtr ) )
						*tempPtr = std::numeric_limits<unsigned short>::max();

			this->doRecordBackground = false;
		}

		if( image->depth != 16 )
			throw std::runtime_error( "requiring image with depth of 16 bit" );

		const unsigned short *inImgPtr = (const unsigned short*)( image->imageData );
		const unsigned short *bgImgPtr = (const unsigned short*)( this->bgImage->imageData );

		unsigned short *outImgPtr = (unsigned short*)( this->outImage->imageData );
		unsigned char *maskPtr = (unsigned char*)( this->mask->imageData );

		for( int j = 0; j < this->outImage->height; j++ )
			for( int i = 0; i < this->outImage->width; i++, inImgPtr++, bgImgPtr++, outImgPtr++, maskPtr++ )
			{
				*outImgPtr = ( (double)*bgImgPtr - *inImgPtr > this->tolerance ? *inImgPtr : 0x00 );
				*maskPtr = ( *outImgPtr != *inImgPtr ? 0xff : 0x00 );	//mask holds information of subtracted pixels
			}
	}
	else
	{
		cvCopy( image, this->outImage );
		cvSet( this->mask, cvScalar( 0x00 ) );
		cvSet( this->noDataMask, cvScalar( 0x0000 ) );
	}

#ifdef WIN32
	sprintf_s( this->desc,
		"[%s] {%dx%dx%dx%d} tolerance: %.03f",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->tolerance );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"[%s] {%dx%dx%dx%d} tolerance: %.03f",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->tolerance );
#endif // WIN32

	return this->outImage;
}

void BackgroundSubtracter::resize( unsigned int width, unsigned int height )
{
	if( this->outImage && width == this->outImage->width && height == this->outImage->height )
		return;

	this->release();

	this->outImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
	this->mask = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	this->noDataMask = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
}

void BackgroundSubtracter::release()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}
	if( this->mask )
	{
		cvReleaseImage( &( this->mask ) );
		this->mask = NULL;
	}
	if( this->noDataMask )
	{
		cvReleaseImage( &( this->noDataMask ) );
		this->noDataMask = NULL;
	}
}

void BackgroundSubtracter::getDebugView( unsigned char *rgbData, size_t stride, bool switchRB )
{
	if( !this->outImage )
		throw std::runtime_error( "background subtracter not initialized!" );

	if( stride < this->outImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned short *outImgPtr = (const unsigned short*)( this->outImage->imageData );
	const unsigned char *maskPtr = (const unsigned char* )( this->mask->imageData );
	const unsigned short *avrgPtr = (const unsigned short* )( this->bgAvrg ? this->bgAvrg->imageData : NULL );

	const unsigned char *colorImagePtr = ( this->colorImage ? (const unsigned char*)( this->colorImage->imageData ) : NULL );

	float r = 1.0f;
	float g = 1.0f;
	float b = 1.0f;

	for( int j = 0; j < this->outImage->height; j++ )
	{
		for( int i = 0; i < this->outImage->width; i++ )
		{
			unsigned char val = 255 - (unsigned char)( ( *outImgPtr ) * 255.0f / 7000.0f );

			if( this->showBgAvrg )
			{
				if( avrgPtr )
				{
					if( *avrgPtr == std::numeric_limits<unsigned short>::max() )
					{
						rgbData[0] = 255;
						rgbData[1] = 0;
						rgbData[2] = 255;
					}
					else
					{
						rgbData[0] = 255 - (unsigned char)( clamp( *avrgPtr / 7000.0f, 0.0f, 1.0f ) * 255.0f );
						rgbData[1] = 255 - (unsigned char)( clamp( *avrgPtr / 7000.0f, 0.0f, 1.0f ) * 255.0f );
						rgbData[2] = 255 - (unsigned char)( clamp( *avrgPtr / 7000.0f, 0.0f, 1.0f ) * 255.0f );
					}
				}
				else
				{
					rgbData[0] = 255;
					rgbData[1] = 255;
					rgbData[2] = 255;
				}
			}
			else
			{
				if( colorImagePtr )
				{
					r = colorImagePtr[0] / 255.0f;
					g = colorImagePtr[1] / 255.0f;
					b = colorImagePtr[2] / 255.0f;
					colorImagePtr += 3;
				}

				if( switchRB )
				{
					rgbData[2] = (unsigned char) ( val * r );
					rgbData[1] = (unsigned char) ( ( *maskPtr?0x00:val ) * g );
					rgbData[0] = (unsigned char) ( ( *maskPtr?0x00:val ) * b );
				}
				else
				{
					rgbData[0] = (unsigned char) ( val * r );
					rgbData[1] = (unsigned char) ( ( *maskPtr?0x00:val ) * g );
					rgbData[2] = (unsigned char) ( ( *maskPtr?0x00:val ) * b );
				}
			}

			rgbData += 3;

			outImgPtr++;
			maskPtr++;
			if( avrgPtr )
				avrgPtr++;
		}

		rgbData += ( stride - this->outImage->width ) * 3;
	}
}


bool BackgroundSubtracter::loadBackground( const std::string &filename )
{
	if( this->bgImage )
	{
		cvReleaseImage( &( this->bgImage ) );
		this->bgImage = NULL;
	}

	this->bgImage = cvLoadImage( filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH );

	if( !this->bgImage )
		return false;

	if( this->bgImage->nChannels != 1 || this->bgImage->depth != IPL_DEPTH_16U )
	{
		cvReleaseImage( &( this->bgImage ) );
		this->bgImage = NULL;

		throw std::runtime_error( "background image in invalid format" );
	}
	if( this->bgImage->width != this->outImage->width || this->bgImage->height != this->outImage->height )
	{
		cvReleaseImage( &( this->bgImage ) );
		this->bgImage = NULL;

		throw std::runtime_error( "loaded background image has invalid dimensions" );
	}

	return true;
}

bool BackgroundSubtracter::saveBackground( const std::string &filename ) const
{
	if( !this->bgImage )
	{
		std::cerr << "there is no background image to save" << std::endl;
		return false;
	}

	return ( cvSaveImage( filename.c_str(), this->bgImage, NULL ) ? true : false );
}

void BackgroundSubtracter::resetBackgroundAvrg()
{
	if( this->bgAvrg )
		cvSet( this->bgAvrg, cvScalar( std::numeric_limits<unsigned short>::max() ) );

	cvSet( this->noDataMask, cvScalar( 0x0000 ) );
}




MorphologicCloser::MorphologicCloser( unsigned int closingIterations ) :
	FrameProcessor(),
	closingIterations( closingIterations ),
	skip( true ),
	depthDifference( 0.0f ),
	mask( NULL ),
	maskOld( NULL ),
	diffMask( NULL )
{
}

MorphologicCloser::~MorphologicCloser()
{
	this->release();
}

const IplImage *MorphologicCloser::process( const IplImage *image, const IplImage *colorImage )
{
	this->colorImage = colorImage;

	this->checkSize( image->width, image->height );

	this->depthDifference = 0.0f;

	if( this->isEnabled )
	{
		const unsigned short *inPtr = ( const unsigned short* )( image->imageData );
		unsigned char *maskPtr = ( unsigned char* )( this->mask->imageData );
		unsigned int size = image->width * image->height;

		for( int i = 0; i < size; i++, inPtr++, maskPtr++ )
			*maskPtr = ( *inPtr ? 0xff : 0x00 );

		cvMorphologyEx( this->mask, this->mask, 0, 0, CV_MOP_CLOSE, this->closingIterations );

		//for first frame do nothing
		if( !this->skip )
		{
			cvXor( this->mask, this->maskOld, this->diffMask );

			this->depthDifference = (float)cvCountNonZero( this->diffMask ) / ( this->diffMask->width * this->diffMask->height );
		}
		else
			this->skip = false;

		cvCopy( this->mask, this->maskOld );
		cvCopy( image, this->outImage );
	}
	else
	{
		cvSet( this->maskOld, cvScalar( 0x00 ) );
		cvSet( this->mask, cvScalar( 0x00 ) );
		cvCopy( image, this->outImage );
		this->skip = true;
	}

#ifdef WIN32
	sprintf_s( this->desc,
		"[%s] {%dx%dx%dx%d} closing: %d",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->closingIterations );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"[%s] {%dx%dx%dx%d} closing: %d",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->closingIterations );
#endif // WIN32

	return this->outImage;
}

void MorphologicCloser::resize( unsigned int width, unsigned int height )
{
	if( this->outImage && width == this->outImage->width && height == this->outImage->height )
		return;

	this->release();

	this->outImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
	this->mask = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	this->maskOld = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
	this->diffMask = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );

	this->skip = true;
}

void MorphologicCloser::release()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}

	if( this->mask )
	{
		cvReleaseImage( &( this->mask ) );
		this->mask = NULL;
	}

	if( this->maskOld )
	{
		cvReleaseImage( &( this->maskOld ) );
		this->maskOld = NULL;
	}

	if( this->diffMask )
	{
		cvReleaseImage( &( this->diffMask ) );
		this->diffMask = NULL;
	}
}

void MorphologicCloser::getDebugView( unsigned char *rgbData, size_t stride, bool switchRB )
{
	if( !this->outImage )
		throw std::runtime_error( "diff extracter not initialized!" );

	if( stride < this->outImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned char *maskPtr = (const unsigned char* )( this->mask->imageData );
	const unsigned char *maskOldPtr = (const unsigned char* )( this->maskOld->imageData );

	for( int j = 0; j < this->outImage->height; j++ )
	{
		for( int i = 0; i < this->outImage->width; i++ )
		{
			if( *maskPtr )
			{
				if( *maskOldPtr )
				{
					rgbData[0] = 0xff;
					rgbData[1] = 0xff;
					rgbData[2] = 0xff;
				}
				else
				{
					if( switchRB )
					{
						rgbData[2] = 0x00;
						rgbData[1] = 0xff;
						rgbData[0] = 0xff;
					}
					else
					{
						rgbData[0] = 0x00;
						rgbData[1] = 0xff;
						rgbData[2] = 0xff;
					}
				}
			}
			else
			{
				if( *maskOldPtr )
				{
					if( switchRB )
					{
						rgbData[2] = 0xff;
						rgbData[1] = 0xff;
						rgbData[0] = 0x00;
					}
					else
					{
						rgbData[0] = 0xff;
						rgbData[1] = 0xff;
						rgbData[2] = 0x00;
					}
				}
				else
				{
					rgbData[0] = 0x00;
					rgbData[1] = 0x00;
					rgbData[2] = 0x00;
				}
			}

			rgbData += 3;

			maskPtr++;
			maskOldPtr++;
		}

		rgbData += ( stride - this->outImage->width ) * 3;
	}
}





Reprojector::Reprojector( const glm::mat4 &camToProj, const glm::mat4 &camToWorld, unsigned int closingIterations, unsigned int widthProjector, unsigned int heightProjector, double focalLengthProjector, double cxProjector, double cyProjector, double focalLengthCamera, double cxCamera, double cyCamera, double clipNear, double clipFar ) :
	FrameProcessor(),
	closingIterations( closingIterations ),
	widthProjector( widthProjector ),
	heightProjector( heightProjector ),
	focalLengthProjector( focalLengthProjector ),
	cxProjector( cxProjector ),
	cyProjector( cyProjector ),
	focalLengthCamera( focalLengthCamera ),
	cxCamera( cxCamera ),
	cyCamera( cyCamera ),
	camToProj( camToProj ),
	camToWorld( camToWorld ),
	clipNear( cvScalar( clipNear ) ),
	clipFar( cvScalar( clipFar ) ),
	clipNearPlaneRotationX( 0.0f ),
	clipNearPlaneRotationY( 0.0f ),
	clipFarPlaneRotationX( 0.0f ),
	clipFarPlaneRotationY( 0.0f ),
	clipRotationXMax( 45 ),
	clipRotationYMax( 65 ),
	clipNearPlaneNormal( unitZ() ),
	clipFarPlaneNormal( unitZ() ),
	cylinderClippingCenterX( 1.0 ),
	cylinderClippingCenterZ( 1.0 ),
	cylinderClippingRadius( 1.0 ),
	doFrustumClipping( true ),
	doCylinderClipping( true )
{
	if( clipNear >= clipFar )
		throw std::runtime_error( "clipnear has to be smaller than clipFar!" );

	this->outImage = cvCreateImage( cvSize( this->widthProjector, this->heightProjector ), IPL_DEPTH_16U, 1 );
}

Reprojector::~Reprojector()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}

	this->release();
}

const IplImage *Reprojector::process( const IplImage *image, const IplImage *colorImage )
{
	this->colorImage = colorImage;

	this->checkSize( image->width, image->height );

	if( this->isEnabled )
	{
		this->calcPointCloud( image );
		this->transformCloud();
		this->projectCloud( this->outImage );

		cvMorphologyEx( this->outImage, this->outImage, 0, 0, CV_MOP_CLOSE, this->closingIterations );
	}
	else
	{
		cvZero( this->outImage );
	}

#ifdef WIN32
	sprintf_s( this->desc,
		"[%s] {%dx%dx%dx%d} closing: %d, {%dx%d} f/cx/cy cam: %.03f/%.03f/%.03f, f/cx/cy prj: %.03f %.03f %.03f",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->closingIterations,
		this->widthProjector,
		this->heightProjector,
		this->focalLengthCamera,
		this->cxCamera,
		this->cyCamera,
		this->focalLengthProjector,
		this->cxProjector,
		this->cyProjector );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"[%s] {%dx%dx%dx%d} closing: %d, {%dx%d} f/cx/cy cam: %.03f/%.03f/%.03f, f/cx/cy prj: %.03f %.03f %.03f",
		( this->isEnabled ? "ON" : "OFF" ),
		this->outImage->width,
		this->outImage->height,
		this->outImage->nChannels,
		this->outImage->depth,
		this->closingIterations,
		this->widthProjector,
		this->heightProjector,
		this->focalLengthCamera,
		this->cxCamera,
		this->cyCamera,
		this->focalLengthProjector,
		this->cxProjector,
		this->cyProjector );
#endif // WIN32

	return this->outImage;
}

void Reprojector::resize( unsigned int width, unsigned int height )
{
	//do nothing -- projector image has to stay the same size, regardless of input dimensions
}

void Reprojector::release()
{}

void Reprojector::calcPointCloud( const IplImage *image )
{
	this->camVertices.clear();
	this->camColors.clear();

	const unsigned short *imgPtr = (const unsigned short*)( image->imageData );
	const unsigned char *colPtr = (const unsigned char*)( this->colorImage ? this->colorImage->imageData : NULL );

	XnVector3D v;
	XnVector3D c;

	for( int j = 0; j < image->height; j++ )
		for( int i = 0; i < image->width; i++ )
		{
			if( *imgPtr != NI_CAMERA_NO_VALUE )
			{
				v.X = i;
				v.Y = j;
				v.Z = -( *imgPtr );

				this->camVertices.push_back( v );

				if( colPtr )
				{
					c.X = colPtr[0] / 255.0f;
					c.Y = colPtr[1] / 255.0f;
					c.Z = colPtr[2] / 255.0f;

					this->camColors.push_back( c );

					colPtr += 3;
				}
			}

			if( colPtr )
				colPtr += 3;

			imgPtr++;
		}

	if( this->camVertices.size() )
		transform2DTo3D( this->camVertices.size(), &( this->camVertices[0] ), &( this->camVertices[0] ), this->cxCamera, this->cyCamera, this->focalLengthCamera, false );
}

void Reprojector::transformCloud()
{
	this->projVertices.clear();
	this->projColors.clear();

	glm::vec4 v;
	glm::vec4 vW;
	XnVector3D xnv;

	double dotNear = 0.0;
	double dotFar = 0.0;
	double nearP0z = -this->clipNear.val[0] * 0.001;
	double farP0z = -this->clipFar.val[0] * 0.001;

	VertexList::const_iterator itV = this->camVertices.begin();
	ColorList::const_iterator itC = this->camColors.begin();
	bool useColor = ( this->camColors.size() != 0 );

	if( useColor && this->camVertices.size() != this->camVertices.size() )
		std::cerr << "cam vertex and color arrays have different sizes, this will cause problems!" << std::endl;

	float clippingRadius2 = std::pow( this->cylinderClippingRadius, 2.0f );
	for( ; itV != this->camVertices.end(); ++itV )
	{
		if( this->doFrustumClipping )
		{
			//TODO: assuming plane P0 to be on z-axis (0/0/clipFar, 0/0/clipNear respectively) for now, thus taking no cx/cy
			// into account -- maybe correct this some time if necessary.
			dotFar =
				( itV->X - 0.0 ) * this->clipFarPlaneNormal.x +
				( itV->Y - 0.0 ) * this->clipFarPlaneNormal.y +
				( itV->Z - farP0z ) * this->clipFarPlaneNormal.z;
			dotNear =
				( itV->X - 0.0 ) * this->clipNearPlaneNormal.x +
				( itV->Y - 0.0 ) * this->clipNearPlaneNormal.y +
				( itV->Z - nearP0z ) * this->clipNearPlaneNormal.z;

			if( dotFar < 0.0 || dotNear > 0.0 )
				continue;
		}

		v.x = itV->X;
		v.y = itV->Y;
		v.z = itV->Z;
		v.w = 1.0f;

		if( this->doCylinderClipping )
		{
			vW = this->camToWorld * v;
			float radius2 = std::pow( vW.x - this->cylinderClippingCenterX, 2.0f ) + std::pow( vW.z - this->cylinderClippingCenterZ, 2.0f );
			if( radius2 > clippingRadius2 )
				continue;
		}

		v = this->camToProj * v;

		xnv.X = v.x;
		xnv.Y = v.y;
		xnv.Z = v.z;

		this->projVertices.push_back( xnv );
		if( useColor )
			this->projColors.push_back( *itC );
	}
}

void Reprojector::projectCloud( IplImage *image )
{
	cvZero( image );

	if( !this->projVertices.size() )
		return;

	VertexList tempList;
	tempList.resize( this->projVertices.size() );

	transform3DTo2D( this->projVertices.size(), &( this->projVertices[0] ), &( tempList[0] ), this->cxProjector, this->cyProjector, this->focalLengthProjector, false );

	unsigned short val = 0;
	unsigned short *tempPtr = NULL;

	for( VertexList::const_iterator it = tempList.begin(); it != tempList.end(); ++it )
	{
		unsigned int x = it->X;
		unsigned int y = it->Y;
		unsigned short depth = -( it->Z );

		if( x >= image->width || y >= image->height )
			continue;

		val = depth;
		tempPtr = (unsigned short*)( image->imageData ) + x + y * image->width;

		if( !( *tempPtr ) || *tempPtr > val )	//z-test
			*tempPtr = val;
	}
}

void Reprojector::getDebugView( unsigned char *rgbData, size_t stride, bool switchRB )
{
	if( !this->outImage )
		throw std::runtime_error( "reprojector not initialized!" );

	if( stride < this->outImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned short *outImagePtr = (const unsigned short*)( this->outImage->imageData );

	for( int j = 0; j < this->outImage->height; j++ )
	{
		for( int i = 0; i < this->outImage->width; i++ )
		{
			unsigned char val = (unsigned char)( ( *outImagePtr ) * 255.0f / 14000.0f );

			if( switchRB )
			{
				rgbData[2] = *outImagePtr?val:0x00;
				rgbData[1] = *outImagePtr?val:0x00;
				rgbData[0] = *outImagePtr?val:0xff;
			}
			else
			{
				rgbData[0] = *outImagePtr?val:0x00;
				rgbData[1] = *outImagePtr?val:0x00;
				rgbData[2] = *outImagePtr?val:0xff;
			}

			rgbData += 3;

			outImagePtr++;
		}

		rgbData += ( stride - this->outImage->width ) * 3;
	}
}

void Reprojector::setClipNearPlaneRotationX( float rot )
{
	this->clipNearPlaneRotationX = clamp( rot, -this->clipRotationXMax, this->clipRotationXMax );

	glm::mat3 m( glm::rotate( glm::rotate( this->clipNearPlaneRotationX, unitX() ), this->clipNearPlaneRotationY, unitY() ) );
	this->clipNearPlaneNormal = m * unitZ();
}

void Reprojector::setClipNearPlaneRotationY( float rot )
{
	this->clipNearPlaneRotationY = clamp( rot, -this->clipRotationYMax, this->clipRotationYMax );

	glm::mat3 m( glm::rotate( glm::rotate( this->clipNearPlaneRotationX, unitX() ), this->clipNearPlaneRotationY, unitY() ) );
	this->clipNearPlaneNormal = m * unitZ();
}

void Reprojector::setClipFarPlaneRotationX( float rot )
{
	this->clipFarPlaneRotationX = clamp( rot, -this->clipRotationXMax, this->clipRotationXMax );;

	glm::mat3 m( glm::rotate( glm::rotate( this->clipFarPlaneRotationX, unitX() ), this->clipFarPlaneRotationY, unitY() ) );
	this->clipFarPlaneNormal = m * unitZ();
}

void Reprojector::setClipFarPlaneRotationY( float rot )
{
	this->clipFarPlaneRotationY = clamp( rot, -this->clipRotationYMax, this->clipRotationYMax );;

	glm::mat3 m( glm::rotate( glm::rotate( this->clipFarPlaneRotationX, unitX() ), this->clipFarPlaneRotationY, unitY() ) );
	this->clipFarPlaneNormal = m * unitZ();
}




FrameProcessorChain::FrameProcessorChain() :
	sendColor( false )
{}

FrameProcessorChain::~FrameProcessorChain()
{
	for( FrameProcessorList::iterator it = this->chain.begin(); it != this->chain.end(); ++it )
		delete( *it );
	this->chain.clear();
}

void FrameProcessorChain::add( FrameProcessor *processor )
{
	this->chain.push_back( processor );
}

FrameProcessor *FrameProcessorChain::getLink( unsigned int index )
{
	if( index >= this->chain.size() )
		return NULL;
	return this->chain[index];
}

void FrameProcessorChain::resize( unsigned int width, unsigned int height )
{
	for( FrameProcessorList::iterator it = this->chain.begin(); it != this->chain.end(); ++it )
		( *it )->resize( width, height );
}

void FrameProcessorChain::update()
{
	const IplImage *depthImage = NULL;
	const IplImage *colorImage = NULL;

	if( this->sendColor )
	{
		CameraSource *cs = this->getLink<CameraSource>();
		if( cs )
			colorImage = cs->getColorImage();
	}

	for( FrameProcessorList::iterator it = this->chain.begin(); it != this->chain.end(); ++it )
		depthImage = ( *it )->process( depthImage, colorImage );
}

