#include "BlobTracker.h"

#include "../HydraNILib_Common/Common.h"

#include <opencv/highgui.h>

using namespace hydraNI;


namespace hydraNI
{
	void drawCross( IplImage *img, const CvPoint &pt, const CvScalar &color, unsigned int len = 5, unsigned int thickness = 1 )
	{
		cvLine( img,
			cvPoint( pt.x - len / 2, pt.y - len / 2 ),
			cvPoint( pt.x + len / 2, pt.y + len / 2 ),
			color, thickness );
		cvLine( img,
			cvPoint( pt.x - len / 2, pt.y + len / 2 ),
			cvPoint( pt.x + len / 2, pt.y - len / 2 ),
			color, thickness );
	}
}

BlobTracker::BlobTracker( unsigned int width, unsigned int height ) :
	isRunning( false ),
	sendAllContours( false ),
	currentTime( 0.0f ),
	flowVectorScale( 0.0f ),
	minSize( 1000 ),
	inMask( cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 ) ),
	outMask( cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 ) ),
	rgbImage( cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 3 ) ),
	memStorage( cvCreateMemStorage( 0 ) )
{
	this->desc[0] = 0;
}

BlobTracker::~BlobTracker()
{
	if( this->inMask )
	{
		cvReleaseImage( &this->inMask );
		this->inMask = NULL;
	}

	if( this->outMask )
	{
		cvReleaseImage( &this->outMask );
		this->outMask = NULL;
	}

	if( this->rgbImage )
	{
		cvReleaseImage( &this->rgbImage );
		this->rgbImage = NULL;
	}
}

void BlobTracker::start()
{
	if( this->isRunning )
		return;

	this->isRunning = true;
}

void BlobTracker::stop()
{
	this->isRunning = false;
}

bool BlobTracker::updateFrame( const IplImage *image, double dt )
{
	this->currentTime += dt;

	if( this->inMask && ( this->inMask->width != image->width || this->inMask->height != image->height ) )
	{
		cvReleaseImage( &this->inMask );
		this->inMask = NULL;
	}
	if( this->outMask && ( this->outMask->width != image->width || this->outMask->height != image->height ) )
	{
		cvReleaseImage( &this->outMask );
		this->outMask = NULL;
	}
	if( this->rgbImage && ( this->rgbImage->width != image->width || this->rgbImage->height != image->height ) )
	{
		cvReleaseImage( &this->rgbImage );
		this->rgbImage = NULL;
	}

	if( !this->inMask )
		this->inMask = cvCreateImage( cvSize( image->width, image->height ), IPL_DEPTH_8U, 1 );
	if( !this->outMask )
		this->outMask = cvCreateImage( cvSize( image->width, image->height ), IPL_DEPTH_8U, 1 );
	if( !this->rgbImage )
		this->rgbImage = cvCreateImage( cvSize( image->width, image->height ), IPL_DEPTH_8U, 3 );

	cvClearMemStorage( this->memStorage );

	cvConvert( image, this->inMask );
	cvThreshold( this->inMask, this->inMask, 1, 255, CV_THRESH_BINARY );

	this->twoBlobsList.clear();
	this->twoContoursList.clear();
	this->flowVectorsList.clear();
	this->comList.clear();

	this->contours.clear();

	if( this->isRunning )
	{
		CvSeq *maxAreaContour = NULL;
		std::list<CvSeq*> sequenceList;

		if( this->sendAllContours )
			this->getAllContours( this->inMask, sequenceList );
		else
			maxAreaContour = this->getMaxAreaContour( this->inMask );

		cvZero( this->outMask );
		cvZero( this->rgbImage );

		if( maxAreaContour || sequenceList.size() )
		{
			Rectangle boundingrect;

			if( sequenceList.size() )
			{
				for( std::list<CvSeq*>::const_iterator it = sequenceList.begin(); it != sequenceList.end(); ++it )
					this->drawAndAdd( *it, boundingrect );
			}
			else if( maxAreaContour )
				this->drawAndAdd( maxAreaContour, boundingrect );

			unsigned int vectorsX = 11;
			unsigned int vectorsY = 11;

			FlowVectors flowVectors;
			flowVectors.pos1 = boundingrect.pos;

			flowVectors.step.x = 2.0f / this->rgbImage->width;	// 2px distance between vectors in pixel space
			flowVectors.step.y = 2.0f / this->rgbImage->height;	// 2px distance between vectors in pixel space

			if( vectorsX * vectorsY * 2 > FLOWVECTORS_MAX )
				throw std::runtime_error( "too many flow vectors" );

			unsigned int centerX = vectorsX / 2;
			unsigned int centerY = vectorsY / 2;

			unsigned int cntr = 0;
			for( int j = 0; j < vectorsY; j++ )
				for( int i = 0; i < vectorsX; i++, cntr++ )
				{
					flowVectors.flowvectors[cntr].x = ( i - centerX ) * this->flowVectorScale / dt;
					flowVectors.flowvectors[cntr].y = ( j - centerY ) * this->flowVectorScale / dt;
				}

			for( cntr; cntr < FLOWVECTORS_MAX; cntr++ )
			{
				flowVectors.flowvectors[cntr].x = 0.0f;
				flowVectors.flowvectors[cntr].y = 0.0f;
			}

			this->flowVectorsList.push_back( flowVectors );

			cntr = 0;
			double sumX = 0.0;
			double sumY = 0.0;
			double sumZ = 0.0;

			unsigned char *maskPtr = (unsigned char*)( this->outMask->imageData );
			unsigned short *imgPtr = (unsigned short*)( image->imageData );

			for( unsigned int j = 0; j < this->outMask->height; j++ )
				for( unsigned int i = 0; i < this->outMask->width; i++, maskPtr++, imgPtr++ )
					if( *maskPtr && *imgPtr )
					{
						sumX += i;
						sumY += j;
						sumZ += *imgPtr;
						cntr++;
					}

			glm::vec3 v(
				sumX / cntr,
				sumY / cntr,
				sumZ / cntr );

			this->comList.push_back( v );
		}

		const CvScalar CVX_RED		= cvScalar( 0xff, 0x00, 0x00 );
		const CvScalar CVX_GREEN	= cvScalar( 0x00, 0xff, 0x00 );

		for( std::vector<TwoBlobs>::const_iterator it = this->twoBlobsList.begin(); it != this->twoBlobsList.end(); ++it )
		{
			cvDrawRect(
				this->rgbImage,
				cvPoint( ( it->blob1.boundingbox.pos.x + 1.0f ) * 0.5f * this->rgbImage->width, ( -it->blob1.boundingbox.pos.y + 1.0f ) * 0.5f * this->rgbImage->height ),
				cvPoint( ( it->blob1.boundingbox.pos.x + it->blob1.boundingbox.width + 1.0f ) * 0.5f * this->rgbImage->width, ( -it->blob1.boundingbox.pos.y + it->blob1.boundingbox.height + 1.0f ) * 0.5f * this->rgbImage->height ),
				CVX_GREEN );

			drawCross(
				this->rgbImage,
				cvPoint( ( it->blob1.centroid.x + 1.0f ) * 0.5f * this->rgbImage->width, ( -it->blob1.centroid.y + 1.0f ) * 0.5f * this->rgbImage->height ),
				CVX_RED );
		}
	}

#ifdef WIN32
	sprintf_s( this->desc,
		"[%s] minSize: %i, search: %s, blobs: %d, contours: %d, vectors: %d, COMs: %d",
		( this->isRunning ? "ON" : "OFF" ),
		this->minSize,
		( this->sendAllContours ? "ALL" : "TALLEST" ),
		this->twoBlobsList.size() / 2,
		this->twoContoursList.size() / 2,
		this->flowVectorsList.size() / 2,
		this->comList.size() );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"[%s] minSize: %i, search: %s, blobs: %d, contours: %d, vectors: %d, COMs: %d",
		( this->isRunning ? "ON" : "OFF" ),
		this->minSize,
		( this->sendAllContours ? "ALL" : "TALLEST" ),
		this->twoBlobsList.size() / 2,
		this->twoContoursList.size() / 2,
		this->flowVectorsList.size() / 2,
		this->comList.size() );
#endif // WIN32

	return true;
}

CvSeq *BlobTracker::getMaxAreaContour( IplImage *image )
{
	CvContourScanner scanner = cvStartFindContours(
		image,
		this->memStorage,
		sizeof( CvContour ),
		CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE );

	CvSeq *contour = NULL;

	double maxArea = 0.0;
	CvSeq *maxAreaContour = NULL;

	while( ( contour = cvFindNextContour( scanner ) ) != NULL )
	{
		double area = cvContourArea( contour );

		if( area < this->minSize )
			cvSubstituteContour( scanner, NULL );
		else
		{
			CvRect r = cvBoundingRect( contour );
			if( r.width > 480 )
			{
				cvSubstituteContour( scanner, NULL );
				continue;
			}

			int maxDist = 1;
			CvSeq *simplified = NULL;

			do
			{
				simplified = cvApproxPoly( contour, sizeof( CvContour ), NULL, CV_POLY_APPROX_DP, maxDist );
				maxDist++;
			}while( simplified->total > CONTOURPOINTS_MAX );

			if( area > maxArea )
			{
				maxArea = area;
				maxAreaContour = simplified;
			}

			cvSubstituteContour( scanner, simplified );
		}
	}

	cvEndFindContours( &scanner );

	return maxAreaContour;
}

void BlobTracker::getAllContours( IplImage *image, std::list<CvSeq*> &list )
{
	list.clear();

	CvContourScanner scanner = cvStartFindContours(
		image,
		this->memStorage,
		sizeof( CvContour ),
		CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE );

	CvSeq *contour = NULL;

	while( ( contour = cvFindNextContour( scanner ) ) != NULL )
	{
		double area = cvContourArea( contour );

		if( area < this->minSize )
			cvSubstituteContour( scanner, NULL );
		else
		{
			int maxDist = 1;
			CvSeq *simplified = NULL;

			do
			{
				simplified = cvApproxPoly( contour, sizeof( CvContour ), NULL, CV_POLY_APPROX_DP, maxDist );
				maxDist++;
			}while( simplified->total > CONTOURPOINTS_MAX );

			list.push_back( simplified );

			cvSubstituteContour( scanner, simplified );
		}
	}

	cvEndFindContours( &scanner );
}

void BlobTracker::drawAndAdd( CvSeq *contour, hydraNI::Rectangle &boundingrect )
{
	const CvScalar CVX_WHITE	= cvScalar( 0xff, 0xff, 0xff );
	const CvScalar CVX_GREY		= cvScalar( 0x7f, 0x7f, 0x7f );

	CvMoments moments;

	CvRect r = cvBoundingRect( contour );
	cvContourMoments( contour, &moments );

	Point2 centroid(
		( moments.m10 / moments.m00 / this->inMask->width * 2.0f - 1.0f ),
		-( moments.m01 / moments.m00 / this->inMask->height * 2.0f - 1.0f ) );

	boundingrect.width = (float)r.width / this->inMask->width * 2.0f,
	boundingrect.height = (float)r.height / this->inMask->height * 2.0f,
	boundingrect.pos = Point2( ( (float)r.x / this->inMask->width * 2.0f - 1.0f ),
				-( (float)r.y / this->inMask->height * 2.0f - 1.0f ) );

	Blob b( centroid, boundingrect );

	this->twoBlobsList.push_back( TwoBlobs( b, b, STEREO_LEFT ) );
	this->twoBlobsList.push_back( TwoBlobs( b, b, STEREO_RIGHT ) );

	cvDrawContours( this->rgbImage, contour, CVX_WHITE, CVX_GREY, -1, 1 );
	cvDrawContours( this->outMask, contour, CVX_WHITE, CVX_GREY, -1, -1 );

	TwoContours twoContoursLeft;
	twoContoursLeft.num_points_1 = contour->total;
	twoContoursLeft.num_points_2 = 0;
	twoContoursLeft.stereopos = STEREO_LEFT;

	TwoContours twoContoursRight;
	twoContoursRight.num_points_1 = contour->total;
	twoContoursRight.num_points_2 = 0;
	twoContoursRight.stereopos = STEREO_RIGHT;

	Contour cont;

	for( int i = 0; i < CONTOURPOINTS_MAX; i++ )
	{
		if( i < contour->total )
		{
			CvPoint *pt = (CvPoint*)CV_GET_SEQ_ELEM( CvPoint, contour, i );

			twoContoursLeft.contourpoints[i].x = ( (float)pt->x / this->rgbImage->width * 2.0f - 1.0f );
			twoContoursLeft.contourpoints[i].y = -( (float)pt->y / this->rgbImage->height * 2.0f - 1.0f );

			twoContoursRight.contourpoints[i].x = twoContoursLeft.contourpoints[i].x;
			twoContoursRight.contourpoints[i].y = twoContoursLeft.contourpoints[i].y;

			cont.push_back( glm::vec2( twoContoursLeft.contourpoints[i].x, twoContoursLeft.contourpoints[i].y ) );
		}
		else
		{
			twoContoursLeft.contourpoints[i].x = 0.0f;
			twoContoursLeft.contourpoints[i].y = 0.0f;

			twoContoursRight.contourpoints[i].x = 0.0f;
			twoContoursRight.contourpoints[i].y = 0.0f;
		}
	}

	this->twoContoursList.push_back( twoContoursLeft );
	this->twoContoursList.push_back( twoContoursRight );

	this->contours.push_back( cont );
}

void BlobTracker::getDebugView( unsigned char *rgbData, size_t stride, bool switchRB )
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
			if( switchRB )
			{
				rgbData[2] = rgbImagePtr[0];
				rgbData[1] = rgbImagePtr[1];
				rgbData[0] = rgbImagePtr[2];
			}
			else
			{
				rgbData[0] = rgbImagePtr[0];
				rgbData[1] = rgbImagePtr[1];
				rgbData[2] = rgbImagePtr[2];
			}
			rgbData += 3;
			rgbImagePtr += 3;
		}

		rgbData += ( stride - this->rgbImage->width ) * 3;
	}
}
