#include "ProjectorImageMerger.h"

#include <opencv/highgui.h>


using namespace hydraNI;


ProjectorImageMerger::ProjectorImageMerger() :
	outImage( NULL )
{
	this->desc[0] = 0;
}

ProjectorImageMerger::~ProjectorImageMerger()
{
	if( this->outImage )
	{
		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}
}

bool ProjectorImageMerger::getUseSource( int id )
{
	return this->useSource[id];
}

void ProjectorImageMerger::setUseSource( int id, bool value )
{
	this->useSource[id] = value;
}

void ProjectorImageMerger::add( IplImage *inImage )
{
	this->list.push_back( inImage );
	this->useSource.push_back( true );

	this->checkSize( inImage->width, inImage->height );
}

const IplImage *ProjectorImageMerger::process()
{
	cvZero( this->outImage );

	int id = 0;
	for( ImageList::const_iterator it = this->list.begin(); it != this->list.end(); ++it, id++ )
	{
		if( !this->useSource[id] )
			continue;

		unsigned short *outPtr = ( unsigned short* )( this->outImage->imageData );
		const unsigned short *inPtr = ( const unsigned short* )( ( *it )->imageData );

		unsigned int size = this->outImage->width * this->outImage->height;

		for( int i = 0; i < size; i++, outPtr++, inPtr++ )
			if( *inPtr && ( !( *outPtr ) || *outPtr > *inPtr ) )	//z-test
				*outPtr = *inPtr;
	}

#ifdef WIN32
	sprintf_s( this->desc,
		"sources: %d",
		this->list.size() );
	for( int i = 0; i < this->useSource.size(); i++ )
		strncat( this->desc, ( this->useSource[i] ? ", ON" : ", OFF" ), sizeof( this->desc ) - strlen( this->desc ) );
#else
	snprintf( this->desc, sizeof( this->desc ),
		"sources: %d",
		this->list.size() );
	for( int i = 0; i < this->useSource.size(); i++ )
		strncat( this->desc, ( this->useSource[i] ? ", ON" : ", OFF" ), sizeof( this->desc ) - strlen( this->desc ) );
#endif // WIN32

	return this->outImage;
}

void ProjectorImageMerger::checkSize( unsigned int width, unsigned int height )
{
	if( this->outImage )
	{
		if( this->outImage && width == this->outImage->width && height == this->outImage->height )
			return;

		cvReleaseImage( &( this->outImage ) );
		this->outImage = NULL;
	}

	if( !this->outImage )
		this->outImage = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
}

void ProjectorImageMerger::getDebugView( unsigned char *rgbData, size_t stride )
{
	if( !this->outImage )
		throw std::runtime_error( "camera source not initialized!" );

	if( stride < this->outImage->width )
		throw std::runtime_error( "target texture too small, resampling not implemented" );

	const unsigned short *outImagePtr = (const unsigned short*)( this->outImage->imageData );

	for( int j = 0; j < this->outImage->height; j++ )
	{
		for( int i = 0; i < this->outImage->width; i++ )
		{
			unsigned char val = (unsigned char)( ( *outImagePtr ) * 255.0f / 14000.0f );

			if( !( *outImagePtr ) )
			{
				rgbData[0] = 0xff;
				rgbData[1] = 0xff;
				rgbData[2] = 0x00;
			}
			else
			{
				rgbData[0] = val;
				rgbData[1] = val;
				rgbData[2] = val;
			}
			rgbData += 3;

			outImagePtr++;
		}

		rgbData += ( stride - this->outImage->width ) * 3;
	}
}
