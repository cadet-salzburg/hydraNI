#pragma once

#include "../HydraNILib_Common/Common.h"

#include "Contours.h"

#include <opencv/cv.h>

#include <string>
#include <vector>
#include <list>

#include <glm/glm.hpp>

namespace hydraNI
{
	typedef std::vector<glm::vec2>	Contour;

	class BlobTracker
	{
	private:
		bool	isRunning;
		bool	sendAllContours;

		float	currentTime;

		double	flowVectorScale;

		unsigned int	minSize;

		IplImage		*inMask;
		IplImage		*outMask;
		IplImage		*rgbImage;

		CvMemStorage	*memStorage;

		std::vector<TwoBlobs>		twoBlobsList;
		std::vector<FlowVectors>	flowVectorsList;
		std::vector<TwoContours>	twoContoursList;
		std::vector<glm::vec3>		comList;

		std::vector<Contour>		contours;

		char			desc[256];

		CvSeq *getMaxAreaContour( IplImage *image );
		void getAllContours( IplImage *image, std::list<CvSeq*> &list );

		void drawAndAdd( CvSeq *contour, hydraNI::Rectangle &boundingrect );

	public:
		BlobTracker( unsigned int width, unsigned int height );
		~BlobTracker();

		void start();
		void stop();

		void setSendAllContours( bool sendAll )	{	this->sendAllContours = sendAll;	}
		bool getSendAllContours() const			{	return this->sendAllContours;		}

		void setMinSize( unsigned int minSize )	{	this->minSize = minSize;		}
		unsigned int getMinSize() const			{	return this->minSize;			}

		void setFlowVectorScale( double s )		{	this->flowVectorScale = s;		}
		double getFlowVectorScale() const		{	return this->flowVectorScale;	}

		bool updateFrame( const IplImage *image, double dt );

		bool running() const				{	return this->isRunning;			}

		const std::vector<TwoBlobs>		&getTwoBlobsList() const		{	return this->twoBlobsList;			}
		const std::vector<TwoContours>	&getTwoContoursList() const		{	return this->twoContoursList;		}
		const std::vector<FlowVectors>	&getFlowVectorsList() const		{	return this->flowVectorsList;		}
		const std::vector<glm::vec3>	&getCOMList() const				{	return this->comList;	}

		const std::vector<Contour>		&getContours() const			{	return this->contours;	}

		void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB );

		const char *getName() const	{	return "BlobTracker";	}
		const char *getDesc()		{	return this->desc;		}
	};
}
