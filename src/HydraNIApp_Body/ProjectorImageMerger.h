#pragma once

#include <opencv/cv.h>

#include "../HydraNILib_Common/Common.h"

#include <vector>

namespace hydraNI
{
	class ProjectorImageMerger
	{
	private:
		IplImage	*outImage;

		typedef	std::vector<IplImage*>	ImageList;

		ImageList	list;

		std::vector<bool>	useSource;

		char	desc[256];

		void checkSize( unsigned int width, unsigned int height );

	public:
		ProjectorImageMerger();
		~ProjectorImageMerger();

		void add( IplImage *inImage );	// ProjectorImageMerger does NOT take ownership!!

		const IplImage *process();

		void getDebugView( unsigned char *rgbData, size_t stride );

		void setUseSource( int id, bool value );
		bool getUseSource( int id );

		const char *getName() const	{	return "ProjectorImageMerger";	}
		const char *getDesc()		{	return this->desc;				}
	};
}
