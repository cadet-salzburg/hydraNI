#pragma once

#include <glm/glm.hpp>

#include <opencv/cv.h>

namespace hydraNI
{
	class ExtrinsicsCalibration
	{
	private:
		bool	enabled;
		bool	useAdaptiveThreshold;
		bool	adaptiveThresholdGaussian;

		unsigned int	adaptiveThresholdBlockSize;
		unsigned int	adaptiveThresholdOffset;

		IplImage	*rgbImage;
		IplImage	*gsImage;
		IplImage	*threshImage;

		CvSize		boardSize;

		CvMat			*objectPointsMat;
		CvPoint3D32f	*objectPoints;

		CvMat			*imagePointsMatCam;
		CvPoint2D32f	*imagePointsCam;

		CvMat			*intrinsicMatrix;
		CvMat			*distortionCoeffs;

		glm::mat4		glExtrinsics;
		glm::mat4		glExtrinsicsInv;

		void checkSize( unsigned int width, unsigned int height );

	public:
		ExtrinsicsCalibration( unsigned int width, unsigned int height, unsigned int boardSizeX, unsigned int boardSizeY, double fieldWidth, double fieldHeight, double fx, double fy, double cx, double cy, double skew );
		~ExtrinsicsCalibration();

		bool getEnabled() const				{	return this->enabled;		}
		void setEnabled( bool enabled )		{	this->enabled = enabled;	}

		bool getAdaptiveThreshold() const						{	return this->useAdaptiveThreshold;					}
		void setAdaptiveThreshold( bool useAdaptiveThreshold )	{	this->useAdaptiveThreshold = useAdaptiveThreshold;	}

		unsigned int getAdaptiveThresholdBlockSize() const				{	return this->adaptiveThresholdBlockSize;		}
		void setAdaptiveThresholdBlockSize( unsigned int blockSize )	{	this->adaptiveThresholdBlockSize = blockSize;	}

		unsigned int getAdaptiveThresholdOffset() const			{	return this->adaptiveThresholdOffset;			}
		void setAdaptiveThresholdOffset( unsigned int offset )	{	this->adaptiveThresholdOffset = offset;			}

		bool update( const unsigned char *rgbData, unsigned int width, unsigned int height, size_t pixelSize, size_t layers );

		void getDebugView( unsigned char *rgbData, size_t stride );

		void setIntrinsics( double fx, double fy, double cx, double cy, double skew );
		void setDistortionCoeffs( const float *c );

		const glm::mat4 &getExtrinsics() const		{	return this->glExtrinsics;			}
		const glm::mat4 &getExtrinsicsInv() const	{	return this->glExtrinsicsInv;		}

		void setExtrinsics( const glm::mat4 &e, const glm::mat4 &eInv );
	};
}