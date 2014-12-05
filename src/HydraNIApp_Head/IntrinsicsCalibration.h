#pragma once

#include <glm/glm.hpp>

#include <opencv/cv.h>

namespace hydraNI
{
	class IntrinsicsCalibration
	{
	private:
		bool			enabled;
		bool			refinementMode;
		bool			foundChessboardInFrame;

		unsigned int	chessboardFrames;
		unsigned int	chessboardCornerCount;

		unsigned int	requiredSteps;
		double			timeUntilNextStep;
		double			timeBetweenSteps;

		IplImage	*inImage;
		IplImage	*rgbImage;

		CvSize		boardSize;

		CvMat			*objectPointsMat;
		CvPoint3D32f	*objectPoints;

		CvMat			*imagePointsMatCam;
		CvPoint2D32f	*imagePointsCam;

		CvMat			*imagePointsOverall;
		CvMat			*objectPointsOverall;
		CvMat			*pointCountsCamera;

		CvMat			*intrinsicMatrix;
		CvMat			*distortionCoeffs;

		glm::mat3		glIntrinsics;

		void checkSize( unsigned int width, unsigned int height );
		void updateGLMatrices();

	public:
		IntrinsicsCalibration( unsigned int width, unsigned int height, unsigned int boardSizeX, unsigned int boardSizeY, double fieldWidth, double fieldHeight, unsigned int requiredSteps, double timeBetweenSteps );
		~IntrinsicsCalibration();

		bool getEnabled() const				{	return this->enabled;		}

		void start();
		void abort();

		bool update( double dt, const unsigned char *rgbData, const unsigned short *depthData, unsigned int width, unsigned int height, size_t pixelSize, size_t layers );

		void getDebugView( unsigned char *rgbData, size_t stride );

		void setIntrinsics( double fx, double fy, double cx, double cy, double skew );
		void setDistortionCoeffs( const float *c );

		const glm::mat3 &getIntrinsics() const		{	return this->glIntrinsics;			}

		const float *getDistortionCoeffs() const	{	return this->distortionCoeffs->data.fl;		}

		bool getFoundChessboardInFrame() const		{	return this->foundChessboardInFrame;	}
	};
}
