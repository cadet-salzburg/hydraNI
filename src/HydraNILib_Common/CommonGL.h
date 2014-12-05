#pragma once

#include "Common.h"

namespace hydraNI
{
	bool checkForGLError();
	bool checkFBOStatus();

	void logGLCaps();

	void drawAxes( double scale, double colorScale = 0.0 );
	void drawLocator( const float *color, double scale );
	void drawRect( float x0, float y0, float x1, float y1 );
	void drawCameraCone( double sensorWidth, double sensorHeight, double focalLength, double clipNear, double clipFar, const float *color );
	void drawCameraCone( double sensorWidth, double sensorHeight, double focalLength, double clipNear, double clipFar, const float *color, float opticalCenterX, float opticalCenterY );
	void drawCameraFrameBuffer( double sensorWidth, double sensorHeight, double focalLength, double coneDepth, const float *color, unsigned int textureID, float *texcoords );
	void drawCameraFrameBuffer( double sensorWidth, double sensorHeight, double focalLength, double coneDepth, const float *color, unsigned int textureID, float *texcoords, float opticalCenterX, float opticalCenterY );

	void drawQuad( float x, float y, float z, float width, float height, const float *color, bool filled = true );
	void drawCircle( float x, float y, float z, float r, const float *color, bool filled = true, int sections = 20 );
	void drawTri( float x, float y, float z, float r, float angle, const float *color, bool filled = true );

	void drawGrid( const float *color, float size = 10, unsigned int subdivisions = 9 );
}