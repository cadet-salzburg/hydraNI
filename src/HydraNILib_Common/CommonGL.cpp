#include "CommonGL.h"

#include <iostream>

#ifdef WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>
#include <GL/glu.h>

using namespace hydraNI;


#ifdef _DEBUG
bool hydraNI::checkForGLError()
{
	GLenum errCode;
	if( ( errCode = glGetError() ) != GL_NO_ERROR )
	{
		std::cerr << "OpenGL error: " << gluErrorString( errCode ) << std::endl;
		return false;
	}
	return true;
}
#else
bool hydraNI::checkForGLError()	{ return true; }
#endif //_DEBUG

bool hydraNI::checkFBOStatus()
{
	switch( glCheckFramebufferStatus( GL_DRAW_FRAMEBUFFER ) )
	{
	case GL_FRAMEBUFFER_COMPLETE:
		std::cout << "FBO complete" << std::endl;
		return true;
	case GL_FRAMEBUFFER_UNDEFINED:
		std::cerr << "FBO undefined" << std::endl;
		return false;
	case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
		std::cerr << "FBO has incomplete attachment" << std::endl;
		return true;
	case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
		std::cerr << "FBO is missig attachment" << std::endl;
		return true;
	case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
		std::cerr << "FBO has incomplete drawbuffer" << std::endl;
		return true;
	case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
		std::cerr << "FBO has incomplete readbuffer" << std::endl;
		return true;
	case GL_FRAMEBUFFER_UNSUPPORTED:
		std::cerr << "FBO not supported" << std::endl;
		return false;
	case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
		std::cerr << "FBO sample numbers do not match" << std::endl;
		return true;
	case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
		std::cerr << "FBO has incomplete layer targets" << std::endl;
		return true;
	}

	std::cerr << "FBO status unknown" << std::endl;
	return true;
}

void hydraNI::logGLCaps()
{
	int temp;

	std::cout << "----- OPENGL CAPABILITIES -----" << std::endl;

	glGetIntegerv( GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &temp );
	std::cout << "max vertex shader textures: " << temp << std::endl;

	glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS, &temp );
	std::cout << "max fragment shader textures: " << temp << std::endl;

	glGetIntegerv( GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &temp );
	std::cout << "max fragment shader textures: " << temp << std::endl;

	glGetIntegerv( GL_MAX_DRAW_BUFFERS, &temp );
	std::cout << "max draw buffers: " << temp << std::endl;

	std::cout << "-------------------------------" << std::endl;
}

void hydraNI::drawAxes( double scale, double colorOffset )
{
	glPushAttrib( GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT );
	{
		glDisable( GL_LIGHTING );
		glDisable( GL_TEXTURE_2D );
		glEnable( GL_COLOR_MATERIAL );

		glBegin( GL_LINES );
		{
			glColor3f( 1.0, colorOffset, colorOffset );
			glVertex3d( 0.0, 0.0, 0.0 );
			glVertex3d( scale, 0.0, 0.0 );

			glColor3f( colorOffset, 1.0, colorOffset );
			glVertex3d( 0.0, 0.0, 0.0 );
			glVertex3d( 0.0, scale, 0.0 );

			glColor3f( colorOffset, colorOffset, 1.0 );
			glVertex3d( 0.0, 0.0, 0.0 );
			glVertex3d( 0.0, 0.0, scale );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawLocator( const float *color, double scale )
{
	glPushAttrib( GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT );
	{
		glDisable( GL_LIGHTING );
		glDisable( GL_TEXTURE_2D );
		glEnable( GL_COLOR_MATERIAL );

		scale /= 2.0;

		if( color )
			glColor3fv( color );

		glBegin( GL_LINES );
		{
			glVertex3d( -scale, 0.0, 0.0 );
			glVertex3d( scale, 0.0, 0.0 );

			glVertex3d( 0.0, -scale, 0.0 );
			glVertex3d( 0.0, scale, 0.0 );

			glVertex3d( 0.0, 0.0, -scale );
			glVertex3d( 0.0, 0.0, scale );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawRect( float x0, float y0, float x1, float y1 )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( x0, y0, 0.0 );
			glVertex3f( x0, y1, 0.0 );
			glVertex3f( x1, y1, 0.0 );
			glVertex3f( x1, y0, 0.0 );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawCameraCone( double sensorWidth, double sensorHeight, double focalLength, double clipNear, double clipFar, const float *color )
{
	hydraNI::drawCameraCone( sensorWidth, sensorHeight, focalLength, clipNear, clipFar, color, sensorWidth * 0.5, sensorHeight * 0.5 );
}

void hydraNI::drawCameraCone( double sensorWidth, double sensorHeight, double focalLength, double clipNear, double clipFar, const float *color, float opticalCenterX, float opticalCenterY )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		float dx = sensorWidth * 0.5f;
		float dy = sensorHeight * 0.5f;

		float dxNear = clipNear * dx / focalLength;
		float dyNear = clipNear * dy / focalLength;
		float dxFar = clipFar * dx / focalLength;
		float dyFar = clipFar * dy / focalLength;

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glVertex3f( -dxFar, dyFar, -clipFar );
			glVertex3f( -dxFar, -dyFar, -clipFar );
		}
		glEnd();

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glVertex3f( dxFar, dyFar, -clipFar );
			glVertex3f( dxFar, -dyFar, -clipFar );
		}
		glEnd();

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( -dxNear, dyNear, -clipNear );
			glVertex3f( -dxNear, -dyNear, -clipNear );
			glVertex3f( dxNear, -dyNear, -clipNear );
			glVertex3f( dxNear, dyNear, -clipNear );
		}
		glEnd();

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( -dxFar, dyFar, -clipFar );
			glVertex3f( -dxFar, -dyFar, -clipFar );
			glVertex3f( dxFar, -dyFar, -clipFar );
			glVertex3f( dxFar, dyFar, -clipFar );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawCameraFrameBuffer( double sensorWidth, double sensorHeight, double focalLength, double coneDepth, const float *color, unsigned int textureID, float *texcoords )
{
	hydraNI::drawCameraFrameBuffer( sensorWidth, sensorHeight, focalLength, coneDepth, color, textureID, texcoords, sensorWidth * 0.5, sensorHeight * 0.5 );
}

void hydraNI::drawCameraFrameBuffer( double sensorWidth, double sensorHeight, double focalLength, double coneDepth, const float *color, unsigned int textureID, float *texcoords, float opticalCenterX, float opticalCenterY )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		float dx0 = coneDepth * ( -opticalCenterX ) / focalLength;
		float dx1 = coneDepth * ( -opticalCenterX + sensorWidth ) / focalLength;
		float dy0 = coneDepth * ( opticalCenterY ) / focalLength;
		float dy1 = coneDepth * ( opticalCenterY - sensorHeight ) / focalLength;

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glVertex3f( dx0, dy0, -coneDepth );
			glVertex3f( dx0, dy1, -coneDepth );
		}
		glEnd();

		glBegin( GL_LINE_LOOP );
		{
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glVertex3f( dx1, dy0, -coneDepth );
			glVertex3f( dx1, dy1, -coneDepth );
		}
		glEnd();

		glEnable( GL_TEXTURE_2D );
		glBindTexture( GL_TEXTURE_2D, textureID );
		glColor3f( 1.0f, 1.0f, 1.0f );

		glBegin( GL_QUADS );
		{
			glTexCoord2f( texcoords[4], texcoords[5] );
			glVertex3f( dx0, dy0, -coneDepth );

			glTexCoord2f( texcoords[6], texcoords[7] );
			glVertex3f( dx0, dy1, -coneDepth );

			glTexCoord2f( texcoords[0], texcoords[1] );
			glVertex3f( dx1, dy1, -coneDepth );

			glTexCoord2f( texcoords[2], texcoords[3] );
			glVertex3f( dx1, dy0, -coneDepth );
		}
		glEnd();

		glDisable( GL_TEXTURE_2D );
	}
	glPopAttrib();
}

void hydraNI::drawQuad( float x, float y, float z, float width, float height, const float *color, bool filled )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		if( filled )
			glBegin( GL_QUADS );
		else
			glBegin( GL_LINE_STRIP );
		{
			glVertex3f( x, y, z );
			glVertex3f( x, y + height, z );
			glVertex3f( x + width, y + height, z );
			glVertex3f( x + width, y, z );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawCircle( float x, float y, float z, float r, const float *color, bool filled, int sections )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		float a = 0.0f;

		if( filled )
		{
			glBegin( GL_TRIANGLE_FAN );
			glVertex3f( x, y, z );
		}
		else
			glBegin( GL_LINE_STRIP );
		for( int i = 0; i <= sections; i++ )
		{
			a = PI() * 2 / sections * i;
			glVertex3f( x + cos( a ) * r, y - sin( a ) * r, z );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawTri( float x, float y, float z, float r, float angle, const float *color, bool filled )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		float a = 0.0f;

		if( filled )
			glBegin( GL_TRIANGLES );
		else
			glBegin( GL_LINE_STRIP );
		for( int i = 0; i < 3; i++ )
		{
			a = angle + PI() * 2 / 3 * i;
			glVertex3f( x + cos( a ) * r, y - sin( a ) * r, z );
		}
		glEnd();
	}
	glPopAttrib();
}

void hydraNI::drawGrid( const float *color, float size, unsigned int subdivisions )
{
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	{
		glEnable( GL_COLOR_MATERIAL );
		glDisable( GL_TEXTURE_2D );

		if( color )
			glColor3fv( color );

		glBegin( GL_LINES );
		{
			float halfSize = size * 0.5f;
			float d = size / ( subdivisions + 1 );

			float x = -halfSize;
			float y = -halfSize;

			for( int i = 0; i <= subdivisions + 1; i++ )
			{
				glVertex3f( x, -halfSize, 0.0 );
				glVertex3f( x, halfSize, 0.0 );

				glVertex3f( -halfSize, y, 0.0 );
				glVertex3f( halfSize, y, 0.0 );

				x += d;
				y += d;
			}
		}
		glEnd();
	}
	glPopAttrib();
}