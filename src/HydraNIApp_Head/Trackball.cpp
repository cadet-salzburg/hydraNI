#include "Trackball.h"

#include "../HydraNILib_Common/Common.h"

#ifdef WIN32
#include <Windows.h>
#endif // WIN32

#include <GL/gl.h>

using namespace hydraNI;

Trackball::Trackball() :
	MouseListener( "unnamed" ),
	translationSpeed( 5.0 / 1024 ),
	rotationSpeed( 180.0 ),
	translation( 0.0f ),
	rotation( 0.0f ),
	doRotate( false )
{}

Trackball::Trackball( const std::string &name, bool rotate, bool zScale ) :
	MouseListener( name ),
	translationSpeed( 5.0 / 1024 ),
	rotationSpeed( 180.0 ),
	translation( 0.0f ),
	rotation( 0.0f ),
	doRotate( rotate ),
	zScale( zScale )
{}

Trackball::Trackball( const glm::vec3 &trans, const std::string &name, bool rotate, bool zScale ) :
	MouseListener( name ),
	translationSpeed( 5.0 / 1024 ),
	rotationSpeed( 180.0 ),
	translation( trans ),
	rotation( 0.0f ),
	doRotate( rotate ),
	zScale( zScale )
{}

Trackball::Trackball( const glm::vec3 &trans, const glm::vec3 &rot, const std::string &name, bool rotate, bool zScale ) :
	MouseListener( name ),
	translationSpeed( 5.0 / 1024 ),
	rotationSpeed( 180.0 ),
	translation( trans ),
	rotation( rot ),
	doRotate( rotate ),
	zScale( zScale )
{}

Trackball::~Trackball()
{}

void Trackball::onMouseRel( int dx, int dy, unsigned int mouseMask )
{
	if( !this->getEnabled() )
		return;

	//left
	if( this->doRotate )
	{
		if( mouseMask & 0x01 << 0 )
		{
			this->rotation[1] += (float)dx * this->rotationSpeed;
			this->rotation[0] += (float)dy * this->rotationSpeed;
		}
	}

	//middle
	if( mouseMask & 0x01 << 1 )
	{
		this->translation[2] -= (float)dy * this->translationSpeed;
		if( zScale )
			this->translation[2] = hydraNI::clamp( this->translation[2], 0.001f, 10.0f );
	}

	//right
	if( mouseMask & 0x01 << 2 )
	{
		this->translation[0] += (float)dx * this->translationSpeed;
		this->translation[1] -= (float)dy * this->translationSpeed;
	}
}

void Trackball::apply()
{
	if( zScale )
	{
		float s = hydraNI::clamp( this->translation[2], 0.001f, 10.0f );

		glScalef( s, s, s );

		glTranslatef( this->translation[0], this->translation[1], 0.0f );
	}
	else
		glTranslatef( this->translation[0], this->translation[1], this->translation[2] );

	glRotatef( this->rotation[0], 1.0f, 0.0f, 0.0f );
	glRotatef( this->rotation[1], 0.0f, 1.0f, 0.0f );
	glRotatef( this->rotation[2], 0.0f, 0.0f, 1.0f );
}
