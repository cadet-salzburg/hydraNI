#include "CameraTransform.h"

#include "../HydraNILib_Common/Common.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

using namespace hydraNI;

CameraTransform::CameraTransform() :
	resX( 1 ),
	resY( 1 ),
	centerX( 0 ),
	centerY( 0 ),
	pxSizeX( 1.0 ),
	pxSizeY( 1.0 ),
	sx( 1.0 ),
	sy( 1.0 ),
	skew( 0.0 ),
	focalLength( 1 ),
	aspectRatio( 1 ),
	glExtrinsics( 1.0f ),
	glExtrinsicsInv( 1.0f ),
	glIntrinsics( 1.0f )
{
	this->updateIntrinsics();
}

CameraTransform::CameraTransform( unsigned int resX, unsigned int resY, double cx, double cy, double psX, double psY, double skew, double f, double ar ) :
	resX( resX ),
	resY( resY ),
	centerX( cx ),
	centerY( cy ),
	pxSizeX( psX ),
	pxSizeY( psY ),
	sx( 1.0f / psX ),
	sy( 1.0f / psY ),
	skew( skew ),
	focalLength( f ),
	aspectRatio( ar ),
	glExtrinsics( 1.0f ),
	glExtrinsicsInv( 1.0f ),
	glIntrinsics( 1.0f )
{
	this->updateIntrinsics();
}

CameraTransform::~CameraTransform()
{}

void CameraTransform::updateIntrinsics()
{
	this->glIntrinsics = glm::mat3( 1.0f );

	this->glIntrinsics[0][0] = this->focalLength * this->sx;
	this->glIntrinsics[1][0] = this->skew;
	this->glIntrinsics[1][1] = this->focalLength * this->sy;
	this->glIntrinsics[2][0] = this->centerX;
	this->glIntrinsics[2][1] = this->centerY;
}

glm::vec3 CameraTransform::getWorldPosition()
{
	return glm::vec3(
		this->glExtrinsics[3][0],
		this->glExtrinsics[3][1],
		this->glExtrinsics[3][2] );
}

void CameraTransform::multLeft( const glm::mat4 &m )
{
	this->glExtrinsics = m * this->glExtrinsics;

	glm::mat3 invRot( glm::transpose( m ) );
	glm::vec3 invTrans( -m[3] );

	glm::mat4 inv( invRot );
	inv[3] = glm::vec4( invTrans, 1.0 );

	this->glExtrinsicsInv = this->glExtrinsicsInv * inv;
}

void CameraTransform::multRight( const glm::mat4 &m )
{
	this->glExtrinsics = this->glExtrinsics * m;

	glm::mat3 invRot( glm::transpose( m ) );
	glm::vec3 invTrans( -m[3] );

	glm::mat4 inv( invRot );
	inv[3] = glm::vec4( invTrans, 1.0 );

	this->glExtrinsicsInv = inv * this->glExtrinsicsInv;
}

void CameraTransform::transLeft( const glm::vec3 &t )
{
	glm::mat4 m( glm::translate( t ) );
	glm::mat4 mInv( glm::translate( -t ) );

	this->glExtrinsics = m * this->glExtrinsics;
	this->glExtrinsicsInv = this->glExtrinsicsInv * mInv;
}

void CameraTransform::transRight( const glm::vec3 &t )
{
	glm::mat4 m( glm::translate( t ) );
	glm::mat4 mInv( glm::translate( -t ) );

	this->glExtrinsics = this->glExtrinsics * m;
	this->glExtrinsicsInv = mInv * this->glExtrinsicsInv;
}

void CameraTransform::rotLeft( const glm::vec3 &r )
{
	glm::mat4 m( glm::rotate( r[2], unitZ() ) );
	m = glm::rotate( m, r[1], unitY() );
	m = glm::rotate( m, r[0], unitX() );

	glm::mat4 mInv( glm::rotate( -r[0], unitX() ) );
	mInv = glm::rotate( mInv, -r[1], unitY() );
	mInv = glm::rotate( mInv, -r[2], unitZ() );

	this->glExtrinsics = m * this->glExtrinsics;
	this->glExtrinsicsInv = this->glExtrinsicsInv * mInv;
}

void CameraTransform::rotRight( const glm::vec3 &r )
{
	glm::mat4 m( glm::rotate( r[2], unitZ() ) );
	m = glm::rotate( m, r[1], unitY() );
	m = glm::rotate( m, r[0], unitX() );

	glm::mat4 mInv( glm::rotate( -r[0], unitX() ) );
	mInv = glm::rotate( mInv, -r[1], unitY() );
	mInv = glm::rotate( mInv, -r[2], unitZ() );

	this->glExtrinsics = this->glExtrinsics * m;
	this->glExtrinsicsInv = mInv * this->glExtrinsicsInv;
}
