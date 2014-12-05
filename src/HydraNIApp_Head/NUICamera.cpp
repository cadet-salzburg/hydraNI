#include "NUICamera.h"

#include "../HydraNILib_Common/Common.h"
#include "../HydraNILib_Common/CommonNI.h"

#include <sstream>
#include <iostream>

#include <XnPropNames.h>

#include <boost/filesystem.hpp>

using namespace hydraNI;


namespace hydraNI
{
	void XN_CALLBACK_TYPE NewUser( xn::UserGenerator& generator, XnUserID user, void* pCookie )
	{
		reinterpret_cast<NUICamera*>( pCookie )->NewUser( user );
	}

	void XN_CALLBACK_TYPE LostUser( xn::UserGenerator& generator, XnUserID user, void* pCookie )
	{
		reinterpret_cast<NUICamera*>( pCookie )->LostUser( user );
	}

	void XN_CALLBACK_TYPE UserExit( xn::UserGenerator& generator, XnUserID user, void* pCookie )
	{
		reinterpret_cast<NUICamera*>( pCookie )->UserExit( user );
	}

	void XN_CALLBACK_TYPE UserReEnter( xn::UserGenerator& generator, XnUserID user, void* pCookie )
	{
		reinterpret_cast<NUICamera*>( pCookie )->UserReEnter( user );
	}
}

NUICamera::NUICamera( const std::string &name, const std::string &serial, const xn::IRGenerator &irGen, const xn::ImageGenerator &imageGen, const xn::DepthGenerator &depthGen, const xn::UserGenerator &userGen ) :
	name( name ),
	serial( serial ),
	online( true ),
	timeSinceLastFrame( 0.0 ),
	offlineTimeOut( NI_CAMERA_OFFLINE_TIMEOUT_DEFAULT ),
	irGenerator( irGen ),
	imageGenerator( imageGen ),
	depthGenerator( depthGen ),
	userGenerator( userGen ),
	irFrameNr( 0 ), depthFrameNr( 0 ), colorFrameNr( 0 ), skeletonFrameNr( 0 ),
	framesIR( 0 ), framesDepth( 0 ), framesColor( 0 ), framesSkeleton( 0 ),
	fpsIR( 0.0 ), fpsDepth( 0.0 ), fpsColor( 0.0 ), fpsSkeleton( 0.0 ),
	timeAccu( 0.0 )
{
	if( this->irGenerator.IsValid() )
		this->irGenerator.GetMetaData( this->irMetaData );
	if( this->depthGenerator.IsValid() )
		this->depthGenerator.GetMetaData( this->depthMetaData );
	if( this->imageGenerator.IsValid() )
		this->imageGenerator.GetMetaData( this->imageMetaData );

	if( this->userGenerator.IsValid() )
	{
		XnCallbackHandle h;
		XnCallbackHandle hCalibStart, hCalibComplete;
		XnStatus s = XN_STATUS_OK;
		if( ( s = this->userGenerator.RegisterUserCallbacks( hydraNI::NewUser, hydraNI::LostUser, this, h ) ) != XN_STATUS_OK )
			std::cerr << "registering user callbacks failed for camera " << this->getName() << std::endl;
		if( ( s = this->userGenerator.RegisterToUserExit( hydraNI::UserExit, this, h ) ) != XN_STATUS_OK )
			std::cerr << "registering user exit callback failed for camera " << this->getName() << std::endl;
		if( ( s = this->userGenerator.RegisterToUserReEnter( hydraNI::UserReEnter, this, hCalibStart ) ) != XN_STATUS_OK )
			std::cerr << "registering user reenter callback failed for camera " << this->getName() << std::endl;

		if( ( s = this->userGenerator.GetSkeletonCap().SetSkeletonProfile( XN_SKEL_PROFILE_ALL ) ) != XN_STATUS_OK )
			std::cerr << "setting skeletonprofile failed for camera " << this->getName() << std::endl;
	}
}

NUICamera::~NUICamera()
{
	this->userGenerator.Release();
	this->depthGenerator.Release();
	this->imageGenerator.Release();
	this->irGenerator.Release();
}

void NUICamera::start()
{
	this->timeSinceLastFrame = 0.0;

	if( this->imageGenerator.IsValid() )
		this->imageGenerator.StartGenerating();
	if( this->irGenerator.IsValid() )
		this->irGenerator.StartGenerating();
	if( this->depthGenerator.IsValid() )
		this->depthGenerator.StartGenerating();
	if( this->userGenerator.IsValid() )
		this->userGenerator.StartGenerating();
}

bool NUICamera::update( double dt )
{
	this->timeAccu += dt;
	this->timeSinceLastFrame += dt;

	bool didUpdate = false;
	if( this->isIRDataNew() )
	{
		this->framesIR++;
		this->irFrameNr++;
		this->timeSinceLastFrame = 0.0;
	}
	if( this->isDepthDataNew() )
	{
		this->framesDepth++;
		this->depthFrameNr++;
		this->timeSinceLastFrame = 0.0;
	}
	if( this->isColorDataNew() )
	{
		this->framesColor++;
		this->colorFrameNr++;
		this->timeSinceLastFrame = 0.0;
	}
	if( this->isSkeletonDataNew() )
	{
		this->framesSkeleton++;
		this->skeletonFrameNr++;
		this->timeSinceLastFrame = 0.0;
	}

	if( this->timeAccu > 1.0 )
	{
		this->fpsIR = this->framesIR / this->timeAccu;
		this->fpsDepth = this->framesDepth / this->timeAccu;
		this->fpsColor = this->framesColor / this->timeAccu;
		this->fpsSkeleton = this->framesSkeleton / this->timeAccu;
		this->framesIR = 0;
		this->framesDepth = 0;
		this->framesColor = 0;
		this->framesSkeleton = 0;

		this->timeAccu = 0.0;
	}

	this->online = ( this->timeSinceLastFrame < this->offlineTimeOut );

	return (
		( this->isIRDataNew() ) ||
		( this->isDepthDataNew() ) ||
		( this->isColorDataNew() ) ||
		( this->isSkeletonDataNew() ) );
}

bool NUICamera::getColorRes( XnUInt16 &xRes, XnUInt16 &yRes ) const
{
	if( !this->hasColor() )
		return false;

	xn::ImageMetaData imageMD;
	this->imageGenerator.GetMetaData( imageMD );
	xRes = imageMD.XRes();
	yRes = imageMD.YRes();

	return true;
}

const XnRGB24Pixel *NUICamera::getColorData() const
{
	if( !this->hasColor() )
		return NULL;

	xn::ImageMetaData imageMD;
	this->imageGenerator.GetMetaData( imageMD );
	return imageMD.RGB24Data();
}

bool NUICamera::getIRRes( XnUInt16 &xRes, XnUInt16 &yRes ) const
{
	if( !this->hasIR() )
		return false;

	xn::IRMetaData irMD;
	this->irGenerator.GetMetaData( irMD );
	xRes = irMD.XRes();
	yRes = irMD.YRes();

	return true;
}

const XnIRPixel *NUICamera::getIRData() const
{
	if( !this->hasIR() )
		return NULL;

	xn::IRMetaData irMD;
	this->irGenerator.GetMetaData( irMD );
	return irMD.Data();
}

bool NUICamera::getDepthRes( XnUInt16 &xRes, XnUInt16 &yRes ) const
{
	if( !this->hasDepth() )
		return false;

	xn::DepthMetaData depthMD;
	this->depthGenerator.GetMetaData( depthMD );
	xRes = depthMD.XRes();
	yRes = depthMD.YRes();

	return true;
}

const XnDepthPixel *NUICamera::getDepthData() const
{
	if( !this->hasDepth() )
		return NULL;

	xn::DepthMetaData depthMD;
	this->depthGenerator.GetMetaData( depthMD );
	return depthMD.Data();
}

bool NUICamera::getSkeletonRes( XnUInt16 &currentUsers, XnUInt16 &jointMax ) const
{
	if( !this->hasSkeleton() )
		return false;

	currentUsers = this->userGenerator.GetNumberOfUsers();
	jointMax = XN_SKEL_PROFILE_ALL;

	return true;
}

const SkeletonData *NUICamera::getSkeletonData()
{
	XnUInt16 userCount = this->userGenerator.GetNumberOfUsers();

	this->skeletonData.resize( userCount );

	if( userCount )
	{
		std::vector<XnUserID> ids;
		ids.resize( userCount );

		this->userGenerator.GetUsers( &( ids[0] ), userCount );

		xn::SkeletonCapability skelCap = this->userGenerator.GetSkeletonCap();
		for( int i = 0; i < userCount; i++ )
		{
			SkeletonData &sd = this->skeletonData[i];

			if( !skelCap.IsTracking( ids[i] ) )
			{
				std::cerr << "not tracking user " << ids[i] << std::endl;
				skelCap.StartTracking( ids[i] );
			}

			sd.userID = ids[i];
			for( int j = 0; j < 24; j++ )
			{
				sd.joints[j].joint = (XnSkeletonJoint) ( j + 1 );
				sd.joints[j].valid = ( skelCap.GetSkeletonJointPosition( sd.userID, sd.joints[j].joint, sd.joints[j].position ) == XN_STATUS_OK );

				if( sd.joints[j].valid )
				{
					sd.joints[j].position.position.X *= 0.001;
					sd.joints[j].position.position.Y *= 0.001;
					sd.joints[j].position.position.Z *= -0.001;
				}
			}
		}

		return &( this->skeletonData[0] );
	}

	return NULL;
}

bool NUICamera::setViewpointIR()
{
	this->resetViewpoints();

	if( this->imageGenerator.IsValid() && this->imageGenerator.IsCapabilitySupported( XN_CAPABILITY_ALTERNATIVE_VIEW_POINT ) )
	{
		if( this->irGenerator.IsValid() )
		{
			this->imageGenerator.GetAlternativeViewPointCap().SetViewPoint( this->irGenerator );
			return true;
		}
		if( this->depthGenerator.IsValid() )
		{
			this->imageGenerator.GetAlternativeViewPointCap().SetViewPoint( this->depthGenerator );
			return true;
		}
	}
	return false;
}

bool NUICamera::setViewpointColor()
{
	this->resetViewpoints();

	if( this->depthGenerator.IsValid() && this->imageGenerator.IsValid() && this->depthGenerator.IsCapabilitySupported( XN_CAPABILITY_ALTERNATIVE_VIEW_POINT ) )
	{
		this->depthGenerator.GetAlternativeViewPointCap().SetViewPoint( this->imageGenerator );
		return true;
	}
	return false;
}

void NUICamera::resetViewpoints()
{
	if( this->depthGenerator.IsValid() && this->depthGenerator.IsCapabilitySupported( XN_CAPABILITY_ALTERNATIVE_VIEW_POINT ) )
		this->depthGenerator.GetAlternativeViewPointCap().ResetViewPoint();
	if( this->irGenerator.IsValid() && this->irGenerator.IsCapabilitySupported( XN_CAPABILITY_ALTERNATIVE_VIEW_POINT ) )
		this->irGenerator.GetAlternativeViewPointCap().ResetViewPoint();
	if( this->imageGenerator.IsValid() && this->imageGenerator.IsCapabilitySupported( XN_CAPABILITY_ALTERNATIVE_VIEW_POINT ) )
		this->imageGenerator.GetAlternativeViewPointCap().ResetViewPoint();
}

bool NUICamera::getIRIntrinsics( float &f, float &sx, float &sy ) const
{
	throw std::exception( "NUICamera::getIRIntrinsics not implemented" );
}

bool NUICamera::getDepthIntrinsics( float &f, float &sx, float &sy ) const
{
	if( !this->depthGenerator.IsValid() )
		return false;

	XnUInt64 zpd;
	XnDouble zpps;
	XnStatus status = XN_STATUS_OK;
	if( ( status = this->depthGenerator.GetIntProperty( "ZPD", zpd ) ) != XN_STATUS_OK )
	{
		std::cerr << "getting focal length failed: \"" << xnGetStatusString( status ) << "\"" << std::endl;
		return false;
	}
	if( ( status = this->depthGenerator.GetRealProperty( "ZPPS", zpps ) ) != XN_STATUS_OK )
	{
		std::cerr << "getting pixel size failed: \"" << xnGetStatusString( status ) << "\"" << std::endl;
		return false;
	}

	// the returned pixel size is related to CMOS resolution which operates on twice the resolution
	// http://community.openni.org/openni/topics/focal_length_camera_intrinsics
	// this is why ZPD / ZPPS = 1140.6844. double the pixel size and you get the expected 570.34.
	// now looking for a way of getting CMOS-resolution out of OpenNI, so I can calculate this
	// dynamically.
	//f = zpd;
	f = 570.34 * zpps;

	sx = zpps;
	sy = zpps;

	return true;
}

bool NUICamera::getColorIntrinsics( float &f, float &sx, float &sy ) const
{
	throw std::exception( "NUICamera::getColorIntrinsics not implemented" );
}

void NUICamera::NewUser( XnUserID user )
{
	std::cout << "New user identified: " << user << std::endl;
}

void NUICamera::LostUser( XnUserID user )
{
	std::cout << "User " << user << " lost" << std::endl;
}

void NUICamera::UserExit( XnUserID user )
{
	std::cout << "User " << user << " exited" << std::endl;
}

void NUICamera::UserReEnter( XnUserID user )
{
	std::cout << "User " << user << " reentered" << std::endl;
}
