#pragma once

#include <vector>
#include <string>
#include <fstream>

#include <XnTypes.h>
#include <XnOpenNI.h>
#include <XnCppWrapper.h>


namespace hydraNI
{
	struct JointData
	{
		bool				valid;
		XnSkeletonJoint		joint;
		XnSkeletonJointPosition	position;
	};

	struct SkeletonData
	{
		XnUserID	userID;

		JointData	joints[24];
	};

	void XN_CALLBACK_TYPE NewUser( xn::UserGenerator &generator, XnUserID user, void *pCookie );
	void XN_CALLBACK_TYPE LostUser( xn::UserGenerator &generator, XnUserID user, void *pCookie );
	void XN_CALLBACK_TYPE UserExit( xn::UserGenerator &generator, XnUserID user, void *pCookie );
	void XN_CALLBACK_TYPE UserReEnter( xn::UserGenerator &generator, XnUserID user, void *pCookie );

	class NUICamera
	{
		friend void XN_CALLBACK_TYPE hydraNI::NewUser( xn::UserGenerator &generator, XnUserID user, void *pCookie );
		friend void XN_CALLBACK_TYPE hydraNI::LostUser( xn::UserGenerator &generator, XnUserID user, void *pCookie );
		friend void XN_CALLBACK_TYPE hydraNI::UserExit( xn::UserGenerator &generator, XnUserID user, void *pCookie );
		friend void XN_CALLBACK_TYPE hydraNI::UserReEnter( xn::UserGenerator &generator, XnUserID user, void *pCookie );

	private:
		std::string	name;
		std::string serial;


		bool	online;

		double	timeSinceLastFrame;
		double	offlineTimeOut;

		xn::IRGenerator		irGenerator;
		xn::ImageGenerator	imageGenerator;
		xn::DepthGenerator	depthGenerator;
		xn::UserGenerator	userGenerator;

		xn::IRMetaData		irMetaData;
		xn::ImageMetaData	imageMetaData;
		xn::DepthMetaData	depthMetaData;
		std::vector<SkeletonData>	skeletonData;

		unsigned int	irFrameNr;
		unsigned int	depthFrameNr;
		unsigned int	colorFrameNr;
		unsigned int	skeletonFrameNr;

		unsigned int	framesIR;
		unsigned int	framesDepth;
		unsigned int	framesColor;
		unsigned int	framesSkeleton;

		double	fpsIR;
		double	fpsDepth;
		double	fpsColor;
		double	fpsSkeleton;

		double	timeAccu;


		void NewUser( XnUserID user );
		void LostUser( XnUserID user );
		void UserExit( XnUserID user );
		void UserReEnter( XnUserID user );

		NUICamera( const NUICamera& );	//prevent from cloning

	public:
		NUICamera( const std::string &name, const std::string &serial, const xn::IRGenerator &irGen, const xn::ImageGenerator &imageGen, const xn::DepthGenerator &depthGen, const xn::UserGenerator &userGen );
		virtual ~NUICamera();

		const std::string &getName() const		{ return this->name; }
		const std::string &getSerial() const	{ return this->serial; }

		double getIRFPS() const					{ return this->fpsIR; }
		double getDepthFPS() const				{ return this->fpsDepth; }
		double getColorFPS() const				{ return this->fpsColor; }
		double getSkeletonFPS() const			{ return this->fpsSkeleton; }

		unsigned int getIRFrameNr() const		{ return this->irFrameNr; }
		unsigned int getDepthFrameNr() const	{ return this->depthFrameNr; }
		unsigned int getColorFrameNr() const	{ return this->colorFrameNr; }
		unsigned int getSkeletonFrameNr() const	{ return this->skeletonFrameNr; }

		virtual bool isOnline() const			{ return this->online; }
		virtual void resetOnlineTimeout()       { this->timeSinceLastFrame = 0.0f; }

		virtual double getOfflineTimeOut() const			{ return this->offlineTimeOut; }
		virtual void setOfflineTimeOut( double timeOut )	{ this->offlineTimeOut = timeOut; }

		virtual void start();
		virtual bool update( double dt );

		virtual bool hasIR() const						{ return this->irGenerator.IsValid(); }
		virtual bool hasDepth() const					{ return this->depthGenerator.IsValid(); }
		virtual bool hasColor() const					{ return this->imageGenerator.IsValid(); }
		virtual bool hasSkeleton() const				{ return this->userGenerator.IsValid() && this->userGenerator.IsCapabilitySupported( XN_CAPABILITY_SKELETON ); }

		virtual bool isIRDataNew() const				{ return ( this->hasIR() && this->irGenerator.IsDataNew() ); }
		virtual bool isDepthDataNew() const				{ return ( this->hasDepth() && this->depthGenerator.IsDataNew() ); }
		virtual bool isColorDataNew() const				{ return ( this->hasColor() && this->imageGenerator.IsDataNew() ); }
		virtual bool isSkeletonDataNew() const			{ return ( this->hasSkeleton() && this->userGenerator.IsDataNew() ); }

		virtual bool getColorRes( XnUInt16 &xRes, XnUInt16 &yRes ) const;
		virtual const XnRGB24Pixel *getColorData() const;

		virtual bool getIRRes( XnUInt16 &xRes, XnUInt16 &yRes ) const;
		virtual const XnIRPixel *getIRData() const;

		virtual bool getDepthRes( XnUInt16 &xRes, XnUInt16 &yRes ) const;
		virtual const XnDepthPixel *getDepthData() const;

		virtual bool getSkeletonRes( XnUInt16 &currentUsers, XnUInt16 &jointMax ) const;
		virtual const SkeletonData * getSkeletonData();

		virtual bool setViewpointColor();
		virtual bool setViewpointIR();

		virtual void resetViewpoints();

		virtual bool getIRIntrinsics( float &f, float &sx, float &sy ) const;
		virtual bool getDepthIntrinsics( float &f, float &sx, float &sy ) const;
		virtual bool getColorIntrinsics( float &f, float &sx, float &sy ) const;
	};

}
