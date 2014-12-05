#pragma once

#include <opencv/cv.h>

#include "../HydraNILib_Common/Common.h"
#include "../HydraNILib_Common/CommonNI.h"

#include <vector>

namespace hydraNI
{
	class NUICamera;

	class FrameProcessor
	{
	protected:
		bool isEnabled;

		const IplImage	*colorImage;

		IplImage	*outImage;

		char		desc[256];

		void checkSize( unsigned int width, unsigned int height );

		FrameProcessor( const FrameProcessor& );	//prevent from cloning

		virtual void release()	=	0;

	public:
		FrameProcessor();
		virtual ~FrameProcessor();

		bool getEnabled() const				{	return this->isEnabled;			}
		void setEnabled( bool enabled )		{	this->isEnabled = enabled;		}

		const IplImage *getImage()			{	return this->outImage;			}

		virtual const IplImage *process( const IplImage *image, const IplImage *colorImage = NULL )	=	0;
		virtual void resize( unsigned int width, unsigned int height )								=	0;

		virtual void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB = false )	=	0;

		virtual const char *getName() const		=	0;
		virtual const char *getDesc()		{	return this->desc;		}
	};

	class CameraSource : public FrameProcessor
	{
	private:
		const NUICamera	*nuiCamera;

		IplImage	*colorSource;

		virtual void release();

	public:
		explicit CameraSource( const NUICamera *nuiCamera );
		virtual ~CameraSource();

		virtual const IplImage *process( const IplImage *image, const IplImage *colorImage = NULL );
		virtual void resize( unsigned int width, unsigned int height );
		virtual void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB = false );

		const NUICamera *getNUICamera() const				{	return this->nuiCamera;			}
		void setNUICamera( const NUICamera *camera )		{	this->nuiCamera = camera;		}

		IplImage *getColorImage();

		virtual const char *getName() const		{	return "CameraSource";	}
	};

	class BackgroundSubtracter : public FrameProcessor
	{
	private:
		bool		doRecordBackground;
		bool		showBgAvrg;

		bool            doResetNoDataAreas;
		unsigned short  resetNoDataThreshold;

		double		    tolerance;

		double		avrgSpeed;

		IplImage	*bgAvrg;
		IplImage	*bgImage;
		IplImage	*mask;
		IplImage	*noDataMask;

		virtual void release();

	public:
		BackgroundSubtracter( const std::string &fileName, double tolerance );
		virtual ~BackgroundSubtracter();

		virtual const IplImage *process( const IplImage *image, const IplImage *colorImage = NULL );
		virtual void resize( unsigned int width, unsigned int height );
		virtual void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB = false );

		double getTolerance() const				{	return this->tolerance;				}
		void setTolerance( double tolerance )	{	this->tolerance = tolerance;		}

		double getAvrgSpeed() const				{	return this->avrgSpeed;				}
		void setAvrgSpeed( double speed )		{	this->avrgSpeed = speed;			}

		bool recordBackground()					{	this->doRecordBackground = true;	return true;	}
		void resetBackgroundAvrg();

		bool getShowBackgroundAvrg() const		{	return this->showBgAvrg;	}
		void setShowBackgroundAvrg( bool show )	{	this->showBgAvrg = show;	}

		bool getResetNoDataAreas() const        {   return this->doResetNoDataAreas;    }
		void setResetNoDataAreas( bool doReset) {   this->doResetNoDataAreas = doReset; }

		unsigned short getResetNoDataThreshold() const       {   return this->resetNoDataThreshold;   }
		void setResetNoDataTheshold( unsigned short thresh ) {   this->resetNoDataThreshold = thresh; }

		bool loadBackground( const std::string &filename );
		bool saveBackground( const std::string &filename ) const;

		virtual const char *getName() const		{	return "BackgroundSubtracter";	}
	};

	class MorphologicCloser : public FrameProcessor
	{
	private:
		unsigned int	closingIterations;

		bool			skip;
		float			depthDifference;

		IplImage		*mask;
		IplImage		*maskOld;
		IplImage		*diffMask;

		virtual void release();

	public:
		explicit MorphologicCloser( unsigned int closingIterations );
		virtual ~MorphologicCloser();

		virtual const IplImage *process( const IplImage *image, const IplImage *colorImage = NULL );
		virtual void resize( unsigned int width, unsigned int height );
		virtual void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB = false );

		unsigned int getClosingIterations() const		{	return this->closingIterations;		}
		void setClosingIterations( unsigned int i )		{	this->closingIterations = i;		}

		float getDepthDifference() const				{	return this->depthDifference;		}

		virtual const char *getName() const				{	return "MorphologicCloser";	}
	};

	class Reprojector : public FrameProcessor
	{
	private:
		unsigned int	closingIterations;

		unsigned int	widthProjector;
		unsigned int	heightProjector;

		double			focalLengthProjector;
		double			cxProjector;
		double			cyProjector;

		double			focalLengthCamera;
		double			cxCamera;
		double			cyCamera;

		glm::mat4		camToProj;
		glm::mat4		camToWorld;

		VertexList		camVertices;
		VertexList		projVertices;

		ColorList		camColors;
		ColorList		projColors;

		//------------------------------------------------------------------------------------
		//TODO: for performance and code architecture considerations i have to do this here
		// although it would be better to have this seperately somewhere -- it does not really
		// fit in this class since it serves a different purpose. maybe change this some time.
		CvScalar	clipNear;		///< @brief clip far in millimeters
		CvScalar	clipFar;		///< @brief clip far in millimeters

		float		clipNearPlaneRotationX;
		float		clipNearPlaneRotationY;
		float		clipFarPlaneRotationX;
		float		clipFarPlaneRotationY;

		float		clipRotationXMax;
		float		clipRotationYMax;

		glm::vec3	clipNearPlaneNormal;
		glm::vec3	clipFarPlaneNormal;

		float		cylinderClippingCenterX;
		float		cylinderClippingCenterZ;
		float		cylinderClippingRadius;

		bool		doFrustumClipping;
		bool		doCylinderClipping;
		//------------------------------------------------------------------------------------

		virtual void release();

		void calcPointCloud( const IplImage *image );
		void transformCloud();
		void projectCloud( IplImage *image );

	public:
		Reprojector( const glm::mat4 &camToProj, const glm::mat4 &camToWorld, unsigned int closingIterations, unsigned int widthProjector, unsigned int heightProjector, double focalLengthProjector, double cxProjector, double cyProjector, double focalLengthCamera, double cxCamera, double cyCamera, double clipNear, double clipFar );
		virtual ~Reprojector();

		virtual const IplImage *process( const IplImage *image, const IplImage *colorImage = NULL );
		virtual void resize( unsigned int width, unsigned int height );
		virtual void getDebugView( unsigned char *rgbData, size_t stride, bool switchRB = false );

		unsigned int getClosingIterations() const		{	return this->closingIterations;		}
		void setClosingIterations( unsigned int i )		{	this->closingIterations = i;		}

		unsigned int getWidthProjector() const			{	return this->widthProjector;		}
		unsigned int getHeightProjector() const			{	return this->heightProjector;		}

		double getFocalLengthProjector() const			{	return this->focalLengthProjector;	}
		void setFocalLengthProjector( double f ) 		{	this->focalLengthProjector = f;		}

		double getCenterXProjector() const				{	return this->cxProjector;			}
		void setCenterXProjector( double cx )	 		{	this->cxProjector = cx;				}

		double getCenterYProjector() const				{	return this->cyProjector;			}
		void setCenterYProjector( double cy )	 		{	this->cyProjector = cy;				}

		double getFocalLengthCamera() const				{	return this->focalLengthCamera;		}
		void setFocalLengthCamera( double f )			{	this->focalLengthCamera = f;		}

		double getCenterXCamera() const					{	return this->cxCamera;				}
		void setCenterXCamera( double cx )	 			{	this->cxCamera = cx;				}

		double getCenterYCamera() const					{	return this->cyCamera;				}
		void setCenterYCamera( double cy )	 			{	this->cyCamera = cy;				}

		void setCamToProjector( const glm::mat4 &camToProj )	{	this->camToProj = camToProj;	}
		const glm::mat4 &getCamToProjector() const				{	return this->camToProj;			}

		void setCamToWorld( const glm::mat4 &camToWorld )	{	this->camToWorld = camToWorld;	}
		const glm::mat4 &getCamToWorld() const				{	return this->camToWorld;		}

		const VertexList &getCamVertices() const		{	return this->camVertices;			}
		const VertexList &getPrjVertices() const		{	return this->projVertices;			}

		const ColorList &getCamColors() const			{	return this->camColors;				}
		const ColorList &getPrjColors() const			{	return this->projColors;			}

		//------------------------------------------------------------------------------------
		//TODO: for performance and code architecture considerations i have to do this here
		// although it would be better to have this seperately somewhere -- it does not really
		// fit in this class since it serves a different purpose maybe change this some time.
		void setClipNear( double clipNear )				{	this->clipNear = cvScalar( hydraNI::max<double>( 0.0, clipNear ) );		}
		double getClipNear() const						{	return this->clipNear.val[0];			}

		void setClipFar( double clipFar )				{	this->clipFar = cvScalar( hydraNI::max<double>( 0.0f, clipFar ) );		}
		double getClipFar() const						{	return this->clipFar.val[0];			}

		float getClipNearPlaneRotationX() const			{	return this->clipNearPlaneRotationX;	}
		void setClipNearPlaneRotationX( float rot );

		float getClipNearPlaneRotationY() const			{	return this->clipNearPlaneRotationY;	}
		void setClipNearPlaneRotationY( float rot );

		float getClipFarPlaneRotationX() const			{	return this->clipFarPlaneRotationX;		}
		void setClipFarPlaneRotationX( float rot );

		float getClipFarPlaneRotationY() const			{	return this->clipFarPlaneRotationY;		}
		void setClipFarPlaneRotationY( float rot );

		const glm::vec3 &getClipNearPlaneNormal() const	{	return this->clipNearPlaneNormal;		}
		const glm::vec3 &getClipFarPlaneNormal() const	{	return this->clipFarPlaneNormal;		}

		void setCylinderClippingCenterX( double x )		{	this->cylinderClippingCenterX = x;		}
		double getCylinderClippingCenterX() const		{	return this->cylinderClippingCenterX;	}

		void setCylinderClippingCenterZ( double z )		{	this->cylinderClippingCenterZ = z;		}
		double getCylinderClippingCenterZ() const		{	return this->cylinderClippingCenterZ;	}

		void setCylinderClippingRadius( double r )		{	this->cylinderClippingRadius = r;		}
		double getCylinderClippingRadius() const		{	return this->cylinderClippingRadius;	}

		bool getFrustumClippingOn() const				{	return this->doFrustumClipping;		}
		void setFrustumClippingOn( bool b )				{	this->doFrustumClipping = b;		}

		bool getCylinderClippingOn() const				{	return this->doCylinderClipping;	}
		void setCylinderClippingOn( bool b )			{	this->doCylinderClipping = b;		}
		//------------------------------------------------------------------------------------

		virtual const char *getName() const				{	return "Reprojector";	}
	};

	class FrameProcessorChain
	{
	private:
		typedef	std::vector<FrameProcessor*>	FrameProcessorList;

		bool	sendColor;

		FrameProcessorList	chain;

		FrameProcessorChain( const FrameProcessorChain& );	//prevent from cloning

	public:
		FrameProcessorChain();
		~FrameProcessorChain();

		void add( FrameProcessor *processor );	// FrameProcessorChain takes ownership!!
		FrameProcessor *getLink( unsigned int index );

		template<class T>
		T *getLink( unsigned int index = 0 )
		{
			T *t = NULL;
			for( FrameProcessorList::iterator it = chain.begin(); it != chain.end(); ++it )
			{
				t = dynamic_cast<T*>( *it );
				if( t )
				{
					if( !index )
						return t;

					index--;
				}
			}

			return NULL;
		}

		void setSendColor( bool sendColor )			{	this->sendColor = sendColor;	}
		bool getSendColor() const					{	return this->sendColor;			}

		void resize( unsigned int width, unsigned int height );
		void update();

		unsigned int getLinkCount() const		{	return this->chain.size();		}
	};
}
