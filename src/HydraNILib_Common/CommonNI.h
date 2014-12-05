#pragma once

#include <vector>

#include <glm/glm.hpp>
#include <XnTypes.h>


#define NI_CAMERA_X_RES	640
#define NI_CAMERA_Y_RES	480
#define NI_CAMERA_PIXELS	( KINECT_X_RES * KINECT_Y_RES )
#define NI_CAMERA_FPS		30

#define NI_CAMERA_OFFLINE_TIMEOUT_DEFAULT	10.0

#define NI_CAMERA_NO_VALUE	0
#define NI_CAMERA_MAX_DEPTH	10000


namespace hydraNI
{
	typedef std::vector<XnVector3D>	VertexList;
	typedef std::vector<XnVector3D>	ColorList;


	inline glm::vec3 xnToVec3( const XnPoint3D &xn )
	{
		return glm::vec3( xn.X, xn.Y, xn.Z );
	}
	
	void transform2DTo3D( size_t size, const XnVector3D *in, XnVector3D *out, unsigned int cx, unsigned int cy, float f, bool switchHandedness );
	void transform3DTo2D( size_t size, const XnVector3D *in, XnVector3D *out, unsigned int cx, unsigned int cy, float f, bool switchHandedness );
}
