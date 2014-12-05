#include "CommonNI.h"

#include <XnCppWrapper.h>


void hydraNI::transform2DTo3D( size_t size, const XnVector3D *in, XnVector3D *out, unsigned int cx, unsigned int cy, float f, bool switchHandedness )
{
	float s = 0.001f / f;
	float a = -( cx * s );
	float b = -( cy * s );
	float zs = 0.001 * ( switchHandedness?-1.0f:1.0f );

	while( size-- )
	{
		out->X = -( in->X * s + a ) * in->Z;
		out->Y = ( in->Y * s + b ) * in->Z;
		out->Z = in->Z * zs;

		in++;
		out++;
	}
}

void hydraNI::transform3DTo2D( size_t size, const XnVector3D *in, XnVector3D *out, unsigned int cx, unsigned int cy, float f, bool switchHandedness )
{
	float a = cx;
	float b = cy;
	float zs = 1000.0f * ( switchHandedness?-1.0f:1.0f );

	while( size-- )
	{
		out->Z = in->Z * zs;

		float s = 1000.0f * f / out->Z;

		out->X = ( -in->X * s + a );
		out->Y = in->Y * s + b;

		in++;
		out++;
	}
}
