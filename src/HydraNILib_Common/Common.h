#pragma once

#include <time.h>

#include <string>
#include <vector>

#include <opencv2/core/types_c.h>

#include <glm/glm.hpp>


std::ostream &operator << ( std::ostream &out, const glm::vec2 &v );
std::ostream &operator << ( std::ostream &out, const glm::vec3 &v );
std::ostream &operator << ( std::ostream &out, const glm::vec4 &v );

std::ostream &operator << ( std::ostream &out, const glm::mat2 &m );
std::ostream &operator << ( std::ostream &out, const glm::mat3 &m );
std::ostream &operator << ( std::ostream &out, const glm::mat4 &m );

std::ostream &operator << ( std::ostream &out, const CvMat &m );

namespace hydraNI
{
	const int HNI_MOUSE_LEFT = 0x00;
	const int HNI_MOUSE_MID = 0x01;
	const int HNI_MOUSE_RIGHT = 0x02;

	const int HNI_KEY_TAB = 0x09;
	const int HNI_KEY_ESC = 0x1b;
	const int HNI_KEY_RETURN = 0x0d;
	const int HNI_KEY_ENTER = 0x0a;
	const int HNI_KEY_BACKSPACE = 0x08;


	inline const glm::vec3 &white()
	{
		static glm::vec3 w( 1.0 );
		return w;
	}

	inline const glm::vec3 &black()
	{
		static glm::vec3 b( 0.0 );
		return b;
	}

	inline const glm::vec3 &red()
	{
		static glm::vec3 r( 1.0, 0.0, 0.0 );
		return r;
	}

	inline const glm::vec3 &green()
	{
		static glm::vec3 g( 0.0, 1.0, 0.0 );
		return g;
	}

	inline const glm::vec3 &blue()
	{
		static glm::vec3 b( 0.0, 0.0, 1.0 );
		return b;
	}

	inline const glm::vec3 &cyan()
	{
		static glm::vec3 c( 0.0, 1.0, 1.0 );
		return c;
	}

	inline const glm::vec3 &magenta()
	{
		static glm::vec3 m( 1.0, 0.0, 1.0 );
		return m;
	}

	inline const glm::vec3 &yellow()
	{
		static glm::vec3 y( 1.0, 1.0, 0.0 );
		return y;
	}

	inline const glm::vec3 &darkgrey()
	{
		static glm::vec3 dg( 0.25 );
		return dg;
	}

	inline const glm::vec3 &grey()
	{
		static glm::vec3 g( 0.5 );
		return g;
	}

	inline const glm::vec3 &lightgrey()
	{
		static glm::vec3 lg( 0.75 );
		return lg;
	}

	inline const glm::vec3 &zero()
	{
		static glm::vec3 vzero( 0.0f, 0.0f, 0.0f );
		return vzero;
	}

	inline const glm::vec3 &unitX()
	{
		static glm::vec3 x( 1.0f, 0.0f, 0.0f );
		return x;
	}

	inline const glm::vec3 &unitY()
	{
		static glm::vec3 y( 0.0f, 1.0f, 0.0f );
		return y;
	}

	inline const glm::vec3 &unitZ()
	{
		static glm::vec3 z( 0.0f, 0.0f, 1.0f );
		return z;
	}

	inline const glm::vec3 &forward()
	{
		static glm::vec3 fwd( 0.0f, 0.0f, -1.0f );
		return fwd;
	}

	inline const glm::vec3 &one()
	{
		static glm::vec3 vOne( 1.0f, 1.0f, 1.0f );
		return vOne;
	}
	
	inline float PI()
	{
#ifdef WIN32
		__asm fldpi;
#else
		return 3.14159265358979323846f;
#endif
	}

	inline float toRad( float deg )
	{
		static float s = PI() / 180.0f;
		return deg * s;
	}

	inline float toDeg( float rad )
	{
		static float s = 180.0f / PI();
		return rad * s;
	}

	inline bool isPo2( unsigned int x )
	{
		if( x )
			return ( ( x & ( x - 1 ) ) == 0 );
		return false;
	}

	inline unsigned int nextPo2( unsigned int x )
	{
		unsigned int npo2 = 0x01;
		while( x > npo2 )
		{
			if( npo2 == (unsigned int) ( 0x01 << 31 ) )
				return 0;
			npo2 <<= 1;
		}
		return npo2;
	}

	//custom min function since std::min produced slow code in vc++
	// http://randomascii.wordpress.com/2013/11/24/stdmin-causing-three-times-slowdown-on-vc/
	template<typename T>
	inline const T min( const T left, const T right )
	{
		return ( right < left?right:left );
	}

	//custom max function since std::max produced slow code in vc++
	// http://randomascii.wordpress.com/2013/11/24/stdmin-causing-three-times-slowdown-on-vc/
	template<typename T>
	inline const T max( const T left, const T right )
	{
		return ( left < right?right:left );
	}

	template<typename T>
	inline const T clamp( const T &value, const T &minValue, const T &maxValue )
	{
		return hydraNI::max( minValue, hydraNI::min( value, maxValue ) );
	}

	template<typename T>
	inline void safeDelete( T* &p )
	{
		if( p )
		{
			delete p;
			p = NULL;
		}
	}

	template<typename T>
	inline void safeDeleteArray( T* &p )
	{
		if( p )
		{
			delete[] p;
			p = NULL;
		}
	}


	template<size_t T>
	void makeDateTimeString( char( &str )[T], time_t rawTime, bool fileSafe = false )
	{
		struct tm *timeinfo;
		timeinfo = localtime( &rawTime );

#ifdef WIN32
		sprintf_s( str,
			( fileSafe?"%d-%02d-%02d %02d-%02d-%02d":"%d-%02d-%02d %02d:%02d:%02d" ),
			timeinfo->tm_year + 1900,
			timeinfo->tm_mon + 1,
			timeinfo->tm_mday,
			timeinfo->tm_hour,
			timeinfo->tm_min,
			timeinfo->tm_sec );
#else
		snprintf( str, T,
			( fileSafe?"%d-%02d-%02d %02d-%02d-%02d":"%d-%02d-%02d %02d:%02d:%02d" ),
			timeinfo->tm_year + 1900,
			timeinfo->tm_mon + 1,
			timeinfo->tm_mday,
			timeinfo->tm_hour,
			timeinfo->tm_min,
			timeinfo->tm_sec );
#endif // WIN32
	}

	template<size_t T>
	void makeDateTimeString( char( &str )[T], bool fileSafe = false )
	{
		time_t rawTime;

		::time( &rawTime );

		makeDateTimeString( str, rawTime, fileSafe );
	}

	std::string getDateTimeString( bool fileSafe = false );

#ifdef WIN32
	std::string formatLastWinError();
#endif // WIN32
}
