#include "Common.h"

#ifdef WIN32
#include <Windows.h>
#endif

#include <opencv/cv.h>


std::ostream &operator << ( std::ostream &out, const glm::vec2 &v )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::vec2 [ "
		<< v.x << ", "
		<< v.y
		<< " ]";
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const glm::vec3 &v )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::vec3 [ "
		<< v.x << ", "
		<< v.y << ", "
		<< v.z
		<< " ]";
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const glm::vec4 &v )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::vec4 [ "
		<< v.x << ", "
		<< v.y << ", "
		<< v.z << ", "
		<< v.w
		<< " ]";
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const glm::mat2 &m )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::mat2 " << std::endl
		<< "[ " << m[0][0] << ", " << m[0][1] << "] " << std::endl
		<< "[ " << m[1][0] << ", " << m[1][1] << "] " << std::endl
		;
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const glm::mat3 &m )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::mat3 " << std::endl
		<< "[ " << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << "] " << std::endl
		<< "[ " << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << "] " << std::endl
		<< "[ " << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << "] " << std::endl
		;
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const glm::mat4 &m )
{
	std::streamsize s = out.precision( 3 );
	out << "glm::mat4 " << std::endl
		<< "[ " << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << ", " << m[0][3] << "] " << std::endl
		<< "[ " << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << ", " << m[1][3] << "] " << std::endl
		<< "[ " << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << ", " << m[2][3] << "] " << std::endl
		<< "[ " << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ", " << m[3][3] << "] " << std::endl
		;
	out.precision( s );

	return out;
}

std::ostream &operator << ( std::ostream &out, const CvMat &m )
{
	std::streamsize s = out.precision( 3 );
	out << "CvMat (" << m.cols << "x" << m.rows << ")" << std::endl;
	for( int i = 0; i < m.rows; i++ )
	{
		out << "[ ";
		for( int j = 0; j < m.cols; j++ )
			out << CV_MAT_ELEM( m, float, i, j ) << ( ( j < m.cols - 1 )?", ":"" );
		out << "] " << std::endl;
	}
	out.precision( s );

	return out;
}

std::string hydraNI::getDateTimeString( bool fileSafe )
{
	char tempStr[32];
	hydraNI::makeDateTimeString( tempStr, fileSafe );
	return std::string( tempStr );
}

#ifdef WIN32
std::string hydraNI::formatLastWinError()
{
	DWORD errCode = GetLastError();

	LPVOID lpMsgBuf;

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_MAX_WIDTH_MASK,
		NULL,
		errCode,
		MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ),
		(LPTSTR) &lpMsgBuf,
		0, NULL );

	std::stringstream sstr;
	sstr << "\"" << (LPTSTR) lpMsgBuf << "\" (ErrorCode: " << errCode << ")";

	LocalFree( lpMsgBuf );

	return sstr.str();
}
#endif //WIN32
