#pragma once

#include <glm/glm.hpp>

namespace hydraNI
{
	class Line
	{
	private:
		glm::vec3		p0;
		glm::vec3		dir;

	public:
		Line() :
			p0( 0.0 ),
			dir( 0.0, 0.0, 1.0 )
		{
		}

		Line( const glm::vec3 &p0, const glm::vec3 &dir ) :
			p0( p0 ),
			dir( dir )
		{
			glm::normalize( this->dir );
		}

		const glm::vec3 &getP0() const				{	return this->p0;	}
		void setP0( const glm::vec3 &p0 )			{	this->p0 = p0;		}

		const glm::vec3 &getDirection() const		{	return this->dir;	}
		void setDirection( const glm::vec3 &dir )	{	this->dir = dir;	glm::normalize( this->dir );	}
	};
}