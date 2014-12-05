#pragma once

#include "Line.h"

#include <glm/glm.hpp>

namespace hydraNI
{
	class Plane
	{
	private:
		glm::vec3	p0;
		glm::vec3	normal;

	public:
		Plane( const glm::vec3 &p0, const glm::vec3 &normal ) :
			p0( p0 ),
			normal( normal )
		{
			glm::normalize( this->normal );
		}

		void setP0( const glm::vec3 &p0 )				{	this->p0 = p0;		}
		const glm::vec3 &getV0() const					{	return this->p0;	}

		void setNormal( const glm::vec3 &normal )		{	this->normal = normal;		}
		const glm::vec3 &getNormal() const				{	return this->normal;		}

		bool getIntersection( const Line &line, glm::vec3 &pos, float &collisionDist )
		{
			float dot = glm::dot( this->normal, line.getDirection() );
			if( std::abs( dot ) < std::numeric_limits<float>::epsilon() )				//normal and ray are perpendicular -> no intersection
				return false;

			//http://www.softsurfer.com/Archive/algorithm_0104/algorithm_0104B.htm
			collisionDist = glm::dot( this->normal, ( this->p0 - line.getP0() ) ) / dot;
			pos = line.getP0() + line.getDirection() * collisionDist;

			return true;
		}
	};
}