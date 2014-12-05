#pragma once

#include "MouseListener.h"

#include <glm/glm.hpp>

namespace hydraNI
{
	class Trackball : public MouseListener
	{
	private:
		double translationSpeed;
		double rotationSpeed;

		glm::vec3 translation;
		glm::vec3 rotation;

		bool	doRotate;
		bool	zScale;

	public:
		Trackball();
		explicit Trackball( const std::string &name, bool rotate = true, bool zScale = false );
		Trackball( const glm::vec3 &trans, const std::string &name, bool rotate = true, bool zScale = false );
		Trackball( const glm::vec3 &trans, const glm::vec3 &rot, const std::string &name, bool rotate = true, bool zScale = false );
		virtual ~Trackball();

		virtual void onMouseRel( int dx, int dy, unsigned int mouseMask );

		void apply();

		double getRotationSpeed() const				{	return this->rotationSpeed;			}
		void setRotationSpeed( double speed )		{	this->rotationSpeed = speed;		}

		double getTranslationSpeed() const			{	return this->translationSpeed;		}
		void setTranslationSpeed( double speed )	{	this->translationSpeed = speed;		}

		const glm::vec3 &getTranslation() const		{	return this->translation;	}
		const glm::vec3 &getRotation() const		{	return this->rotation;		}
	};
}