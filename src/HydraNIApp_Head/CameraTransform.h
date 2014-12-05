#pragma once

#include <glm/glm.hpp>

namespace hydraNI
{
	class CameraTransform
	{
	private:
		unsigned int	resX;
		unsigned int	resY;

		double	centerX;
		double	centerY;

		double	pxSizeX;
		double	pxSizeY;

		double	sx;
		double	sy;

		double	skew;

		double	focalLength;
		double	aspectRatio;

		glm::mat4 glExtrinsics;
		glm::mat4 glExtrinsicsInv;

		glm::mat3 glIntrinsics;

		void updateIntrinsics();

	public:
		CameraTransform();
		CameraTransform( unsigned int resX, unsigned int resY, double cx, double cy, double psX, double psY, double skew, double f, double ar );
		virtual ~CameraTransform();

		unsigned int getResolutionX() const			{	return this->resX;				}
		unsigned int getResolutionY() const			{	return this->resY;				}
		void setResolution( unsigned int resX, unsigned int resY )
		{
			this->resX = resX;
			this->resY = resY;
			this->aspectRatio = (double)this->resX / this->resY;

			this->updateIntrinsics();
		}

		double getSensorWidth() const				{	return this->resX * this->pxSizeX;	}
		double getSensorHeight() const				{	return this->resY * this->pxSizeY;	}

		double	getPixelSizeX() const				{	return this->pxSizeX;		}
		double	getPixelSizeY() const				{	return this->pxSizeY;		}
		void setPixelSize( double psX, double psY )
		{
			this->sx = 1.0 / psX;
			this->sy = 1.0 / psY;
			this->pxSizeX = psX;
			this->pxSizeY = psY;
		}

		double	getCenterX() const					{	return this->centerX;		}
		double	getCenterY() const					{	return this->centerY;		}
		double	getSX() const						{	return this->sx;			}
		double	getSY() const						{	return this->sy;			}
		double	getSkew() const						{	return this->skew;			}
		void setIntrinsicParams( double f, double cx, double cy, double skew )
		{
			this->focalLength = f;
			this->centerX = cx;
			this->centerY = cy;
			this->skew = skew;

			this->updateIntrinsics();
		}

		double getFocalLength() const				{	return this->focalLength;		}
		double getAspectRatio() const				{	return this->aspectRatio;		}

		//TODO: calculate translation and rotation vectors from that -- they are no longer valid!!
		void setExtrinsics( const glm::mat4 &extrinsics, const glm::mat4 &extrinsicsInv )
		{
			this->glExtrinsics = extrinsics;
			this->glExtrinsicsInv = extrinsicsInv;
		}

		const glm::mat4 &getExtrinsics() const		{	return this->glExtrinsics;		}
		const glm::mat4 &getExtrinsicsInv() const	{	return this->glExtrinsicsInv;	}

		glm::vec3 getWorldPosition();

		void multLeft( const glm::mat4 &m );
		void multRight( const glm::mat4 &m );

		void transLeft( const glm::vec3 &t );
		void transRight( const glm::vec3 &t );

		void rotLeft( const glm::vec3 &r );
		void rotRight( const glm::vec3 &r );
	};
}
