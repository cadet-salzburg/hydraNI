#pragma once

#include <string>

namespace hydraNI
{
	class MouseListener
	{
	private:
		bool enabled;
		std::string	name;

	public:
		explicit MouseListener( const std::string &name = "" ) : enabled( true ), name( name )	{}
		virtual ~MouseListener()			{}

		bool getEnabled() const			{	return this->enabled;		}
		void setEnabled( bool enabled )	{	this->enabled = enabled;	}

		virtual void onMouse( int x, int y, unsigned int mouseMask )		{}
		virtual void onMouseRel( int dx, int dy, unsigned int mouseMask )	{}

		const std::string &getName() const			{	return this->name;		}
	};
}