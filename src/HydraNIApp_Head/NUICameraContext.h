#pragma once

#include <map>
#include <vector>
#include <string>

#include <XnTypes.h>
#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#ifdef linux
#include <libusb-1.0/libusb.h>
#else
#include <libusb/libusb.h>
#endif // linux

namespace hydraNI
{
	class NUICamera;

	enum StreamType
	{
		ST_RGB = 0x01 << 0,
		ST_DEPTH = 0x01 << 1,
		ST_IR = 0x01 << 2,
		ST_SKELETON = 0x01 << 3
	};

	class NUICameraContext
	{
	private:
		xn::Context		context;

		libusb_context  *usbContext;
		std::map<std::string, unsigned int>			serialsToBusID;

#ifdef GCC
		std::vector<std::string>					serials;
		std::multimap<unsigned int, std::string>	busIDToCreationInfo;
		std::map<std::string, std::string>			serialsToCreationInfo;
#else
		std::vector<const std::string>					serials;
		std::multimap<unsigned int, const std::string>  busIDToCreationInfo;
		std::map<const std::string, const std::string>	serialsToCreationInfo;
#endif // GCC

		bool findDeviceCreationInfo( const xn::NodeInfo &nodeInfo, std::string &creationInfo );

		bool findGenerator( XnProductionNodeType type, const std::string &creationInfo, xn::Generator &generator );
		bool findGenerator( XnProductionNodeType type, const std::string &creationInfo, const XnMapOutputMode mode, xn::MapGenerator &generator );

	public:
		NUICameraContext();
		~NUICameraContext();

#ifdef GCC
		const std::vector<std::string> &getSerials() const			{ return this->serials; }
#else
		const std::vector<const std::string> &getSerials() const			{ return this->serials; }
#endif // GCC

		void rescan();
		void update();

		NUICamera *createNUICamera( const std::string &name, const std::string &serial, StreamType streams );
	};
}