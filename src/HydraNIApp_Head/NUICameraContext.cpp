#include "NUICameraContext.h"
#include "NUICamera.h"

#include "../HydraNILib_Common/Common.h"
#include "../HydraNILib_Common/CommonNI.h"

#include <sstream>
#include <iostream>
#include <algorithm>

#include <XnPropNames.h>

using namespace hydraNI;

#define VENDOR_ID_MSFT		0x045e
#define PRODUCT_ID_KINECT	0x02ae

#define VENDOR_ID_ASUS		0x1d27
#define PRODUCT_ID_XTION	0x0600

namespace hydraNI
{
	bool getAlternateSerial( const char *creationInfo, char *serial, size_t size )
	{
		if( !size )
			return false;

		if( !strlen( creationInfo ) )
		{
			serial[0] = 0;
			return false;
		}

		int cntr = 0;

		char *infoStr = new char[nextPo2( strlen( creationInfo ) * 2 )];
		strcpy( infoStr, creationInfo );

		char *tok = strtok( infoStr, "#" );
		while( tok )
		{
			cntr++;
			tok = strtok( NULL, "#" );

			if( !tok )
				break;

			if( cntr == 2 )
				strcpy( serial, tok );
		}

		delete[] infoStr;

		return ( cntr > 1?true:false );
	}

	unsigned int getBusID( const char *creationInfo )
	{
		//creationinfo is in format "vvvv/pppp@bb/dd" in linux where
		// v = vendor id
		// p = product id
		// b = bus id
		// d = device id (canges on re-plug on same bus)
		//thus "045e/02ae@15/21" for example
		unsigned int busID = ~0x00;

		const char *busStart = NULL;
		if( ( busStart = strchr( creationInfo, '@' ) ) != NULL )
		{
			busStart++;
			const char *busEnd = NULL;
			if( ( busEnd = strchr( busStart, '/' ) ) != NULL )
			{
				char tempStr[16];
				char *tempPtr = tempStr;
				const char *busPtr = busStart;
				while( busPtr != busEnd )
					*( tempPtr++ ) = *( busPtr++ );
				*tempPtr = 0;

				busID = atoi( tempStr );
			}
		}

		return busID;
	}

	bool isKinectDevice( const libusb_device_descriptor *usbDeviceDesc )
	{
		return( usbDeviceDesc->idVendor == VENDOR_ID_MSFT && usbDeviceDesc->idProduct == PRODUCT_ID_KINECT );
	}

	bool isXtionDevice( const libusb_device_descriptor *usbDeviceDesc )
	{
		return( usbDeviceDesc->idVendor == VENDOR_ID_ASUS && usbDeviceDesc->idProduct == PRODUCT_ID_XTION );
	}

	int getSerial( const libusb_device *usbDevice, const libusb_device_descriptor *usbDeviceDesc, std::string &outString )
	{
		unsigned char serialIndex = usbDeviceDesc->iSerialNumber;

		int ret = LIBUSB_SUCCESS;
		libusb_device_handle *handle = NULL;
		if( ( ret = libusb_open( const_cast<libusb_device*>( usbDevice ), &handle ) ) != LIBUSB_SUCCESS )
			return ret;

		char serialStr[128];
		libusb_get_string_descriptor_ascii( handle, serialIndex, (unsigned char*) serialStr, sizeof( serialStr ) );

		libusb_close( handle );

		outString.assign( serialStr );

		return ret;
	}
}


NUICameraContext::NUICameraContext() :
	usbContext( NULL )
{
	int r = libusb_init( &( this->usbContext ) );

	if( r < 0 || !this->usbContext )
		throw std::runtime_error( "unable to initialize libusb" );

	this->rescan();
}

NUICameraContext::~NUICameraContext()
{
	if( this->usbContext )
		libusb_exit( this->usbContext );

	this->context.Release();
}

void NUICameraContext::rescan()
{
	this->context.StopGeneratingAll();
	this->context.Shutdown();
	this->context.Release();

	XnStatus status = this->context.Init();

	if( status != XN_STATUS_OK )
	{
		std::stringstream sstr;
		sstr << "initializing xn::Context failed: " << xnGetStatusString( status );

		throw std::runtime_error( sstr.str() );
	}

	this->serials.clear();
	this->serialsToBusID.clear();
	this->busIDToCreationInfo.clear();
	this->serialsToCreationInfo.clear();

	libusb_device **usbDeviceList = NULL;
	ssize_t count = libusb_get_device_list( this->usbContext, &usbDeviceList );
	if( !count )
		throw std::runtime_error( "no usb devices found" );
	std::cout << "found " << count << " USB devices overall" << std::endl;

	for( int i = 0; i < count; i++ )
	{
		libusb_device *usbDevice = usbDeviceList[i];
		unsigned char busID = libusb_get_bus_number( usbDevice );

		struct libusb_device_descriptor desc;
		if( libusb_get_device_descriptor( usbDevice, &desc ) != LIBUSB_SUCCESS )
		{
			std::cerr << "could not get device descriptor of usb device #" << i << " on bus " << (unsigned int) busID << std::endl;
			continue;
		}

		if( !isKinectDevice( &desc ) && !isXtionDevice( &desc ) )
			continue;

		int ret = LIBUSB_SUCCESS;
		std::string serial;
		if( ( ret = getSerial( usbDevice, &desc, serial ) ) != LIBUSB_SUCCESS )
		{
			std::cerr << "could not get serial of usb device #" << i << " on bus " << (unsigned int) busID << " libusb error code " << ret << std::endl;

#ifdef WIN32
			if( ret == LIBUSB_ERROR_NOT_SUPPORTED )
				std::cerr << "NOTE: did you install the libusb windows backend? see http://www.libusb.org/wiki/windows_backend" << std::endl;
#endif

			continue;
		}

		std::transform( serial.begin(), serial.end(), serial.begin(), tolower );

		std::cout << "usb device #" << i << " on bus " << (unsigned int) busID << " has serial " << serial << std::endl;

		this->serialsToBusID.insert( std::pair<std::string, unsigned char>( serial, busID ) );
	}
	libusb_free_device_list( usbDeviceList, 1 );

	static xn::NodeInfoList devices;
	status = this->context.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, NULL, devices, NULL );
	if( status != XN_STATUS_OK && devices.Begin() != devices.End() )
	{
		std::cerr << "Enumerating devices failed: " << xnGetStatusString( status ) << std::endl;
	}
	else if( devices.Begin() == devices.End() )
	{
		std::cerr << "No devices found." << std::endl;
	}
	else
	{
		int nodeCntr = 0;
		
		for( xn::NodeInfoList::Iterator it = devices.Begin(); it != devices.End(); ++it, ++nodeCntr )
		{
			xn::NodeInfo deviceNodeInfo = *it;

			xn::Device deviceNode;
			deviceNodeInfo.GetInstance( deviceNode );
			XnBool bExists = deviceNode.IsValid();
			if( !bExists )
			{
				this->context.CreateProductionTree( deviceNodeInfo, deviceNode );
				// this might fail.
			}

			if( deviceNode.IsValid() )
			{
				if( deviceNode.IsCapabilitySupported( XN_CAPABILITY_DEVICE_IDENTIFICATION ) )
				{
					unsigned int busID = ~0x00;
					char strDeviceName[256];
					char strSerialNumber[256];

					deviceNode.GetIdentificationCap().GetDeviceName( strDeviceName, sizeof( strDeviceName ) );
					deviceNode.GetIdentificationCap().GetSerialNumber( strSerialNumber, sizeof( strSerialNumber ) );

					if( strlen( strSerialNumber ) <= 1 )
					{
						const char *strCreationInfo = deviceNode.GetInfo().GetCreationInfo();
						if( !getAlternateSerial( strCreationInfo, strSerialNumber, sizeof( strSerialNumber ) ) )
						{
							std::cerr << "SN of device #" << nodeCntr << " not found, tried creationinfo \"" << strCreationInfo << "\"" << std::endl;

							busID = getBusID( strCreationInfo );
							std::cout << "inserting bus ID " << busID << " for device " << strDeviceName << std::endl;
						}
						else
						{
							std::transform( strSerialNumber, strSerialNumber + strlen( strSerialNumber ), strSerialNumber, tolower );
							std::cout << "SN of device #" << nodeCntr << ": " << strSerialNumber << std::endl;
						}
					}
					else
						std::transform( strSerialNumber, strSerialNumber + strlen( strSerialNumber ), strSerialNumber, tolower );

					std::cout << "device #" << nodeCntr << " \"" << strDeviceName << "\" SN: " << strSerialNumber << std::endl;

					this->serials.push_back( std::string( strSerialNumber ) );
#ifdef GCC
					this->busIDToCreationInfo.insert( std::pair<unsigned int, std::string>( busID, std::string( deviceNodeInfo.GetCreationInfo() ) ) );
					this->serialsToCreationInfo.insert( std::pair<std::string, std::string>( std::string( strSerialNumber ), std::string( deviceNodeInfo.GetCreationInfo() ) ) );
#else
					this->busIDToCreationInfo.insert( std::pair<unsigned int, const std::string>( busID, std::string( deviceNodeInfo.GetCreationInfo() ) ) );
					this->serialsToCreationInfo.insert( std::pair<const std::string, const std::string>( std::string( strSerialNumber ), std::string( deviceNodeInfo.GetCreationInfo() ) ) );
#endif // GCC
				}
				else
				{
					char info[256];
#ifdef WIN32
					strcpy_s( info, deviceNodeInfo.GetCreationInfo() );
#else
					strncpy( info, deviceNodeInfo.GetCreationInfo(), sizeof( info ) );
#endif // WIN32

					int cntr = 0;
					char strSerialNumber[256];
					strSerialNumber[0] = 0;

					char *tok = strtok( info, "#" );
					while( tok )
					{
						cntr++;
						tok = strtok( NULL, "#" );

						if( !tok )
							break;

						if( cntr == 2 )
							strcpy( strSerialNumber, tok );
					}

					if( strlen( strSerialNumber ) )
					{
						std::cerr << "device #" << nodeCntr << " serial not found (using creation info substring \"" << strSerialNumber << "\")" << std::endl;

						this->serials.push_back( std::string( strSerialNumber ) );
						this->serialsToCreationInfo.insert( std::pair<const std::string, const std::string>( std::string( strSerialNumber ), std::string( deviceNodeInfo.GetCreationInfo() ) ) );
					}
					else
						std::cerr << "device #" << nodeCntr << " cannot be identified" << std::endl;
				}
			}
			else
				std::cerr << "device #" << nodeCntr << " could not create production tree" << std::endl;

			// release the device if we created it
			if( !bExists && deviceNode.IsValid() )
			{
				deviceNode.Release();
			}
		}

		std::cout << "found " << nodeCntr << " devices" << std::endl;
	}
}

bool NUICameraContext::findDeviceCreationInfo( const xn::NodeInfo &nodeInfo, std::string &creationInfo )
{
	//searching parents for device with requested serial
	bool found = false;
	xn::NodeInfoList needed = nodeInfo.GetNeededNodes();
	for( xn::NodeInfoList::Iterator it = needed.Begin(); it != needed.End() && !found; ++it )
	{
		if( ( *it ).GetDescription().Type == XN_NODE_TYPE_DEVICE )
		{
			creationInfo = ( *it ).GetCreationInfo();
			found = true;
		}
		else
		{
			found = this->findDeviceCreationInfo( *it, creationInfo );
		}
	}

	return found;
}

bool NUICameraContext::findGenerator( XnProductionNodeType type, const std::string &creationInfo, xn::Generator &generator )
{
	xn::NodeInfoList nil;
	XnStatus status = this->context.EnumerateProductionTrees( type, NULL, nil, NULL );
	if( status != XN_STATUS_OK && nil.Begin() != nil.End() )
	{
		std::stringstream sstr;
		sstr << "Enumerating generators of type " << xnProductionNodeTypeToString( type ) << " failed: " << xnGetStatusString( status );

		throw std::runtime_error( sstr.str() );
	}
	else if( nil.Begin() == nil.End() )
	{
		std::stringstream sstr;
		sstr << "No generators of type " << xnProductionNodeTypeToString( type ) << " found.";

		throw std::runtime_error( sstr.str() );
	}

	bool found = false;
	int nodeCntr = 0;

	for( xn::NodeInfoList::Iterator it = nil.Begin(); it != nil.End(); ++it, ++nodeCntr )
	{
		if( found )
			continue;

		if( ( *it ).GetDescription().Type != type )
		{
			std::cerr << "found node of type " << xnProductionNodeTypeToString( ( *it ).GetDescription().Type ) << " when looking for " << xnProductionNodeTypeToString( type ) << " nodes -- skipping this one..." << std::endl;
			continue;
		}

		std::string parentCreationInfo;
		if( this->findDeviceCreationInfo( *it, parentCreationInfo ) )
		{
			if( !parentCreationInfo.compare( creationInfo ) )
			{
				//found device with requested serial -- color node is part of this device

				XnStatus status = XN_STATUS_OK;

				xn::NodeInfo ni = *it;  //gcc wants me to do this -- it wouldn't convert *it to xn::NodeInfo& (?)
				if( ( status = this->context.CreateProductionTree( ni ) ) != XN_STATUS_OK )
				{
					std::stringstream sstr;
					sstr << "creating production tree for " << xnProductionNodeTypeToString( type ) << " failed: " << xnGetStatusString( status );

					throw std::runtime_error( sstr.str() );
				}

				if( ( status = ( *it ).GetInstance( generator ) ) != XN_STATUS_OK )
				{
					std::stringstream sstr;
					sstr << "getting " << xnProductionNodeTypeToString( type ) << " instance failed: " << xnGetStatusString( status );

					throw std::runtime_error( sstr.str() );
				}

				generator.GetMirrorCap().SetMirror( false );

				found = true;
			}
		}
	}

	std::cout << "found " << nodeCntr << " generators of type " << xnProductionNodeTypeToString( type ) << std::endl;

	return found;
}

bool NUICameraContext::findGenerator( XnProductionNodeType type, const std::string &creationInfo, const XnMapOutputMode mode, xn::MapGenerator &generator )
{
	xn::NodeInfoList nil;
	XnStatus status = this->context.EnumerateProductionTrees( type, NULL, nil, NULL );
	if( status != XN_STATUS_OK && nil.Begin() != nil.End() )
	{
		std::stringstream sstr;
		sstr << "Enumerating generators of type " << xnProductionNodeTypeToString( type ) << " failed: " << xnGetStatusString( status );

		throw std::runtime_error( sstr.str() );
	}
	else if( nil.Begin() == nil.End() )
	{
		std::stringstream sstr;
		sstr << "No generators of type " << xnProductionNodeTypeToString( type ) << " found.";

		throw std::runtime_error( sstr.str() );
	}

	bool found = false;
	int nodeCntr = 0;

	for( xn::NodeInfoList::Iterator it = nil.Begin(); it != nil.End(); ++it, ++nodeCntr )
	{
		if( found )
			continue;

		if( ( *it ).GetDescription().Type != type )
		{
			std::cerr << "found node of type " << xnProductionNodeTypeToString( ( *it ).GetDescription().Type ) << " when looking for " << xnProductionNodeTypeToString( type ) << " nodes -- skipping this one..." << std::endl;
			continue;
		}

		std::string parentCreationInfo;
		if( this->findDeviceCreationInfo( *it, parentCreationInfo ) )
		{
			if( !parentCreationInfo.compare( creationInfo ) )
			{
				//found device with requested serial -- color node is part of this device

				XnStatus status = XN_STATUS_OK;

				xn::NodeInfo ni = *it;  //gcc wants me to do this -- it wouldn't convert *it to xn::NodeInfo& (?)
				if( ( status = this->context.CreateProductionTree( ni ) ) != XN_STATUS_OK )
				{
					std::stringstream sstr;
					sstr << "creating production tree for " << xnProductionNodeTypeToString( type ) << " failed: " << xnGetStatusString( status );

					throw std::runtime_error( sstr.str() );
				}

				if( ( status = ( *it ).GetInstance( generator ) ) != XN_STATUS_OK )
				{
					std::stringstream sstr;
					sstr << "getting " << xnProductionNodeTypeToString( type ) << " instance failed: " << xnGetStatusString( status );

					throw std::runtime_error( sstr.str() );
				}

				generator.GetMirrorCap().SetMirror( false );
				generator.SetMapOutputMode( mode );

				found = true;
			}
		}
	}

	std::cout << "found " << nodeCntr << " generators of type " << xnProductionNodeTypeToString( type ) << std::endl;

	return found;
}

NUICamera *NUICameraContext::createNUICamera( const std::string &name, const std::string &serial, StreamType streams )
{
	std::string srl( serial );

	if( ( streams & ST_DEPTH ) && ( streams &ST_IR ) )
	{
		std::cerr << "cannot create camera with both IR and DEPTH streams" << std::endl;
		return NULL;
	}
	if( ( streams & ST_SKELETON ) && ( streams &ST_IR ) )
	{
		std::cerr << "cannot create camera with both SKELETON and DEPTH streams" << std::endl;
		return NULL;
	}

	std::string creationInfo;

	if( serial.length() )
	{
#ifdef GCC
		std::map<std::string, std::string>::const_iterator mapping = this->serialsToCreationInfo.find( serial );
		if( mapping == this->serialsToCreationInfo.end() )
		{
			std::map<std::string, unsigned int>::const_iterator it = this->serialsToBusID.find( serial );
			if( it == this->serialsToBusID.end() )
			{
				std::cerr << "camera " << name << " with serial \"" << serial << "\" not found at any of the usb busses" << std::endl;

				for( it = this->serialsToBusID.begin(); it != this->serialsToBusID.end(); ++it )
					std::cout << "-> cam " << it->first << " is on bus " << it->second << std::endl;
				return NULL;
			}

			unsigned int busID = it->second;
			std::cout << "camera " << name << " with serial \"" << serial << "\" found at usb bus " << busID << std::endl;

			//try bus ID
			std::map<unsigned int, std::string>::const_iterator mapping2 = this->busIDToCreationInfo.find( busID );
			if( mapping2 == this->busIDToCreationInfo.end() )
			{
				std::cerr << "camera " << name << " with serial \"" << serial << "\" not registered" << std::endl;
				return NULL;
			}
			else
				creationInfo = mapping2->second;
		}
		else
			creationInfo = mapping->second;
#else
		std::map<const std::string, const std::string>::const_iterator mapping = this->serialsToCreationInfo.find( serial );
		if( mapping == this->serialsToCreationInfo.end() )
		{
			std::cerr << "camera " << name << " with serial \"" << serial << "\" not registered" << std::endl;
			return NULL;
		}
		else
			creationInfo = mapping->second;
#endif //GCC
	}
	else if( this->serialsToCreationInfo.size() )
	{
		std::cout << "no serial number specified, creating first available camera" << std::endl;

		srl = this->serialsToCreationInfo.begin()->first;
		creationInfo = this->serialsToCreationInfo.begin()->second;
	}
	else
	{
		std::cerr << "no cameras connected" << std::endl;
		return NULL;
	}

	XnMapOutputMode mode;
	mode.nXRes = NI_CAMERA_X_RES;
	mode.nYRes = NI_CAMERA_Y_RES;
	mode.nFPS = NI_CAMERA_FPS;

	xn::IRGenerator		irGenerator;
	xn::DepthGenerator	depthGenerator;
	xn::ImageGenerator	imageGenerator;
	xn::UserGenerator	userGenerator;

	if( streams & ST_IR )
	{
		if( !this->findGenerator( XN_NODE_TYPE_IR, creationInfo, mode, irGenerator ) )
		{
			std::stringstream sstr;
			sstr << "initializing IR generator for camera " << name << " with serial " << serial << " failed";

			throw std::runtime_error( sstr.str() );
		}
	}
	if( streams & ST_DEPTH )
	{
		if( !this->findGenerator( XN_NODE_TYPE_DEPTH, creationInfo, mode, depthGenerator ) )
		{
			std::stringstream sstr;
			sstr << "initializing depth generator for camera " << name << " with serial " << serial << " failed";

			throw std::runtime_error( sstr.str() );
		}
	}
	if( streams & ST_RGB )
	{
		if( !this->findGenerator( XN_NODE_TYPE_IMAGE, creationInfo, mode, imageGenerator ) )
		{
			std::stringstream sstr;
			sstr << "initializing image generator for camera " << name << " with serial " << serial << " failed";

			throw std::runtime_error( sstr.str() );
		}
	}
	if( streams & ST_SKELETON )
	{
		if( !this->findGenerator( XN_NODE_TYPE_USER, creationInfo, userGenerator ) )
		{
			std::stringstream sstr;
			sstr << "initializing user generator for camera " << name << " with serial " << serial << " failed";

			throw std::runtime_error( sstr.str() );
		}
		if( !userGenerator.IsCapabilitySupported( XN_CAPABILITY_SKELETON ) )
		{
			std::stringstream sstr;
			sstr << "user generator for camera " << name << " with serial " << serial << " doesn't support skeletons";

			throw std::runtime_error( sstr.str() );
		}
	}

	return new NUICamera( name, srl, irGenerator, imageGenerator, depthGenerator, userGenerator );
}

void NUICameraContext::update()
{
	this->context.WaitNoneUpdateAll();
}
