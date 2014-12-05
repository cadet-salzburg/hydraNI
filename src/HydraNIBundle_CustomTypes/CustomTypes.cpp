/*
	CADET - Center for Advances in Digital Entertainment Technologies
	Copyright 2012 Fachhochschule Salzburg GmbH

		http://www.cadet.at

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "_2RealBundle.h"

#include <vector>

void getBundleMetainfo( _2Real::bundle::BundleMetainfo &info )
{
	// --- basic information

	info.setAuthor( "Ars Electronica Futurelab" );
	info.setDescription( "bundle for hydraNI master and client applications" );
	info.setCategory( "tracking" );
	info.setContact( "info@cadet.at" );
	info.setVersion( 0, 0, 0 );

	_2Real::HumanReadableNameVisitor name;
	// -- types ( by name )
	info.exportsType( "depthFrame", { 
		_2Real::declareField( "CameraName", "string" ),
		_2Real::declareField( "CameraSerial", "string" ),
		_2Real::declareField( "FrameNr", "int" ),
		_2Real::declareField( "TimeStamp", "ulong" ),
		_2Real::declareField( "Width", "uint" ),
		_2Real::declareField( "Height", "uint" ),
		_2Real::declareField( "DepthData", name( std::vector<uint16_t>() ) ) } );

	info.exportsType("blobFrame", {
		_2Real::declareField("Contour", name(std::vector<float>() ) ),
		_2Real::declareField("COM", name(std::vector<float>() ) )
			} );
}

void getTypeMetainfo( _2Real::bundle::CustomTypeMetainfo & info, _2Real::bundle::TypeMetainfoCollection const& existingTypes )
{
	if( info.getName() == "depthFrame" )
	{
		// ordering is preserved
		info.setDescription( "hydraNI depth frame" );
		info.setInitialFieldValue( "CameraName", std::string( "not set" ) );
		info.setInitialFieldValue( "CameraSerial", std::string( "not set" ) );
		info.setInitialFieldValue( "FrameNr", (int32_t) 0xffffffff );
		info.setInitialFieldValue( "TimeStamp", (uint64_t) 0xffffffffffffffff );
		info.setInitialFieldValue( "Width", (uint32_t)0 );
		info.setInitialFieldValue( "Height", (uint32_t)0 );
		info.setInitialFieldValue( "DepthData", std::vector<uint16_t>() );
	}
	else if( info.getName() == "blobFrame" )
	{
		//ordering is preserved
		info.setDescription( "hydraNI blob frame" );
		info.setInitialFieldValue( "Contour", std::vector<float>() );
		info.setInitialFieldValue( "COM", std::vector<float>() );
	}
}