#pragma once


#define CONTOURPOINTS_MAX	1000
#define FLOWVECTORS_MAX		1000

namespace hydraNI
{
	/* x and y in range of [-1 1] */
	//================================================================
	struct Point2
	{
		Point2 () : x (0), y (0) {}
		Point2 (float x, float y) : x (x), y (y) {}
		float x;
		float y;

		Point2 &operator= (const Point2 &p)
		{
			x = p.x;
			y = p.y;

			return *this;
		}
	};

	/* width and height are in range [0 2] */
	//================================================================
	struct Rectangle
	{
		Rectangle () : width (0), height (0) {}
		Rectangle (float width, float height, const Point2 &pos ) : width( width ), height( height ), pos( pos )	{}
		Point2 pos;
		float width;
		float height;


		Rectangle &operator= (const Rectangle &r)
		{
			pos = r.pos;
			width = r.width;
			height = r.height;

			return *this;
		}
	};

	//================================================================
	struct Blob
	{
		Point2 centroid;
		Rectangle boundingbox;

		Blob()	{}
		Blob( const Point2 &centroid, const Rectangle &boundingbox ) : centroid( centroid ), boundingbox( boundingbox )	{}

		Blob &operator= (const Blob &b)
		{
			centroid = b.centroid;
			boundingbox = b.boundingbox;

			return *this;
		}
	};

	//================================================================
	enum StereoPosition
	{
		STEREO_LEFT,
		STEREO_RIGHT
	};

	//================================================================
	struct TwoBlobs
	{
		TwoBlobs() : selected_blob (0) {}
		TwoBlobs( const Blob &blob1, const Blob &blob2, StereoPosition stereopos ) : blob1( blob1 ), blob2( blob2 ), selected_blob( 0 ), stereopos( stereopos )	{}

		Blob blob1;
		Blob blob2;

		int selected_blob;

		StereoPosition stereopos;
	};

	//================================================================
	struct TwoContours
	{
		TwoContours () : num_points_1 (0), num_points_2 (0)
		{}
		int num_points_1;
		int num_points_2;

		Point2 contourpoints [CONTOURPOINTS_MAX];

		StereoPosition stereopos;
	};

	struct FlowVectors
	{
		FlowVectors () : vectors_x1 (0), vectors_y1 (0), vectors_x2 (0), vectors_y2 (0) {}

		Point2 pos1;
		int vectors_x1;
		int vectors_y1;

		Point2 pos2;
		int vectors_x2;
		int vectors_y2;

		Point2 flowvectors [FLOWVECTORS_MAX];

		Point2 step;
	};
}