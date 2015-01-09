
HydraNI Bundle
==============

This Bundle provides data types used for communication between HydraNI multi-depthsensor tracking applications.
The Bundle exports one single Type, currently used to transport pre-processed depth frames recorded from a depth sensing device such as Asus Xtion or Microsoft Kinect.
Depth Frame Type
Depth frame data, containing a single framebuffer of 1 channel with depth of 16 bits. The frame also contains additional meta-information about its source and timing, so multiple sources can send in parallel.

-	<string> CameraName
	Custom name descriptor. Meant to provide an additional means for identifying the sender.
-	<string> CameraSerial
	Serial number of source depth sensing device.
-	<int> FrameNr
	ID of frame – starts from 0 and is incremented by 1 each time the depth sensing device delivers a new frame. Frame IDs are not synchronized in between senders and different timings and frame rates of clients will result in diverging frame numbers.
-	<ulong> TimeStamp
	Unix-Timestamp of the sender machine. This is a very loose description of timing and is not appropriate nor intended for frame-synchronization. 
-	<uint> Width
	Number for vertical frame pixels.
-	<uint> Height
	Number of horizontal frame pixels.
-	<std::vector<ushort>> DepthData
-	Array of depth values. Usually, values are specified as millimeters, while a value of 0 means “out of range” aka. “no data”.
Blob Frame Type
Blob frame data, containing blob contours and COM (center of mass)
-	<std::vector<float>> Contour
	Array of 2D points, describing a closed blob contour. Size of vector is a multiple of 2, since points are represented as couples of float values (x y), stringed one after another. Thus, the data block can be cast to an array of two-float structs such as glm::vec2 (http://glm.g-truc.net/0.9.5/index.html)
-	<std::vector<float>> COM
	Couple of floats, making up a 2D-point (x z), which is the position of the center of mass, projected to the XZ-plane (floor).
Dependencies
The bundle is depending on the boost library (www.boost.org) and was built against version 1.55, newer versions should be compatible without any issues, however.
Applications
As the only purpose of this bundle is to provide a means of communication for the HydraNI Server and Client applications (ADD LINK), its only application is exactly this.
