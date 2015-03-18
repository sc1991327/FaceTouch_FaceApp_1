// HDFace.cpp : ƒRƒ“ƒ\[ƒ‹ ƒAƒvƒŠƒP[ƒVƒ‡ƒ“‚ÌƒGƒ“ƒgƒŠ ƒ|ƒCƒ“ƒg‚ð’è‹`‚µ‚Ü‚·B
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

//#include "stdafx.h"
#include <stdio.h>
#include <tchar.h>
#include <SDKDDKVer.h>
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <bitset>
#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <thread>

// for osc
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"
#define ADDRESS "127.0.0.1"
#define PORTOUT 7000
#define PORTIN  7001
#define OUTPUT_BUFFER_SIZE 4096

using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

struct infoControl
{
	// H
	int iLowH;
	int iHighH;
	// S
	int iLowS;
	int iHighS;
	// V
	int iLowV;
	int iHighV;

	int iAreaSize;
};

int STATE_TOUCH = 0;	// receive signal + 1
int STATE_TOUCHED = 0;	// handle signal + 1
bool STATE_SIGNAL = false;	// if use one touch, connect detection

// for osc receive

class ExamplePacketListener : public osc::OscPacketListener {
protected:

    virtual void ProcessMessage( const osc::ReceivedMessage& m, 
				const IpEndpointName& remoteEndpoint )
    {
        (void) remoteEndpoint; // suppress unused parameter warning

        try{
            // example of parsing single messages. osc::OsckPacketListener
            // handles the bundle traversal.
            
            if( std::strcmp( m.AddressPattern(), "/touched" ) == 0 ){
                // example #2 -- argument iterator interface, supports
                // reflection for overloaded messages (eg you can call 
                // (*arg)->IsBool() to check if a bool was passed etc).
                osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
                int a = (arg++)->AsInt32();
                if( arg != m.ArgumentsEnd() )
                    throw osc::ExcessArgumentException();
				// set the system state
				STATE_TOUCH++;								
                std::cout << "received '/touched' message with arguments: "<< a << "\n";
			}
			else{
				std::cout << "received messages" << std::endl;
			}
        }catch( osc::Exception& e ){
            // any parsing errors such as unexpected argument types, or 
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: "
                << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};

void ReceiveMsg(){

	// for osc receive -- need multi-thread

	ExamplePacketListener listener;
    UdpListeningReceiveSocket s(
            IpEndpointName( IpEndpointName::ANY_ADDRESS, PORTIN ),
            &listener );

    std::cout << "press ctrl-c to end\n";

	s.RunUntilSigInt();

}

// for osc send
bool SendMsg1DData(char *msgBegin, int msgID){
	
	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORTOUT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(msgBegin) << msgID << osc::EndMessage
		<< osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );

	return true;
}

bool SendMsg2DData(char *msgBegin, int msgID, int rx, int ry){

	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORTOUT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(msgBegin) << msgID << rx << ry << osc::EndMessage
		<< osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );

	return true;
}

bool SendMsgControl(char *msgBegin, char *msgData){

	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORTOUT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(msgBegin) << msgData << osc::EndMessage
		<< osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );

	return true;
}

// the input PosOpt is the defined opt face point, the PosNow is the finger pos now
// this method only work for the special defined PosOpt.
void MsgPrepare(vector<Vec6i> &PosOpt, Vec2i PosFinger){
	
	for (int it = 0; it < PosOpt.size(); it++){
		// check the finger in a opt position or not
		int lx = ( PosFinger[0] - PosOpt[it][2] );
		int ly = ( PosFinger[1] - PosOpt[it][3] );
		if ( abs(lx) < PosOpt[it][1] && abs(ly) < PosOpt[it][1] ){
			STATE_SIGNAL = true;
			// use 1d data
			if ( PosOpt[it][4] == 1 ){
				SendMsg1DData("/face-touch", it);
			}
			// use 2d data
			if ( PosOpt[it][4] == 2 ){
				STATE_SIGNAL = false;
				if ( STATE_TOUCH - STATE_TOUCHED > 5 ){
					STATE_TOUCHED += 5;	
				}
				SendMsg2DData("/face-move", it, lx, ly);
			}
		}
	}

}

// find all connect components
void FindBlobs(const Mat &binary, vector < vector<Point2i> > &blobs)
{
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            Rect rect;
            floodFill(label_image, Point(x,y), label_count, &rect, 0, 0, 4);

            vector <Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }
}

// input and output CV_8UC3 RGB image
Vec2i FindFingerPoint(Mat &imgFinger, const Mat &imgOriginal, infoControl infocontrol){

	Vec2i fpos;
	fpos[0] = 0;
	fpos[1] = 0;

	//Convert the captured frame from BGR to HSV
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	
	//Threshold the image
	Mat imgThresholded;
	inRange(imgHSV, Scalar(infocontrol.iLowH, infocontrol.iLowS, infocontrol.iLowV), Scalar(infocontrol.iHighH, infocontrol.iHighS, infocontrol.iHighV), imgThresholded);

	//morphological opening (removes small objects from the foreground)
	erode( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	// 1. ------ Calculate the moments of the thresholded image
	/*
	imgThresholded *= 1./255;

	vector <vector<cv::Point2i > > blobs;
	FindBlobs(imgThresholded, blobs);

	if( blobs.size() > 0 ){

		// find the bigest blob.
		for(size_t i = 0; i < blobs.size(); i++){
			for(size_t j=0; j < blobs[i].size(); j++) {
				int x = blobs[i][j].x;
				int y = blobs[i][j].y;

				imgFinger.at<Vec3b>(y,x)[0] = 128;
				imgFinger.at<Vec3b>(y,x)[1] = 128;
				imgFinger.at<Vec3b>(y,x)[2] = 128;
			}
		}
	}
	*/

	// 2. ------ Find The bigest connect component and calculate the center of it.
		
	///*
	imgThresholded *= 1./255;

	vector <vector<cv::Point2i > > blobs;
	FindBlobs(imgThresholded, blobs);

	if( blobs.size() > 0 ){

		// find the bigest blob.
		int LBlob = 0;						// mark the largest blob.
		int LBlobSize = blobs[0].size();	// mark the largest blob's size.
		for(size_t i = 0; i < blobs.size(); i++){
			if( blobs[i].size() > LBlobSize ){
				LBlob = i;
				LBlobSize = blobs[i].size();
			}
		}

		// color the bigest blob
		if ( blobs[LBlob].size() > infocontrol.iAreaSize ){
			int cxall = 0;
			int cyall = 0;
			for(size_t j=0; j < blobs[LBlob].size(); j++) {
				int x = blobs[LBlob][j].x;
				int y = blobs[LBlob][j].y;
				cxall += x;
				cyall += y;

				imgFinger.at<Vec3b>(y,x)[0] = 0;
				imgFinger.at<Vec3b>(y,x)[1] = 60;
				imgFinger.at<Vec3b>(y,x)[2] = 0;
			}
			fpos[1] = int(cyall / blobs[LBlob].size());
			fpos[0] = int(cxall / blobs[LBlob].size());
			cv::circle( imgFinger, cv::Point( fpos[0], fpos[1] ), 5, static_cast<cv::Scalar>( Vec3b(0, 255, 0) ), -1, CV_AA );
		}
	}
	//*/

	//imshow("Thresholded Image", imgFinger);			//show the thresholded image
	//imshow("Original", imgOriginal);				//show the original image

	return fpos;
}

void ChooseFingerColor(infoControl &infocontrol){
	
	//create a window called "Control"
    namedWindow("Control",CV_WINDOW_AUTOSIZE);

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &infocontrol.iLowH, 255);		//Hue (0 - 255)
	cvCreateTrackbar("HighH", "Control", &infocontrol.iHighH, 255);

	cvCreateTrackbar("LowS", "Control", &infocontrol.iLowS, 255);		//Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &infocontrol.iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &infocontrol.iLowV, 255);		//Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &infocontrol.iHighV, 255);

	cvCreateTrackbar("AreaSizeThreshold", "Control", &infocontrol.iAreaSize, 1000);//Value (0 - 100000)

}

// return the nearest face point.
Vec4i GetTouchPosition(vector<Vec4i> &facepos, Vec2i &fingerpos){
	
	// input control
	//cout << "Face Points Size: " << facepos.size() << endl;
	//cout << "Finger Position: " << fingerpos[0] << ", " << fingerpos[1] << endl;

	if(facepos.size() > 0 && fingerpos[0] != 0 && fingerpos[1] != 0){
		int Near_id = facepos[0][0];
		int Near_dist = abs(facepos[0][1] - fingerpos[0]) + abs(facepos[0][2] - fingerpos[1]);
		for(int i=0; i<facepos.size(); i++){
			int tempdist = abs(facepos[i][1] - fingerpos[0]) + abs(facepos[i][2] - fingerpos[1]);
			if( Near_dist > tempdist ){
				Near_id = facepos[i][0];
				Near_dist = tempdist;
			}
		}
		//cout << "F: " << fingerpos << endl;
		//cout << "N: " << facepos[Near_id] << endl;
		//cout << endl;
		return facepos[Near_id];
	}else{
		return Vec4i(-1, -1, -1, -1);
	}

}

int getAllInfo(){

	// ---------- kinect data input --------------

	// Sensor perpare
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Get Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Coordinate Mapper
	// - Project and unproject depth from 2D image space to 3D camera space
	// - Map between locations on the depth image and their corresponding locations on the color image
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	// Read Source
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Get Color Image Data Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	// Use the kinect Description - image size >> use to define the memory size.
	int width = 0;
	int height = 0;
	pDescription->get_Width(&width);	// 1920
	pDescription->get_Height(&height);	// 1080
	
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);	// color buffer size

	cv::Mat bufferMat(height, width, CV_8UC4);						// to store color image
	cv::Mat depthMat(height, width, CV_8UC4);
	cv::Mat FaceMat = Mat::zeros(height, width, CV_8UC4);			// to store face info

	cv::Mat ShowBufferMat(height / 2, width / 2, CV_8UC4);			// to show color image by 1/2 size
	cv::Mat ShowDepthMat(height / 2, width / 2, CV_8UC4);
	cv::Mat ShowFaceMat(height / 2, width / 2, CV_8UC4);			// to show face info by 1/2 size
	cv::Mat ShowFingerMat = Mat::zeros(height / 2, width / 2, CV_8UC3);		// to show finger info by 1/2 size
	cv::Mat ShowMat = Mat::zeros(height / 2, width / 2, CV_8UC3);

	// ------ Interface for Face Info ------

	// Color Table - to show the detect info.
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b(0, 0, 255);
	color[1] = cv::Vec3b(0, 0, 255);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(0, 0, 255);
	color[4] = cv::Vec3b(0, 0, 255);
	color[5] = cv::Vec3b(0, 0, 255);

	// define
	IHighDefinitionFaceFrameSource* pHDFaceSource[BODY_COUNT];	// Represents high definition face frame sources array. - data from pScener
	IHighDefinitionFaceFrameReader* pHDFaceReader[BODY_COUNT];	// Represents high definition face frame readers array. - data from pHDFaceSource
	IFaceModelBuilder* pFaceModelBuilder[BODY_COUNT];			// Represents face model builders array.				- data from pHDFaceSource
	bool produce[BODY_COUNT] = { false };
	IFaceAlignment* pFaceAlignment[BODY_COUNT];					// Stores face alignment datas array.
	IFaceModel* pFaceModel[BODY_COUNT];							// Represents face models array.
	std::vector<std::vector<float>> deformations(BODY_COUNT, std::vector<float>(FaceShapeDeformations::FaceShapeDeformations_Count));
	
	vector<Vec6i>	PosOptData;		// <ID, R, x, y, type, not use> if in, to 
	vector<Vec4i>	PosFaceData;
	Vec2i			PosFingerData;

	// pre-define opt position 
	// use to touch (1d info)
	PosOptData.push_back(Vec6i(496, 1, 0, 0, 1, 0));
	PosOptData.push_back(Vec6i(126, 1, 0, 0, 1, 0));
	PosOptData.push_back(Vec6i(1063, 1, 0, 0, 1, 0));
	PosOptData.push_back(Vec6i(5, 1, 0, 0, 1, 0));
	// use 2d info
	PosOptData.push_back(Vec6i(273, 1, 0, 0, 2, 0));	
	PosOptData.push_back(Vec6i(936, 1, 0, 0, 2, 0));	
	
	// initial - for each body
	for (int count = 0; count < BODY_COUNT; count++){
		
		// Source - Creates a high definition face frame source.
		hResult = CreateHighDefinitionFaceFrameSource(pSensor, &pHDFaceSource[count]);
		if (FAILED(hResult)){
			std::cerr << "Error : CreateHighDefinitionFaceFrameSource()" << std::endl;
			return -1;
		}

		// Reader
		hResult = pHDFaceSource[count]->OpenReader(&pHDFaceReader[count]);
		if (FAILED(hResult)){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		// Open Face Model Builder
		hResult = pHDFaceSource[count]->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &pFaceModelBuilder[count]);
		if (FAILED(hResult)){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenModelBuilder()" << std::endl;
			return -1;
		}

		// Start Collection Face Data
		hResult = pFaceModelBuilder[count]->BeginFaceDataCollection();
		if (FAILED(hResult)){
			std::cerr << "Error : IFaceModelBuilder::BeginFaceDataCollection()" << std::endl;
			return -1;
		}

		// Create Face Alignment
		hResult = CreateFaceAlignment(&pFaceAlignment[count]);
		if (FAILED(hResult)){
			std::cerr << "Error : CreateFaceAlignment()" << std::endl;
			return -1;
		}

		// Create Face Model
		hResult = CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &deformations[count][0], &pFaceModel[count]);
		if (FAILED(hResult)){
			std::cerr << "Error : CreateFaceModel()" << std::endl;
			return -1;
		}
	}

	UINT32 vertex = 0;
	hResult = GetFaceModelVertexCount(&vertex);		// 1347 - all vertex for each face model by kinect
	if (FAILED(hResult)){
		std::cerr << "Error : GetFaceModelVertexCount()" << std::endl;
		return -1;
	}

	// ------ Control Window ------
	// Use to control the ROI Color.

	infoControl ifctl;
	// H
	ifctl.iLowH = 38;
	ifctl.iHighH = 83;
	// S
	ifctl.iLowS = 100;
	ifctl.iHighS = 200;
	// V
	ifctl.iLowV = 60;
	ifctl.iHighV = 200;
	// Area Size Threshold
	ifctl.iAreaSize = 80;

	ChooseFingerColor(ifctl);

	while (1){
		
		// Color Frame
		// - pColorReader >> pColorFrame >> bufferMat
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)){
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)){
				cv::resize(bufferMat, ShowBufferMat, cv::Size(), 0.5, 0.5);	// resize to 1/2
			}
		}
		SafeRelease(pColorFrame);

		// Body Frame
		// - pBodyReader >> pBodyFrame >1:n> pBody >1:m> joint
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)){
				for (int count = 0; count < BODY_COUNT; count++){
					BOOLEAN bTrackingIdValid = false;
					hResult = pHDFaceSource[count]->get_IsTrackingIdValid(&bTrackingIdValid);
					if (!bTrackingIdValid){
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked(&bTracked);
						if (SUCCEEDED(hResult) && bTracked){
							
							// Joint
							Joint joint[JointType::JointType_Count];
							hResult = pBody[count]->GetJoints( JointType::JointType_Count, joint );
							if( SUCCEEDED( hResult ) ){
								for( int type = 0; type < JointType::JointType_Count; type++ ){
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
										cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA );
									}	
								}
							}

							// Set TrackingID to Detect Face
							UINT64 trackingId = _UI64_MAX;
							hResult = pBody[count]->get_TrackingId(&trackingId);
							if (SUCCEEDED(hResult)){
								pHDFaceSource[count]->put_TrackingId(trackingId);
							}
						}
					}
				}
			}
			for (int count = 0; count < BODY_COUNT; count++){
				SafeRelease(pBody[count]);
			}
		}
		SafeRelease(pBodyFrame);

		// HD Face Frame
		// - pHDFaceReader >> pHDFaceFrame >> pFaceAlignment
		for (int count = 0; count < BODY_COUNT; count++){
			IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
			hResult = pHDFaceReader[count]->AcquireLatestFrame(&pHDFaceFrame);
			if (SUCCEEDED(hResult) && pHDFaceFrame != nullptr){
				BOOLEAN bFaceTracked = false;
				hResult = pHDFaceFrame->get_IsFaceTracked(&bFaceTracked);
				if (SUCCEEDED(hResult) && bFaceTracked){
					hResult = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(pFaceAlignment[count]);
					if (SUCCEEDED(hResult) && pFaceAlignment[count] != nullptr){
						// Face Model Building
						// - pFaceModelBuilder >> pFaceModelData >> pFaceModel
						if (!produce[count]){
							std::system("cls");
							FaceModelBuilderCollectionStatus collection;
							hResult = pFaceModelBuilder[count]->get_CollectionStatus(&collection);
							if (collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete){
								// collection data complete to do

								std::cout << "Status : Complete" << std::endl;
								cv::putText(bufferMat, "Status : Complete", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
								IFaceModelData* pFaceModelData = nullptr;
								hResult = pFaceModelBuilder[count]->GetFaceData(&pFaceModelData);
								if (SUCCEEDED(hResult) && pFaceModelData != nullptr){
									hResult = pFaceModelData->ProduceFaceModel(&pFaceModel[count]);
									if (SUCCEEDED(hResult) && pFaceModel[count] != nullptr){
										produce[count] = true;
									}
								}
								SafeRelease(pFaceModelData);

							}
							else{
								// collection data not complete to do

								std::cout << "Status : " << collection << std::endl;
								cv::putText(bufferMat, "Status : " + std::to_string(collection), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);

								// Collection Status
								if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded){
									std::cout << "Need : Tilted Up Views" << std::endl;
									cv::putText(bufferMat, "Need : Tilted Up Views", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
								}
								else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded){
									std::cout << "Need : Right Views" << std::endl;
									cv::putText(bufferMat, "Need : Right Views", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
								}
								else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded){
									std::cout << "Need : Left Views" << std::endl;
									cv::putText(bufferMat, "Need : Left Views", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
								}
								else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded){
									std::cout << "Need : Front ViewFrames" << std::endl;
									cv::putText(bufferMat, "Need : Front ViewFrames", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
								}

								// Capture Status
								FaceModelBuilderCaptureStatus capture;
								hResult = pFaceModelBuilder[count]->get_CaptureStatus(&capture);
								switch (capture){
								case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
									std::cout << "Error : Face Too Far from Camera" << std::endl;
									cv::putText(bufferMat, "Error : Face Too Far from Camera", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
									break;
								case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
									std::cout << "Error : Face Too Near to Camera" << std::endl;
									cv::putText(bufferMat, "Error : Face Too Near to Camera", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
									break;
								case FaceModelBuilderCaptureStatus_MovingTooFast:
									std::cout << "Error : Moving Too Fast" << std::endl;
									cv::putText(bufferMat, "Error : Moving Too Fast", cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
									break;
								default:
									break;
								}
							}
						}

						// ****** Use Face Data to show on the image. ******
						// HD Face Points
						// - pFaceModel >(pFaceAlignment)> facePoints[CameraSpacePoint] >(pCoordinateMapper)> colorSpacePoint[ColorSpacePoint]
						std::vector<CameraSpacePoint> facePoints(vertex);
						FaceMat = Mat::zeros(height, width, CV_8UC4);
						hResult = pFaceModel[count]->CalculateVerticesForAlignment(pFaceAlignment[count], vertex, &facePoints[0]);
						if (SUCCEEDED(hResult)){
							PosFaceData.clear();
							int Xmax = 0, Ymax = 0, Xmin = width, Ymin = height;
							for (int point = 0; point < vertex; point++){
								ColorSpacePoint colorSpacePoint;
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(facePoints[point], &colorSpacePoint);
								if (SUCCEEDED(hResult)){
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									// use to find the face location
									if ( Xmax < x ){ Xmax = x; }
									if ( Ymax < y ){ Ymax = y; }
									if ( Xmin > x ){ Xmin = x; }
									if ( Ymin > y ){ Ymin = y; }
									// use location
									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)){
										PosFaceData.push_back(Vec4i(point, static_cast<int>(colorSpacePoint.X / 2), static_cast<int>(colorSpacePoint.Y / 2), -1));
										cv::circle(FaceMat, cv::Point(static_cast<int>(colorSpacePoint.X), static_cast<int>(colorSpacePoint.Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
									}
								}
							}
							//cout << "Max" << Ymax << endl;
							//cout << "Min" << Ymin << endl;
							for (int point = 0; point < vertex; point++){
								// if the point is a defined opt point, update it's position
								for (int it = 0; it < PosOptData.size(); it++){
									if( PosOptData[it][0] == point ){
										PosOptData[it][1] = (Ymax - Ymin) * PosOptData[it][4] / 25;
										PosOptData[it][2] = PosFaceData[point][1];
										PosOptData[it][3] = PosFaceData[point][2];
										break;
									}
								}
							}
						}
					}
				}
			}
			SafeRelease(pHDFaceFrame);
		}

		// Retrieved Depth Frame Size
		// To Reserve Depth Frame Buffer
		// Acquire Latest Depth Frame
		IDepthFrame* pDepthFrame = nullptr;
		IFrameDescription* pDepthDescription = nullptr;
		int depthWidth = 0;
		int depthHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;

		hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
		if(SUCCEEDED(hResult)){
			hResult = pDepthFrame->get_FrameDescription( &pDepthDescription );
			pDepthDescription->get_Width( &depthWidth );
			pDepthDescription->get_Height( &depthHeight );
			UINT16 *depthBuffer = new UINT16[depthHeight * depthWidth];
			if (SUCCEEDED(hResult)) 
				hResult = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			if (SUCCEEDED(hResult)) 
				hResult = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			if (SUCCEEDED(hResult)){
				hResult = pDepthFrame->CopyFrameDataToArray(depthHeight * depthWidth, depthBuffer);
				if( FAILED( hResult ) ){
					std::cerr << "Error : IDepthFrame::CopyFrameDataToArray()" << std::endl;
				}  
				cv::Mat depthMap = cv::Mat(depthHeight, depthWidth, CV_16U, depthBuffer);
				// show depth image
				/*cv::Mat img0 = cv::Mat::zeros(depthHeight, depthWidth, CV_8UC1);
				cv::Mat img1;
				double scale = 255.0 / (nDepthMaxReliableDistance - nDepthMinReliableDistance);
				depthMap.convertTo(img0, CV_8UC1, scale);
				applyColorMap(img0, img1, cv::COLORMAP_JET);
				cv::imshow("Depth Only", img1);*/
			}
			if (depthBuffer != nullptr) {
				delete[] depthBuffer;
				depthBuffer = nullptr;
			}
		}
		SafeRelease( pDepthFrame );

		// --------- handle results -------

		cvtColor(ShowBufferMat, ShowMat, COLOR_BGRA2BGR);

		cv::resize(FaceMat, ShowFaceMat, cv::Size(), 0.5, 0.5);
		cvtColor(ShowFaceMat, ShowFaceMat, COLOR_BGRA2BGR);

		// get finger data
		ShowFingerMat = Mat::zeros(height / 2, width / 2, CV_8UC3);
		PosFingerData = FindFingerPoint(ShowFingerMat, ShowMat, ifctl);
		
		// get touch position
		Vec4i nearfacepoint = GetTouchPosition(PosFaceData, PosFingerData);
		cv::circle(ShowFaceMat, cv::Point(static_cast<int>(nearfacepoint[1]), static_cast<int>(nearfacepoint[2])), 5, static_cast<cv::Scalar>(Vec3b(255, 255, 255)), -1, CV_AA);
		if (PosOptData[0][2] != 0){
			for (int it = 0; it < PosOptData.size(); it++){
				cv::circle(ShowFaceMat, cv::Point(PosOptData[it][2], PosOptData[it][3]), PosOptData[it][1], static_cast<cv::Scalar>(Vec3b(128, 0, 0)), -1, CV_AA);
			}
		}

		Mat ShowTotalMat = ShowFaceMat + ShowFingerMat;
		Mat ShowTotalMat2 = ShowTotalMat + ShowMat;
		cv::imshow("HDImage", ShowTotalMat);
		cv::imshow("HDInfo", ShowTotalMat2);

		char key = cv::waitKey(10);
		// keyboard control
		/*
		if (key == 's'){
			// handle the press 
			if (PosOptData[0][2] != 0){
				// to processing
				MsgPrepare(PosOptData, PosFingerData);
			}
		}
		*/
		// touch control
		///*
		if ( STATE_TOUCH > STATE_TOUCHED && STATE_SIGNAL == false ){
			STATE_TOUCHED++;
			// handle the press 
			if (PosOptData[0][2] != 0){
				// to processing
				MsgPrepare(PosOptData, PosFingerData);
			}
		}else if( STATE_TOUCH > STATE_TOUCHED){
			if ( STATE_TOUCH - STATE_TOUCHED > 5 ){
				STATE_TOUCHED += 5;	
			}else{
				STATE_TOUCHED++;
			}
		}else{
			STATE_SIGNAL = false;
		}
		cout << "STATE: " << STATE_TOUCH << ", " << STATE_TOUCHED << ", " << STATE_SIGNAL << endl;
		//*/
		// start call in.
		if (key == 'c'){
			SendMsgControl("/face-control","call");
		}
		// refresh all
		if (key == 'r'){
			SendMsgControl("/face-control","refresh");
		}

	}

	SafeRelease(pColorSource);
	SafeRelease(pBodySource);
	SafeRelease(pColorReader);
	SafeRelease(pBodyReader);
	SafeRelease(pDescription);
	SafeRelease(pCoordinateMapper);
	for (int count = 0; count < BODY_COUNT; count++){
		SafeRelease(pHDFaceSource[count]);
		SafeRelease(pHDFaceReader[count]);
		SafeRelease(pFaceModelBuilder[count]);
		SafeRelease(pFaceAlignment[count]);
		SafeRelease(pFaceModel[count]);
	}
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();

	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
	cv::setUseOptimized(true);

	// ---------- multi threads for receive message from processing ----------

	std::thread t1(ReceiveMsg);
	std::thread t2(getAllInfo);

	t1.join();
	t2.join();

	return 0;
}

