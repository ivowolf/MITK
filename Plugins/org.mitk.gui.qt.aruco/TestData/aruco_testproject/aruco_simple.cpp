#include <iostream>
#include <fstream>
#include <aruco/aruco.h>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/cvdrawingutils.h>
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace aruco;

Point2f getBoardCenterPoint(Board &B,const CameraParameters &CP)
{
Mat objectPoints (4,3,CV_32FC1);
objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
objectPoints.at<float>(1,0)=2*B[0].ssize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=2*B[0].ssize;objectPoints.at<float>(2,2)=0;
objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=2*B[0].ssize;

vector<Point2f> imagePoints;//vermutlich Punkt-Coords im Bild

//! projects points from the model coordinate space to the image coordinates. Also computes
//! derivatives of the image coordinates w.r.t the intrinsic and extrinsic camera parameters
//! tranformation von weltkoordinaten in bildkoordinaten
projectPoints( objectPoints, B.Rvec,B.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

return imagePoints[0];
}

int main(int argc,char **argv)
{
    try
    {
        VideoCapture cap(-1);

//        fstream f;
//        f.open("/home/riecker/BoardCenterPoint.txt", ios::out);

        while( cap.isOpened() )
        {
            Mat frame;
            if ( ! cap.read(frame) )
                break;
            int k = waitKey(33);

            MarkerDetector MDetector;
            BoardDetector BDetector;
            vector<Marker> Markers;
            BoardConfiguration BC;
            CameraParameters CP;

            /** Define the BoardConfiguration u want to detect and the camera intrinsics */
            BC.readFromFile("/home/riecker/Downloads/aruco_testproject/TestData/chessboardinfo_pix.yml");
            CP.readFromXMLFile("/home/riecker/Downloads/aruco_testproject/TestData/out_camera_data.xml");

            /** Detect the single Markers */
            MDetector.detect(frame,Markers,CP,100);

            /** Detect the defined Board */
            BDetector.setParams(BC,CP,100);
            /*float prob =*/ BDetector.detect(frame);
            Board b = BDetector.getDetectedBoard();

            /** For drawing into the single markers */
            for (unsigned int i=0;i<Markers.size();i++) {
//                cout<<Markers[i]<<endl;
//                cv::circle(frame,Markers[i].getCenter(),20,Scalar(255,0,0),3);
//                Markers[i].draw(frame,Scalar(0,0,255),2,false);
//                CvDrawingUtils::draw3dAxis(frame,Markers[i],CP);
//                CvDrawingUtils::draw3dCube(frame,Markers[i],CP,false);
            }

            /** For drawing into the detected board */
            if(!b.empty())
            {
//                b.draw(frame,Scalar(255,0,0),2,false);
                CvDrawingUtils::draw3dAxis(frame,b,CP);
                cv::circle(frame,getBoardCenterPoint(b,CP),20,Scalar(0,0,255),10);
                cout << "BoardCenter: " << getBoardCenterPoint(b,CP) << endl;
//                f << getBoardCenterPoint(b,CP) << endl;
//                CvDrawingUtils::draw3dCube(frame,b,CP,false);
            }

            /** Show the camera stream and thresholded image */
            cv::imshow("in",frame);
            cv::imshow("thres",MDetector.getThresholdedImage());
        }
        return 0;
    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
