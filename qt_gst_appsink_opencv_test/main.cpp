#include <QCoreApplication>

#include "gst_appsink_opencv.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <string>

#include <gst/gst.h>

using namespace std;

int main(int argc, char *argv[])
{
    gst_init( &argc, &argv );

    string pipeline = "v4l2src device=/dev/video0 ! "
                      "video/x-raw,format=I420,framerate=30/1,width=1280,height=720 ! "
                      "videoconvert";

    GstSinkOpenCV* ocvAppsink = GstSinkOpenCV::Create( pipeline, 5, 15, GstSinkOpenCV::DEBUG_NONE );

    while(1)
    {
        // Getting last frame from appsink
        cv::Mat frame = ocvAppsink->getLastFrame();

        if( !frame.empty() )
        {
            cv::imshow( "Frame", frame );
        }

        if( cv::waitKey(5) == 'q' )
        {
            break;
        }
    }

    delete ocvAppsink;
}
