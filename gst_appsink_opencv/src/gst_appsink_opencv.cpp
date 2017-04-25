#include "gst_appsink_opencv.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <chrono>

using namespace std;

GstSinkOpenCV::GstSinkOpenCV( std::string input_pipeline, int bufferSize, DebugLevel debugLvl )
{
    mPipelineStr = input_pipeline;
    mPipeline = NULL;
    mSink = NULL;

    mMaxBufferSize = bufferSize;

    mDebugLvl = debugLvl;

    mSinkStats.fps = 0.0;
    mSinkStats.frameCount = 0;
    mSinkStats.frameDropped = 0;
    mSinkStats.errorCount = 0;
    mSinkStats.timeout = 0;

    mFrameTimeoutMsec = 50;
}

GstSinkOpenCV::~GstSinkOpenCV()
{
    if( mPipeline )
    {
        /* cleanup and exit */
        gst_element_set_state( mPipeline, GST_STATE_NULL );
        gst_object_unref( mPipeline );
        mPipeline = NULL;
    }
}

GstSinkOpenCV* GstSinkOpenCV::Create(string input_pipeline, int bufferSize, int timeout_sec , DebugLevel debugLvl )
{
    GstSinkOpenCV* gstSinkOpencv = new GstSinkOpenCV( input_pipeline, bufferSize, debugLvl );

    if( !gstSinkOpencv->init( timeout_sec ) )
    {
        delete gstSinkOpencv;
        return NULL;
    }

    return gstSinkOpencv;
}

bool GstSinkOpenCV::init( int timeout_sec )
{
    GError *error = NULL;
    GstStateChangeReturn ret;

    mPipelineStr += " ! appsink name=sink caps=\"video/x-raw,format=BGR\"";

    switch(mDebugLvl)
    {
    case DEBUG_NONE:
        break;
    case DEBUG_FULL:
    case DEBUG_VERBOSE:
    case DEBUG_INFO:
        cout << "[GstSinkOpenCV] Input pipeline:" << endl << mPipelineStr << endl << endl;
    }

    mPipeline = gst_parse_launch( mPipelineStr.c_str(), &error );

    if (error != NULL)
    {
        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            cout << "[GstSinkOpenCV] could not construct pipeline: " << error->message << endl;
        }

        g_clear_error (&error);
        return NULL;
    }

    /* set to PAUSED to make the first frame arrive in the sink */
    ret = gst_element_set_state (mPipeline, GST_STATE_PLAYING);
    switch (ret)
    {
    case GST_STATE_CHANGE_FAILURE:

        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            cout << "[GstSinkOpenCV] Failed to play the pipeline" << endl;
        }

        return false;

    case GST_STATE_CHANGE_NO_PREROLL:
        /* for live sources, we need to set the pipeline to PLAYING before we can
           * receive a buffer. We don't do that yet */
        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            cout << "[GstSinkOpenCV] Waiting for first frame" << endl;
            break;
        }

    default:
        break;
    }

    /* This can block for up to "timeout_sec" seconds. If your machine is really overloaded,
       * it might time out before the pipeline prerolled and we generate an error. A
       * better way is to run a mainloop and catch errors there. */
    ret = gst_element_get_state( mPipeline, NULL, NULL, timeout_sec * GST_SECOND );
    if (/*ret == GST_STATE_CHANGE_FAILURE*/ret!=GST_STATE_CHANGE_SUCCESS)
    {
        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            cout << "[GstSinkOpenCV] Source connection timeout" << endl;
            break;
        }

        return false;
    }

    /* get sink */
    mSink = gst_bin_get_by_name (GST_BIN (mPipeline), "sink");

    GstSample *sample;
    g_signal_emit_by_name (mSink, "pull-preroll", &sample, NULL);

    /* if we have a buffer now, convert it to a pixbuf. It's possible that we
       * don't have a buffer because we went EOS right away or had an error. */
    if (sample)
    {
        GstBuffer *buffer;
        GstCaps *caps;
        GstStructure *s;
        GstMapInfo map;

        gint width, height;

        gboolean res;

        /* get the snapshot buffer format now. We set the caps on the appsink so
         * that it can only be an rgb buffer. The only thing we have not specified
         * on the caps is the height, which is dependant on the pixel-aspect-ratio
         * of the source material */
        caps = gst_sample_get_caps (sample);
        if (!caps)
        {
            switch(mDebugLvl)
            {
            case DEBUG_NONE:
                break;
            case DEBUG_FULL:
            case DEBUG_VERBOSE:
            case DEBUG_INFO:
                cout << "[GstSinkOpenCV] could not get the format of the first frame" << endl;
                break;
            }

            return false;
        }
        s = gst_caps_get_structure (caps, 0);

        /* we need to get the final caps on the buffer to get the size */
        res = gst_structure_get_int (s, "width", &width);
        res |= gst_structure_get_int (s, "height", &height);
        if (!res)
        {
            switch(mDebugLvl)
            {
            case DEBUG_NONE:
                break;
            case DEBUG_FULL:
            case DEBUG_VERBOSE:
            case DEBUG_INFO:
                cout << "[GstSinkOpenCV] could not get the dimensions of the first frame" << endl;
                break;
            }

            return false;
        }

        //cerr << width << height << endl;

        /* create pixmap from buffer and save, gstreamer video buffers have a stride
         * that is rounded up to the nearest multiple of 4 */
        buffer = gst_sample_get_buffer (sample);

        /* Mapping a buffer can fail (non-readable) */
        if (gst_buffer_map (buffer, &map, GST_MAP_READ))
        {
            mWidth = width;
            mHeight = height;
            mChannels = map.size / (mWidth*mHeight);

            //mFrameBuffer[mPushIdx] = cv::Mat( mHeight, mWidth, CV_8UC3 );

            cv::Mat frame( mHeight, mWidth, CV_8UC3 );

            memcpy( frame.data, map.data, map.size );

            mFrameBuffer.push( frame );

            switch(mDebugLvl)
            {
            case DEBUG_NONE:
                break;
            case DEBUG_FULL:
                cv::imshow( "Sink frame", frame );
                cv::waitKey( 100 );
            case DEBUG_VERBOSE:
            case DEBUG_INFO:
                cout << "[GstSinkOpenCV] Received first frame [" << mWidth << "x" << mHeight << "x" << mChannels << "]" << endl;
            }

            gst_buffer_unmap( buffer, &map );
        }
        gst_sample_unref (sample);
    }
    else
    {
        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            cout << "[GstSinkOpenCV] could not get first frame" << endl;
        }

        return false;
    }

    /* we use appsink in push mode, it sends us a signal when data is available
       * and we pull out the data in the signal callback. We want the appsink to
       * push as fast as it can, hence the sync=false */
    g_object_set (G_OBJECT (mSink), "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect( mSink, "new-sample", G_CALLBACK(on_new_sample_from_sink), this );

    return true;
}

GstFlowReturn GstSinkOpenCV::on_new_sample_from_sink( GstElement* elt, GstSinkOpenCV* sinkData )
{
    GstSample *sample;
    //GstFlowReturn ret;

    /* get the sample from appsink */
    sample = gst_app_sink_pull_sample( GST_APP_SINK (elt) );

    if (sample)
    {
        // >>>>> FPS calculation and automatic frame timeout
        static std::chrono::time_point<std::chrono::system_clock> start = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();

        double elapsed_msec = std::chrono::duration_cast<std::chrono::microseconds>( end-start ).count()/1000.0;
        start = end;
        sinkData->mSinkStats.fps = 1000.0/elapsed_msec;

        sinkData->mFrameTimeoutMsec = static_cast<int>(elapsed_msec*1.5);
        // <<<<< FPS calculation and automatic frame timeout

        GstBuffer *buffer;
        GstCaps *caps;
        GstStructure *s;
        GstMapInfo map;

        gint width, height;

        gboolean res;

        /* get the snapshot buffer format now. We set the caps on the appsink so
         * that it can only be an rgb buffer. The only thing we have not specified
         * on the caps is the height, which is dependant on the pixel-aspect-ratio
         * of the source material */
        caps = gst_sample_get_caps (sample);
        if (!caps)
        {
            switch(sinkData->mDebugLvl)
            {
            case DEBUG_NONE:
                break;
            case DEBUG_FULL:
            case DEBUG_VERBOSE:
            case DEBUG_INFO:
                cout << "[GstSinkOpenCV] could not get the format of the frame" << endl;
                break;
            }

            return GST_FLOW_CUSTOM_ERROR;
        }
        s = gst_caps_get_structure (caps, 0);

        /* we need to get the final caps on the buffer to get the size */
        res = gst_structure_get_int (s, "width", &width);
        res |= gst_structure_get_int (s, "height", &height);
        if (!res)
        {
            switch(sinkData->mDebugLvl)
            {
            case DEBUG_NONE:
                break;
            case DEBUG_FULL:
            case DEBUG_VERBOSE:
            case DEBUG_INFO:
                cout << "[GstSinkOpenCV] could not get the dimensions of the frame" << endl;
                break;
            }

            return GST_FLOW_CUSTOM_ERROR;
        }

        /* create pixmap from buffer and save, gstreamer video buffers have a stride
         * that is rounded up to the nearest multiple of 4 */
        buffer = gst_sample_get_buffer (sample);

        /* Mapping a buffer can fail (non-readable) */
        if (gst_buffer_map (buffer, &map, GST_MAP_READ))
        {
            if( width!=sinkData->mWidth || height!=sinkData->mHeight ) // New size?
            {
                switch(sinkData->mDebugLvl)
                {
                case DEBUG_NONE:
                    break;
                case DEBUG_FULL:
                case DEBUG_VERBOSE:
                case DEBUG_INFO:
                    cout << "[GstSinkOpenCV] New Frame Size [" << width << "x" << height << "x" << sinkData->mChannels << "]" << endl;
                    break;
                }

                sinkData->mWidth = width;
                sinkData->mHeight = height;
                sinkData->mChannels = map.size / (sinkData->mWidth*sinkData->mHeight);

                if( sinkData->mChannels!=3 )
                {
                    switch(sinkData->mDebugLvl)
                    {
                    case DEBUG_NONE:
                        break;
                    case DEBUG_FULL:
                    case DEBUG_VERBOSE:
                        cout << "[GstSinkOpenCV] Only BGR image are supported" << endl;
                    case DEBUG_INFO:
                        break;
                    }

                    return GST_FLOW_CUSTOM_ERROR;
                }
            }

            sinkData->mFrameMutex.lock();

            int bufferSize = static_cast<int>(sinkData->mFrameBuffer.size());

            if( bufferSize==sinkData->mMaxBufferSize )
            {
                sinkData->mSinkStats.frameDropped++;

                switch(sinkData->mDebugLvl)
                {
                case DEBUG_NONE:
                    break;
                case DEBUG_FULL:
                case DEBUG_VERBOSE:
                    cout << "[GstSinkOpenCV] Dropped frame #" << sinkData->mSinkStats.frameDropped << endl;
                    cout << "[GstSinkOpenCV] bufferSize: " << bufferSize << "/" << sinkData->mMaxBufferSize << endl;
                case DEBUG_INFO:
                    break;
                }
            }
            else
            {
                cv::Mat frame(height,width,CV_8UC3);

                memcpy( frame.data, map.data, map.size );

                sinkData->mFrameBuffer.push( frame );

                sinkData->mSinkStats.frameCount++;

                switch(sinkData->mDebugLvl)
                {
                case DEBUG_NONE:
                    break;
                case DEBUG_FULL:
                    cv::imshow( "Sink frame", frame );
                    cv::waitKey( 1 );
                case DEBUG_VERBOSE:
                    cout << "[GstSinkOpenCV] Received frame #" << sinkData->mSinkStats.frameCount << endl;
                    cout << "[GstSinkOpenCV] bufferSize: " << bufferSize << "/" << sinkData->mMaxBufferSize << endl;
                case DEBUG_INFO:
                    break;
                }
            }

            sinkData->mFrameMutex.unlock();

            gst_buffer_unmap (buffer, &map);
        }
        gst_sample_unref (sample);


    }
    else
    {
        sinkData->mSinkStats.errorCount++;

        switch(sinkData->mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
            cout << "[GstSinkOpenCV] Error receiving frame" << endl;
        case DEBUG_INFO:
            break;
        }

        return GST_FLOW_CUSTOM_ERROR;
    }


    return GST_FLOW_OK;
}

cv::Mat GstSinkOpenCV::getLastFrame()
{
    if( !mFrameMutex.try_lock_for( std::chrono::milliseconds(mFrameTimeoutMsec) ) )
    {
        mSinkStats.timeout++;

        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
        case DEBUG_VERBOSE:
            cout << "[GstSinkOpenCV] Getting frame timeout #" << mSinkStats.timeout << endl;
        case DEBUG_INFO:
            break;
        }

        return cv::Mat();
    }

    if( mFrameBuffer.size()==0 )
    {
        switch(mDebugLvl)
        {
        case DEBUG_NONE:
            break;
        case DEBUG_FULL:
            cout << "[GstSinkOpenCV] Frame Buffer empty" << endl;
        case DEBUG_VERBOSE:
        case DEBUG_INFO:
            break;
        }

        mFrameMutex.unlock();

        return cv::Mat();
    }

    cv::Mat frame = mFrameBuffer.front();

    mFrameBuffer.pop();

    mFrameMutex.unlock();

    return frame;
}
