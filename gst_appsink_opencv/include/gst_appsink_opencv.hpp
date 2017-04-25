#include <iostream>
#include <gst/gst.h>

#include <opencv2/core/core.hpp>

#include <mutex>
#include <queue>
#include <chrono>

class GstSinkOpenCV
{
public:
    typedef enum _debug_lvl
    {
        DEBUG_NONE,
        DEBUG_INFO,
        DEBUG_VERBOSE,
        DEBUG_FULL
    } DebugLevel;

    typedef struct _sink_stats
    {
        int frameCount;
        int frameDropped;
        int errorCount;
        int timeout;
        double fps;
    } SinkStats;

    static GstSinkOpenCV* Create(std::string input_pipeline , int bufferSize=3, int timeout_sec=15, DebugLevel debugLvl=DEBUG_NONE );
    ~GstSinkOpenCV();

    cv::Mat getLastFrame();

private:
    GstSinkOpenCV(std::string input_pipeline, int bufferSize, DebugLevel debugLvl );
    bool init(int timeout_sec);

    static GstFlowReturn on_new_sample_from_sink(GstElement* elt, GstSinkOpenCV* sinkData );

protected:

private:
    std::string mPipelineStr;

    GstElement* mPipeline;
    GstElement* mSink;

    //std::vector<cv::Mat> mFrameBuffer;
    std::queue<cv::Mat> mFrameBuffer;

    int mWidth;
    int mHeight;
    int mChannels;

    int mMaxBufferSize;

    DebugLevel mDebugLvl;
    SinkStats mSinkStats;

    std::timed_mutex mFrameMutex;

    int mFrameTimeoutMsec;
};
