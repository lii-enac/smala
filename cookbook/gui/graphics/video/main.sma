use core
use base
use display
use gui

_native_code_
%{
#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"

//  brew install opencv@3
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>

#include <unistd.h> // sleep

#include <iostream>
using namespace std;
using namespace cv;
// or ibavdevice: see https://github.com/leixiaohua1020/simplest_ffmpeg_device/blob/master/simplest_ffmpeg_readcamera/simplest_ffmpeg_readcamera.cpp

void
cpp_action(Process *p)
{
    //cerr << "initing opencv" << endl<<flush;
    Mat frame;
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return;
    }

    auto w = cap.get(CAP_PROP_FRAME_WIDTH);
    auto h = cap.get(CAP_PROP_FRAME_HEIGHT);
    auto format = cap.get(CAP_PROP_CODEC_PIXEL_FORMAT);
    //std::cerr << format << __FL__;
    string frame_data;
    frame_data.reserve(w*h*3);

    get_exclusive_access(DBG_GET);
    auto * im = dynamic_cast<DataImage*>(p->get_parent()->find_child("im"));
    string*& data = im->get_data_ref();
    
    Process * update = nullptr;
    //if (im->find_layer ()) update = im->find_layer()->damaged ();
    //else
    if (im->get_frame ()) update = im->get_frame ()->damaged ();
    assert(update);
    data = &frame_data;
    
    im->width()->set_value (w, true);
    im->height()->set_value (h, true);
    im->format()->set_value(29, true); // frame is BGR888, QImage::Format_BGR888 = 29
    release_exclusive_access(DBG_REL);

    //--- GRAB AND WRITE LOOP
    //cout << "Start grabbing" << endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        
        get_exclusive_access(DBG_GET);        
        frame_data.assign((const char*)frame.ptr(), w*h*3);
        im->set_invalid_cache (true);
        update->activate();
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
        
    }
    
}

%}

_main_
Component root
{
    Frame f ("my frame", 0, 0, 800, 800)
    Exit ex (0, 1)
    f.close -> ex
    //mouseTracking = 1

    //FillColor _(255,255,0)
    //Rectangle _(0,0,10,10)

    //Scaling _(0.1,0.1,0,0)
    DataImage im(0,0,0,0)
    NativeAsyncAction na(cpp_action, root, 0)
}