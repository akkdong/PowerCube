// CubeMonitor.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "CubeMonitor.h"
#include "CubeFrame.h"


#ifndef wxHAS_IMAGES_IN_RESOURCES
#include "monitor.xpm"
#endif


///////////////////////////////////////////////////////////////////////////////////////
//

class CubeApp : public wxApp
{
public:
    virtual bool OnInit();
};



///////////////////////////////////////////////////////////////////////////////////////
//

bool CubeApp::OnInit()
{
    // call the base class initialization method, currently it only parses a
    // few common command-line options but it could be do more in the future
    if (!wxApp::OnInit())
        return false;

    // create the main application window
    CubeFrame* frame = new CubeFrame(wxT("Serial Com"));

    // and show it (the frames, unlike simple controls, are not shown when
    // created initially)
    frame->Show(true);

    // success: wxApp::OnRun() will be called which will enter the main message
    // loop and the application will run. If we returned false here, the
    // application would exit immediately.
    return true;
}


IMPLEMENT_APP(CubeApp)
