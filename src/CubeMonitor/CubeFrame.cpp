// CubeFrame.cpp
//

#include "framework.h"
#include "CubeFrame.h"



// constants
// ----------------------------------------------------------------------------

enum
{
    ID_BTNSEND = 101,
    ID_TXTSEND,
    ID_TXTRECV,

    ID_CHECK_RTS,
    ID_CHECK_DTR,
    ID_CHECK_CTS,
    ID_CHECK_DSR,
    ID_CHECK_RI,
    ID_CHECK_CD,

    ID_TIMER,

    ID_OPTION_PORT,
    ID_OPTION_BAUDRATE,
    ID_OPTION_DATASIZE,
    ID_OPTION_PARITY,
    ID_OPTION_STOPBITS,

    // predefined-id: wxID_LOWEST(4999) ~ wxID_HIGHEST(5999)
    ID_EDIT_CLEAR = wxID_CLEAR,

    ID_SERIAL_OPEN = wxID_OPEN,
    ID_SERIAL_CLOSE = wxID_CLOSE,

    ID_APP_QUIT = wxID_EXIT,

    // it is important for the id corresponding to the "About" command to have
    // this standard value as otherwise it won't be handled properly under Mac
    // (where it is special and put into the "Apple" menu)
    ID_APP_ABOUT = wxID_ABOUT
};


/////////////////////////////////////////////////////////////////////////////
//

CubeFrame::CubeFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(800, 600), wxDEFAULT_FRAME_STYLE /*/^ wxRESIZE_BORDER*/)
    , m_timer(this, ID_TIMER)
{
    //
    MakeMenuBar();
    MakeStatusBar();
    MakeWidgets();

    //
    LayoutWidgets();

    //
    m_timer.Start(250);
}


void CubeFrame::MakeMenuBar()
{
    // create a menu bar
    wxMenu* fileMenu = new wxMenu;
    fileMenu->Append(ID_SERIAL_OPEN, wxT("&Open\tAlt-O"), wxT("Open serial port"));
    fileMenu->Append(ID_SERIAL_CLOSE, wxT("&Close\tAlt-C"), wxT("Close serial port"));
    fileMenu->AppendSeparator();
    fileMenu->Append(ID_OPTION_PORT, wxT("&Serial Port\tAlt-S"), wxT("Select serial port"));
    fileMenu->Append(ID_OPTION_BAUDRATE, wxT("&Baud Rate\tAlt-B"), wxT("Set baud rate"));
    fileMenu->Append(ID_OPTION_DATASIZE, wxT("&Data Size\tAlt-D"), wxT("Set data size"));
    fileMenu->Append(ID_OPTION_PARITY, wxT("&Parity\tAlt-P"), wxT("Set parity"));
    fileMenu->Append(ID_OPTION_STOPBITS, wxT("S&top Bits\tAlt-t"), wxT("Set stop bits"));
    fileMenu->AppendSeparator();
    fileMenu->Append(ID_APP_QUIT, wxT("E&xit\tAlt-X"), wxT("Quit this program"));

    //Edit menu
    wxMenu* editMenu = new wxMenu;
    editMenu->Append(ID_EDIT_CLEAR, wxT("Clea&r\tAlt-R"), wxT("Clear text"));

    // the "About" item should be in the help menu
    wxMenu* helpMenu = new wxMenu;
    helpMenu->Append(ID_APP_ABOUT, wxT("&About\tF1"), wxT("Show about dialog"));


    // now append the freshly created menu to the menu bar...
    wxMenuBar* menuBar = new wxMenuBar();
    menuBar->Append(fileMenu, wxT("&File"));
    menuBar->Append(editMenu, wxT("&Edit"));
    menuBar->Append(helpMenu, wxT("&Help"));

    // ... and attach this menu bar to the frame
    SetMenuBar(menuBar);
}

void CubeFrame::MakeStatusBar()
{
    // create a status bar just for fun (by default with 1 pane only)
    CreateStatusBar(2);

    SetStatusText(wxT("Serial Communication"));
    SetStatusText(wxT("READY"), 1);
}

void CubeFrame::MakeWidgets()
{
    //
    wxPanel* panel = new wxPanel(this, -1);

    m_pBtnSend = new wxButton(panel, ID_BTNSEND, wxT("Send"), wxDefaultPosition, wxDefaultSize);
    m_pTextSend = new wxTextCtrl(panel, ID_TXTSEND, wxT("Hello!"), wxDefaultPosition, wxDefaultSize);
    m_pTextRecv = new wxTextCtrl(panel, ID_TXTRECV, wxT(""), wxPoint(-1, -1), wxSize(-1, -1), wxTE_MULTILINE);

    m_pCheckRTS = new wxCheckBox(panel, ID_CHECK_RTS, wxT("RTS"), wxDefaultPosition, wxDefaultSize);
    m_pCheckDTR = new wxCheckBox(panel, ID_CHECK_DTR, wxT("DTR"), wxDefaultPosition, wxDefaultSize);
    m_pCheckCTS = new wxCheckBox(panel, ID_CHECK_CTS, wxT("CTS"), wxDefaultPosition, wxDefaultSize);
    m_pCheckDSR = new wxCheckBox(panel, ID_CHECK_DSR, wxT("DSR"), wxDefaultPosition, wxDefaultSize);
    m_pCheckRI = new wxCheckBox(panel, ID_CHECK_RI, wxT("RI"), wxDefaultPosition, wxDefaultSize);
    m_pCheckCD = new wxCheckBox(panel, ID_CHECK_CD, wxT("CD"), wxDefaultPosition, wxDefaultSize);

    m_pMainPanel = panel;

    //
    Connect(ID_SERIAL_OPEN, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::OnOpen));
    Connect(ID_SERIAL_CLOSE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::OnClose));
    Connect(ID_OPTION_PORT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::SelPort));
    Connect(ID_OPTION_BAUDRATE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::SetBaud));
    Connect(ID_OPTION_DATASIZE, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::SetDataSize));
    Connect(ID_OPTION_PARITY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::SetParity));
    Connect(ID_OPTION_STOPBITS, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::SetStopBits));
    //
    Connect(ID_TIMER, wxEVT_TIMER, wxTimerEventHandler(CubeFrame::OnTimer));
    //
    Connect(ID_EDIT_CLEAR, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::ClearText));
    Connect(ID_APP_ABOUT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::OnAbout));
    Connect(ID_APP_QUIT, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(CubeFrame::OnQuit));

    Connect(ID_BTNSEND, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CubeFrame::OnSend));
    Connect(ID_CHECK_RTS, wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(CubeFrame::OnChkRTS));
    Connect(ID_CHECK_DTR, wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(CubeFrame::OnChkDTR));

    //Bind(wxEVT_MENU, &MyFrame::OnClose, this, Serial_Close);
    //Bind(wxEVT_SIZE, &MyFrame::OnSize, this, 0);

    m_pCheckCTS->Disable();
    m_pCheckDSR->Disable();
    m_pCheckRI->Disable();
    m_pCheckCD->Disable();
}


void CubeFrame::LayoutWidgets()
{
    //
    //  +----------+ +-------------------------------+
    //  |   Send   | |                               |
    //  +----------+ +-------------------------------+
    //  +--------------------------------------------+
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  |                                            |
    //  +--------------------------------------------+
    //  +----+ +----+      +----+ +----+ +----+ +----+
    //  |    | |    |      |    | |    | |    | |    |
    //  +----+ +----+      +----+ +----+ +----+ +----+
    //

    wxBoxSizer* vbox = new wxBoxSizer(wxVERTICAL);

    wxBoxSizer* hbox1 = new wxBoxSizer(wxHORIZONTAL);
    hbox1->Add(m_pBtnSend, 0, wxRIGHT, 8);
    hbox1->Add(m_pTextSend, 1);

    vbox->Add(hbox1, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    vbox->Add(-1, 8);

    wxBoxSizer* hbox2 = new wxBoxSizer(wxHORIZONTAL);
    hbox2->Add(m_pTextRecv, 1, wxEXPAND);

    vbox->Add(hbox2, 1, wxLEFT | wxRIGHT | wxEXPAND, 6);
    vbox->Add(-1, 10);

    wxBoxSizer* hbox3 = new wxBoxSizer(wxHORIZONTAL);
    hbox3->Add(m_pCheckRTS, 0, wxLEFT, 8);
    hbox3->Add(m_pCheckDTR, 0, wxLEFT, 8);
    hbox3->AddStretchSpacer(1);
    hbox3->Add(m_pCheckCTS, 0, wxRIGHT, 8);
    hbox3->Add(m_pCheckDSR, 0, wxRIGHT, 8);
    hbox3->Add(m_pCheckRI, 0, wxRIGHT, 8);
    hbox3->Add(m_pCheckCD, 0, wxRIGHT, 8);

    vbox->Add(hbox3, 0, wxEXPAND | wxBOTTOM, 6);

    m_pMainPanel->SetSizer(vbox);
    m_pMainPanel->SetAutoLayout(true);
}



void CubeFrame::OnQuit(wxCommandEvent& event)
{
    // true is to force the frame to close
    Close(true);
}

void CubeFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox(wxString::Format(
        wxT("Serial Communication! \n ")
        wxT("Author: Yan Naing Aye \n ")
        wxT("Web: https://github.m_serial/yan9a/serial")
    ),
        wxT("About Serial Comm"),
        wxOK | wxICON_INFORMATION,
        this);
}

void CubeFrame::OnTimer(wxTimerEvent& event)
{
    char ch; bool r;
    do {
        ch = m_serial.ReadChar(r);
        if (r && ch != '\r')
            ProcessChar(ch);
    } while (r);

    UpdateCommStatus();
}


void CubeFrame::OnOpen(wxCommandEvent& event)
{
    if (m_serial.Open())
        m_pTextRecv->AppendText(wxString::Format(wxT("Error opening port %s.\n"), m_serial.GetPort()));
    else
        m_pTextRecv->AppendText(wxString::Format(wxT("Port %s is opened.\n"), m_serial.GetPort()));
}

void CubeFrame::OnClose(wxCommandEvent& event)
{
    m_serial.Close();
    m_pTextRecv->AppendText(wxString::Format(wxT("Port %s is closed.\n"), m_serial.GetPort()));
}

void CubeFrame::SelPort(wxCommandEvent& event)
{
    if (m_serial.IsOpened()) 
    {
        m_pTextRecv->AppendText(wxString::Format(wxT("Close Port %s first.\n"), m_serial.GetPort()));
    }
    else 
    {
        wxString cdev = wxString::Format(wxT("%s"), m_serial.GetPort());
        wxString device = wxGetTextFromUser(wxT("Enter the port"), wxT("Set Port"), cdev);
        std::string str = device.ToStdString();
        if (str.length() > 0) 
            m_serial.SetPortName(str);

        m_pTextRecv->AppendText(wxString::Format(wxT("Port: %s\n"), m_serial.GetPort()));
    }
}

void CubeFrame::SetDataSize(wxCommandEvent& event)
{
    if (m_serial.IsOpened()) 
    {
        m_pTextRecv->AppendText(wxString::Format(wxT("Close port %s first.\n"), m_serial.GetPort()));
    }
    else 
    {
        long n = wxGetNumberFromUser(wxT("Enter the data size"), wxT("Data Size"), wxT("Set Data Size"), m_serial.GetDataSize(), 5, 8);
        if (n >= 0)
            m_serial.SetDataSize(n);

        m_pTextRecv->AppendText(wxString::Format(wxT("Data size: %ld\n"), m_serial.GetDataSize()));
    }
}

void CubeFrame::SetParity(wxCommandEvent& event)
{
    if (m_serial.IsOpened()) 
    {
        m_pTextRecv->AppendText(wxString::Format(wxT("Close Port %s first.\n"), m_serial.GetPort()));
    }
    else 
    {
        wxString cdev = wxString::Format(wxT("%c"), m_serial.GetParity());
#if defined(__WINDOWS__)
        wxString parity = wxGetTextFromUser(wxT("Enter the parity ( N, E, O, M, or S )"), wxT("Set Parity"), cdev);
#else
        wxString parity = wxGetTextFromUser(wxT("Enter the parity ( N, E, or O )"), wxT("Set Parity"), cdev);
#endif

        std::string pstr = parity.ToStdString();
        if (pstr.length() > 0)
            m_serial.SetParity(pstr.at(0));

        m_pTextRecv->AppendText(wxString::Format(wxT("Parity: %c\n"), m_serial.GetParity()));
    }
}

void CubeFrame::SetStopBits(wxCommandEvent& event)
{
    if (m_serial.IsOpened()) 
    {
        m_pTextRecv->AppendText(wxString::Format(wxT("Close port %s first.\n"), m_serial.GetPort()));
    }
    else {
        long n = wxGetNumberFromUser(wxT("Enter the number of stop bits"), wxT("Data Size"), wxT("Set stop bits"), long(m_serial.GetStopBits()), 1, 2);
        if (n > 0) 
            m_serial.SetStopBits(float(n));

        m_pTextRecv->AppendText(wxString::Format(wxT("Stop bits: %ld\n"), long(m_serial.GetStopBits())));
    }
}

void CubeFrame::SetBaud(wxCommandEvent& event)
{
    if (m_serial.IsOpened()) 
    {
        m_pTextRecv->AppendText(wxString::Format(wxT("Close port %s first.\n"), m_serial.GetPort()));
    }
    else 
    {
        long n = wxGetNumberFromUser(wxT("Enter the baud rate"), wxT("Baud rate"), wxT("Set Baud Rate"), m_serial.GetBaudRate(), 0, 1000000);
        if (n >= 0) 
            m_serial.SetBaudRate(n);

        m_pTextRecv->AppendText(wxString::Format(wxT("Baud rate: %ld\n"), m_serial.GetBaudRate()));
    }
}

void CubeFrame::OnSend(wxCommandEvent& event)
{
    wxString str = m_pTextSend->GetValue();
    str += "\r\n";
    wxCharBuffer buffer = str.ToUTF8();
    if (m_serial.Write(buffer.data())) 
        m_pTextRecv->AppendText(str);
    else 
        m_pTextRecv->AppendText(wxT("Write error.\n"));
}

void CubeFrame::ProcessChar(char ch)
{
    m_pTextRecv->AppendText(wxString::Format(wxT("%c"), ch));
}

void CubeFrame::ClearText(wxCommandEvent& event)
{
    m_pTextRecv->Clear();
}

void CubeFrame::OnChkRTS(wxCommandEvent& event)
{
    if (!m_serial.SetRTS(m_pCheckRTS->IsChecked())) 
        m_pTextRecv->AppendText(wxT("RTS error.\n"));
}

void CubeFrame::OnChkDTR(wxCommandEvent& event)
{
    if (!m_serial.SetDTR(m_pCheckDTR->IsChecked()))
        m_pTextRecv->AppendText(wxT("DTR error.\n"));
}

void CubeFrame::UpdateCommStatus()
{
    bool s;
    bool v;
    v = m_serial.GetCTS(s);
    if (s) m_pCheckCTS->SetValue(v);
    v = m_serial.GetDSR(s);
    if (s) m_pCheckDSR->SetValue(v);
    v = m_serial.GetRI(s);
    if (s) m_pCheckRI->SetValue(v);
    v = m_serial.GetCD(s);
    if (s) m_pCheckCD->SetValue(v);
}
