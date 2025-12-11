// CubeFrame.h
//

#pragma once

#include "ceserial.h"


/////////////////////////////////////////////////////////////////////////////
//

class CubeFrame : public wxFrame
{
public:
	CubeFrame(const wxString& title);


public:
    // event handlers (these functions should _not_ be virtual)
    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    void OnOpen(wxCommandEvent& event);
    void OnClose(wxCommandEvent& event);
    void SelPort(wxCommandEvent& event);
    void SetDataSize(wxCommandEvent& event);
    void SetParity(wxCommandEvent& event);
    void SetStopBits(wxCommandEvent& event);
    void SetBaud(wxCommandEvent& event);
    void OnSend(wxCommandEvent& event);
    void OnTimer(wxTimerEvent& event);
    void ProcessChar(char ch);
    void ClearText(wxCommandEvent& event);
    void OnChkRTS(wxCommandEvent& event);
    void OnChkDTR(wxCommandEvent& event);
    void UpdateCommStatus();

protected:
    void MakeMenuBar();
    void MakeStatusBar();
    void MakeWidgets();

    void LayoutWidgets();

protected:
    wxPanel* m_pMainPanel;
    wxButton* m_pBtnSend;
    wxTextCtrl* m_pTextSend;
    wxTextCtrl* m_pTextRecv;

    wxCheckBox* m_pCheckRTS;
    wxCheckBox* m_pCheckDTR;
    wxCheckBox* m_pCheckCTS;
    wxCheckBox* m_pCheckDSR;
    wxCheckBox* m_pCheckRI;
    wxCheckBox* m_pCheckCD;

    //
    wxTimer m_timer;
    ceSerial m_serial;

};