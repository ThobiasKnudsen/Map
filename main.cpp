#include <wx/wx.h>
#include "mapRenderer.hpp"
#include "DBG.hpp"
#include <fmt/core.h>

// vanlige main.cpp filer har en main() funksjon som starter programmet
// denne main.cpp filen har ikke main() funksjonen fordi wxWidgets fra #include <wx/wx.h> starter programmet fra App.OnInit()
// all koden her er hvordan GUI-en ser ut. GUI=graphical user interface, 
// altså GUI er hvordan appen ser ut og hvordan brukeren kan interagere med appen gjennom, for eksempel, knapper og tekst felt


// her vises ikke innholdet i funksjonene. det vises sener i MainFrame::MainFrame, 
// MainFrame::OnEditMapButtonClicked og MainFrame::OnUpdateControls
class MainFrame : public wxFrame{
// funksjoner
public:
						MainFrame(const wxString &title);
private:
	void 				UpdateControls();
	void 				OnEditMapButtonClicked(wxCommandEvent& event);
	void				OnTimer(wxTimerEvent& event);
	
	void 				OnTextEnterMousePosLatLon(wxCommandEvent& event);
	void 				OnTextEnterMousePosMGRS(wxCommandEvent& event);
	void 				OnTextKillFocusMousePosLatLon(wxFocusEvent& event);
	void 				OnTextKillFocusMousePosMGRS(wxFocusEvent& event);
//variabler
public:
	MapRenderer* 	    mapRenderer;
private:
	wxPanel*         	topPanel;
	wxTextCtrl* 		mousePosLatLon;
    wxTextCtrl* 		mousePosMGRS;
	wxTextCtrl* 		elevationMeters;
	wxTimer 			m_timer;
};

// SettingsDialog er det som vises når du trykker på "settings" øverst til høyre i appen.
// da popper det opp et vindu der du kan endre:
// 1. hvordan kartet ser ut
// 2. file mappen der dataen er
// 3. vis/ikke vis synlig område for musen, meter over bakken for musen og meter over bakken der musen ser
class SettingsDialog : public wxDialog {
public:
    SettingsDialog(MainFrame* parent, const wxString& title)
        : wxDialog(parent, wxID_ANY, title, wxDefaultPosition, wxSize(-1, -1)), mainFrame(parent) {
        wxPanel* panel = new wxPanel(this);
        wxBoxSizer* vbox = new wxBoxSizer(wxVERTICAL); 

        // Map type setting
        wxBoxSizer* hbox = new wxBoxSizer(wxHORIZONTAL);
        wxStaticText* label = new wxStaticText(panel, wxID_ANY, wxT("Angi kart type (0-1):"));
        hbox->Add(label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 10);
        mapType = new wxTextCtrl(panel, wxID_ANY, fmt::format("{}", mainFrame->mapRenderer->m_map->mapType), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
        mapType->Bind(wxEVT_TEXT_ENTER, &SettingsDialog::OnTextEnterMapType, this);
        hbox->Add(mapType, 1, wxALL | wxALIGN_CENTER_VERTICAL, 10); 
        vbox->Add(hbox, 0, wxEXPAND | wxALL, 10);

        // Folder path setting
        wxBoxSizer* hbox2 = new wxBoxSizer(wxHORIZONTAL);
        wxStaticText* label2 = new wxStaticText(panel, wxID_ANY, wxT("Specify folder path:"));
        hbox2->Add(label2, 0, wxALL | wxALIGN_CENTER_VERTICAL, 10);
        folderPath = new wxTextCtrl(panel, wxID_ANY, fmt::format("{}", mainFrame->mapRenderer->m_map->dataFolderPath), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
		folderPath->Bind(wxEVT_TEXT_ENTER, &SettingsDialog::OnTextEnterFolderPath, this);
		hbox2->Add(folderPath, 1, wxALL | wxALIGN_CENTER_VERTICAL, 10);
		vbox->Add(hbox2, 0, wxEXPAND | wxALL, 10);

        panel->SetSizer(vbox);  
    }

private:
    MainFrame* mainFrame;  // Pointer to MainFrame
    wxTextCtrl* mapType;
    wxTextCtrl* folderPath;  // New text control for the folder path

    void OnTextEnterMapType(wxCommandEvent& event) {
        wxTextCtrl* ctrl = dynamic_cast<wxTextCtrl*>(event.GetEventObject());
        if (ctrl == mapType) {
            long value;
            if (!mapType->GetValue().ToLong(&value) || value < 0 || value > 4) {
				wxMessageBox("given value is not valid: " + ctrl->GetValue()); 
                mapType->SetValue("0");
                mainFrame->mapRenderer->m_map->mapType = 0;
            } else {
                mapType->SetValue(wxString::Format("%ld", value));
                mainFrame->mapRenderer->m_map->mapType = value;
            }
            mainFrame->mapRenderer->m_map->terrainAndTreeDataToRGBarray();
        }
    }
	void OnTextKillFocusMapType(wxFocusEvent& event) {
        wxTextCtrl* ctrl = dynamic_cast<wxTextCtrl*>(event.GetEventObject());
        if (ctrl == mapType) {
            long value;
            if (!mapType->GetValue().ToLong(&value) || value < 0 || value > 4) {
				wxMessageBox("given value is not valid: " + ctrl->GetValue()); 
                mapType->SetValue("0");
                mainFrame->mapRenderer->m_map->mapType = 0;
            } else {
                mapType->SetValue(wxString::Format("%ld", value));
                mainFrame->mapRenderer->m_map->mapType = value;
            }
            mainFrame->mapRenderer->m_map->terrainAndTreeDataToRGBarray();
        }
		event.Skip();
    }
	
	void OnTextEnterFolderPath(wxCommandEvent& event) {
		wxTextCtrl* ctrl = dynamic_cast<wxTextCtrl*>(event.GetEventObject());
		if (!mainFrame->mapRenderer->m_map->setDataFolderPath(string(ctrl->GetValue()))) {
			wxMessageBox("given data folder path is not valid: " + ctrl->GetValue());  
			folderPath->SetValue(fmt::format("{}", mainFrame->mapRenderer->m_map->dataFolderPath));
			return;
		}
		folderPath->SetValue(ctrl->GetValue());
    }
	void OnTextKillFocusFolderPath(wxFocusEvent& event) {
		wxTextCtrl* ctrl = dynamic_cast<wxTextCtrl*>(event.GetEventObject());
		if (!mainFrame->mapRenderer->m_map->setDataFolderPath(string(ctrl->GetValue()))) {
			wxMessageBox("given data folder path is not valid: " + ctrl->GetValue());  
			folderPath->SetValue(fmt::format("{}", mainFrame->mapRenderer->m_map->dataFolderPath));
			return;
		}
		folderPath->SetValue(ctrl->GetValue());
		event.Skip(); 
	}
};

MainFrame::MainFrame(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxDefaultSize) {
    DBG::Scope scope(__LINE__, __func__, __FILE__);
	
	m_timer.Start(100);
	m_timer.Bind(wxEVT_TIMER, &MainFrame::OnTimer, this);
	
	SetClientSize(800, 600);
    Center();

    topPanel = new wxPanel(this, wxID_ANY);
	topPanel->SetBackgroundColour(wxColor(220, 220, 220));
    wxButton* editMapButton = new wxButton(topPanel, wxID_ANY, "Settings");
    
    mousePosLatLon = new wxTextCtrl(topPanel, wxID_ANY, "1234567890123456789", wxDefaultPosition, wxSize(190, -1), wxTE_PROCESS_ENTER);
    mousePosMGRS = new wxTextCtrl(topPanel, wxID_ANY,   "00XXX1234512345", wxDefaultPosition, wxSize(170, -1), wxTE_PROCESS_ENTER);
	elevationMeters = new wxTextCtrl(topPanel, wxID_ANY, "0000.0m", wxDefaultPosition, wxSize(90, -1), wxTE_READONLY);

    mapRenderer = new MapRenderer(this);

    wxBoxSizer* topSizer = new wxBoxSizer(wxHORIZONTAL);
	
	topSizer->Add(editMapButton, 1, wxALL | wxEXPAND, 3);
	topSizer->Add(mousePosLatLon, 1, wxALL | wxEXPAND, 3);
	topSizer->Add(mousePosMGRS, 1, wxALL | wxEXPAND, 3);
	topSizer->Add(elevationMeters, 1, wxALL | wxEXPAND, 3);

	topPanel->SetSizer(topSizer);

    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);
    mainSizer->Add(topPanel, 0, wxALIGN_LEFT);
    mainSizer->Add(mapRenderer->m_panel, 1, wxEXPAND);

    SetSizer(mainSizer);
    Layout();

    this->Show(true);
	
	editMapButton->Bind(wxEVT_BUTTON, &MainFrame::OnEditMapButtonClicked, this);
	
	mousePosLatLon->Bind(wxEVT_TEXT_ENTER, &MainFrame::OnTextEnterMousePosLatLon, this);
    mousePosLatLon->Bind(wxEVT_KILL_FOCUS, &MainFrame::OnTextKillFocusMousePosLatLon, this);
	mousePosMGRS->Bind(wxEVT_TEXT_ENTER, &MainFrame::OnTextEnterMousePosMGRS, this);
    mousePosMGRS->Bind(wxEVT_KILL_FOCUS, &MainFrame::OnTextKillFocusMousePosMGRS, this);
	
}
void MainFrame::OnEditMapButtonClicked(wxCommandEvent& event) {
    (void)event;
	SettingsDialog dlg(this, wxT("Settings"));
	dlg.ShowModal();
}
void MainFrame::UpdateControls() {
	mousePosLatLon->SetValue(this->mapRenderer->mousePosLatLon);
    mousePosMGRS->SetValue(this->mapRenderer->mousePosMGRS);
	elevationMeters->SetValue(this->mapRenderer->elevationMeters);
}
void MainFrame::OnTimer(wxTimerEvent& event) {
	(void)event;
	MainFrame::UpdateControls();
}
void MainFrame::OnTextEnterMousePosLatLon(wxCommandEvent& event) {
    wxString text = mousePosLatLon->GetValue();
		wxMessageBox("given data folder path is not valid: " + text);  
}
void MainFrame::OnTextKillFocusMousePosLatLon(wxFocusEvent& event) {
    wxString text = mousePosLatLon->GetValue();
		wxMessageBox("given data folder path is not valid: " + text);  
    event.Skip(); 
}
void MainFrame::OnTextEnterMousePosMGRS(wxCommandEvent& event) {
    wxString text = mousePosMGRS->GetValue();
		wxMessageBox("given data folder path is not valid: " + text);  
}
void MainFrame::OnTextKillFocusMousePosMGRS(wxFocusEvent& event) {
    wxString text = mousePosMGRS->GetValue();
		wxMessageBox("given data folder path is not valid: " + text);  
    event.Skip(); 
}

class App : public wxApp {
public:
	App() {}
    bool OnInit() {
		DBG::init();
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		MainFrame* mainFrame = new MainFrame("milMap");
		return true;
	}
};

// må være her for å få appen til å funke
wxIMPLEMENT_APP(App);