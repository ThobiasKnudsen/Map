#pragma once

#include <wx/wx.h>
#include <GL/glew.h> // må inkluderes før glcanvas
#include <wx/glcanvas.h>
#include <wx/display.h>
#include <GL/gl.h>
#include <fmt/core.h>
#include <sstream>
#include <iomanip>

#include "mapAPI.hpp"
#include "DBG.hpp"


class MapRenderer {
//funksjoner
public: 
	MapRenderer(wxFrame* parentFrame) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		InitializePanel(parentFrame);
		InitializeGlCanvas();
		InitializeGlContext();
		InitializeOpenGL();
		InitializeMap();
		InitializeTexture();

		m_glCanvas->Bind(wxEVT_PAINT, &this->OnPaint, this);
		m_glCanvas->Bind(wxEVT_SIZE, &this->OnSize, this);

		m_glCanvas->Bind(wxEVT_LEFT_DOWN, &this->OnMouseDown, this);
		m_glCanvas->Bind(wxEVT_MOTION, &this->OnMouseMove, this);
		m_glCanvas->Bind(wxEVT_MOUSEWHEEL, &this->OnMouseWheel, this);

		DBG::note(__LINE__, "OpenGLCanvasMap done");
	}
    ~MapRenderer();
	void UpdateTexture(const unsigned char* imageData, int imageWidth, int imageHeight, int nrChannels) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (m_textureID == 0) {
			DBG::error(__LINE__, "m_textureID not initialized");
		}
		if (!imageData) {
			DBG::error(__LINE__, "imageData not initialized");
		}
		if (imageWidth < 0 or imageWidth>=10000) {
			DBG::error(__LINE__, fmt::format("imageWidth < 0 or imageWidth>=10000. imageWidth={}", imageWidth));
		}
		if (imageHeight < 0 or imageHeight>=10000) {
			DBG::error(__LINE__, fmt::format("imageHeight < 0 or imageHeight>=10000. imageHeight={}", imageHeight));
		}
		if (nrChannels!=3 and nrChannels!=4) {
			DBG::error(__LINE__, "nrChannels is not 3 or 4");
		}

		// checks if given imageWidth and imageHeight is larger than current texture.
		// if so it resizes the texture
		int lastTextureWidth, lastTextureHeight;
		this->GetTextureSize(&lastTextureWidth, &lastTextureHeight);
		int maxTextureWidth = std::max(lastTextureWidth, imageWidth);
		int maxTextureHeight = std::max(lastTextureHeight, imageHeight);

		glBindTexture(GL_TEXTURE_2D, m_textureID);																			debug(__LINE__);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);																				debug(__LINE__);
		// new texture and write imageData
		if (maxTextureWidth > lastTextureWidth or maxTextureHeight > lastTextureHeight) {
			// nrChannels==3
			if (nrChannels == 3) {
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData);	debug(__LINE__);
				glGenerateMipmap(GL_TEXTURE_2D);																			debug(__LINE__);
			}
			// nrChannels==4
			else {
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageWidth, imageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData);	debug(__LINE__);
				glGenerateMipmap(GL_TEXTURE_2D);																			debug(__LINE__);
			}
			DBG::note(__LINE__, fmt::format("texture resized ({}, {}). previously ({}, {})",
											maxTextureWidth,
											maxTextureHeight,
											lastTextureWidth,
											lastTextureHeight));
		}
		// write imageData
		else {
			// nrChannels==3
			if (nrChannels == 3) {
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth, imageHeight, GL_RGB, GL_UNSIGNED_BYTE, imageData);	debug(__LINE__);
				glGenerateMipmap(GL_TEXTURE_2D);																		debug(__LINE__);
			}
			// nrChannels==4
			else {
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, imageWidth, imageHeight, GL_RGBA, GL_UNSIGNED_BYTE, imageData);	debug(__LINE__);
				glGenerateMipmap(GL_TEXTURE_2D);																		debug(__LINE__);
			}
		}
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);																			debug(__LINE__);
		glBindTexture(GL_TEXTURE_2D, 0);																				debug(__LINE__);

		// setting veritces
		float x2 = static_cast<float>(imageWidth)/static_cast<float>(maxTextureWidth);
		float y2 = static_cast<float>(imageHeight)/static_cast<float>(maxTextureHeight);
		DBG::note(__LINE__, fmt::format("x2={} y2={}", x2, y2));
		float vertices[] = {
			// Positions    // Texture Coords
			-1.0f, -1.0f,    0.0f, y2, // Bottom left
			 1.0f, -1.0f,    x2,   y2, // Bottom right
			-1.0f,  1.0f,    0.0f, 0.0f, // Top left
			 1.0f,  1.0f,    x2,   0.0f  // Top right
		};
		glBindVertexArray(m_VAO);																						debug(__LINE__);
		glBindBuffer(GL_ARRAY_BUFFER, m_VBO);																			debug(__LINE__);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);										debug(__LINE__);
		glBindVertexArray(0);																							debug(__LINE__);
	}
	void GetTextureSize(int* returnWidth, int* returnHeight) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		glBindTexture(GL_TEXTURE_2D, m_textureID);debug(__LINE__);
		if (returnWidth) {
			glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, returnWidth);debug(__LINE__);
		}
		if (returnHeight) {
			glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, returnHeight);debug(__LINE__);
		}
		glBindTexture(GL_TEXTURE_2D, 0);debug(__LINE__);
	}
private:
    void OnPaint(wxPaintEvent &event) {
		(void)event;
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (!m_glContext->IsOK()) {
			return; // OpenGL context not OK, skip rendering
		}

		m_glCanvas->SetCurrent(*m_glContext);

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glUseProgram(m_shaderProgram);

		glBindTexture(GL_TEXTURE_2D, m_textureID);
		glBindVertexArray(m_VAO);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
		glBindTexture(GL_TEXTURE_2D, 0);

		m_glCanvas->SwapBuffers();
	}
    void OnSize(wxSizeEvent &event) {
		(void)event;
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		int winWidth, winHeight;
		m_panel->GetSize(&winWidth, &winHeight);
		DBG::note(__LINE__, fmt::format("winWidth={} winHeight={}", winWidth, winHeight));
		DBG::note(__LINE__, fmt::format("map data folder path: {}", m_map->dataFolderPath.c_str()));
		DBG::note(__LINE__, fmt::format("map metersView ({} {})", m_map->metersWidth, m_map->metersHeight));
		DBG::note(__LINE__, fmt::format("map winSize ({} {})", m_map->winWidth, m_map->winHeight));

		this->m_map->updateSize(winWidth, winHeight);
		this->rgbArrayWidth = winWidth;
		this->rgbArrayHeight = winHeight;
		this->rgbArray.resize(3*winWidth*winHeight);
		this->m_map->writeToRGBarrayPtr(&this->rgbArray[0], this->rgbArrayWidth, this->rgbArrayHeight, m_lastX, m_lastY);
		this->UpdateTexture(&this->rgbArray[0], this->rgbArrayWidth, this->rgbArrayHeight, 3);
		auto viewPortSize = m_panel->GetSize() * m_glCanvas->GetContentScaleFactor();
		glViewport(0, 0, viewPortSize.x, viewPortSize.y);debug(__LINE__);
		
		DBG::note(__LINE__, fmt::format("rgbArray size = {}", this->rgbArray.size()));
		
		DBG::print();

		event.Skip();
	}
	void OnMouseDown(wxMouseEvent& event) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		m_panel->SetFocus();
		event.GetPosition(&m_lastX, &m_lastY);
		event.Skip();
	}
	void OnMouseMove(wxMouseEvent& event) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (oddMouseMoveEvent) {
			oddMouseMoveEvent=false;
			return;
		}

		m_panel->SetFocus();

		int currX, currY;
		event.GetPosition(&currX, &currY);

		//wxSize totalSize = m_panel->GetSize();
		//wxSize clientSize = m_panel->GetClientSize();
		//fmt::print("tot({}, {}) client({}, {}) mouse()\n", totalSize.GetWidth(), totalSize.GetHeight(), clientSize.GetWidth(), clientSize.GetHeight());

		if (event.Dragging() && event.LeftIsDown()) {

			int deltaX = currX-m_lastX;
			int deltaY = currY-m_lastY;

			m_map->moveView(deltaX, deltaY);
			m_map->writeToRGBarrayPtr(&this->rgbArray[0], this->rgbArrayWidth, this->rgbArrayHeight, currX, currY);
			int winWidth, winHeight;
			m_panel->GetSize(&winWidth, &winHeight);
			this->UpdateTexture(&this->rgbArray[0], winWidth, winHeight, 3);
			m_glCanvas->Refresh();
			
			DBG::note(__LINE__, fmt::format("rgbArray size = {}", this->rgbArray.size()));
		}
		// hovering
		else {
			m_map->writeToRGBarrayPtr(&this->rgbArray[0], this->rgbArrayWidth, this->rgbArrayHeight, currX, currY);
			int winWidth, winHeight;
			m_panel->GetSize(&winWidth, &winHeight);
			this->UpdateTexture(&this->rgbArray[0], winWidth, winHeight, 3);
			m_glCanvas->Refresh();
		}

		m_lastX = currX;
		m_lastY = currY;
		
		Math::Point latLon = m_map->getLatLonForPixel(currX, currY);
		this->mousePosMGRS = MGRS::latLonToMGRS(latLon.x, latLon.y);
		int precision = 6;

		std::ostringstream stream;
		stream << std::fixed << std::setprecision(precision) << latLon.x << "," << latLon.y;
		this->mousePosLatLon = stream.str();
		
		int index = currX + m_map->winWidth*currY;
		double elevation = static_cast<float>(m_map->DTM[index])*(1024.f/65536.f);
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(1);
		oss << std::setw(6) << std::setfill('0') << elevation;
		this->elevationMeters = oss.str()+"m";
		
		oddMouseMoveEvent=true;

		event.Skip();
	}
	void OnMouseWheel(wxMouseEvent& event) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		if (oddMouseWheelEvent) {
			oddMouseWheelEvent=false;
			return;
		}
		m_panel->SetFocus();
		static float zoomFactor = 2;
		int currX, currY;
		event.GetPosition(&currX, &currY);

		m_map->zoom(currX, currY, event.GetWheelRotation() > 0 ? zoomFactor : 1.0f/zoomFactor);
		m_map->writeToRGBarrayPtr(&this->rgbArray[0], this->rgbArrayWidth, this->rgbArrayHeight, currX, currY);
		int winWidth, winHeight;
		m_panel->GetSize(&winWidth, &winHeight);
		this->UpdateTexture(&this->rgbArray[0], winWidth, winHeight, 3);
		m_glCanvas->Refresh();

		DBG::note(__LINE__, fmt::format("rgbArray size = {}", this->rgbArray.size()));
		oddMouseWheelEvent=true;
		event.Skip();
	}
	void InitializePanel(wxFrame* parentFrame) {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (!parentFrame) {
			DBG::error(__LINE__, "parentFrame is not initialized");
		}

		m_panel = new wxPanel(parentFrame, wxID_ANY);
		if (!m_panel) {
			DBG::error(__LINE__, "failed to initialize m_panel");
		}

		DBG::note(__LINE__, "InitializePanel done");
	}
	void InitializeGlCanvas() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (!m_panel) {
			DBG::error(__LINE__, "this->panel is not initialized");
		}

		wxGLAttributes vAttrs;
		vAttrs.PlatformDefaults().Defaults().EndList();

		if (!wxGLCanvas::IsDisplaySupported(vAttrs)) {
			DBG::error(__LINE__,  "wxGLCanvas does not support display");
		}

		m_glCanvas = new wxGLCanvas(m_panel, vAttrs);

		// Set a sizer for the panel to manage the OpenGL canvas
		wxBoxSizer* panelSizer = new wxBoxSizer(wxVERTICAL);
		panelSizer->Add(m_glCanvas, 1, wxEXPAND); // Fill the panel entirely
		m_panel->SetSizer(panelSizer);
		m_panel->Layout();

		DBG::note(__LINE__, "InitializeGlCanvas done");
	}
	void InitializeGlContext() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (!m_glCanvas) {
			DBG::error(__LINE__, "this->glCanvas is not initialized");
		}

		wxGLContextAttrs ctxAttrs;
		ctxAttrs.PlatformDefaults().CoreProfile().OGLVersion(3, 3).EndList();
		m_glContext = new wxGLContext(m_glCanvas, nullptr, &ctxAttrs);

		if (!m_glContext->IsOK())
		{
			delete m_glContext; // Corrected variable name
			m_glContext = nullptr;
			DBG::error(__LINE__, "OpenGL version error");
		}

		DBG::note(__LINE__, "InitializeGlContext done");
	}
	void InitializeOpenGL() {
	DBG::Scope scope(__LINE__, __func__, __FILE__);

    if (!m_glContext) {
        DBG::error(__LINE__, "this->glContext is not initialized");
	}

    m_glCanvas->SetCurrent(*m_glContext);

    GLenum err = glewInit();debug(__LINE__);
    if (GLEW_OK != err) {
        DBG::error(__LINE__, fmt::format("OpenGL GLEW initialization failed: %s", reinterpret_cast<const char *>(glewGetErrorString(err))));
	}

    printf("Status: Using GLEW %s\n", reinterpret_cast<const char *>(glewGetString(GLEW_VERSION)));
    printf("OpenGL version: %s\n", reinterpret_cast<const char *>(glGetString(GL_VERSION)));
    printf("OpenGL vendor: %s\n", reinterpret_cast<const char *>(glGetString(GL_VENDOR)));

    constexpr auto vertexShaderSource = R"(
        #version 330 core
		layout (location = 0) in vec2 aPos;
		layout (location = 1) in vec2 aTexCoord;

		out vec2 TexCoord;

		void main()
		{
			gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
			TexCoord = aTexCoord;
		}
    )";

    constexpr auto fragmentShaderSource = R"(
        #version 330 core
		out vec4 FragColor; 

		in vec2 TexCoord;

		uniform sampler2D texture1;

		void main()
		{
			FragColor = texture(texture1, TexCoord);
		}
    )";

    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);debug(__LINE__);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);debug(__LINE__);
    glCompileShader(vertexShader);debug(__LINE__);

    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);debug(__LINE__);

    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);debug(__LINE__);
        DBG::error(__LINE__, fmt::format("Vertex Shader Compilation Failed: {}", infoLog));
	}

    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);debug(__LINE__);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);debug(__LINE__);
    glCompileShader(fragmentShader);debug(__LINE__);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);debug(__LINE__);

    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);debug(__LINE__);
		DBG::error(__LINE__, fmt::format("Fragment Shader Compilation Failed: {}", infoLog));
	}

    m_shaderProgram = glCreateProgram();debug(__LINE__);
    glAttachShader(m_shaderProgram, vertexShader);debug(__LINE__);
    glAttachShader(m_shaderProgram, fragmentShader);debug(__LINE__);
    glLinkProgram(m_shaderProgram);debug(__LINE__);

    glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);debug(__LINE__);

    if (!success) {
        glGetProgramInfoLog(m_shaderProgram, 512, nullptr, infoLog);debug(__LINE__);
		DBG::error(__LINE__, fmt::format("Shader Program Linking Failed: %s", infoLog));
    }

    glDeleteShader(vertexShader);debug(__LINE__);
    glDeleteShader(fragmentShader);debug(__LINE__);

    float vertices[] = {
		// Positions    // Texture Coords
		-1.0f, -1.0f,    0.0f, 1.0f, // Bottom left
		 1.0f, -1.0f,    1.0f, 1.0f, // Bottom right
		-1.0f,  1.0f,    0.0f, 0.0f, // Top left
		 1.0f,  1.0f,    1.0f, 0.0f  // Top right
	};

	unsigned int indices[] = {
		0, 1, 2, // First triangle
		1, 3, 2  // Second triangle
	};

	glGenVertexArrays(1, &m_VAO);debug(__LINE__);
	glGenBuffers(1, &m_VBO);debug(__LINE__);
	glGenBuffers(1, &m_EBO);debug(__LINE__);

	glBindVertexArray(m_VAO);debug(__LINE__);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);debug(__LINE__);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);debug(__LINE__);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);debug(__LINE__);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);debug(__LINE__);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);debug(__LINE__);
	glEnableVertexAttribArray(0);debug(__LINE__);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));debug(__LINE__);
	glEnableVertexAttribArray(1);debug(__LINE__);

	// Note: Don't unbind the EBO while the VAO is active as the EBO is stored in the VAO; You can unbind the VBO though
	glBindBuffer(GL_ARRAY_BUFFER, 0);debug(__LINE__);

	// Unbind VAO for now to avoid accidental modifications to it.
	glBindVertexArray(0);debug(__LINE__);

	DBG::note(__LINE__, "InitializeOpenGL done");
}
	void InitializeMap() {
	DBG::Scope scope(__LINE__, __func__, __FILE__);

	if (!m_panel) {
		DBG::error(__LINE__, "m_panel is not initialized");
	}

	m_map = new Map("C:/dev/GeoTIFF/test/bin", 69.720420, 29.908827, 2000, 1000);
	m_map->updateSize(2000, 1000);
	m_map->mapType=1;
	
	this->rgbArrayWidth = 2000;
	this->rgbArrayHeight = 1000;
	this->rgbArray.resize(3*2000*1000);

	DBG::note(__LINE__, "InitializeMap done");
}
	void InitializeTexture() {
		DBG::Scope scope(__LINE__, __func__, __FILE__);

		if (m_VAO==0 or m_VBO==0 or m_EBO==0 or m_shaderProgram==0) {
			DBG::error(__LINE__, "openGL not initialized");
		}

		if (!m_map) {
			DBG::error(__LINE__, "map is not initlialized");
		}

		glGenTextures(1, &m_textureID);debug(__LINE__);
		glBindTexture(GL_TEXTURE_2D, m_textureID);debug(__LINE__);

		// Set the texture wrapping parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);debug(__LINE__);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);debug(__LINE__);

		// Set texture filtering parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);debug(__LINE__);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);debug(__LINE__);

		glBindTexture(GL_TEXTURE_2D, 0);debug(__LINE__);


		UpdateTexture(&this->rgbArray[0], m_map->winWidth, m_map->winHeight, 3);

		DBG::note(__LINE__, "InitializeTexture done");
	}
	void debug(int line)  {
		DBG::Scope scope(__LINE__, __func__, __FILE__);
		DBG::note(line, "calls debug()");
		m_glCanvas->SetCurrent(*m_glContext);

		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR) {
			const char* errorStr = "";
			switch (err) {
				case GL_INVALID_ENUM:
					errorStr = "GL_INVALID_ENUM";
					break;
				case GL_INVALID_VALUE:
					errorStr = "GL_INVALID_VALUE";
					break;
				case GL_INVALID_OPERATION:
					errorStr = "GL_INVALID_OPERATION";
					break;
				case GL_STACK_OVERFLOW:
					errorStr = "GL_STACK_OVERFLOW";
					break;
				case GL_STACK_UNDERFLOW:
					errorStr = "GL_STACK_UNDERFLOW";
					break;
				case GL_OUT_OF_MEMORY:
					errorStr = "GL_OUT_OF_MEMORY";
					break;
				default:
					errorStr = "Unknown Error";
					break;
			}
			DBG::error(__LINE__, std::string(errorStr));
		}
	}
    
	
// variabler
public:
	int 			m_lastX, m_lastY;
	std::string 	mousePosLatLon="";
	std::string 	mousePosMGRS="";
	std::string 	elevationMeters="";
	wxPanel*    	m_panel{nullptr};
	Map* 			m_map{nullptr};
private:
	wxGLCanvas* 	m_glCanvas{nullptr};
	wxGLContext* 	m_glContext{nullptr};
	unsigned int 	m_VAO=0, m_VBO=0, m_EBO=0, m_shaderProgram=0;
	std::vector<unsigned char> 	rgbArray;
	int							rgbArrayWidth;
	int 						rgbArrayHeight;
	unsigned int 				m_textureID=0;
	
	bool oddMouseWheelEvent=false;
	bool oddMouseMoveEvent=false;
};