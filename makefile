# Compiler
CXX=C:/dev/mingw64/bin/g++

# Compiler flags
CXXFLAGS=-DNOMINMAX -D_CONSOLE -fopenmp -O2 -s -Wall -Wextra -Wshadow -Wpedantic -mfpmath=sse -msse -msse2 -DGLEW_STATIC
# Additional flags for profiling
PROFILE_GENERATE_FLAGS=-fprofile-generate
PROFILE_USE_FLAGS=-fprofile-use

# Include directories
INCLUDES=-IC:/dev/cppLibraries/fmt-master/include \
		 -IC:/dev/cppLibraries/geographiclib/include \
         -IC:/dev/cppLibraries/wxWidgets-3.2.4/include \
         -IC:/dev/cppLibraries/wxWidgets-3.2.4/lib/gcc_lib/mswu \
		 -IC:/dev/cppLibraries/GLEW/include

# Libraries and library paths
LDFLAGS=-LC:/dev/cppLibraries/geographiclib/lib \
        -LC:/dev/cppLibraries/gdal/lib \
        -LC:/dev/cppLibraries/SQLite/lib \
        -LC:/dev/cppLibraries/fmt-master/lib \
        -LC:/dev/cppLibraries/epoxy/lib \
        -LC:/dev/cppLibraries/wxWidgets-3.2.4/lib/gcc_lib \
		-LC:/dev/cppLibraries/GLEW/lib

LDLIBS=-lGeographicLib -lgomp -lfmt -lepoxy \
       -lwxmsw32u_core -lwxbase32u -lwxmsw32u_gl -lwxtiff -lwxjpeg -lwxpng -lwxzlib -lwxregexu -lwxexpat \
       -lole32 -loleaut32 -luuid -luxtheme -luser32 -lcomdlg32 -lcomctl32 -loleacc -lwinspool -lshlwapi -lversion \
	   -lglew32 -lopengl32 -lwinmm -lgdi32
       
       # -lwxbase32u -lwxbase32u_net -lwxbase32u_xml -lwxexpat -lwxjpeg -lwxmsw32u_adv -lwxmsw32u_aui -lwxmsw32u_core -lwxmsw32u_gl -lwxmsw32u_html -lwxmsw32u_media -lwxmsw32u_propgrid -lwxmsw32u_ribbon -lwxmsw32u_richtext -lwxmsw32u_stc -lwxmsw32u_webview -lwxmsw32u_xrc -lwxpng -lwxregexu -lwxscintilla -lwxtiff -lwxzlib
       #-lwxregexu -lwxscintilla -lwxtiff -lwxzlib -lwxexpat -lwxjpeg -lwxlexilla \
       #-lgdi32 -lcomctl32 -luxtheme -lole32 -luuid -lrpcrt4 -lcomdlg32 -loleaut32 \
       #-lshlwapi -lversion -lwinspool

# Output binary
TARGET=main.exe

# Object files
OBJS=main.o DBG.o

# Resource file
RESOURCE=application.res

# Default target
all: $(TARGET)

$(TARGET): $(OBJS) $(RESOURCE)
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

# Resource compilation
$(RESOURCE): application.rc
	windres application.rc -O coff -o $(RESOURCE)

# Compile with profile generate
profile-generate: CXXFLAGS += $(PROFILE_GENERATE_FLAGS)
profile-generate: clean $(TARGET)

# Use generated profile to compile
profile-use: CXXFLAGS += $(PROFILE_USE_FLAGS)
profile-use: clean $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean target
clean:
	rm -f *.o $(TARGET) *.res *.gcda *.gcno

.PHONY: all clean profile-generate profile-use
