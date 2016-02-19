CC=g++

PTHREADINCLUDE=C:/cygwin64/usr/i686-pc-mingw32/sys-root/mingw/include
PTHREADLIB=C:/cygwin64/usr/i686-pc-mingw32/sys-root/mingw/lib/libpthread.a 

OPTIMIZE=-O3
CFLAGS=$(OPTIMIZE) -pthread -I$(PTHREADINCLUDE) -c -std=c++11 -Wall
LDFLAGS=
LDLIBS=$(PTHREADLIB) -lstdc++

SOURCEPATH=src
SOURCES=$(SOURCEPATH)/aabb.cpp \
		$(SOURCEPATH)/anisotropicparticlemesher.cpp \
		$(SOURCEPATH)/collision.cpp \
		$(SOURCEPATH)/cuboidfluidsource.cpp \
		$(SOURCEPATH)/fluidbrickgrid.cpp \
		$(SOURCEPATH)/fluidmaterialgrid.cpp \
		$(SOURCEPATH)/fluidsimulation.cpp \
		$(SOURCEPATH)/fluidsimulationsavestate.cpp \
		$(SOURCEPATH)/fluidsource.cpp \
		$(SOURCEPATH)/gridindexkeymap.cpp \
		$(SOURCEPATH)/gridindexvector.cpp \
		$(SOURCEPATH)/implicitpointprimitive.cpp \
		$(SOURCEPATH)/implicitsurfacescalarfield.cpp \
		$(SOURCEPATH)/interpolation.cpp \
		$(SOURCEPATH)/isotropicparticlemesher.cpp \
		$(SOURCEPATH)/levelset.cpp \
		$(SOURCEPATH)/logfile.cpp \
		$(SOURCEPATH)/macvelocityfield.cpp \
		$(SOURCEPATH)/main.cpp \
		$(SOURCEPATH)/polygonizer3d.cpp \
		$(SOURCEPATH)/pressuresolver.cpp \
		$(SOURCEPATH)/spatialpointgrid.cpp \
		$(SOURCEPATH)/sphericalfluidsource.cpp \
		$(SOURCEPATH)/stopwatch.cpp \
		$(SOURCEPATH)/threading.cpp \
		$(SOURCEPATH)/trianglemesh.cpp \
		$(SOURCEPATH)/turbulencefield.cpp \
		$(SOURCEPATH)/vmath.cpp

OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=fluidsim

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(SOURCEPATH)/*.o $(EXECUTABLE)
