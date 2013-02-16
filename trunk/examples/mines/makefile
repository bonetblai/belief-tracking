BELIEF_TRACKING_HOME=	../..
MDP_ENGINE_HOME=	../../../mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(MDP_ENGINE_HOME)/engine

CXX=			clang++
CXXEXT=			-std=c++0x
CFLAGS=			-Wall -O3 $(CXXEXT) $(INCLUDES) -DNDEBUG

mines:			main.cc c_api.o
			$(CXX) $(CFLAGS) -o mines main.cc c_api.o

c_api.o:		c_api.cc c_api.h
			$(CXX) $(CFLAGS) -c c_api.cc

clean:
			rm -f mines c_api.o *~

