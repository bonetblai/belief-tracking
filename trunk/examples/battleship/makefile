BELIEF_TRACKING_HOME=	../..
MDP_ENGINE_HOME=	../../../mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(MDP_ENGINE_HOME)/engine

CXX=			g++
CXXEXT=			-std=c++0x
CFLAGS=			-Wall -Werror -O3 $(CXXEXT) $(INCLUDES) -DNDEBUG

battleship:		main.cc
			$(CXX) $(CFLAGS) -o battleship main.cc

clean:
			rm -f battleship *~

