BELIEF_TRACKING_HOME=	../..
MDP_ENGINE_HOME=	../../../mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine -I $(MDP_ENGINE_HOME)/engine

CXX=			g++
CXXEXT=			-std=c++0x
CFLAGS=			-Wall -Werror -O3 $(CXXEXT) $(INCLUDES) #-DNDEBUG

mines:			main.cc c_api.o
			$(CXX) $(CFLAGS) -o mines main.cc c_api.o

c_api.o:		c_api.cc c_api.h mines_agent.h mines_belief.h mines_grid_belief.h
			$(CXX) $(CFLAGS) -c c_api.cc

clean:
			rm -f mines c_api.o *~

