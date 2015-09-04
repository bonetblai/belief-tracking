ROOT=                   ../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=		$(ROOT)/mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(MDP_ENGINE_HOME)/engine

CXX=			g++
CXXEXT=			-std=c++0x
CFLAGS=			-Wall -Werror -O3 $(CXXEXT) $(INCLUDES) -DNDEBUG

battleship:		main.cc battleship_api.h battleship_belief.h battleship_wrapper_belief.h base_belief.h lookahead.h
				$(CXX) $(CFLAGS) -o battleship main.cc

clean:
				rm -f battleship *~

