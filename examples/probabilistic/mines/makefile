ROOT=		            ../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=		$(ROOT)/mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine #-I $(MDP_ENGINE_HOME)/engine

CXX=			c++
CXXEXT=			-std=c++11
#CXXEXT2=		-std=c++11 -I ~/include -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/lib -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp
CXXEXT2=		-std=c++11 -DMACOSX -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp

#c++ -std=c++11 -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -DMACOSX -arch x86_64 -O3 -g -DDAI_DEBUG  -L ~/software/libdai/lib -o mines2 main2.cc c_api.o -ldai -lgmpxx -lgmp -arch x86_64

CFLAGS=			-Wall -Werror -O3 $(CXXEXT) $(INCLUDES) #-DNDEBUG

prob_mines:		prob_main.cc c_api.o
				$(CXX) $(CXXEXT2) -o prob_mines prob_main.cc c_api.o -ldai -lgmpxx -lgmp

c_api.o:		c_api.cc c_api.h mines_agent.h mines_belief.h mines_grid_belief.h
				$(CXX) $(CFLAGS) -c c_api.cc

clean:
				rm -f prob_mines c_api.o *~

