ROOT=			../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=	$(ROOT)/mdp-engine
LIBDAI=			~/software/libdai

INCLUDES=		-I $(LIBDAI)/include -I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine -I ~/include -I /usr/local/include
LIBS=			-L $(LIBDAI)/lib -L ~/lib -ldai

CXX=			g++
#CXXEXT=			-std=c++11
#CXXEXT2=		-std=c++11 -I ~/include -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/lib -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp
CXXEXT=			-Wall -W -Wextra -O3 -std=c++11 -DMACOSX -Wno-deprecated -Wno-overloaded-virtual -Wno-unused-local-typedef -Wno-unused-parameter

CFLAGS=			$(CXXEXT) $(INCLUDES) -DNDEBUG

prob_mines:		prob_main.cc
			$(CXX) $(CFLAGS) -o prob_mines prob_main.cc $(LIBS)

clean:
			rm -f prob_mines c_api.o dummy* *~

clean-tmp:
			rm -fr dummy* *~

