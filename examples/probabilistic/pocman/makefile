BELIEF_TRACKING_HOME=	../..
MDP_ENGINE_HOME=	../../../mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine -I $(MDP_ENGINE_HOME)/engine

CXX=			c++
CXXEXT=			-std=c++11
CXXEXT2=		-std=c++11 -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -DMACOSX -arch x86_64 -O3 -DDAI_DEBUG -L ~/software/libdai/lib -ldai -lgmpxx -lgmp

CFLAGS=			-Wall -Werror -O3 $(CXXEXT) $(INCLUDES) #-DNDEBUG

pocman:			main.cc
			$(CXX) $(CXXEXT2) -o pocman main.cc

clean:
			rm -f pocman *~

