BELIEF_TRACKING_HOME=	../..
MDP_ENGINE_HOME=	../../../mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine -I $(MDP_ENGINE_HOME)/engine

CXX=			c++
CXXEXT=			-std=c++11
#CXXEXT2=		-std=c++11 -I ~/include -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/lib -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp
CXXEXT2=		-std=c++11 -DMACOSX -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/software/libdai/lib -Wno-overloaded-virtual #-ldai -lgmpxx

#c++ -std=c++11 -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -DMACOSX -arch x86_64 -O3 -g -DDAI_DEBUG  -L ~/software/libdai/lib -o mines2 main2.cc c_api.o -ldai -lgmpxx -lgmp -arch x86_64

CFLAGS=			-Wall -O3 $(CXXEXT2) $(INCLUDES) #-DNDEBUG

maplearning:		main.cc
			$(CXX) $(CFLAGS) -o maplearning main.cc -ldai -lgmpxx -lgmp

clean:
			rm -f maplearning *~

