ROOT=			../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=	$(ROOT)/mdp-engine

DEPS=			action_selection.h cellmap.h slam_action_selection.h tracking.h utils.h \
			particle_filter.h \
			sis.h \
			sir.h motion_model_sir.h optimal_sir.h \
			rbpf.h \
			slam_particle_types.h slam2_particle_types.h \
			inference.h

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine #-I $(MDP_ENGINE_HOME)/engine

CXX=			c++
CXXEXT=			-std=c++11
#CXXEXT2=		-std=c++11 -I ~/include -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/lib -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp
  CXXEXT2=		-std=c++11 -DMACOSX -I ~/software/libdai/include -I ~/include -I /usr/local/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/software/libdai/lib -L ~/lib -L /usr/local/lib -Wno-overloaded-virtual -Wno-unused-local-typedef -Wno-unused-parameter

#c++ -std=c++11 -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -DMACOSX -arch x86_64 -O3 -g -DDAI_DEBUG  -L ~/software/libdai/lib -o mines2 main2.cc c_api.o -ldai -lgmpxx -lgmp -arch x86_64

CFLAGS=			-Wall -O3 $(CXXEXT2) $(INCLUDES) #-DNDEBUG

colorslam.test:		main.cc $(DEPS)
				$(CXX) $(CFLAGS) -o colorslam.test main.cc -ldai -lgmpxx -lgmp

clean:
			rm -f colorslam dummy* *~

