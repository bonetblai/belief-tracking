ROOT=			../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=	$(ROOT)/mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine -I $(BELIEF_TRACKING_HOME)/probabilistic_engine #-I $(MDP_ENGINE_HOME)/engine

CXX=			c++
MPICXX=			mpic++

CXXEXT=			-std=c++11
#CXXEXT2=		-std=c++11 -I ~/include -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/lib -L ~/software/libdai/lib #-ldai -lgmpxx -lgmp
CXXEXT2=		-std=c++11 -DMACOSX -I ~/software/libdai/include -I ~/include -I /usr/local/include -Wno-deprecated -Wall -W -Wextra -fPIC -O3 -DDAI_DEBUG -L ~/software/libdai/lib -L ~/lib -L /usr/local/lib -Wno-overloaded-virtual -Wno-unused-local-typedef #-Wno-unused-parameter

#c++ -std=c++11 -I ~/software/libdai/include -Wno-deprecated -Wall -W -Wextra -fPIC -DMACOSX -arch x86_64 -O3 -g -DDAI_DEBUG  -L ~/software/libdai/lib -o mines2 main2.cc c_api.o -ldai -lgmpxx -lgmp -arch x86_64

CFLAGS=			-Wall -O3 $(CXXEXT2) $(INCLUDES) #-DNDEBUG

slam:
			$(CXX) $(CFLAGS) -o slam main.cc -ldai -lgmpxx -lgmp

slam.mpi:
			$(MPICXX) $(CFLAGS) -DUSE_MPI -o slam.mpi main.cc -ldai -lgmpxx -lgmp

mpi_worker:
			$(MPICXX) $(CFLAGS) -DUSE_MPI -o mpi_worker mpi_worker.cc -ldai -lgmpxx -lgmp

all:			slam slam.mpi mpi_worker
mpi:			slam.mpi mpi_worker

clean:
			rm -f slam slam.mpi mpi_worker dummy* *~

# dependencies

slam:			main.cc \
			cellmap.h utils.h \
  			action_selection.h slam_action_selection.h \
			tracking.h particle_filter.h \
			sis.h sir.h motion_model_sir.h optimal_sir.h rbpf.h \
			slam_particles.h slam2_particles.h \
			inference.h

slam.mpi:		main.cc \
			cellmap.h utils.h \
  			action_selection.h slam_action_selection.h \
			tracking.h particle_filter.h \
			sis.h sir.h motion_model_sir.h optimal_sir.h rbpf.h \
			slam_particles.h slam2_particles.h \
			inference.h \
			mpi_api.h mpi_slam.h

mpi_worker:		mpi_worker.cc mpi_api.h mpi_slam.h inference.h utils.h

