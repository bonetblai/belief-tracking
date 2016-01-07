ROOT=			../../../..
BELIEF_TRACKING_HOME=	$(ROOT)/belief-tracking
MDP_ENGINE_HOME=	$(ROOT)/mdp-engine

INCLUDES=		-I $(BELIEF_TRACKING_HOME)/engine #-I $(BELIEF_TRACKING_HOME)/probabilistic_engine #-I $(MDP_ENGINE_HOME)/engine

CXX=			c++
MPICXX=			mpic++

CXXEXT=			-std=c++11 -DMACOSX -I ~/software/libdai/include -I /usr/local/include -Wno-deprecated -Wall -W -Wextra -DDAI_DEBUG -L ~/software/libdai/lib -L /usr/local/lib -Wno-overloaded-virtual -Wno-unused-local-typedef -Wno-unused-parameter

CFLAGS=			-Wall -O3 $(CXXEXT) $(INCLUDES) #-DNDEBUG

slam:
			$(CXX) $(CFLAGS) -o slam main.cc -ldai #-lgmpxx -lgmp

mpi.master:
			$(MPICXX) $(CFLAGS) -DUSE_MPI -o mpi.master main.cc -ldai #-lgmpxx -lgmp

mpi.worker:
			$(MPICXX) $(CFLAGS) -DUSE_MPI -o mpi.worker mpi_worker.cc -ldai #-lgmpxx -lgmp

all:			slam mpi
mpi:			mpi.master mpi.worker

clean:
			rm -f slam mpi.master mpi.worker dummy* *~

# dependencies

slam:		main.cc \
			cellmap.h utils.h \
  			action_selection.h slam_action_selection.h \
			tracking.h particle_filter.h \
			sis.h sir.h motion_model_sir.h optimal_sir.h rbpf.h \
			slam_particles.h slam2_particles.h slam3_particles.h slam4_particles.h\
			kappa.h \
			generate_plot.h stats.h \
			inference.h

mpi.master:	main.cc \
			cellmap.h utils.h \
  			action_selection.h slam_action_selection.h \
			tracking.h particle_filter.h \
			sis.h sir.h motion_model_sir.h optimal_sir.h rbpf.h \
			slam_particles.h slam2_particles.h slam3_particles.h slam4_particles.h\
			kappa.h \
			generate_plot.h stats.h \
			inference.h \
			mpi_api.h mpi_slam.h
mpi.worker:	mpi_worker.cc mpi_api.h mpi_slam.h inference.h utils.h

