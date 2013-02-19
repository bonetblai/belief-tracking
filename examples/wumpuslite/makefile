# Mac users should change the compiler (CXX) to clang++ and
# set the CLANG flags (for CC and LD) as the default flags.
# Also, the java include directory should be changed by
# replacing SOLARIS_JAVA with MACOSX java.


BELIEF_TRACKING_HOME=		../..
MDP_ENGINE_HOME=		../../../mdp-engine
WUMPUS_HOME=			$(BELIEF_TRACKING_HOME)/examples/wumpus

SOLARIS_JAVA=			-I /usr/java/include -I /usr/java/include/solaris
MACOSX_JAVA=                    -I /System/Library/Frameworks/JavaVM.framework/Versions/A/Headers
JAVA_INCLUDES=			$(SOLARIS_JAVA)

INCLUDES=			-I $(BELIEF_TRACKING_HOME)/engine \
				-I $(MDP_ENGINE_HOME)/engine \
				-I $(WUMPUS_HOME) \
				$(JAVA_INCLUDES)

GNU_CC_FLAGS=			-std=c++0x
GNU_LD_FLAGS=			-shared -Wl,-soname,libmean.so.1 -o platform/libagent.so AgentFunctionProxy.o
CLANG_CC_FLAGS=			-std=c++11 -std=c++0x
CLANG_LD_FLAGS=                 -dynamiclib -o platform/libagent.dylib AgentFunctionProxy.o -framework JavaVM
CXX_FLAGS=			$(GNU_CC_FLAGS)
LD_FLAGS=			$(GNU_LD_FLAGS)

CXX=				g++
CFLAGS=				-Wall -O3 -fPIC $(CXX_FLAGS) $(INCLUDES) -DNDEBUG

all:				wumpus.jar

clean:
				rm -f platform/* AgentFunctionProxy.{h,o} *.class *~

WorldApplication.class:
				javac WorldApplication.java

AgentFunctionProxy.class:
				javac AgentFunctionProxy.java

AgentFunctionProxy.h:
				javah -force AgentFunctionProxy

AgentFunctionProxy.o:
				$(CXX) -c AgentFunctionProxy.cc $(CFLAGS)

platform/libagent.so:
				$(CXX) $(LD_FLAGS)

wumpus.jar:
				jar cmf Manifest.txt wumpus.jar *.class platform/*


wumpus.jar:			WorldApplication.class platform/libagent.so
platform/libagent.so:		AgentFunctionProxy.o
AgentFunctionProxy.o:		AgentFunctionProxy.h AgentFunctionProxy.cc
AgentFunctionProxy.h:		AgentFunctionProxy.class
AgentFunctionProxy.class:	AgentFunctionProxy.java



