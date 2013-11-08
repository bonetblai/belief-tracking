# Mac users should change the compiler (CXX) to clang++ and
# set the CLANG flags (for CC and LD) as the default flags.
# Also, the java include directory should be changed by
# replacing SOLARIS_JAVA with MACOSX java.


BELIEF_TRACKING_HOME=		../..
MDP_ENGINE_HOME=		../../../mdp-engine
EXAMPLES_HOME=			$(BELIEF_TRACKING_HOME)/examples

SOLARIS_JAVA=			-I /usr/java/include -I /usr/java/include/solaris
MACOSX_JAVA=                    -I /System/Library/Frameworks/JavaVM.framework/Versions/A/Headers
JAVA_INCLUDES=			$(SOLARIS_JAVA)

INCLUDES=			-I $(BELIEF_TRACKING_HOME)/engine \
				-I $(MDP_ENGINE_HOME)/engine \
				-I $(EXAMPLES_HOME) \
				$(JAVA_INCLUDES)

GNU_CC_FLAGS=			-std=c++0x
GNU_LD_FLAGS=			-shared -Wl,-soname,libmean.so.1 -o platform/libagent.so WumpusAgentProxy.o
CLANG_CC_FLAGS=			-std=c++11 -std=c++0x
CLANG_LD_FLAGS=                 -dynamiclib -o platform/libagent.dylib WumpusAgentProxy.o -framework JavaVM
CXX_FLAGS=			$(GNU_CC_FLAGS)
LD_FLAGS=			$(GNU_LD_FLAGS)

CXX=				g++
CFLAGS=				-Wall -Werror -O3 -fPIC $(CXX_FLAGS) $(INCLUDES) -DNDEBUG

all:				platform/libagent.so  wumpus.jar

clean:				clean-java clean-jni

clean-java:
				rm -f *.class wumpus.jar core *~

clean-jni:
				rm -f platform/* WumpusAgentProxy.{h,o} core *~

WorldApplication.class:
				javac WorldApplication.java

WumpusAgentProxy.class:
				javac WumpusAgentProxy.java

WumpusAgentProxy.h:
				javah -force WumpusAgentProxy

WumpusAgentProxy.o:
				$(CXX) -c WumpusAgentProxy.cc $(CFLAGS)

platform/libagent.so:
				$(CXX) $(LD_FLAGS)

wumpus.jar:
				jar cmf Manifest.txt wumpus.jar *.class platform/libagent.so


wumpus.jar:			WorldApplication.class platform/libagent.so
platform/libagent.so:		WumpusAgentProxy.o
WumpusAgentProxy.o:		WumpusAgentProxy.h WumpusAgentProxy.cc
WumpusAgentProxy.h:		WumpusAgentProxy.class
WumpusAgentProxy.class:		WumpusAgentProxy.java

