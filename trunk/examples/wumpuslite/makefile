# The following variable detect the operating system. On
# supported OS, this should work without modification.
# For unsupported OS, the variable must set manually.

OS=auto

ifeq ($(OS), auto)
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
OS=osx
endif
ifeq ($(UNAME), Linux)
OS=linux
endif
ifeq ($(UNAME), SunOS)
OS=solaris
endif
ifeq ($(OS), auto)
$(warning OS detection failed -- setting to Linux and hoping for the best!)
OS=linux
endif
endif

ifeq ($(OS), osx)
LIBRARY=			platform/libagent.dylib
CXX=				clang++
CC_FLAGS=			-std=c++11 -std=c++0x
LD_FLAGS=                 	-dynamiclib -o $(LIBRARY) WumpusAgentProxy.o -framework JavaVM
JAVA_SDK=			$(shell /usr/libexec/java_home -v 1.7)
JAVA_INCLUDES=                  -I /System/Library/Frameworks/JavaVM.framework/Versions/A/Headers -I $(JAVA_SDK)/include -I $(JAVA_SDK)/include/darwin
else
LIBRARY=			platform/libagent.so
CXX=				g++
CC_FLAGS=			-std=c++0x
LD_FLAGS=			-shared -Wl,-soname,libmean.so.1 -o $(LIBRARY) WumpusAgentProxy.o
ifeq ($(OS), solaris)
JAVA_INCLUDES=			-I /usr/java/include -I /usr/java/include/solaris
endif
ifeq ($(OS), linux)
JAVA_INCLUDES=			-I /usr/lib/jvm/default-java/include
endif
endif

CFLAGS=				-Wall -Werror -O3 -fPIC $(CC_FLAGS) $(INCLUDES) -DNDEBUG

BELIEF_TRACKING_HOME=		../..
MDP_ENGINE_HOME=		../../../mdp-engine
EXAMPLES_HOME=			$(BELIEF_TRACKING_HOME)/examples

INCLUDES=			-I $(BELIEF_TRACKING_HOME)/engine \
				-I $(MDP_ENGINE_HOME)/engine \
				-I $(EXAMPLES_HOME) \
				$(JAVA_INCLUDES)


all:				$(LIBRARY) wumpus.jar

clean:				clean-java clean-jni

clean-java:
				rm -f *.class wumpus.jar core *~

clean-jni:
				rm -f $(LIBRARY) WumpusAgentProxy.{h,o} core *~

WorldApplication.class:
				javac WorldApplication.java

WumpusAgentProxy.class:
				javac WumpusAgentProxy.java

WumpusAgentProxy.h:
				javah -force WumpusAgentProxy

WumpusAgentProxy.o:
				$(CXX) -c WumpusAgentProxy.cc $(CFLAGS)

$(LIBRARY):
				$(CXX) $(LD_FLAGS)

wumpus.jar:
				jar cmf Manifest.txt wumpus.jar *.class $(LIBRARY)


wumpus.jar:			WorldApplication.class $(LIBRARY)
$(LIBRARY):			WumpusAgentProxy.o
WumpusAgentProxy.o:		WumpusAgentProxy.h WumpusAgentProxy.cc
WumpusAgentProxy.h:		WumpusAgentProxy.class
WumpusAgentProxy.class:		WumpusAgentProxy.java

