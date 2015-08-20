Provides a collection of templates and tools for implementing a sound and complete belief tracking module for non-deterministic contingent planning. It also supports approximate belief tracking that is sound, incomplete, yet it proves to be quite powerful in challenging domains.

The project includes three example agents for the games of battleship, minesweeper and wumpus. The code for wumpus is based on the `wumpuslite` source code developed by James
Biagioni ([email](mailto:jbiagi1@uic.edu),[website](http://cs.uic.edu/Bits/JamesBiagioni)).


For more information about the belief tracking algorithms, see

  * B. Bonet and H. Geffner. _Width and Complexity of Belief Tracking in Non-deterministic Conformant and Contingent Planning_. Proc. of 16th Conf. on Artificial Intelligence (AAAI). 2012. [(PDF)](http://ldc.usb.ve/~bonet/reports/AAAI12-tracking.pdf)

  * B. Bonet and H. Geffner. _Causal Belief Decomposition for Planning with Sensing: Completeness Results and Practical Approximation_. Proc. of 23rd Int. Joint Conf. on Artificial Intelligence (IJCAI). 2013. [(PDF)](http://ldc.usb.ve/~bonet/reports/IJCAI13-tracking.pdf)


---


Notes:

  * For obtaining the code, go to [Checkout Source](http://code.google.com/p/belief-tracking/source/checkout).
  * The original wumpuslite (version 0.21a) is available in [Downloads](http://code.google.com/p/belief-tracking/downloads/list).
  * To compile the examples, you need to checkout the **`mdp-engine`** project that is available [here](http://code.google.com/p/mdp-engine). Before compiling the examples, verify that the path for mdp-engine (specified in the makefiles) is correct as it may be different from the default.
  * A movie showing an fully-autonomous agent using the belief-tracking algorithm to play minesweeper can be seen [here](https://www.youtube.com/watch?v=U98ow4n87RA).
  * Binary distributions of a **graphical** implementation of the games of Battleship, Minesweeper and Wumpus, and a fully-autonomous player that uses the belief-tracking engine are available in [Downloads](http://code.google.com/p/belief-tracking/downloads/list). The implementation was done using the [tewnta framework](http://code.google.com/p/tewnta/) developed in Java.


---

