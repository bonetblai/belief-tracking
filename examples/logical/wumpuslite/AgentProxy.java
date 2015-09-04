/*
 * Proxy class for accessing the C++ code that 
 * implements the agent.
 * 
 * Written by Blai Bonet upon a code of James
 * P. Biagioni (jbiagi1@uic.edu).
 * 
 * DISCLAIMER:
 * Elements of this application were borrowed from
 * the client-server implementation of the Wumpus
 * World Simulator written by Kruti Mehta at
 * The University of Texas at Arlington.
 * 
 */

interface AgentProxy {

    public void prepareNewTrial();
    public void setPolicyParameters(int numExpansions, int MDPHorizon);
    public int process(int[] location, char direction, TransferPercept tp);
    public boolean isWorldExplored();
    public boolean isThereAnUnvisitedSafeCell();
    public String getAgentName();
}

