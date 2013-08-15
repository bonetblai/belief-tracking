/*
 * Class that defines the agent.
 * 
 * Written by James P. Biagioni (jbiagi1@uic.edu)
 * for CS511 Artificial Intelligence II
 * at The University of Illinois at Chicago
 * 
 * Last modified 3/5/07
 * 
 * DISCLAIMER:
 * Elements of this application were borrowed from
 * the client-server implementation of the Wumpus
 * World Simulator written by Kruti Mehta at
 * The University of Texas at Arlington.
 * 
 */

import java.util.Random;

class Agent {
    private boolean movingWumpus;
    private int[] wumpusLocation;

    private int[] location;
    private char direction;
    private char agentIcon;

    private int numArrows = 1;
    private int worldSize;

    private boolean isDead;
    private boolean hasGold;

    private Environment wumpusWorld;
    private TransferPercept percept;
    //private AgentFunctionProxy agentFunction;
    private AgentProxy agent;

    public Agent(Environment world, TransferPercept perceptTrans, boolean movingWumpus, boolean diagonalWumpus) {
        // set deterministic/non-deterministic
        this.movingWumpus = movingWumpus;

        // initial conditions
        isDead = false;
        hasGold = false;

        wumpusWorld = world;
        worldSize = wumpusWorld.getWorldSize();
        agent = new WumpusAgentProxy(movingWumpus, worldSize, world.getNumPits(), world.getNumWumpus(), world.getNumArrows(), diagonalWumpus);
        percept = perceptTrans;

        // initial location
        location = wumpusWorld.getAgentLocation();
        direction = wumpusWorld.getAgentDirection();
        if (movingWumpus) wumpusLocation = wumpusWorld.getWumpusLocation();
        setDirection(direction);
    }

    public void prepareNewTrial() {
        agent.prepareNewTrial();
    }

    public void setPolicyParameters(int numExpansions, int MDPHorizon) {
        agent.setPolicyParameters(numExpansions, MDPHorizon);
    }

    public void setIsDead(boolean dead) {
        isDead = dead;
    }

    public boolean getIsDead() {
        return isDead;
    }

    public void setHasGold(boolean possessGold) {
        hasGold = possessGold;
    }

    public boolean getHasGold() {
        return hasGold;
    }

    public boolean getExplored() {
        return agent.isWorldExplored();
    }

    public boolean getUnvisitedSafeCell() {
        return agent.isThereAnUnvisitedSafeCell();
    }

    public String getName() {
        return agent.getAgentName();
    }

    public int chooseAction() {
        return agent.process(location, direction, percept);
    }

    public char getAgentIcon() {
        return agentIcon;
    }

    public void goForward() {
        if (direction == 'N') {
            if (location[0]+1 < worldSize) location[0] += 1;
            else wumpusWorld.setBump(true);
        } else if (direction == 'E') {
            if (location[1]+1 < worldSize) location[1] += 1;
            else wumpusWorld.setBump(true);
        } else if (direction == 'S') {
            if (location[0]-1 >= 0) location[0] -= 1;
            else wumpusWorld.setBump(true);
        } else if (direction == 'W') {
            if (location[1]-1 >= 0) location[1] -= 1;
            else wumpusWorld.setBump(true);
        }

        if (movingWumpus) {
            Random rand = new Random();
            switch (rand.nextInt(4)) {
                case 0:
                    if (wumpusLocation[0]+1 < worldSize) wumpusLocation[0] += 1;
                    break;
                case 1:
                    if (wumpusLocation[1]+1 < worldSize) wumpusLocation[1] += 1;
                    break;
                case 2:
                    if (wumpusLocation[0]-1 >= 0) wumpusLocation[0] -= 1;
                    break;
                case 3:
                    if (wumpusLocation[1]-1 >= 0) wumpusLocation[1] -= 1;
                    break;
            }
        }
    }

    public boolean shootArrow() {
        if (numArrows == 1) {
            numArrows -= 1;
            return true;
        } else {
            return false;
        }
    }
	
    public void turnRight() {
        if (direction == 'N') setDirection('E');
        else if (direction == 'E') setDirection('S');
        else if (direction == 'S') setDirection('W');
        else if (direction == 'W') setDirection('N');
    }

    public void turnLeft() {
        if (direction == 'N') setDirection('W');
        else if (direction == 'E') setDirection('N');
        else if (direction == 'S') setDirection('E');
        else if (direction == 'W') setDirection('S');
    }

    public void setDirection(char newDirection) {
        direction = newDirection;
        if (direction == 'N') agentIcon = 'A';
        if (direction == 'E') agentIcon = '>';
        if (direction == 'S') agentIcon = 'V';
        if (direction == 'W') agentIcon = '<';
    }

    public char getDirection() {
        return direction;
    }

    public void setLocation(int[] newLocation) {
        location[0] = newLocation[0];
        location[1] = newLocation[1];
    }

    public int[] getLocation() {
        return location;
    }

    public int[] getWumpusLocation() {
        return wumpusLocation;
    }
}

