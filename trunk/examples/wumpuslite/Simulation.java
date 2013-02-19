/*
 * Class that defines the simulation environment.
 * 
 * Written by James P. Biagioni (jbiagi1@uic.edu)
 * for CS511 Artificial Intelligence II
 * at The University of Illinois at Chicago
 * 
 * Last modified 4/14/08 
 * 
 * DISCLAIMER:
 * Elements of this application were borrowed from
 * the client-server implementation of the Wumpus
 * World Simulator written by Kruti Mehta at
 * The University of Texas at Arlington.
 * 
 */

class Simulation {
    private boolean death = false;
    private boolean gold = false;
    private int currScore = 0;
    private int steps = 0;

    private Timing timing;
    private long elapsedTime = 0;

    private static int actionCost = -1;
    private static int deathCost = -1000;
    private static int shootCost = -10;
    private int stepCounter = 0;
    private int lastAction = 0;

    private boolean simulationRunning;

    private Agent agent;
    private Environment environment;
    private TransferPercept transferPercept;

    public Simulation(Environment wumpusEnvironment, int maxSteps, boolean nonDeterministic, boolean verbose) {
        // start the simulator
        timing = new Timing();
        long startTime = timing.getCpuTime();
        simulationRunning = true;

        transferPercept = new TransferPercept(wumpusEnvironment);
        environment = wumpusEnvironment;
        agent = new Agent(environment, transferPercept, nonDeterministic);
        agent.prepareNewTrial();

        environment.placeAgent(agent);
        if (verbose) environment.printEnvironment();
        if (verbose) printCurrentPerceptSequence();

        try {
            if (verbose) System.out.println("Current score: " + currScore);
            while (simulationRunning == true && stepCounter < maxSteps) {
                if (verbose) System.out.println("Last action: " + Action.printAction(lastAction));
                if (verbose) System.out.println("Time step: " + stepCounter);

                ++steps;
                handleAction(agent.chooseAction());
                wumpusEnvironment.placeAgent(agent);

                if (verbose) environment.printEnvironment();								
                if (verbose) printCurrentPerceptSequence();
                if (verbose) System.out.println("Current score: " + currScore);

                stepCounter += 1;
                if (stepCounter == maxSteps || simulationRunning == false) {
                    if (verbose) System.out.println("Last action: " + Action.printAction(lastAction));
                    if (verbose) System.out.println("Time step: " + stepCounter);
                    lastAction = Action.END_TRIAL;
                }

                if (agent.getHasGold() == true) {
                    if (verbose) System.out.println("\n" + agent.getName() + " found the GOLD!!");
                }
                if (agent.getIsDead() == true) {
                    if (verbose) System.out.println("\n" + agent.getName() + " is DEAD!!");
                }
            }
        } catch (Exception e) {
            System.out.println("An exception was thrown: " + e);
        }		
        if (verbose) printEndWorld();
        elapsedTime = timing.getCpuTime() - startTime;
    }

    public void printEndWorld() {
        try {
            environment.printEnvironment();
            System.out.println("Final score: " + currScore);
            System.out.println("Last action: " + Action.printAction(lastAction));
        } catch (Exception e) {
            System.out.println("An exception was thrown: " + e);
        }
    }

    public void printCurrentPerceptSequence() {
        try {
            System.out.print("Percept: <");	
            if (transferPercept.getBump() == true) {
                System.out.print("bump,");
            } else if (transferPercept.getBump() == false) {
                System.out.print("none,");
            }
            if (transferPercept.getGlitter() == true) {
                System.out.print("glitter,");
            } else if (transferPercept.getGlitter() == false) {
                System.out.print("none,");
            }
            if (transferPercept.getBreeze() == true) {
                System.out.print("breeze,");
            } else if (transferPercept.getBreeze() == false) {
                System.out.print("none,");
            }
            if (transferPercept.getStench() == true) {
                System.out.print("stench,");
            } else if (transferPercept.getStench() == false) {
                System.out.print("none,");
            }
            if (transferPercept.getScream() == true) {
                System.out.print("scream>\n");
            } else if (transferPercept.getScream() == false) {
                System.out.print("none>\n");
            }
        } catch (Exception e) {
            System.out.println("An exception was thrown: " + e);
        }
    }

    public void handleAction(int action) {
        try {
            if (action == Action.GO_FORWARD) {
                if (environment.getBump() == true) environment.setBump(false);
                agent.goForward();
                environment.placeAgent(agent);
                if (environment.checkDeath() == true) {
                    currScore += deathCost;
                    simulationRunning = false;
                    agent.setIsDead(true);
                } else {
                    currScore += actionCost;
                }
                if (environment.getScream() == true) environment.setScream(false);
                lastAction = Action.GO_FORWARD;
            } else if (action == Action.TURN_RIGHT) {
                currScore += actionCost;
                agent.turnRight();		
                environment.placeAgent(agent);
                if (environment.getBump() == true) environment.setBump(false);
                if (environment.getScream() == true) environment.setScream(false);
                lastAction = Action.TURN_RIGHT;
            } else if (action == Action.TURN_LEFT) {
                currScore += actionCost;
                agent.turnLeft();		
                environment.placeAgent(agent);
                if (environment.getBump() == true) environment.setBump(false);
                if (environment.getScream() == true) environment.setScream(false);
                lastAction = Action.TURN_LEFT;
            } else if (action == Action.GRAB) {
                if (environment.grabGold() == true) {
                    currScore += 1000;
                    simulationRunning = false;
                    agent.setHasGold(true);
                } else {
                    currScore += actionCost;
                }
                environment.placeAgent(agent);
                if (environment.getBump() == true) environment.setBump(false);
                if (environment.getScream() == true) environment.setScream(false);
                lastAction = Action.GRAB;
            } else if (action == Action.SHOOT) {
                if (agent.shootArrow() == true) {
                    if (environment.shootArrow() == true) environment.setScream(true);
                    currScore += shootCost;					
                } else {
                    if (environment.getScream() == true) environment.setScream(false);
                    currScore += actionCost;
                }
                environment.placeAgent(agent);
                if (environment.getBump() == true) environment.setBump(false);
                lastAction = Action.SHOOT;
            } else if (action == Action.NO_OP) {
                environment.placeAgent(agent);
                if (environment.getBump() == true) environment.setBump(false);
                if (environment.getScream() == true) environment.setScream(false);
                lastAction = Action.NO_OP;
            }
        } catch (Exception e) {
            System.out.println("An exception was thrown: " + e);
        }
    }

    public int getScore() {
        return currScore;
    }

    public int getSteps() {
        return steps;
    }

    public long getElapsedTime() {
        return elapsedTime;
    }

    public boolean getDeath() {
        return agent.getIsDead();
    }

    public boolean getGold() {
        return agent.getHasGold();
    }

    public boolean getExplored() {
        return agent.getExplored();
    }

    public boolean getUnvisitedSafeCell() {
        return agent.getUnvisitedSafeCell();
    }
}

