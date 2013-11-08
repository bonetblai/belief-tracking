/*
 * Wumpus-Lite, version 0.21 alpha
 * A lightweight Java-based Wumpus World Simulator
 * 
 * Written by James P. Biagioni (jbiagi1@uic.edu)
 * for CS511 Artificial Intelligence II
 * at The University of Illinois at Chicago
 * 
 * Thanks to everyone who provided feedback and
 * suggestions for improving this application,
 * especially the students from Professor
 * Gmytrasiewicz's Spring 2007 CS511 class.
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

import java.util.Random;

class WorldApplication {
    private static String VERSION = "v0.21a";

    public static void usage() {
        System.out.println("");
        System.out.println("Usage: WorldApplication [{-d | --dimension} <dim>]");
        System.out.println("                        [{-p | --npits} <npits>]");
        System.out.println("                        [{-w | --nwumpus} <nwumpus>]");
        System.out.println("                        [{-t | --ntrials} <ntrials>]");
        System.out.println("                        [{-s | --max-steps} <steps>]");
        System.out.println("                        [{-c | --check-explored}]");
        //System.out.println("                        [{-e | --num-expansions} <numexp>]");
        //System.out.println("                        [{-h | --mdp-horizon} <horiz>]");
        System.out.println("                        [{-r | --random-seed} <seed>]");
        System.out.println("                        [{-m | --moving}]");
        System.out.println("                        [{-x | --check-crossing}]");
        System.out.println("                        [--diagonal]");
        System.out.println("                        [{-v | --verbose}]");
        System.out.println("                        [--help]");
        System.out.println("");
        System.out.println("where <dim> is the side dimension of the square grid world, <npits> and");
        System.out.println("<nwumpus> are the number of pits and wumpus in the world, <max-steps> is");
        System.out.println("the maximum number of actions that agent can execute, and <ntrials> is");
        System.out.println("the number of trials for the experiment.");
        //System.out.println("                            , and <numexp> and <horiz> are the parameters");
        //System.out.println("for the lookahead policy.");
        System.out.println("");
        System.out.println("By default, the grid world is static meaning that the wumpuses don't");
        System.out.println("move aroung. If --moving is specified, then the world is dynamic and the");
        System.out.println("wumpus moves non-deterministically at each forward movement of the agent.");
        System.out.println("In the moving case, the number of wumpus must be 1. In the diagonal case,");
        System.out.println("specified with --diagonal, the wumpuses are located along the main");
        System.out.println("diagonal; this is a simplified version that had been used in benchmarks");
        System.out.println("for planning with sensing.");
        System.out.println("");
        System.out.println("The options --check-explored and --check-crossing check whether the world");
        System.out.println("had been explored extensively given all the information acquired and");
        System.out.println("whether the wumpus crosses the agent in a single move. The former option");
        System.out.println("only applies to static world and the check is performed at the end of the");
        System.out.println("trial; a check that involves calls to a SAT solver. The latter option only");
        System.out.println("applies to dynamic worlds (cf. --moving) and implies that in such crosses,");
        System.out.println("the agent dies.");
        System.out.println("");
        System.out.println("The default values for the parameters are the following:");
        System.out.println("");
        System.out.println("    <dim>            =   4");
        System.out.println("    <npits>          =   2");
        System.out.println("    <nwumpus>        =   2");
        System.out.println("    <ntrials>        =   1");
        System.out.println("    <steps>          =   0 (meaning 3 * <dim> * <dim>)");
        //System.out.println("    <numexp>         =   -1");
        //System.out.println("    <horiz>          =   -1");
        System.out.println("    <seed>           =   [random seed]");
        System.out.println("");
        System.out.println("    moving           =   false");
        System.out.println("    diagonal         =   false");
        System.out.println("    check-explored   =   false");
        System.out.println("    check-crossing   =   false");
        System.out.println("");


    }

    public static void main(String args[]) {
        int worldSize = 4;
        int numTrials = 1;
        int maxSteps = 0;
        int numPits = 2;
        int numWumpus = 2;

        int numExpansions = -1;
        int MDPHorizon = -1;

        boolean verbose = false;
        boolean checkExploration = false; // minisat should be in the path for this to work
        boolean randomAgentLoc = false;
        boolean userDefinedSeed = false;
        int randomSeed = (new Random()).nextInt();

        boolean movingWumpus = false;
        boolean checkCrossing = false;
        boolean diagonalWumpus = false;
		
        //String outFilename = "wumpus_out.txt";

        // iterate through command-line parameters
        for (int i = 0; i < args.length; i++) {
            String arg = args[i];

            // if the world dimension is specified
            if (arg.equals("-d") || arg.equals("--dimension")) {
                if (Integer.parseInt(args[i+1]) > 1) {
                    worldSize = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if number of pits is specified
            else if (arg.equals("-p") || arg.equals("--npits")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    numPits = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if number of wumpus is specified
            else if (arg.equals("-w") || arg.equals("--nwumpus")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    numWumpus = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if the maximum number of steps is specified
            else if (arg.equals("-s") || arg.equals("--max-steps")) {
                maxSteps = Integer.parseInt(args[i+1]);
                i++;
            }	    	

            // if the number of trials is specified
            else if (arg.equals("-t") || arg.equals("--ntrials")) {
                numTrials = Integer.parseInt(args[i+1]);
                i++;
            }

            // if exploration status should be checked at the end of each trial
            else if (arg.equals("-c") || arg.equals("--check-explored")) {
                checkExploration = true;
            }

            // if the number of expansions is specified
            else if (arg.equals("-e") || arg.equals("--num-expansions")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    numExpansions = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if the MDP horizon is specified
            else if (arg.equals("-h") || arg.equals("--mdp-horizon")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    MDPHorizon = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if the random agent location value is specified
            else if (arg.equals("-a")) {
                randomAgentLoc = Boolean.parseBoolean(args[i+1]);
                i++;
            }

            // if the random number seed is specified
            else if (arg.equals("-r") || arg.equals("--random-seed")) {
                randomSeed = Integer.parseInt(args[i+1]);
                userDefinedSeed = true;
                i++;
            }

            // if the non-determinism is specified
            else if (arg.equals("-m") || arg.equals("--moving")) {
                movingWumpus = true;
                diagonalWumpus = false;
            }

            // if crossing between agent and moving agent should be checked
            else if (arg.equals("-x") || arg.equals("--check-crossing")) {
                checkCrossing = true;
            }


            // if diagonal version is to be used
            else if (arg.equals("--diagonal")) {
                diagonalWumpus = true;
                movingWumpus = false;
            }

            // if verbose is specified
            else if (arg.equals("-v") || arg.equals("--verbose")) {
                verbose = true;
            }

            // help
            else if (arg.equals("--help")) {
                usage();
                return;
            }
        }

        // create the random generator used in *entire* application
        Random randomGenerator = new Random(randomSeed);

        // calculate max number of steps if not given
        if (maxSteps == 0) {
            maxSteps = 3 * worldSize * worldSize;
            //System.out.println("Setting maxSteps to " + maxSteps);
        }

        try {
            System.out.println("Wumpus-Lite " + VERSION + "\n");
            System.out.println("Dimensions: " + worldSize + "x" + worldSize);
            System.out.println("Maximum number of steps: " + maxSteps);
            System.out.println("Number of trials: " + numTrials);
            System.out.println("Random Agent Location: " + randomAgentLoc);
            System.out.println("Random number seed: " + randomSeed);
            System.out.println("Moving Wumpus: " + movingWumpus + "\n");
            System.out.println("Diagonal Version: " + diagonalWumpus + "\n");

            int trialScores[] = new int[numTrials];
            int trialSteps[] = new int[numTrials];
            boolean deaths[] = new boolean[numTrials];
            boolean golds[] = new boolean[numTrials];
            boolean explored[] = new boolean[numTrials];
            boolean unvisitedSafeCell[] = new boolean[numTrials];
            long elapsedTime[] = new long[numTrials];

            for (int currTrial = 0; currTrial < numTrials; currTrial++) {
                char[][][] wumpusWorld = generateRandomWumpusWorld(randomGenerator, worldSize, numPits, numWumpus, randomAgentLoc, diagonalWumpus);	
                Environment wumpusEnvironment = new Environment(worldSize, numPits, numWumpus, movingWumpus, checkCrossing, wumpusWorld);
                Simulation trial = new Simulation(randomGenerator, wumpusEnvironment, maxSteps, movingWumpus, diagonalWumpus, numExpansions, MDPHorizon, verbose);
                trialScores[currTrial] = trial.getScore();
                trialSteps[currTrial] = trial.getSteps();
                deaths[currTrial] = trial.getDeath();
                golds[currTrial] = trial.getGold();
                if (checkExploration) {
                    explored[currTrial] = !golds[currTrial] && trial.getExplored();
                } else {
                    explored[currTrial] = true;
                }
                unvisitedSafeCell[currTrial] = trial.getUnvisitedSafeCell();
                elapsedTime[currTrial] = trial.getElapsedTime();

                if (verbose) System.out.println("\n\n___________________________________________\n");

                //char[][][] wumpusWorld = generateRandomWumpusWorld(randomGenerator, worldSize, numPits, numWumpus, randomAgentLoc, diagonalWumpus);	
                //Environment wumpusEnvironment = new Environment(worldSize, numPits, numWumpus, movingWumpus, checkCrossing, wumpusWorld);
                System.runFinalization();
                if (!verbose) {
                    System.out.println("Trial " + (currTrial+1) + ":" +
                                       " steps= " + trialSteps[currTrial] +
                                       ", gold= " + golds[currTrial] +
                                       ", death= " + deaths[currTrial] +
                                       ", explored= " + explored[currTrial] +
                                       ", unvisitedSafeCell= " + unvisitedSafeCell[currTrial] +
                                       ", time= " + elapsedTime[currTrial]);
                }
            }

            float totalScore = 0;
            float totalSteps = 0;
            float totalETime = 0;
            float totalDeaths = 0;
            float totalGolds = 0;
            float totalExplored = 0;
            float totalUnvisitedSafeCell = 0;
            for (int i = 0; i < numTrials; i++) {
                if (verbose)
                    System.out.println("Trial " + (i+1) + " score=" + trialSteps[i] +
                        " death=" + deaths[i] + " gold=" + golds[i] +
                        " explored=" + explored[i] + " unvisitedSafeCell=" + unvisitedSafeCell[i]);
                totalScore += trialScores[i];
                totalSteps += trialSteps[i];
                totalETime += elapsedTime[i];
                if (deaths[i]) ++totalDeaths;
                if (golds[i]) ++totalGolds;
                if (explored[i]) ++totalExplored;
                if (unvisitedSafeCell[i]) ++totalUnvisitedSafeCell;
            }

            float scoreAvg = totalScore / numTrials;
            float stepsAvg = totalSteps / numTrials;
            float etimeAvg = totalETime / numTrials;
            float deathAvg = totalDeaths / numTrials;
            float goldAvg = totalGolds / numTrials;

            float scoreStdev = 0;
            float stepsStdev = 0;
            float etimeStdev = 0;
            //float goldStdev = 0;
            //float deathStdev = 0;
            for (int i = 0; i < numTrials; i++) {
                float d = trialScores[i] - scoreAvg;
                scoreStdev += d * d;
                d = trialSteps[i] - stepsAvg;
                stepsStdev += d * d;
                d = elapsedTime[i] - etimeAvg;
                etimeStdev += d * d;
                //float d = golds[i] - goldAvg;
                //goldStdev += d * d;
                //float d = deaths[i] - deathAvg;
                //deathStdev += d * d;
            }
            scoreStdev = (float)Math.sqrt(scoreStdev / (numTrials - 1));
            stepsStdev = (float)Math.sqrt(stepsStdev / (numTrials - 1));
            etimeStdev = (float)Math.sqrt(etimeStdev / (numTrials - 1));
            //goldStdev = (float)Math.sqrt(goldStdev / (numTrials - 1));
            //deathStdev = (float)Math.sqrt(deathStdev / (numTrials - 1));

            System.out.println("\nScore: Total= " + totalScore + " Avg= " + scoreAvg + " Stdev= " + scoreStdev);
            System.out.println("Steps: Total= " + totalSteps + " Avg= " + stepsAvg + " Stdev= " + stepsStdev);
            System.out.println("ETime (ms): Total= " + totalETime + " Avg= " + etimeAvg + " Stdev= " + etimeStdev);
            System.out.println("Death: Total= " + totalDeaths + " Avg= " + deathAvg);
            System.out.println("Gold: Total= " + totalGolds + " Avg= " + goldAvg);
            System.out.println("Explored: Total= " + totalExplored);
            System.out.println("UnvisitedSafeCell: Total= " + totalUnvisitedSafeCell);
        } catch (Exception e) {
            System.out.println("An exception was thrown: " + e);
        }
        System.out.println("\nFinished.");	    
    }
	
    public static char[][][] generateRandomWumpusWorld(Random randomGenerator, int size, int numPits, int numWumpus, boolean randomlyPlaceAgent, boolean diagonalWumpus) {
        char[][][] newWorld = new char[size][size][4];
        boolean[][] pit_occupied = new boolean[size][size];
        boolean[][] wumpus_occupied = new boolean[size][size];

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                for (int k = 0; k < 4; k++) {
                    newWorld[i][j][k] = ' '; 
                }
            }
        }

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                pit_occupied[i][j] = false;
                wumpus_occupied[i][j] = false;
            }
        }

        if (!diagonalWumpus) {
            // default agent location and orientation
            int agentXLoc = 0;
            int agentYLoc = 0;
            char agentIcon = 'A';

            // randomly generate agent location and orientation
            if (randomlyPlaceAgent == true) {
                agentXLoc = randomGenerator.nextInt(size);
                agentYLoc = randomGenerator.nextInt(size);
                switch (randomGenerator.nextInt(4)) {
                    case 0: agentIcon = 'A'; break;
                    case 1: agentIcon = '>'; break;
                    case 2: agentIcon = 'V'; break;
                    case 3: agentIcon = '<'; break;
                }
            }

            // place agent in the world
            newWorld[agentXLoc][agentYLoc][3] = agentIcon;

            // Pit generation
            for (int i = 0; i < numPits; i++) {
                int x = randomGenerator.nextInt(size);
                int y = randomGenerator.nextInt(size);
                while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || ((x < 2) && (y < 2))) {
                    x = randomGenerator.nextInt(size);
                    y = randomGenerator.nextInt(size);    	   
                }
                pit_occupied[x][y] = true;
                newWorld[x][y][0] = 'P';
            }

            // Wumpus Generation
            for (int i = 0; i < numWumpus; i++) {
                int x = randomGenerator.nextInt(size);
                int y = randomGenerator.nextInt(size);
                while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || wumpus_occupied[x][y] || ((x < 2) && (y < 2))) {
                    x = randomGenerator.nextInt(size);
                    y = randomGenerator.nextInt(size);   
                }
                wumpus_occupied[x][y] = true;
                newWorld[x][y][1] = 'W';
            }

            // Gold Generation
            int x = randomGenerator.nextInt(size);
            int y = randomGenerator.nextInt(size);
            while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || wumpus_occupied[x][y]) {
                x = randomGenerator.nextInt(size);
                y = randomGenerator.nextInt(size);   
            }
            newWorld[x][y][2] = 'G';
        } else {
            // In diagonal version, the agent is at lowe-left corner, the gold
            // is at upper-right corner, there are no pits, and there are n-2
            // wumpus along the diagonal.

            // Place agent
            newWorld[0][0][3] = 'A';

            // Place gold
            newWorld[size - 1][size - 1][2] = 'G';

            // Place wumpus
            for (int i = 0; i < size - 2; ++i) {
                int p = randomGenerator.nextInt(2);
                if (p == 0) { // place this wumpus above diagonal
                    newWorld[1 + i][2 + i][1] = 'W';
                } else {
                    newWorld[2 + i][1 + i][1] = 'W';
                }
            }
        }
        return newWorld;
    }
}

