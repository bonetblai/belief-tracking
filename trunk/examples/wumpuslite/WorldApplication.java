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

    public static void main(String args[]) {
        int worldSize = 4;
        int numTrials = 1;
        int maxSteps = 0;
        int numPits = 2;
        int numWumpus = 2;

        int numExpansions = 50;
        int MDPHorizon = 50;

        boolean moving = false;
        boolean verbose = false;
        boolean checkExploration = false; // minisat should be in the path for this to work
        boolean randomAgentLoc = false;
        boolean userDefinedSeed = false;
		
        //String outFilename = "wumpus_out.txt";

        Random rand = new Random();
        int seed = rand.nextInt();

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
            if (arg.equals("-p") || arg.equals("--npits")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    numPits = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if number of wumpus is specified
            if (arg.equals("-w") || arg.equals("--nwumpus")) {
                if (Integer.parseInt(args[i+1]) >= 0) {
                    numWumpus = Integer.parseInt(args[i+1]);
                }
                i++;
            }

            // if the maximum number of steps is specified
            else if (arg.equals("-s") || arg.equals("--steps")) {
                maxSteps = Integer.parseInt(args[i+1]);
                i++;
            }	    	

            // if the number of trials is specified
            else if (arg.equals("-t") || arg.equals("--trials")) {
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
                seed = Integer.parseInt(args[i+1]);
                userDefinedSeed = true;
                i++;
            }

            // if the non-determinism is specified
            else if (arg.equals("-m") || arg.equals("--moving")) {
                moving = true;
            }

            // if verbose is specified
            else if (arg.equals("-v") || arg.equals("--verbose")) {
                verbose = true;
            }
        }

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
            System.out.println("Random number seed: " + seed);
            System.out.println("Non-Deterministic Behavior: " + moving + "\n");

            char[][][] wumpusWorld = generateRandomWumpusWorld(seed, worldSize, numPits, numWumpus, randomAgentLoc);
            Environment wumpusEnvironment = new Environment(worldSize, numPits, numWumpus, wumpusWorld);

            int trialScores[] = new int[numTrials];
            int trialSteps[] = new int[numTrials];
            boolean deaths[] = new boolean[numTrials];
            boolean golds[] = new boolean[numTrials];
            boolean explored[] = new boolean[numTrials];
            boolean unvisitedSafeCell[] = new boolean[numTrials];
            long elapsedTime[] = new long[numTrials];

            for (int currTrial = 0; currTrial < numTrials; currTrial++) {
                Simulation trial = new Simulation(wumpusEnvironment, maxSteps, moving, verbose);
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

                if (userDefinedSeed == true) {
                    wumpusWorld = generateRandomWumpusWorld(++seed, worldSize, numPits, numWumpus, randomAgentLoc);	
                } else {
                    wumpusWorld = generateRandomWumpusWorld(rand.nextInt(), worldSize, numPits, numWumpus, randomAgentLoc);
                }

                wumpusEnvironment = new Environment(worldSize, numPits, numWumpus, wumpusWorld);
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
	
    public static char[][][] generateRandomWumpusWorld(int seed, int size, int numPits, int numWumpus, boolean randomlyPlaceAgent) {
        char[][][] newWorld = new char[size][size][4];
        boolean[][] pit_occupied = new boolean[size][size];
        boolean[][] wumpus_occupied = new boolean[size][size];

        Random randGen = new Random(seed);

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

        // default agent location and orientation
        int agentXLoc = 0;
        int agentYLoc = 0;
        char agentIcon = 'A';

        // randomly generate agent location and orientation
        if (randomlyPlaceAgent == true) {
            agentXLoc = randGen.nextInt(size);
            agentYLoc = randGen.nextInt(size);
            switch (randGen.nextInt(4)) {
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
            int x = randGen.nextInt(size);
            int y = randGen.nextInt(size);
            while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || ((x < 2) && (y < 2))) {
                x = randGen.nextInt(size);
                y = randGen.nextInt(size);    	   
            }
            pit_occupied[x][y] = true;
            newWorld[x][y][0] = 'P';
        }

        // Wumpus Generation
        for (int i = 0; i < numWumpus; i++) {
            int x = randGen.nextInt(size);
            int y = randGen.nextInt(size);
            while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || wumpus_occupied[x][y] || ((x < 2) && (y < 2))) {
                x = randGen.nextInt(size);
                y = randGen.nextInt(size);   
            }
            wumpus_occupied[x][y] = true;
            newWorld[x][y][1] = 'W';
        }
		
        // Gold Generation
        int x = randGen.nextInt(size);
        int y = randGen.nextInt(size);
        while ((x == agentXLoc && y == agentYLoc) || pit_occupied[x][y] || wumpus_occupied[x][y]) {
            x = randGen.nextInt(size);
            y = randGen.nextInt(size);   
        }
        newWorld[x][y][2] = 'G';
        return newWorld;
    }
}

