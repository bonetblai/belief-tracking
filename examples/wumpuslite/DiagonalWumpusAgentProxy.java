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

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.net.URL;
import java.io.IOException;

class DiagonalWumpusAgentProxy implements AgentProxy {

    // Dynamic link the C++ code
    static {
        if( !loadEmbeddedLibrary() ) {
            System.loadLibrary("agent");
        }
    }

    private static boolean loadEmbeddedLibrary() {
        boolean libraryFound = false;

        // Attempt to locate native library within JAR at following location:
        // /platform/libagent.[so|dylib|dll]
        String[] allowedExtensions = new String[] {"so", "dylib", "dll"};
        StringBuilder url = new StringBuilder();
        url.append("/platform/libagent.");

        // Loop through extensions, stopping after finding first one
        URL nativeLibraryUrl = null;
        for( String ext : allowedExtensions ) {
            nativeLibraryUrl = DiagonalWumpusAgentProxy.class.getResource(url.toString() + ext);
            if( nativeLibraryUrl != null ) break;
        }

        if( nativeLibraryUrl != null ) {
            // native library found within JAR, extract and load
            try {
                final File libfile = File.createTempFile("libagent-", ".lib");
                libfile.deleteOnExit(); // just in case

                final InputStream in = nativeLibraryUrl.openStream();
                final OutputStream out = new BufferedOutputStream(new FileOutputStream(libfile));

                int len = 0;
                byte[] buffer = new byte[8192];
                while ((len = in.read(buffer)) > -1)
                    out.write(buffer, 0, len);
                out.close();
                in.close();

                System.load(libfile.getAbsolutePath());
                libfile.delete();
                libraryFound = true;
            } catch (IOException x) {
                // failed, do nothing
            }
        }
        return libraryFound;
    }

    // Native (C++) methods
    private long agentPtr_;
    private native void init_cpp_side(int dim);
    private native void set_policy_parameters(int num_expansions, int mdp_horizon);
    private native void prepare_new_trial();
    private native void update(int obs);
    private native void apply(int action);
    private native int select_action();

    // string to store the agent's name (do not remove this variable)
    private String agentName = "Agent Smith";

    private int[] actionTable;
    private int pending_action;
    private int low_level_pending_action;
    private char target_direction;

    public DiagonalWumpusAgentProxy(int size) {
        // Initialize the C++ side
        init_cpp_side(size);

        // this integer array will store the agent actions
        actionTable = new int[7];
        actionTable[0] = Action.GO_FORWARD;
        actionTable[1] = Action.TURN_RIGHT;
        actionTable[2] = Action.TURN_LEFT;
        actionTable[3] = -1; // Noop
        actionTable[4] = Action.SHOOT;
        actionTable[5] = Action.GRAB;
        actionTable[6] = -1; // EXIT

        // initialize
        pending_action = -1;
        low_level_pending_action = -1;
        target_direction = 'U';
    }

    public void prepareNewTrial() {
        prepare_new_trial();
    }

    public void setPolicyParameters(int numExpansions, int MDPHorizon) {
        set_policy_parameters(numExpansions, MDPHorizon);
    }

    public int process(int[] location, char direction, TransferPercept tp) {
        System.out.println("Current Pos = (" + location[0] + "," + location[1] + ")");
        // read in the current percepts
        boolean bump = tp.getBump();
        boolean glitter = tp.getGlitter();
        boolean breeze = tp.getBreeze();
        boolean stench = tp.getStench();
        boolean scream = tp.getScream();

        // build observation
        int obs = 0;
        if( glitter == true ) obs += 1;
        if( breeze == true ) obs += 2;
        if( stench == true ) obs += 4;

        //  process observation
        System.out.println("OBS=" + obs);
        update(obs);

        // while direction is not intendent, turn right
        System.out.println("tdir=" + target_direction + ", dir=" + direction);
        if( (target_direction != 'U') && (target_direction != direction) ) {
            System.out.println("Action=TURN-RIGHT");
            return actionTable[1];
        }

        // if pending action, apply it
        if( pending_action != -1 ) {
            apply(low_level_pending_action);
            int action = actionTable[pending_action];
            pending_action = -1;
            System.out.println("ACTION=" + action);
            return action;
        }

        // obtain action and set target direction
        low_level_pending_action = select_action();
        pending_action = low_level_pending_action;
        target_direction = direction;
        if( low_level_pending_action < 4 ) {
            pending_action = 0;
            if( low_level_pending_action == 0 ) target_direction = 'N';
            if( low_level_pending_action == 1 ) target_direction = 'E';
            if( low_level_pending_action == 2 ) target_direction = 'S';
            if( low_level_pending_action == 3 ) target_direction = 'W';
        }
        System.out.println("act=" + pending_action + ", low-act=" + low_level_pending_action + ", tdir=" + target_direction);
        return process(location, direction, tp);
    }

    public boolean isWorldExplored() {
        return true;
    }

    public boolean isThereAnUnvisitedSafeCell() {
        return false;
    }
	
    // public method to return the agent's name (do not remove this method)
    public String getAgentName() {
        return agentName;
    }
}

