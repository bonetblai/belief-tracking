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

class AgentFunctionProxy {

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
            nativeLibraryUrl = AgentFunctionProxy.class.getResource(url.toString() + ext);
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
    private native void init_cpp_side(boolean moving, int nrows, int ncols, int npits, int nwumpus, int narrows, boolean nesw_movements);
    private native void set_policy_parameters(int num_expansions, int mdp_horizon);
    private native void prepare_new_trial();
    private native void update(int obs);
    private native void apply_action_and_update(int action, int obs);
    private native int select_action();
    private native int is_world_explored();
    private native int is_there_a_safe_cell();

    // string to store the agent's name (do not remove this variable)
    private String agentName = "Agent Smith";

    private int[] actionTable;
    private int last_action;

    public AgentFunctionProxy(boolean moving, int size, int npits, int nwumpus, int narrows) {
        // Initialize the C++ side
        init_cpp_side(moving, size, size, npits, nwumpus, 0, false);

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
        last_action = -1;
    }

    public void prepareNewTrial() {
        prepare_new_trial();
    }

    public void setPolicyParameters(int numExpansions, int MDPHorizon) {
        set_policy_parameters(numExpansions, MDPHorizon);
    }

    public int process(TransferPercept tp) {
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

        // update agent and get next action
        if( last_action == -1 ) {
            update(obs);
        } else {
            apply_action_and_update(last_action, obs);
        }
        last_action = select_action();
        return actionTable[last_action];
    }

    public boolean isWorldExplored() {
        return is_world_explored() == 1;
    }

    public boolean isThereAnUnvisitedSafeCell() {
        return is_there_a_safe_cell() == 1;
    }
	
    // public method to return the agent's name (do not remove this method)
    public String getAgentName() {
        return agentName;
    }
}

