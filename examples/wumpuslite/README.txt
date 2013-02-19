Text-interface implementation of Wumpus

To compile, execute "make"

Main program is in WorldApplication.  The parameters are:

Parameter: -d <n>  or  --dimension <n>
Sets the dimension of the square grid.  Default is 4.

Parameter: -p <n>  or  --npits <n>
Sets the number of pits in the workd.  Default is 2.

Parameter: -w <n>  or  --nwumpus <n>
Sets the number of wumpus in the workd.  Default is 2.

Parameter: -s <n>  or  --steps <n>
Sets the maximun number of steps in a given trial.  A value of 0
results in the value of 3*n^2 where n is the dimension.  Default is 0.

Parameter: -t <n>  or  --trials <n>
Sets the number of trials to execute.  Default is 1.

Parameter: -e <n>  or  --num-expansions <n>
Sets the number of expansions for the lookahead tree.  Default is 50.

Parameter: -h <n>  or  --mdp-horizon <n>
Sets the depth of the lookahead tree.  Default is 50.

Parameter: -a
Specifies that the agent should be located randomly in the grid.
Default is no.

Parameter: -r <n>  or  --random-seet <n>
Sets the seed of the random number generator.

Parameter: -m  or  --moving
Specifies that the wumpus moves non-deterministically.  Default is no.

Parameter: -v  or  --verbose
Specifies verbose output.  Default is no.

