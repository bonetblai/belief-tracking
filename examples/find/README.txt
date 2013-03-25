Find domains comprises a grid in which an object of unknown
size and shape is hidden, and an agent that must locate the
object by probing the cells of the grid. Every time a cell
is probed, the agent learns whether or not the cell is part
of the object, and the number of neighboring cells that
are part of it.

The only available action is to probe cell (x,y), called
probe(x,y), which returns two tokens of information: a
boolean indicating whether the cell (x,y) is part of the
object, and a number between 1 and 8 counting the neighbor
cells that are part of the object. The goal is to have each
cell part of the object probed.

Hence, there are boolean variables obj(x,y) and probed(x,y) that
denote whether the cell (x,y) is part of the object or has been
probed, an observable boolean variable positive(x,y) that denote
whether the probed cell (x,y) is part of the object, and an
observable integer variable count(x,y) that denote the number
of cells adjacent to (x,y) that are part of the object.

The causal width of the domain is 8 while its width is unbounded.

