/*
 *  Copyright (C) 2012 Universidad Simon Bolivar
 *
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#ifndef WUMPUS_DEFS_H
#define WUMPUS_DEFS_H

// actions
enum { ActionMoveNorth = 0, ActionMoveEast = 1, ActionMoveSouth = 2, ActionMoveWest = 3 };
enum { ActionMoveForward = 0, ActionTurnRight = 1, ActionTurnLeft = 2, ActionNoop = 3 };
enum { ActionShoot = 4, ActionGrab = 5, ActionExit = 6 };

const char* nesw_movements_action_str[] = {
    "MoveNorth",
    "MoveEast",
    "MoveSouth",
    "MoveWest",
    "Shoot",
    "Grab",
    "Exit"
};

const char* no_nesw_movements_action_str[] = {
    "MoveForward",
    "TurnRight",
    "TurnLeft",
    "Noop",
    "Shoot",
    "Grab",
    "Exit"
};

inline const char* action_name(int action, bool nesw_movements) {
    return nesw_movements ? nesw_movements_action_str[action] : no_nesw_movements_action_str[action];
}

// heading and position of agent
enum { North = 0, East = 1, South = 2, West = 3 };
enum { OutsideCave = -1 };

// feedback
enum { Glitter = 0x1, Breeze = 0x2, Stench = 0x4, Fell = 8, Eaten = 9 };

const char* obs_str[] = {
    "(null,null,null)",
    "(glitter,null,null)",
    "(null,breeze,null)",
    "(glitter,breeze,null)",
    "(null,null,stench)",
    "(glitter,null,stench)",
    "(null,breeze,stench)",
    "(glitter,breeze,stench)",
    "Fell in Pit",
    "Eaten by Wumpus"
};

inline const char* obs_name(int obs) {
    if( obs <= Eaten ) {
        return obs_str[obs];
    } else {
        return "wumpus_at_cell";
    }
}

const char* heading_str[] = { "north", "east", "south", "west" };

inline const char* heading_name(int heading) {
    return heading_str[heading];
}

inline int target_cell(int pos, int heading, int action, int rows, int cols, bool nesw_movements) {
    int npos = pos;
    int row = pos / cols;
    int col = pos % cols;

    if( nesw_movements ) {
        if( action == ActionMoveNorth ) {
            if( row < rows - 1 ) npos = (row + 1) * cols + col;
        } else if( action == ActionMoveEast ) {
            if( col < cols - 1 ) npos = row * cols + col + 1;
        } else if( action == ActionMoveSouth ) {
            if( row > 0 ) npos = (row - 1) * cols + col;
        } else if( action == ActionMoveWest ) {
            if( col > 0 ) npos = row * cols + col - 1;
        }
    } else {
        if( action == ActionMoveForward ) {
            if( heading == North ) {
                if( row < rows - 1 ) npos = (row + 1) * cols + col;
            } else if( heading == East ) {
                if( col < cols - 1 ) npos = row * cols + (col + 1);
            } else if( heading == South ) {
                if( row > 0 ) npos = (row - 1) * cols + col;
            } else {
                assert(heading == West);
                if( col > 0 ) npos = row * cols + (col - 1);
            }
        }
    }
    return npos;
}

inline int target_heading(int heading, int action) {
    if( (action == ActionTurnLeft) || (action == ActionTurnRight) ) {
        return (action == ActionTurnLeft ? heading + 3 : heading + 1) & 0x3;
    } else {
        return heading;
    }
}

#endif

