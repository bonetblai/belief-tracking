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

#include <cassert>
#include <string>
#include <iostream>
#include <string.h>
#include <vector>
#include <stdlib.h>

#include "battleship_api.h"

namespace Algorithm {
    unsigned g_seed = 0;
};

namespace Online {
    unsigned g_seed = 0;
};

using namespace std;

struct cell_t {
    int id_;
    bool hit_;
    bool sunk_;
    int segment_;
    int corner_;
    int orientation_;
    bool blocked_;
    cell_t() : id_(0), hit_(false), sunk_(false), segment_(0), corner_(0), orientation_(0), blocked_(false) { }

    void set_ship_id(int ship_id) { id_ = ship_id; }
    void set_ship_segment(int t) { segment_ = t; }
    void set_corner_type(int t) { corner_ = t; }
    void set_horizontal_orientation(bool orientation) { orientation_ = orientation; }

    void set_as_hit() { hit_ = true; }
    void set_as_sunk() { sunk_ = true; }
    void set_as_blocked() { blocked_ = true; }

    int ship_id() const { return id_; }
    bool is_ship_segment() const { return segment_ != 0; }
    bool is_hit() const { return hit_; }
    bool is_sunk() const { return sunk_; }
    bool is_blocked() const { return blocked_; }
};

struct field_t {
    int nrows_;
    int ncols_;
    int ncells_;
    bool allow_adjacent_ships_;

    vector<int> ship_inventory_;
    vector<int> num_ship_segments_to_sink_;
    vector<vector<int> > ship_segments_;

    vector<cell_t> cells_;
    int num_fired_torpedos_;
    int num_remaining_ships_;

    field_t(int nrows, int ncols, const vector<int> &ship_inventory, bool allow_adjacent_ships)
      : nrows_(nrows),
        ncols_(ncols),
        ncells_(nrows_ * ncols_),
        allow_adjacent_ships_(allow_adjacent_ships),
        ship_inventory_(ship_inventory) {
        cells_ = vector<cell_t>(ncells_);
    }

    int num_remaining_ships() const { return num_remaining_ships_; }

    void sample_new_field() {
        int nships = 0;
        for( unsigned i = 0; i < ship_inventory_.size(); ++i ) {
            nships += ship_inventory_[i];
        }
        num_ship_segments_to_sink_ = vector<int>(nships, 0);
        ship_segments_ = vector<vector<int> >(nships);
        cells_ = vector<cell_t>(ncells_);

        num_fired_torpedos_ = 0;
        num_remaining_ships_ = 0;

        int ship_id = 0;
        for( unsigned i = 1; i < ship_inventory_.size(); ++i ) {
            for( int j = 0; j < ship_inventory_[i]; ++j ) {
                ship_segments_[ship_id] = vector<int>();
                add_ship(ship_id, i);
                ++ship_id;
            }
        }
    }

    void add_ship(int ship_id, int size) {
        vector<int> vertical, horizontal;

        // calculate available anchors
        for( int c = 0; c < ncols_; ++c ) {
            for( int r = 0; r < nrows_; ++r ) {
                // check for vertical placement
                if( r + size - 1 < nrows_ ) {
                    bool good = true;
                    for( int d = 0; good && (d < size); ++d ) {
                        good = !cells_[(r + d) * ncols_ + c].is_blocked();
                    }
                    if( good ) {
                        vertical.push_back(r * ncols_ + c);
                    }
                }

                // check for horizontal placement
                if( c + size - 1 < ncols_ ) {
                    bool good = true;
                    for( int d = 0; good && (d < size); ++d ) {
                        good = !cells_[r * ncols_ + (c + d)].is_blocked();
                    }
                    if( good ) {
                        horizontal.push_back(r * ncols_ + c);
                    }
                }
            }
        }

        // pick random anchor
        int anchor = 0;
        bool horizontal_ship = false;
        int pick = lrand48() % 2;

        if( vertical.empty() && horizontal.empty() ) {
            return;
        } else if( horizontal.empty() || (pick == 0 && !vertical.empty()) ) {
            // pick from vertical
            int index = lrand48() % vertical.size();
            anchor = vertical[index];
        } else {
            // pick from horizontal
            int index = lrand48() % horizontal.size();
            anchor = horizontal[index];
            horizontal_ship = true;
        }

        // place selected ship
        int r = anchor / ncols_;
        int c = anchor % ncols_;
        num_ship_segments_to_sink_[ship_id] = size;
        ship_segments_[ship_id].push_back(anchor);
#if 0
        cout << "new ship: id=" << ship_id
             << " @ (" << c << "," << r << ")"
             << ", size=" << size
             << ", horiz=" << (horizontal_ship ? "true" : "false")
             << endl;
#endif

        if( size == 1 ) {
            cells_[r * ncols_ + c].set_ship_id(ship_id);
            cells_[r * ncols_ + c].set_horizontal_orientation(true);
            cells_[r * ncols_ + c].set_ship_segment(2);
            cells_[r * ncols_ + c].set_corner_type(2);
            cells_[r * ncols_ + c].set_as_blocked();
            if( !allow_adjacent_ships_ ) block_surrounding_cells(r, c);
        } else if( horizontal_ship ) {
            cells_[r * ncols_ + c].set_ship_id(ship_id);
            cells_[r * ncols_ + c].set_horizontal_orientation(true);
            cells_[r * ncols_ + c].set_ship_segment(2);
            cells_[r * ncols_ + c].set_corner_type(0);
            cells_[r * ncols_ + c].set_as_blocked();
            if( !allow_adjacent_ships_ ) block_surrounding_cells(r, c);
            for( int d = 1; d < size; ++d ) {
                cells_[r * ncols_ + (c + d)].set_ship_id(ship_id);
                cells_[r * ncols_ + (c + d)].set_horizontal_orientation(true);
                cells_[r * ncols_ + (c + d)].set_ship_segment(1);
                cells_[r * ncols_ + (c + d)].set_as_blocked();
                if( !allow_adjacent_ships_ ) block_surrounding_cells(r, c + d);
                ship_segments_[ship_id].push_back(r * ncols_ + (c + d));
            }
            cells_[r * ncols_ + (c + size - 1)].set_ship_segment(2);
            cells_[r * ncols_ + (c + size - 1)].set_corner_type(1);
        } else {
            cells_[r * ncols_ + c].set_ship_id(ship_id);
            cells_[r * ncols_ + c].set_horizontal_orientation(false);
            cells_[r * ncols_ + c].set_ship_segment(2);
            cells_[r * ncols_ + c].set_corner_type(0);
            cells_[r * ncols_ + c].set_as_blocked();
            if( !allow_adjacent_ships_ ) block_surrounding_cells(r, c);
            for( int d = 1; d < size; ++d ) {
                cells_[(r + d) * ncols_ + c].set_ship_id(ship_id);
                cells_[(r + d) * ncols_ + c].set_horizontal_orientation(false);
                cells_[(r + d) * ncols_ + c].set_ship_segment(1);
                cells_[(r + d) * ncols_ + c].set_as_blocked();
                if( !allow_adjacent_ships_ ) block_surrounding_cells(r + d, c);
                ship_segments_[ship_id].push_back((r + d) * ncols_ + c);
            }
            cells_[(r + size - 1) * ncols_ + c].set_ship_segment(2);
            cells_[(r + size - 1) * ncols_ + c].set_corner_type(1);
        }
        ++num_remaining_ships_;
    }

    void block_surrounding_cells(int r, int c) {
        for( int dc = -1; dc <= 1; ++dc ) {
            if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
            for( int dr = -1; dr <= 1; ++dr ) {
                if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
                cells_[(r + dr) * ncols_ + (c + dc)].set_as_blocked();
            }
        }
    }

    void sample_new_field_2() {
        int nships = 0;
        for( unsigned i = 0; i < ship_inventory_.size(); ++i ) {
            nships += ship_inventory_[i];
        }
        num_ship_segments_to_sink_ = vector<int>(nships, 0);
        ship_segments_ = vector<vector<int> >(nships);
        cells_ = vector<cell_t>(ncells_);

        num_fired_torpedos_ = 0;
        num_remaining_ships_ = 0;

        int ship_id = 0;
        for( int ship_size = ship_inventory_.size(); ship_size > 0; --ship_size ) {
            int nships_to_place = ship_inventory_[ship_size - 1];
            for( ; nships_to_place > 0; --nships_to_place ) {
                int dir = 0, pos = 0;
                do {
                    dir = lrand48() % 4;
                    pos = lrand48() % ncells_;
                } while( collision(dir, pos, ship_size - 1) );
                add_ship(ship_id, dir, pos, ship_size - 1);
                ++ship_id;
            }
        }
    }

    void add_ship(int ship_id, int dir, int pos, int ship_size) {
        int row = pos / ncols_, col = pos % ncols_;

        // canonize direction: 0=North, 1=East, 2=South, 3=West
        if( dir == 2 ) {
            row -= ship_size - 1;
            dir = 0;
        } else if( dir == 3 ) {
            col -= ship_size - 1;
            dir = 1;
        }
        assert((dir == 0) || (dir == 1));
        assert((row >= 0) && (row < nrows_));
        assert((col >= 0) && (col < ncols_));

        // place ship
        ++num_remaining_ships_;
        num_ship_segments_to_sink_[ship_id] = ship_size;
        for( int d = 0; d < ship_size; ++d ) {
            int r = row + (dir == 0 ? d : 0);
            int c = col + (dir == 1 ? d : 0);
            int pos = r * ncols_ + c;
            ship_segments_[ship_id].push_back(pos);
            cells_[pos].set_ship_id(ship_id);
            cells_[pos].set_horizontal_orientation(dir == 1);
            cells_[pos].set_as_blocked();
            if( !allow_adjacent_ships_ ) block_surrounding_cells(r, c);
            cells_[pos].set_ship_segment(1);
        }
        cells_[row * ncols_ + col].set_ship_segment(2);
        cells_[row * ncols_ + col].set_corner_type(0);
        row += dir == 0 ? ship_size - 1 : 0;
        col += dir == 1 ? ship_size - 1 : 0;
        cells_[row * ncols_ + col].set_ship_segment(2);
        cells_[row * ncols_ + col].set_corner_type(1);
        if( ship_size == 1 ) {
            cells_[row * ncols_ + col].set_horizontal_orientation(true);
            cells_[row * ncols_ + col].set_corner_type(2);
        }
    }

    bool collision(int dir, int pos, int ship_size) {
        int row = pos / ncols_, col = pos % ncols_;

        // canonize direction: 0=North, 1=East, 2=South, 3=West
        if( dir == 2 ) {
            row -= ship_size - 1;
            dir = 0;
        } else if( dir == 3 ) {
            col -= ship_size - 1;
            dir = 1;
        }
        assert((dir == 0) || (dir == 1));

        // check that ship falls within bounds
        if( (row < 0) || (row >= nrows_) ) return true;
        if( (col < 0) || (col >= ncols_) ) return true;

        // check that ship doesn't collide with another ship or border
        for( int d = 0; d < ship_size; ++d ) {
            int r = row + (dir == 0 ? d : 0);
            int c = col + (dir == 1 ? d : 0);
            if( (r >= nrows_) || (c >= ncols_) ) return true;
            if( cells_[r * ncols_ + c].is_blocked() ) return true;
        }

        // there is no collision
        return false;
    }

    int fire_torpedo(int r, int c, bool verbose = false) {
        if( verbose ) cout << "torpedo fired @ (" << c << "," << r << "):";
        ++num_fired_torpedos_;
        int index = r * ncols_ + c;
        if( cells_[index].is_ship_segment() ) {
            if( verbose ) cout << " obs=hit" << endl;
            bool was_sunk = cells_[index].is_sunk();
            int ship_id = cells_[index].ship_id();
            if( !cells_[index].is_hit() && (num_ship_segments_to_sink_[ship_id] > 0) ) {
                //if( verbose ) cout << "hit: id=" << ship_id << endl;
                if( --num_ship_segments_to_sink_[ship_id] == 0 ) {
                    for( unsigned i = 0; i < ship_segments_[ship_id].size(); ++i ) {
                        int s = ship_segments_[ship_id][i];
                        int sc = s % ncols_, sr = s / ncols_;
                        cells_[sr * ncols_ + sc].set_as_sunk();
                    }
                    //if( verbose ) cout << "sunk: id=" << ship_id << endl;
                }
            }
            cells_[index].set_as_hit();

            if( !was_sunk && cells_[index].is_sunk() ) {
                if( --num_remaining_ships_ == 0 ) {
                    cout << "Entire fleet sunk w/ #torpedos = " << num_fired_torpedos_ << endl;
                }
            }
            return 1;
        } else {
            if( verbose ) cout << " obs=water" << endl;
            return 0;
        }
    }

    void print(ostream &os) const {
        os << "  ";
        for( int c = 0; c < ncols_; ++c )
            os << c;
        os << endl;
        for( int r = 0; r < nrows_; ++r ) {
            os << r << " ";
            for( int c = 0; c < ncols_; ++c ) {
                os << (cells_[r * ncols_ + c].is_ship_segment() ? "X" : " ");
            }
            os << endl;
        }
    }
};

void usage(ostream &os) {
    os << endl
       << "Usage: battleship [{-t | --ntrials} <ntrials>]" << endl
       << "                  [{-r | --nrows} <nrows>]" << endl
       << "                  [{-c | --ncols} <ncols>]" << endl
       << "                  [{-i | --inventory} <inventory>]" << endl
       << "                  [{-s | --seed} <seed>]" << endl
       << "                  [{-v | --verbose}]" << endl
       << "                  [{-S | --store-fields}]" << endl
       << "                  [{-p | --policy} <policy>]" << endl
       << "                  [{-? | --help}]" << endl
       << endl
       << "where <ntrials> is a non-negative integer tells the number of games to" << endl
       << "play (default is 1), <nrows> and <ncols> are positive integers tell the" << endl
       << "dimensions of the battlefield (default is 10x10), <inventory> is a string" << endl
       << "that tells the number and size of ships to be randomly located in the" << endl
       << "battlefield (default is \"2:1,3:1,4:1,5:1\"), <seed> tells the seed for the" << endl
       << "random number generator (default is 0), and <policy> is a string describing"  << endl
       << "the policy to use," << endl
       << endl
       << "For example," << endl
       << endl
       << "  ./battleship -r 10 -c 10 -i 2:1,3:1,4:1,5:1 -t 100" << endl
       << endl
       << "performs an experiment consisting of 100 trials on a 10x10 battlefield" << endl
       << "with 4 ships of sizes 2, 3, 4 and 5 (one of each size)." << endl
       << endl
       << "If \"store-fields\" is enabled, all the <ntrials> instances of the game" << endl
       << "are generated and store before solving anyone. This is useful when comparing" << endl
       << "different (randomized) policies across the exactly the same collection of" << endl
       << "instances." << endl
       << endl;
}

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 10;
    int ncols = 10;
    bool verbose = false;
    bool store_fields = false;
    string policy = "no-such-policy";
    bool allow_adjacent_ships = false;
    char *inventory_str = strdup("2:1,3:1,4:1,5:1");
    vector<int> ship_inventory;

    // parse arguments
    for( ++argv, --argc; (argc > 1) && (**argv == '-'); ++argv, --argc ) {
        if( ((*argv)[1] == 't') || (string(*argv) == "--ntrials") ) {
            ntrials = strtol(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'r') || (string(*argv) == "--nrows") ) {
            nrows = strtol(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'c') || (string(*argv) == "--ncols") ) {
            ncols = strtol(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'i') || (string(*argv) == "--inventory") ) {
            free(inventory_str);
            inventory_str = strdup(argv[1]);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 's') || (string(*argv) == "--seed") ) {
            Online::g_seed = strtoul(argv[1], 0, 0);
            ++argv;
            --argc;
        } else if( ((*argv)[1] == 'v') || (string(*argv) == "--verbose") ) {
            verbose = true;
        } else if( ((*argv)[1] == 'S') || (string(*argv) == "--store-fields") ) {
            store_fields = true;
        } else if( ((*argv)[1] == 'p') || (string(*argv) == "--policy") ) {
            policy = argv[1];
            ++argv;
            --argc;
        } else if( ((*argv)[1] == '?') || (string(*argv) == "--help") ) {
            usage(cout);
            exit(-1);
        } else {
            cout << "error: unexpected argument: " << argv[0] << endl;
        }
    }

    // parse ship inventory
    int max_ship_size = 0;
    vector<pair<int,int> > ships;
    char *ptr = strtok(inventory_str, ":");
    while( ptr != 0 ) {
        int ship_size = atoi(ptr);
        char *qptr = strtok(0, ",");
        int ship_qty = atoi(qptr);
        ships.push_back(make_pair(ship_size, ship_qty));
        max_ship_size = ship_size > max_ship_size ? ship_size : max_ship_size;
        ptr = strtok(0, ":");
    }
    free(inventory_str);
    ship_inventory = vector<int>(1 + max_ship_size, 0);
    for( unsigned i = 0; i < ships.size(); ++i ) {
        ship_inventory[ships[i].first] = ships[i].second;
    }

    // set seed
    cout << "main: seed= " << Online::g_seed << endl;
    Random::set_seed(Online::g_seed);

    // create empty battleship field
    vector<field_t*> fields;
    if( store_fields ) {
        // if store_fields, generate all the battleship games before playing.
        // Used to recreate games for solving them with another algorithm.
        fields.reserve(ntrials);
        for( int trial = 0; trial < ntrials; ++trial ) {
            fields.push_back(new field_t(nrows, ncols, ship_inventory, allow_adjacent_ships));
            fields.back()->sample_new_field_2();
        }
    } else {
        fields.push_back(new field_t(nrows, ncols, ship_inventory, allow_adjacent_ships));
    }

    // create player
    Battleship::api_t player(nrows, ncols, &ship_inventory[0], max_ship_size, true, allow_adjacent_ships);
    if( policy != "no-such-policy" ) player.select_policy(policy);
    cout << "using: policy=" << player.current_policy()->name() << endl;

    // run for the specified number of trials
    int max_steps = nrows * ncols;
    int total_torpedos = 0;
    for( int trial = 0; trial < ntrials; ++trial ) {
        cout << "trial: t=" << trial << ": " << flush;
        if( verbose ) cout << endl;

        // get field
        field_t *field = 0;
        if( store_fields ) {
            field = fields[trial];
        } else {
            field = fields[0];
            field->sample_new_field_2();
        }
        if( verbose ) field->print(cout);

        // play the game
        player.prepare_new_trial();
        for( int shoots = 0; shoots < max_steps; ++shoots ) {
            int cell = player.select_action();
            int obs = field->fire_torpedo(cell % ncols, cell / ncols, verbose);
            player.apply_action_and_update(cell, obs);
            ++total_torpedos;
            if( field->num_remaining_ships() == 0 ) break;
        }

        if( field->num_remaining_ships() > 0 )
            cout << "Max steps " << max_steps << " reached!" << endl;
    }
    cout << "avg. #torpedos=" << (float)total_torpedos / (float)ntrials << endl;

    // release memory
    for( size_t i = 0; i < fields.size(); ++i )
        delete fields[i];

    return 0;
}

