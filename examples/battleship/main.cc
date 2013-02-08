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

    std::vector<int> ship_inventory_;
    std::vector<int> num_ship_segments_to_sink_;
    std::vector<std::vector<int> > ship_segments_;

    std::vector<cell_t> cells_;
    int num_fired_torpedos_;
    int num_remaining_ships_;

    field_t(int nrows, int ncols, const std::vector<int> &ship_inventory, bool allow_adjacent_ships)
      : nrows_(nrows),
        ncols_(ncols),
        ncells_(nrows_ * ncols_),
        allow_adjacent_ships_(allow_adjacent_ships),
        ship_inventory_(ship_inventory) {
        cells_ = std::vector<cell_t>(ncells_);
    }

    int num_remaining_ships() const { return num_remaining_ships_; }

    void sample_new_field() {
        int nships = 0;
        for( unsigned i = 0; i < ship_inventory_.size(); ++i ) {
            nships += ship_inventory_[i];
        }
        num_ship_segments_to_sink_ = std::vector<int>(nships, 0);
        ship_segments_ = std::vector<std::vector<int> >(nships);
        cells_ = std::vector<cell_t>(ncells_);

        num_fired_torpedos_ = 0;
        num_remaining_ships_ = 0;

        int ship_id = 0;
        for( unsigned i = 1; i < ship_inventory_.size(); ++i ) {
            for( int j = 0; j < ship_inventory_[i]; ++j ) {
                ship_segments_[ship_id] = std::vector<int>();
                add_ship(ship_id, i);
                ++ship_id;
            }
        }
    }

    void add_ship(int ship_id, int size) {
        std::vector<int> vertical, horizontal;

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
        std::cout << "new ship: id=" << ship_id
                  << " @ (" << c << "," << r << ")"
                  << ", size=" << size
                  << ", horiz=" << (horizontal_ship ? "true" : "false")
                  << std::endl;
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
        num_ship_segments_to_sink_ = std::vector<int>(nships, 0);
        ship_segments_ = std::vector<std::vector<int> >(nships);
        cells_ = std::vector<cell_t>(ncells_);

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
        if( verbose ) std::cout << "torpedo fired @ (" << c << "," << r << "):";
        ++num_fired_torpedos_;
        int index = r * ncols_ + c;
        if( cells_[index].is_ship_segment() ) {
            if( verbose ) std::cout << " obs=hit" << std::endl;
            bool was_sunk = cells_[index].is_sunk();
            int ship_id = cells_[index].ship_id();
            if( !cells_[index].is_hit() && (num_ship_segments_to_sink_[ship_id] > 0) ) {
                //if( verbose ) std::cout << "hit: id=" << ship_id << std::endl;
                if( --num_ship_segments_to_sink_[ship_id] == 0 ) {
                    for( unsigned i = 0; i < ship_segments_[ship_id].size(); ++i ) {
                        int s = ship_segments_[ship_id][i];
                        int sc = s % ncols_, sr = s / ncols_;
                        cells_[sr * ncols_ + sc].set_as_sunk();
                    }
                    //if( verbose ) std::cout << "sunk: id=" << ship_id << std::endl;
                }
            }
            cells_[index].set_as_hit();

            if( !was_sunk && cells_[index].is_sunk() ) {
                if( --num_remaining_ships_ == 0 ) {
                    std::cout << "Entire fleet sunk w/ #torpedos = " << num_fired_torpedos_ << std::endl;
                }
            }
            return 1;
        } else {
            if( verbose ) std::cout << " obs=water" << std::endl;
            return 0;
        }
    }

    void print(std::ostream &os) const {
        os << "  ";
        for( int c = 0; c < ncols_; ++c )
            os << c;
        os << std::endl;
        for( int r = 0; r < nrows_; ++r ) {
            os << r << " ";
            for( int c = 0; c < ncols_; ++c ) {
                os << (cells_[r * ncols_ + c].is_ship_segment() ? "X" : " ");
            }
            os << std::endl;
        }
    }
};

int main(int argc, const char **argv) {
    int ntrials = 1;
    int nrows = 10;
    int ncols = 10;
    int seed = 0;
    bool verbose = false;
    bool save_fields = false;

    string policy = "base-policy:direct";
    int width = 100;
    int depth = 10;

    bool allow_adjacent_ships = false;
    char *inventory_str = strdup("2:1,3:1,4:1,5:1");
    std::vector<int> ship_inventory;

    --argc;
    ++argv;
    while( (argc > 0) && (**argv == '-') ) {
        if( !strcmp(argv[0], "-t") || !strcmp(argv[0], "--ntrials") ) {
            ntrials = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-r") || !strcmp(argv[0], "--nrows") ) {
            nrows = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-c") || !strcmp(argv[0], "--ncols") ) {
            ncols = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-i") || !strcmp(argv[0], "--inventory") ) {
            free(inventory_str);
            inventory_str = strdup(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-s") || !strcmp(argv[0], "--seed") ) {
            seed = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-v") || !strcmp(argv[0], "--verbose") ) {
            verbose = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-S") || !strcmp(argv[0], "--save-fields") ) {
            save_fields = true;
            --argc;
            ++argv;
        } else if( !strcmp(argv[0], "-p") || !strcmp(argv[0], "--policy") ) {
            policy = argv[1];
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-w") || !strcmp(argv[0], "--width") ) {
            width = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else if( !strcmp(argv[0], "-d") || !strcmp(argv[0], "--depth") ) {
            depth = atoi(argv[1]);
            argc -= 2;
            argv += 2;
        } else {
            std::cout << "error: unexpected argument: " << argv[0] << std::endl;
            --argc;
            ++argv;
        }
    }

    // parse ship inventory
    int max_ship_size = 0;
    std::vector<std::pair<int,int> > ships;
    char *ptr = strtok(inventory_str, ":");
    while( ptr != 0 ) {
        int ship_size = atoi(ptr);
        char *qptr = strtok(0, ",");
        int ship_qty = atoi(qptr);
        ships.push_back(std::make_pair(ship_size, ship_qty));
        max_ship_size = ship_size > max_ship_size ? ship_size : max_ship_size;
        ptr = strtok(0, ":");
    }
    free(inventory_str);
    ship_inventory = std::vector<int>(1 + max_ship_size, 0);
    for( unsigned i = 0; i < ships.size(); ++i ) {
        ship_inventory[ships[i].first] = ships[i].second;
    }

    // set seed
    unsigned short seeds[3];
    seeds[0] = seeds[1] = seeds[2] = seed;
    seed48(seeds);

    // create empty battleship field
    std::vector<field_t*> fields;
    if( save_fields ) {
        // if save_fields, generate all the battleship games before playing.
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
    Battleship::api_t player(nrows, ncols, &ship_inventory[0], max_ship_size, true, allow_adjacent_ships, false);

    // select policy and parameters
    unsigned pos = policy.find(":");
    string policy_arg1 = policy.substr(0, pos);
    string policy_arg2 = policy.substr(pos + 1);

    if( policy_arg2.find("aot") != string::npos ) {
        player.set_policy_parameters(width, depth, 0.5, 1);
    } else if( policy_arg2.find("uct") != string::npos ) {
        player.set_policy_parameters(width, depth, 0, 0);
    } else {
        player.set_policy_parameters(width, depth, 1, 0);
    }
    player.select_policy(policy_arg1, policy_arg2);

    // run for the specified number of trials
    int max_steps = nrows * ncols;
    int total_torpedos = 0;
    for( int trial = 0; trial < ntrials; ++trial ) {
        std::cout << "TRIAL " << trial << ": " << std::flush;
        if( verbose ) std::cout << std::endl;

        // get field
        field_t *field = 0;
        if( save_fields ) {
            field = fields[trial];
        } else {
            field = fields[0];
            field->sample_new_field_2();
        }
        if( verbose ) field->print(std::cout);

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
            std::cout << "Max steps " << max_steps << " reached!" << std::endl;
    }
    std::cout << "avg. #torpedos=" << (float)total_torpedos / (float)ntrials << std::endl;

    return 0;
}

