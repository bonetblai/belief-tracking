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

#ifndef BATTLESHIP_BELIEF_H
#define BATTLESHIP_BELIEF_H

#include "base_belief.h"

#include <stdlib.h>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <list>
#include <vector>
#include <math.h>

#pragma GCC diagnostic ignored "-Wunused-variable"

//#define DEBUG

namespace Battleship {

class belief_t : public base_belief_t {
  private:
    static std::list<belief_t*> beliefs_;

    static bool simple_observations_;
    static bool allow_adjacent_ships_;
    static int *ship_inventory_;
    static float *cell_probabilities_;

  protected:
    arc_consistency_t beams_;

  public:
    belief_t() : base_belief_t(), beams_(ncols_, allow_adjacent_ships_) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_.set_domain(cell, new var_beam_t(1, num_particles_));
        }
    }
    explicit belief_t(const belief_t &bel)
      : base_belief_t(bel), beams_(ncols_, allow_adjacent_ships_) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_.set_domain(cell, new var_beam_t(*bel.beams_.domain(cell)));
        }
    }
#if 0
    belief_t(belief_t &&bel)
      : base_belief_t(std::move(bel)), beams_(ncols_, allow_adjacent_ships_) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_.set_domain(cell, bel.beams_.domain(cell));
            bel.beams_.set_domain(cell, 0);
        }
    }
#endif
    virtual ~belief_t() { }

    static belief_t* allocate() {
        if( beliefs_.empty() ) {
            return new belief_t;
        } else {
            belief_t *belief = beliefs_.front();
            beliefs_.pop_front();
            assert(belief != 0);
            belief->clear();
            return belief;
        }
    }
    static void deallocate(belief_t *belief) {
        if( belief != 0 ) {
            beliefs_.push_front(belief);
        }
    }
    static void release_memory() {
        for( std::list<belief_t*>::const_iterator it = beliefs_.begin(); it != beliefs_.end(); ++it )
            delete *it;
        beliefs_.clear();
    }

    static void initialize(int nrows,
                           int ncols,
                           int max_ship_size,
                           const int *ship_inventory,
                           bool simple_observations,
                           bool allow_adjacent_ships) {
        base_belief_t::initialize(nrows,
                                  ncols,
                                  max_ship_size,
                                  ship_inventory,
                                  allow_adjacent_ships);
        simple_observations_ = simple_observations;
        allow_adjacent_ships_ = allow_adjacent_ships;
        ship_inventory_ = new int[1 + max_ship_size_];
        cell_probabilities_ = new float[1 + max_ship_size_];
        memcpy(ship_inventory_, ship_inventory, (1 + max_ship_size_) * sizeof(int));
    }
    static void finalize() {
        release_memory();
        delete[] cell_probabilities_;
        delete[] ship_inventory_;
        base_belief_t::finalize();
    }

    static bool consistent_particle(int row, int col, int particle) {
        placement_t plc(particle);
        if( plc.hit_ || (plc.nhits_ > 0) ) {
            return false;
        } else if( (plc.size_ > 0) &&
                   (ship_inventory_[plc.size_] == 0) ) {
            return false;
        } else if( (plc.size_ == 0) &&
                   (plc.horiz_ || (plc.anchor_ > 0) || (plc.nhits_ > 0)) ) {
            return false;
        } else if( (plc.size_ == 1) && plc.horiz_ ) {
            return false;
        } else if( (plc.size_ > 0) && (plc.size_ <= plc.anchor_) ) {
            return false;
        } else if( plc.horiz_ ) {
            return (col - plc.anchor_ >= 0) &&
                   (col + plc.size_ - plc.anchor_ - 1 < ncols_);
        } else {
            return (row - plc.anchor_ >= 0) &&
                   (row + plc.size_ - plc.anchor_ - 1 < nrows_);
        }
    }

    size_t hash() const {
        uint32_t value = 0;
        for( int cell = 0; cell < ncells_; ++cell ) {
            value += beams_.domain(cell)->hash();
            value += (value << 10);
            value ^= (value >> 6);
        }
        value += (value << 3);
        value ^= (value >> 11);
        value += (value << 15);
        return value;
    }

    bool consistent() const {
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( !beams_.domain(cell)->consistent() ) return false;
        }
        return true;
    }

    void clear() {
        for( int cell = 0; cell < ncells_; ++cell ) {
            beams_.domain(cell)->clear();
        }
    }

    void set_initial_configuration() {
        // Insert particles in beams. Two-pass method per beam: 1st pass calculate
        // number of particles in each beam, 2nd pass allocates space and insert
        // particles.
        for( int cell = 0; cell < ncells_; ++cell ) {
            var_beam_t &beam = *beams_.domain(cell);
            beam.clear();
            int col = cell % ncols_, row = cell / ncols_;
            int num_particles_in_beam = 0;
            for( int pass = 0; pass < 2; ++pass ) {
                if( pass == 1 ) beam.reserve(num_particles_in_beam);
                for( int p = 0; p < num_particles_; ++p ) {
                    if( consistent_particle(row, col, p) ) {
                        if( pass == 0 )
                            ++num_particles_in_beam;
                        else {
                            bool inserted = beam.push_back(p);
                            assert(inserted);
                        }
                    }
                }
            }
            beam.set_initial_size(beam.size());
            assert(beam.initial_size() > 0);
        }

#if 0 // DEPRECATED
        // remove inconsistent particles in (border) beams
        for( int cell = 0; cell < ncells_; ++cell ) {
            std::vector<int> indices_to_erase;
            indices_to_erase.reserve(num_particles_);
            int col = cell % ncols_;
            int row = cell / ncols_;
            for( int index = 0; index < num_particles_; ++index ) {
                bool erase = false;
                placement_t plc(index);
                if( plc.hit_ || (plc.nhits_ > 0) ) {
                    erase = true;
                } else if( (plc.size_ > 0) && (ship_inventory_[plc.size_] == 0) ) {
                    erase = true;
                } else if( (plc.size_ == 0) && (plc.horiz_ || (plc.anchor_ > 0) || (plc.nhits_ > 0)) ) {
                    erase = true;
                } else if( (plc.size_ == 1) && plc.horiz_ ) {
                    erase = true;
                } else if( (plc.size_ > 0) && (plc.size_ <= plc.anchor_) ) {
                    erase = true;
                } else if( plc.horiz_ ) {
                    erase = (col - plc.anchor_ < 0) || (col + plc.size_ - plc.anchor_ - 1 >= ncols_);
                } else {
                    erase = (row - plc.anchor_ < 0) || (row + plc.size_ - plc.anchor_ - 1 >= nrows_);
                }
                if( erase ) indices_to_erase.push_back(index);
            }
            beams_.domain(cell)->erase_ordered_indices(indices_to_erase);
        }
#endif

        // propagate initial settings
        beams_.add_all_edges_to_worklist();
        static std::vector<int> revised_beams;
        beams_.ac3(revised_beams);
        assert(consistent());

#ifdef DEBUG
        std::cout << "Initial Belief:" << std::endl;
        print(std::cout);
#endif
    }

    const belief_t& operator=(const belief_t &bel) {
        for( int cell = 0; cell < ncells_; ++cell ) {
            assert(beams_.domain(cell) != 0);
            *beams_.domain(cell) = *bel.beams_.domain(cell);
        }
        return *this;
    }
    const belief_t& operator=(const base_belief_t &bel) {
        *this = dynamic_cast<const belief_t&>(bel);
        return *this;
    }

    bool operator==(const belief_t &bel) const {
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( *beams_.domain(cell) != *bel.beams_.domain(cell) ) return false;
        }
        return true;
    }
    virtual bool operator==(const base_belief_t &bel) const {
        return *this == dynamic_cast<const belief_t&>(bel);
    }

    bool operator!=(const belief_t &bel) const {
        return *this == bel ? false : true;
    }
    virtual bool operator!=(const base_belief_t &bel) const {
        return *this != dynamic_cast<const belief_t&>(bel);
    }

    void print_cell(std::ostream &os, int cell) const {
        os << "cell(" << (cell % ncols_) << "," << (cell / ncols_) << ")={";
        for( var_beam_t::const_iterator it = beams_.domain(cell)->begin(); it != beams_.domain(cell)->end(); ++it ) {
            os << *it << "=" << placement_t(*it) << ",";
        }
        os << "}" << std::endl;
    }
    void print(std::ostream &os) const {
        for( int cell = 0; cell < ncells_; ++cell ) {
            print_cell(os, cell);
        }
    }

    void fire_torpedo(int cell) {
        var_beam_t &beam = *beams_.domain(cell);
        var_beam_t new_beam(1, num_particles_);
        new_beam.set_initial_size(beam.initial_size());
        new_beam.reserve(beam.size());

        int col = cell % ncols_;
        int row = cell / ncols_;

#ifdef DEBUG
        std::cout << "before firing:" << std::endl;
        print(std::cout);
        std::cout << "fire torpedo @ cell(" << col << "," << row << "):" << std::endl;
#endif

        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            placement_t plc(*it);
#ifdef DEBUG
            std::cout << "  processing p=" << *it << "=" << plc << std::endl;
#endif
            if( !plc.hit_ ) {
                if( plc.size_ == 0 ) {
                    assert(!plc.horiz_);
                    assert(plc.anchor_ == 0);
                    assert(plc.nhits_ == 0);
                    plc.hit_ = true;
                    new_beam.insert(plc.encode());
                } else if( plc.size_ == 1 ) {
                    assert(!plc.horiz_);
                    assert(plc.anchor_ == 0);
                    assert(plc.nhits_ == 0);
                    plc.nhits_ = 1;
                    plc.hit_ = true;
                    new_beam.insert(plc.encode());
                } else {
                    placement_t aux_plc = plc;
                    for( int d = 0; d < plc.size_; ++d ) {
                        if( d != plc.anchor_ ) {
                            int aux_col = col, aux_row = row;
                            aux_plc.anchor_ = d;
                            if( plc.horiz_ ) {
                                aux_col += d - plc.anchor_;
                                assert((aux_col >= 0) && (aux_col < ncols_));
                            } else {
                                aux_row += d - plc.anchor_;
                                assert((aux_row >= 0) && (aux_row < nrows_));
                            }
                            int aux_cell = aux_row * ncols_ + aux_col;
                            var_beam_t &aux_beam = *beams_.domain(aux_cell);
                            int p = aux_plc.encode();
                            if( aux_beam.contains(p) ) {
#ifdef DEBUG
                                std::cout << "     erase: cell(" << aux_col << "," << aux_row << "):"
                                          << " p=" << p
                                          << "=" << placement_t(p)
                                          << std::endl;
#endif
                                aux_beam.erase(p);
                                ++aux_plc.nhits_;
                                aux_beam.insert(aux_plc.encode());
#ifdef DEBUG
                                std::cout << "    insert: cell(" << aux_col << "," << aux_row << "):"
                                          << " p=" << aux_plc.encode()
                                          << "=" << aux_plc
                                          << std::endl;
#endif
                                --aux_plc.nhits_;
                            } else if( aux_beam.contains(1 + p) ) {
                                aux_beam.erase(1 + p);
#ifdef DEBUG
                                std::cout << "     erase: cell(" << aux_col << "," << aux_row << "):"
                                          << " p=" << 1 + p
                                          << "=" << placement_t(1 + p) << std::endl;
#endif
                                ++aux_plc.nhits_;
                                aux_beam.insert(1 + aux_plc.encode());
#ifdef DEBUG
                                std::cout << "    insert: cell(" << aux_col << "," << aux_row << "):"
                                          << " p=" << 1 + aux_plc.encode()
                                          << "=" << placement_t(1 + aux_plc.encode())
                                          << std::endl;
#endif
                                --aux_plc.nhits_;
                            }
                        }
                    }
                    assert(beam.contains(plc.encode()));
                    ++plc.nhits_;
                    plc.hit_ = true;
                    new_beam.insert(plc.encode());
                }
            } else {
                new_beam.insert(*it);
            }
        }

        // replace old beam by new beam
        beam = new_beam;

#ifdef DEBUG
        std::cout << "AFTER firing torpedo @ cell(" << col << "," << row << "):" << std::endl;
        print(std::cout);
#endif
    }

    void filter(int cell, int obs) {
        assert(!simple_observations_ || (obs == 0) || (obs == 1));
        var_beam_t &beam = *beams_.domain(cell);

#ifdef DEBUG
        int col = cell % ncols_;
        int row = cell / ncols_;
        std::cout << "filter: obs @ cell(" << col << "," << row << ") is "
                  << (obs == 0 ? "WATER" : obs == 1 ? "HIT" : "SUNK:")
                  << std::flush;
#endif

        // decode observation
        if( obs == 0 ) {
            assert(beam.contains(placement_t::empty_cell_hit));
            beam.clear();
            beam.insert(placement_t::empty_cell_hit);
        } else if( obs == 1 ) {
            std::vector<int> indices_to_erase;
            indices_to_erase.reserve(num_particles_);
            for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                placement_t plc(*it);
                if( !plc.hit_ || (plc.size_ == 0) ||
                    (!simple_observations_ && (plc.nhits_ == plc.size_)) ) {
                    indices_to_erase.push_back(it.index());
                }
            }
            beam.erase_ordered_indices(indices_to_erase);
        } else {
            // decode observation token
            obs = obs - 2;
            bool horiz = obs % 2 == 0 ? false : true;
            obs = obs >> 1;
            int anchor = obs % max_ship_size_;
            int size = obs / max_ship_size_;

#ifdef DEBUG
            std::cout << " horiz=" << (horiz ? "T" : "F")
                      << ", anchor=" << anchor
                      << ", size=" << size
                      << std::flush;
#endif

            // filter beam
            std::vector<int> indices_to_erase;
            indices_to_erase.reserve(num_particles_);
            for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
                placement_t plc(*it);
                if( (plc.nhits_ != plc.size_) || (plc.size_ != size) ||
                    (plc.anchor_ != anchor) || (plc.horiz_ != horiz) ) {
                    indices_to_erase.push_back(it.index());
                }
            }
            beam.erase_ordered_indices(indices_to_erase);
        }
        beams_.add_to_worklist(cell);

#ifdef DEBUG
        std::cout << std::endl << "AFTER local filtering:" << std::endl;
        print(std::cout);
#endif

        // propagate filtering
        static std::vector<int> revised_beams;
        beams_.ac3(revised_beams);

#ifdef DEBUG
        std::cout << "AFTER AC3:" << std::endl;
        print(std::cout);
#endif
    }

    bool hit_status(int cell) const {
        const var_beam_t &beam = *beams_.domain(cell);
        assert(!beam.empty());
        placement_t plc(*beam.begin());
        return (plc.size_ > 0) && plc.hit_;
    }

    float projected_probability(int r, int c, int tr, int tc, float prior_water_at_cell) const {
        assert((r == tr) || (c == tc));
        assert((r != tr) || (c != tc));
        int cell = r * ncols_ + c;
        float water_mass = 0, total_mass = 0;
        const var_beam_t &beam = *beams_.domain(cell);
        assert(!beam.empty());
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            placement_t plc(*it);
            if( plc.size_ == 0 ) {
                water_mass += prior_water_at_cell;
                total_mass += prior_water_at_cell;
            } else {
                float mass = (1.0 - prior_water_at_cell) / (beam.initial_size() - 1);
                if( mass <= 0 ) {
                    std::cout << mass << ", "
                              << prior_water_at_cell << ", "
                              << beam.initial_size()
                              << std::endl;
                }

                assert(mass > 0);
                if( (r == tr) && plc.horiz_ ) {
                    if( c < tc ) {
                        water_mass += c + plc.size_ - plc.anchor_ - 1 < tc ? mass : 0;
                    } else {
                        assert(tc < c);
                        water_mass += c - plc.anchor_ > tc ? mass : 0;
                    }
                } else if( (c == tc) && !plc.horiz_ ) {
                    if( r < tr ) {
                        water_mass += r + plc.size_ - plc.anchor_ - 1 < tr ? mass : 0;
                    } else {
                        assert(tr < r);
                        water_mass += r - plc.anchor_ > tr ? mass : 0;
                    }
                } else {
                    water_mass += mass;
                }
                total_mass += mass;
            }
        }
        assert(total_mass > 0);
#if 0
        std::cout << "ncell=" << cell << ":";
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            placement_t plc(*it);
            std::cout << " " << plc;
        }
        std::cout << ", proj=" << water_mass / total_mass << std::endl;
#endif
        return water_mass / total_mass;
    }

    float water_probability_at_cell(int cell, float prior_water_at_cell) const {
        int r = cell / ncols_, c = cell % ncols_;
        float probability = 1 - ship_probability_at_cell(cell, prior_water_at_cell);
        for( int dr = -1; dr < 2; ++dr ) {
            if( (r + dr < 0) || (r + dr >= nrows_) ) continue;
            for( int dc = -1; dc < 2; ++dc ) {
                if( (c + dc < 0) || (c + dc >= ncols_) ) continue;
                if( (dr != 0) && (dc != 0) ) continue;
                if( (dr == 0) && (dc == 0) ) continue;
                float p = projected_probability(r + dr, c + dc, r, c, prior_water_at_cell);
                probability = p < probability ? p : probability;
            }
        }
        return probability;
    }

    float ship_probability_at_cell(int cell, float prior_water_at_cell) const {
        const var_beam_t &beam = *beams_.domain(cell);
        int first_particle = *beam.begin();
        if( (first_particle == placement_t::empty_cell) ||
            (first_particle == placement_t::empty_cell_hit) ) {
            float mass_for_ship = (beam.size() - 1) * (1 - prior_water_at_cell);
            mass_for_ship /= beam.initial_size() - 1;
            return mass_for_ship / (mass_for_ship + prior_water_at_cell);
        } else {
            // all particles are for ships
            return 1;
        }
    }

#if 0 
    bool compute_probabilities_for_cell(int cell, float prior_water_at_cell) const {
        memset(cell_probabilities_, 0, (1 + max_ship_size_) * sizeof(float));
        const var_beam_t &beam = *beams_.domain(cell);
        assert(!beam.empty());
        float total_mass = 0;
        for( var_beam_t::const_iterator it = beam.begin(); it != beam.end(); ++it ) {
            placement_t plc(*it);
            if( plc.hit_ ) return false;
            float mass = plc.size_ == 0 ? prior_water_at_cell : 1 - prior_water_at_cell;
            //if( plc.size_ > 0 ) mass /= beam.initial_size() - 1; // CHECK: ADDED
            cell_probabilities_[plc.size_] += mass;
            total_mass += mass;
        }
        for( int d = 0; d <= max_ship_size_; ++d ) {
            cell_probabilities_[d] /= total_mass;
        }
#ifdef DEBUG
        print(std::cout);
        std::cout << "probabilities for cell("
                  << (cell % ncols_) << "," << (cell / ncols_) << "):";
        for( int d = 0; d <= max_ship_size_; ++d )
            std::cout << " [" << d << "]=" << cell_probabilities_[d];
        std::cout << std::endl;
#endif
        return true;
    }

    int select_action_old(float prior_water_at_cell) const {
        static std::vector<int> best_cells;
        best_cells.reserve(ncells_);
        best_cells.clear();
        float target_expected_value = 0;
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( !compute_probabilities_for_cell(cell, prior_water_at_cell) ) continue;
            float expected_value = 0;
            for( int d = 1; d <= max_ship_size_; ++d ) {
                expected_value += d * cell_probabilities_[d];
                //expected_value += cell_probabilities_[d];
            }
            if( expected_value >= target_expected_value ) {
                if( expected_value > target_expected_value ) {
                    target_expected_value = expected_value;
                    best_cells.clear();
                }
                best_cells.push_back(cell);
            }
        }
        assert(!best_cells.empty());

        // compute best cell and clear data structures
        int best_cell = best_cells[lrand48() % best_cells.size()];
        best_cells.clear();

#ifdef DEBUG
        std::cout << "target: cell=(" << (best_cell % ncols_) << "," << (best_cell / ncols_)
                  << "), expected_value=" << target_expected_value << std::endl;
#endif

        // return selection
        return best_cell;
    }
#endif

    int select_action(float prior_water_at_cell) const {
        static std::vector<int> best_cells;
        best_cells.reserve(ncells_);
        best_cells.clear();
        float target_probability = 1;
        for( int cell = 0; cell < ncells_; ++cell ) {
            if( placement_t(*beams_.domain(cell)->begin()).hit_ ) continue;
            float p = water_probability_at_cell(cell, prior_water_at_cell);
            if( p <= target_probability ) {
                if( p < target_probability ) {
                    target_probability = p;
                    best_cells.clear();
                }
                best_cells.push_back(cell);
            }
        }
        assert(!best_cells.empty());

        // compute best cell and clear data structures
        int best_cell = best_cells[lrand48() % best_cells.size()];
        best_cells.clear();

#ifdef DEBUG
        //std::cout << "target: cell=(" << (best_cell % ncols_) << "," << (best_cell / ncols_)
        //          << "), expected_value=" << target_expected_value << std::endl;
#endif

        // return selection
        return best_cell;
    }

    int select_random_action() const {
        static std::vector<int> best_cells;
        best_cells.reserve(ncells_);
        best_cells.clear();
        for( int cell = 0; cell < ncells_; ++cell ) {
            int p = *beams_.domain(cell)->begin();
            placement_t plc(p);
            if( !plc.hit_ ) best_cells.push_back(cell);
        }
        assert(!best_cells.empty());

        // compute best cell and clear data structures
        int best_cell = best_cells[lrand48() % best_cells.size()];
        best_cells.clear();
        return best_cell;
    }
};

std::list<belief_t*> belief_t::beliefs_;

bool belief_t::simple_observations_ = false;
bool belief_t::allow_adjacent_ships_ = true;
int *belief_t::ship_inventory_ = 0;
float *belief_t::cell_probabilities_ = 0;

}; // end of namespace Battleship

inline std::ostream& operator<<(std::ostream &os, const Battleship::belief_t &bel) {
    bel.print(os);
    return os;
}

#undef DEBUG

#endif

