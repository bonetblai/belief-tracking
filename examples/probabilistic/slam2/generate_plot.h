/*
 *  Copyright (C) 2015 Universidad Simon Bolivar
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

#ifndef GENERATE_PLOT
#define GENERATE_PLOT

#include <cassert>
#include <string>
#include <vector>

#include "tracking.h"
#include "cellmap.h"

#if 0
#include <cassert>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdlib.h>
#include <math.h>

#include "slam_action_selection.h"

////#include "pcbt.h"

#include "sis.h"
#include "motion_model_sir.h"
#include "optimal_sir.h"
#include "rbpf.h"
////#include "ppcbt2.h"

#include "slam_particles.h"
#include "slam2_particles.h"
#endif

inline void generate_R_plot_commons() {
    // R libraries
    std::cout << "library(ggplot2)" << std::endl
              << "library(reshape)" << std::endl
              << "library(zoo)" << std::endl
              << "#library(gtable)" << std::endl
              << "library(grid)" << std::endl
              << "library(gridExtra)" << std::endl;

    std::cout << std::endl;

    // useful R functions
    std::cout << "define_region <- function(row, col) {" << std::endl
              << "    viewport(layout.pos.row = row, layout.pos.col = col)" << std::endl
              << "}" << std::endl
              << "get_legend <- function(myggplot) {" << std::endl
              << "    tmp <- ggplot_gtable(ggplot_build(myggplot))" << std::endl
              << "    leg <- which(sapply(tmp$grobs, function(x) x$name) == \"guide-box\")" << std::endl
              << "    legend <- tmp$grobs[[leg]]" << std::endl
              << "    return(legend)" << std::endl
              << "}" << std::endl;

    std::cout << std::endl;

    // color themes from http://colorbrewer2.org
    std::cout << "color_theme1 <- c(\"#543005\", \"#8c510a\", \"#bf812d\", \"#dfc27d\", \"#f6e8c3\", \"#f5f5f5\", \"#c7eae5\", \"#80cdc1\", \"#35978f\", \"#01665e\", \"#003c30\")"
              << std::endl
              << "color_theme2 <- c(\"#276419\", \"#4d9221\", \"#7fbc41\", \"#b8e186\", \"#e6f5d0\", \"#f7f7f7\", \"#fde0ef\", \"#f1b6da\", \"#de77ae\", \"#c51b7d\", \"#8e0152\")"
              << std::endl
              << "color_theme3 <- c(\"#00441b\", \"#1b7837\", \"#5aae61\", \"#a6dba0\", \"#d9f0d3\", \"#f7f7f7\", \"#e7d4e8\", \"#c2a5cf\", \"#9970ab\", \"#762a83\", \"#40004b\")"
              << std::endl
              << "color_theme4 <- c(\"#7f3b08\", \"#b35806\", \"#e08214\", \"#fdb863\", \"#fee0b6\", \"#f7f7f7\", \"#d8daeb\", \"#b2abd2\", \"#8073ac\", \"#542788\", \"#2d004b\")"
              << std::endl
              << "color_theme5 <- c(\"#053061\", \"#2166ac\", \"#4393c3\", \"#92c5de\", \"#d1e5f0\", \"#f7f7f7\", \"#fddbc7\", \"#f4a582\", \"#d6604d\", \"#b2182b\", \"#67001f\")"
              << std::endl
              << "color_theme6 <- c(\"#67001f\", \"#b2182b\", \"#d6604d\", \"#f4a582\", \"#fddbc7\", \"#ffffff\", \"#e0e0e0\", \"#bababa\", \"#878787\", \"#4d4d4d\", \"#1a1a1a\")"
              << std::endl
              << "color_theme7 <- c(\"#313695\", \"#4575b4\", \"#74add1\", \"#abd9e9\", \"#e0f3f8\", \"#e8e8e8\", \"#fee090\", \"#fdae61\", \"#f46d43\", \"#d73027\", \"#a50026\")"
              << std::endl
              << "color_theme8 <- c(\"#006837\", \"#1a9850\", \"#66bd63\", \"#a6d96a\", \"#d9ef8b\", \"#b8b8b8\", \"#fee08b\", \"#fdae61\", \"#f46d43\", \"#d73027\", \"#a50026\")"
              << std::endl
              << "color_theme9 <- c(\"#5e4fa2\", \"#3288bd\", \"#66c2a5\", \"#abdda4\", \"#e6f598\", \"#ffffbf\", \"#fee08b\", \"#fdae61\", \"#f46d43\", \"#d53e4f\", \"#9e0142\")"
              << std::endl;

    std::cout << std::endl;

    // prepare colors and labels for legend: 10 levels
    std::cout << "quantile_range <- c(0, .01, .05, .1, .25, .45, .55, .75, .9, .95, .99, 1)"
              << std::endl
              << "color_palette <- colorRampPalette(color_theme2)(length(quantile_range) - 1)"
              << std::endl
              << "label_text <- rollapply(round(quantile_range, 2), width = 2, by = 1, FUN = function(i) paste(i, collapse = \"-\"))"
              << std::endl;

    std::cout << std::endl;

    // define minimal theme
    std::cout << "minimal_theme <- theme(" << std::endl
              << "    plot.background = element_blank()," << std::endl
              << "    panel.grid.minor = element_blank()," << std::endl
              << "    panel.grid.major = element_blank()," << std::endl
              << "    panel.background = element_blank()," << std::endl
              << "    panel.border = element_blank()," << std::endl
              << "    axis.line = element_blank()," << std::endl
              << "    axis.ticks = element_blank()," << std::endl
              << "    axis.text.x = element_blank()," << std::endl
              << "    axis.text.y = element_blank()," << std::endl
              << "    axis.title.x = element_blank()," << std::endl
              << "    axis.title.y = element_blank()," << std::endl
              << "    plot.margin = unit(c(-.25, -.5, -1.25, -.5), \"lines\")," << std::endl
              << "    legend.title = element_blank()" << std::endl
              << ")" << std::endl;

    std::cout << std::endl;
}

inline void generate_R_plot(const std::string &output_file,
                            const tracking_t<cellmap_t> &tracker,
                            const repository_t &repo,
                            const cellmap_t &cellmap,
                            const std::string &inference_algorithm,
                            float map_epsilon,
                            float map_threshold) {
    // set output file
    std::cout << std::endl
              << "pdf(\"" << output_file << "\", width = 12, height = 10)"
              << std::endl;

    // define time steps for analysis
    std::set<int> focus;
    int nsteps = int(repo.size());
    for( int t = 0; t < nsteps; ++t ) {
        if( t == int(.25 * nsteps) ) // raw data at 25% time of total steps
            focus.insert(t);
        else if( t == int(.5 * nsteps) ) // raw data at 50% time of total steps
            focus.insert(t);
        else if( t == int(.75 * nsteps) ) // raw data at 75% time of total steps
            focus.insert(t);
        else if( t == nsteps - 1 ) // raw data at end of time
            focus.insert(t);
    }

    std::cout << std::endl;

    // extract marginals and maps for selected time steps
    std::cout << "time_steps <- list()" << std::endl
              << "raw_data <- list()" << std::endl;
    for( int t = 0; t < nsteps; ++t ) {
        if( focus.find(t) != focus.end() ) {
            std::cout << "time_steps <- append(time_steps, list(" << t << "))" << std::endl;

            // raw data for field
            std::cout << "raw_data <- append(raw_data, list(matrix(c(";
            for( int var = 0; var < cellmap.nrows_ * cellmap.ncols_; ++var ) {
                const cell_t &cell = cellmap.cells_[var];
                std::cout << (cell.label_ ? 1 : 0);
                if( 1 + var < cellmap.nrows_ * cellmap.ncols_ ) std::cout << ", ";
            }
            std::cout << "), ncol=" << cellmap.ncols_ << ", byrow=T)))" << std::endl;

            // raw data for marginals
            std::cout << "raw_data <- append(raw_data, list(matrix(c(";
            for( int var = 0; var < cellmap.nrows_ * cellmap.ncols_; ++var ) {
                const float *marginal = &repo[t][cellmap.variable_offset(var)];
                std::cout << marginal[1];
                if( var + 1 < cellmap.nrows_ * cellmap.ncols_ ) std::cout << ", ";
                assert(fabs(marginal[0] + marginal[1] - 1.0) < EPSILON);
            }
            std::cout << "), ncol=" << cellmap.ncols_ << ", byrow=T)))" << std::endl;

            // raw data for maps on marginals
            std::cout << "raw_data <- append(raw_data, list(matrix(c(";
            for( int var = 0; var < cellmap.nrows_ * cellmap.ncols_; ++var ) {
                std::vector<std::pair<float, int> > map_values;
                tracker.MAP_on_var(repo, t, var, map_values, map_epsilon);
                assert(!map_values.empty());
                if( map_values[0].first < map_threshold )
                    std::cout << .50;
                else
                    std::cout << (map_values.size() == 1 ? map_values[0].second : .50);
                if( 1 + var < cellmap.nrows_ * cellmap.ncols_ ) std::cout << ", ";
            }
            std::cout << "), ncol=" << cellmap.ncols_ << ", byrow=T)))" << std::endl;
        }
    }

    std::cout << std::endl;

    // apply quantile discretization to field, marginals and maps
    std::cout << "mod_data <- lapply(raw_data, function(d) { m <- matrix(findInterval(d, quantile_range, all.inside = TRUE), nrow = nrow(d)); m; })" << std::endl;

    std::cout << std::endl;

    // generate plots and plots without legends
    std::cout << "plots <- lapply(mod_data, function(m) { p <- ggplot(melt(m), aes(x = X2, y = X1, fill = factor(value, levels = 1:(length(quantile_range) - 1)))) + geom_tile(color = \"black\") + scale_fill_manual(values = color_palette, name = \"\", labels = label_text, drop = FALSE) + minimal_theme; p })" << std::endl
              << "plots_nl <- lapply(plots, function(p) { pnl <- p + theme(legend.position = \"none\"); pnl })" << std::endl
              << "plot_legend <- get_legend(plots[[1]])" << std::endl;

    std::cout << std::endl;

    // calculate # errors in maps
    std::cout << "map_threshold <- " << map_threshold << std::endl;
    std::cout << "n_rows <- " << cellmap.nrows_ << std::endl;
    std::cout << "n_cols <- " << cellmap.ncols_ << std::endl;
    std::cout << "n_time_steps <- " << focus.size() << std::endl;
    std::cout << "equals_in_map <- sapply(seq(1, 3 * n_time_steps, 3), function(i) { sum(raw_data[[i+2]] == raw_data[[i]]) })" << std::endl;
    std::cout << "unknowns_in_map <- sapply(seq(1, 3 * n_time_steps, 3), function(i) { sum(raw_data[[i+2]] == 0.5) })" << std::endl;
    std::cout << "errors_in_map <- n_rows * n_cols - equals_in_map - unknowns_in_map" << std::endl;

    std::cout << std::endl;

    // put plots together using viewports and display them
    std::cout << "pushViewport(viewport(layout = grid.layout(2 + n_time_steps, 5, heights = unit(c(1.75, rep(4, n_time_steps), .5), rep(\"null\", 2 + n_time_steps)), widths = unit(rep(4, 5), rep(\"null\", 5)))))" << std::endl
              << "sapply(seq_along(plots_nl), function(i) { print(plots_nl[[i]], vp = define_region(2 + ((i - 1) %/% 3), 2 + ((i - 1) %% 3))); i })" << std::endl
              << "grid.text(\"field\", vp = define_region(2 + n_time_steps, 2))" << std::endl
              << "grid.text(\"marginals\", vp = define_region(2 + n_time_steps, 3))" << std::endl
              << "grid.text(\"map on marginals\", vp = define_region(2 + n_time_steps, 4))" << std::endl
              << "grid.text(paste(n_rows, \"x\", n_cols, \" grid problem\\n"
              << tracker.id()
              << "\\n"
              << inference_algorithm
              << "\", sep=\"\"), vp = viewport(layout.pos.row = 1, layout.pos.col = 1:5))" << std::endl
              << "sapply(seq_along(time_steps), function(i) { grid.text(paste(\"After\", time_steps[[i]], \"steps\\n#error(s) in MAP =\", errors_in_map[[i]], \"\\n#unknown(s) in MAP =\", unknowns_in_map[[i]], sep=\" \"), vp = define_region(1 + i, 1)); i })" << std::endl
              << "pushViewport(viewport(just = c(\"center\", \"center\"), layout.pos.row = 2:(1 + n_time_steps), layout.pos.col = 5))" << std::endl
              << "grid.draw(plot_legend)" << std::endl;

    // close device
    std::cout << "dev.off()" << std::endl;
}

#endif

