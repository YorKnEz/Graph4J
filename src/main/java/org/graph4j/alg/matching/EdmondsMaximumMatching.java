/*
 * Copyright (C) 2023 Cristian Frăsinaru and contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.graph4j.alg.matching;

import org.graph4j.Graph;
import org.graph4j.alg.SimpleGraphAlgorithm;
import org.graph4j.util.Matching;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * TODO
 * @author Cristian Frăsinaru
 */
public class EdmondsMaximumMatching extends SimpleGraphAlgorithm {
    // we store n and m for faster memory access (faster than a method call)
    private final int n; // number of vertices
    private final int m; // number of edges
    private final int minV; // minimum index of a vertex in the original graph
    // we use an adjacency matrix instead of an adjacency list for O(1) retrieval of an edge number, given two vertices i, j
    // adj[i][j] = edge number of edge (i,j) or 0 if the edge doesn't exist
    private final int[][] adj;
    private final int[] edge; // given the number of an edge (i, j), n(ij), edge[2 * n(ij) - 1] = i and edge[2 * n(ij)] = j
    // label[i] can be 4 things:
    // - -1 - non-outer
    // - 0 - start label
    // - [1, n] - vertex label
    // - [n + 1, n + 2 * m] - edge label
    private int[] label;
    // first[i] is the first non-outer vertex on the path from i to the start vertex s
    private int[] first;
    // ij in matching <=> mate[i] = j and mate[j] = i
    private int[] mate;

    // queue used for the search
    // TODO: maybe implement a queue using an `int[]` instead of this for memory efficiency
    private Deque<Integer> q;
    public EdmondsMaximumMatching(Graph graph) {
        super(graph);
    }

}
