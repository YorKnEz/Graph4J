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
public class EdmondsMaximumMatching extends SimpleGraphAlgorithm implements MatchingAlgorithm {
    // we store n and m for faster memory access (faster than a method call)
    private final int n; // number of vertices
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
    private final int[] label;
    // first[i] is the first non-outer vertex on the path from i to the start vertex s
    private final int[] first;
    // ij in matching <=> mate[i] = j and mate[j] = i
    private final int[] mate;

    // queue used for the search
    // TODO: maybe implement a queue using an `int[]` instead of this for memory efficiency
    private final Deque<Integer> q;

    public EdmondsMaximumMatching(Graph graph) {
        super(graph);

        n = graph.numVertices();
        // number of edges
        int m = (int) graph.numEdges();
        adj = new int[n + 1][n + 1];
        edge = new int[(int) (n + 1 + 2 * m)];
        label = new int[n + 1];
        first = new int[n + 1];
        mate = new int[n + 1];
        q = new ArrayDeque<>();

        // find the min vertex label in order to normalize all labels in interval [1, n]
        minV = Arrays.stream(graph.vertices()).min().orElse(0) - 1;

        int i = 1;
        for (var e : graph.edges()) {
            edge[n + 2 * i - 1] = e.source() - minV;
            edge[n + 2 * i] = e.target() - minV;
            adj[e.source() - minV][e.target() - minV] = adj[e.target() - minV][e.source() - minV] = n + 2 * i;
            i++;
        }
    }

    // recursively augment the path P(x)
    // TODO: implement recursion using a stack for memory efficiency
    private void augment(int x, int y) {
        // match x to y (y is assumed to have been matched to x by the caller of this method)
        int t = mate[x];
        mate[x] = y;

        if (mate[t] != x) {
            return;
        }

        // if x has a vertex label
        if (1 <= label[x] && label[x] <= n) {
            mate[t] = label[x]; // match t to label[x]
            augment(label[x], t); // match label[x] to t and the rest of the path to starting vertex
            return;
        }

        // else x must have an edge label, so we retrieve the vertices forming the said edge and
        int v = edge[label[x] - 1], w = edge[label[x]];
        augment(v, w);
        augment(w, v);
    }

    // recursively label non-outer vertices in paths P(x) and P(y)
    // TODO: implement recursion using a stack for memory efficiency
    private void label(int x, int y) {
        int edgeLabel = adj[x][y];
        int r = first[x];
        int s = first[y];
        int join = 0; // this will be the index of the first non-outer vertex both on P(x) and P(y) (variable will also be used as an aux for swap)

        // if they have the same non-outer vertex as the first on their paths to the start, there are no new vertices
        // that we can label
        if (r == s) {
            return;
        }

        // flag r and s
        label[r] = -edgeLabel;
        label[s] = -edgeLabel;

        // alternatively flag the non-outer vertices on the paths P(x) and P(y) until we reach the common root which we will store in join
        while (s != 0) {
            join = r;
            r = s;
            s = join;

            r = first[label[mate[r]]];

            if (label[r] == -edgeLabel) {
                join = r;
                break;
            }

            label[r] = -edgeLabel;
        }

        // mark all non-outer vertices on P(x) and P(y) (excluding join) with an edge label
        // use r as the iterator
        r = first[x];
        while (r != join) {
            label[r] = edgeLabel;
            first[r] = join;
            q.addLast(r);
            r = first[label[mate[r]]];
        }

        r = first[y];
        while (r != join) {
            label[r] = edgeLabel;
            first[r] = join;
            q.addLast(r);
            r = first[label[mate[r]]];
        }

        // update the first of all outer nodes to join by checking which nodes have their first labeled with -edgeLabel
        // TODO: could probably be done better than O(n)
        for (int i = 0; i <= n; i++) {
            if (label[first[i]] == -edgeLabel) {
                first[i] = join;
            }
        }
    }

    @Override
    public Matching getMatching() {
        int u = 1;
        int x; // current vertex of search
        int y;
        int v; // temporary variable

        // initialize data structures
        for (int i = 0; i <= n; i++) {
            label[i] = -1;
            mate[i] = 0;
        }
        q.clear();

        while (u <= n) {
            // skip matched vertices
            if (mate[u] != 0) {
                u++;
                continue;
            }

            // start the search from unmatched vertex u
            label[u] = first[u] = 0;
            q.addLast(u);

            // at this stage, we begin the search in a BFS manner, storing a queue of outer edges (label[i] >= 0)
            // which we use to examine edges xy in which x is taken from the queue and is an outer edge
            search:
            while (!q.isEmpty()) {
                x = q.removeFirst(); // take first outer edge from the queue

                // walk through its edge list
                for (var e : graph.edgesOf(x + minV)) {
                    y = e.target() - minV;

                    // if we find an edge that is unmatched, it means we can augment the path and stop the current search
                    if (mate[y] == 0 && y != u) {
                        mate[y] = x;
                        augment(x, y);
                        break search;
                    }

                    // if y is outer it means we found two paths P(x), P(y) that can be joined (thus forming a blossom)
                    if (label[y] >= 0) {
                        label(x, y);
                        continue;
                    }

                    v = mate[y];

                    // if mate of y is non-outer, it means we can extend the path P(x) with edge (y, mate[y])
                    if (label[v] < 0) {
                        label[v] = x;
                        first[v] = y;
                        q.addLast(v);
                    }
                }
            }

            // prepare the data structures for the next search
            // TODO: probably can be done better?
            for (int i = 0; i <= n; i++) {
                label[i] = -1;
                first[i] = 0;
            }
            q.clear();

            u++;
        }

        // matching is found, build it from mate array
        Matching m = new Matching(graph);
        for (int i = 1; i <= n; i++) {
            m.add(i + minV, mate[i] + minV); // take into account minV to denormalize the vertex indices
        }

        return m;
    }
}
