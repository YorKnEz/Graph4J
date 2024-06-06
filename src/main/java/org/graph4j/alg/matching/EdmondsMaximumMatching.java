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

import java.util.Arrays;

/**
 * An implementation of Edmonds' maximum matching Blossom algorithm, as described by Gabow in his 1976 paper
 * `An implementation of Edmonds' algorithm for maximum matching`. The original implementation of Edmonds had a
 * computation time of O(n^4), while this algorithm has a computation time proportional to O(n^3). Its key being that,
 * as opposed to Edmonds implementation, it doesn't actually shrink blossoms, it instead uses some clever data structures
 * which are used for finding alternating paths.
 *
 * @author Alexandru Mitreanu
 * @author Alina Căprioară
 */
public class EdmondsMaximumMatching extends SimpleGraphAlgorithm implements MatchingAlgorithm {
    // we store n and m for faster memory access (faster than a method call)
    private final int n; // number of vertices
    private final int minV; // minimum index of a vertex in the original graph
//    // label[i] can be 4 things:
//    // - -1                           - non-outer
//    // - 0                            - start label
//    // - [1, n]                       - vertex label
//    // - y << 32 + x, for an edge xy  - edge label
    private final long[] label;
    // first[i] is the first non-outer vertex on the path from i to the start vertex s
    private final int[] first;
    // ij in matching <=> mate[i] = j and mate[j] = i
    private final int[] mate;

    // queue used for the search, we use our own implementation because this queue also stores the outer nodes in the
    // current search, which we use in the label method and when resetting the search
    int[] q;
    int qFirst, qLast;

    public EdmondsMaximumMatching(Graph graph) {
        super(graph);

        n = graph.numVertices();
        label = new long[n + 1];
        first = new int[n + 1];
        mate = new int[n + 1];
        q = new int[n];
        qFirst = qLast = 0;

        // find the min vertex label in order to normalize all labels in interval [1, n]
        minV = Arrays.stream(graph.vertices()).min().orElse(0) - 1;

    }

    // recursively augment the path P(x)
    private void augment(int x, int y) {
        // match x to y (y is assumed to have been matched to x by the caller of this method)
        int t = mate[x];
        mate[x] = y;

        if (mate[t] != x) {
            return;
        }

        // if x has a vertex label
        if (1 <= label[x] && label[x] <= n) {
            mate[t] = (int) label[x]; // match t to label[x]
            augment((int) label[x], t); // match label[x] to t and the rest of the path to starting vertex
            return;
        }

        // else x must have an edge label, so we retrieve the vertices forming the said edge and
        int v = (int) (label[x] & 0xFFFFFFFFL), w = (int) (label[x] >> 32);
        augment(v, w);
        augment(w, v);
    }

    // label non-outer vertices in paths P(x) and P(y)
    private void label(int x, int y) {
        long edgeLabel = (long) x + ((long) y << 32);
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

            r = first[(int)label[mate[r]]];

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
            q[qLast++] = r;
            r = first[(int)label[mate[r]]];
        }

        r = first[y];
        while (r != join) {
            label[r] = edgeLabel;
            first[r] = join;
            q[qLast++] = r;
            r = first[(int)label[mate[r]]];
        }

        // update all outer vertices i that have their first[i] marked by the method to have their first[i] = join
        for (int i = 0, qi; i < qLast; i++) {
            qi = q[i];
            if (label[qi] >= 0 && label[first[qi]] >= 0) {
                first[qi] = join;
            }
        }
    }

    @Override
    public Matching getMatching() {
        int u = 1;
        int x; // current vertex of search
        int v; // temporary variable

        // initialize data structures
        for (int i = 0; i <= n; i++) {
            label[i] = -1;
            mate[i] = 0;
        }
        qFirst = qLast = 0;

        while (u <= n) {
            // skip matched vertices
            if (mate[u] != 0) {
                u++;
                continue;
            }

            // start the search from unmatched vertex u
            label[u] = first[u] = 0;
            q[qLast++] = u;

            // at this stage, we begin the search in a BFS manner, storing a queue of outer edges (label[i] >= 0)
            // which we use to examine edges xy in which x is taken from the queue and is an outer edge
            search:
            while (qFirst < qLast) {
                x = q[qFirst++];

                // walk through its edge list
                for (int y : graph.neighbors(x + minV)) {
                    y -= minV;

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
                        q[qLast++] = v;
                    }
                }
            }

            // prepare the data structures for the next search
            label[0] = -1;
            for (int i = 0, qi; i < qLast; i++) {
                qi = q[i];
                label[qi] = label[mate[qi]] = -1;
                first[qi] = first[mate[qi]] = 0;
            }
            qFirst = qLast = 0;

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
