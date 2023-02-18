/*
 * Copyright (C) 2022 Cristian Frăsinaru and contributors
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
package algorithms;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import ro.uaic.info.graph.GraphBuilder;
import ro.uaic.info.graph.alg.flow.*;
import ro.uaic.info.graph.generate.EdgeWeightsGenerator;
import ro.uaic.info.graph.generate.GnpGraphGenerator;

/**
 *
 * @author Cristian Frăsinaru
 */
public class MaximumFlowTest {

    //@Test
    public void simpleFF() {
        //exemplul din curs
        var g = GraphBuilder.numVertices(6).buildDigraph();
        int s = 0;
        int t = 5;
        g.addEdge(s, 1, 9);
        g.addEdge(s, 4, 8);
        g.addEdge(1, 2, 10);
        g.addEdge(1, 3, 5);
        g.addEdge(2, t, 7);
        g.addEdge(3, t, 6);
        g.addEdge(4, 1, 2);
        g.addEdge(4, 3, 5);
        var alg = new FordFulkersonMaximumFlow(g, s, t);
        assertEquals(13, alg.getValue());
        assertEquals(7, alg.getValue(2, t));
        assertEquals(6, alg.getValue(3, t));

        assertTrue(alg.getSourcePartition().contains(s));
        assertTrue((alg.getSinkPartition().contains(t)));

        double minCutValue = 0;
        for (var e : alg.getCutEdges().edges()) {
            minCutValue += g.getEdgeWeight(e[0], e[1]);
        }
        assertEquals(alg.getValue(), minCutValue);
    }

    @Test
    public void simpleEK() {
        //exemplul din curs
        var g = GraphBuilder.numVertices(6).buildDigraph();
        int s = 0;
        int t = 5;
        g.addEdge(s, 1, 9);
        g.addEdge(s, 4, 8);
        g.addEdge(1, 2, 10);
        g.addEdge(1, 3, 5);
        g.addEdge(2, t, 7);
        g.addEdge(3, t, 6);
        g.addEdge(4, 1, 2);
        g.addEdge(4, 3, 5);
        var alg = new EdmondsKarpMaximumFlow(g, s, t);
        assertEquals(13, alg.getValue());
        assertEquals(13, alg.getFlow().value());
        assertEquals(7, alg.getValue(2, t));
        assertEquals(6, alg.getValue(3, t));

        assertTrue(alg.getSourcePartition().contains(s));
        assertTrue((alg.getSinkPartition().contains(t)));

        double minCutValue = 0;
        for (var e : alg.getCutEdges().edges()) {
            minCutValue += g.getEdgeWeight(e[0], e[1]);
        }
        assertEquals(alg.getValue(), minCutValue);
    }

    @Test
    public void crossTest() {
        int n = 10;
        int s = 0;
        int t = n - 1;
        var g = new GnpGraphGenerator(n, Math.random()).createDigraph();
        EdgeWeightsGenerator.randomIntegers(g, 0, n);
        var alg1 = new EdmondsKarpMaximumFlow(g, s, t);
        var alg2 = new FordFulkersonMaximumFlow(g, s, t);
        //var alg3 = new EdmondsKarpMFImpl(Tools.createJGraph(g));
        double x = alg1.getValue();
        double y = alg2.getValue();
        //double z = alg3.calculateMaximumFlow(s, t);
        assertEquals(x, y);
    }

}
