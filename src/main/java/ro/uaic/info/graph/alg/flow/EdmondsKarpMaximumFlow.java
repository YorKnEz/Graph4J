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
package ro.uaic.info.graph.alg.flow;

import java.util.Arrays;
import ro.uaic.info.graph.Digraph;
import ro.uaic.info.graph.Edge;
import ro.uaic.info.graph.alg.DirectedGraphAlgorithm;
import ro.uaic.info.graph.model.EdgeSet;
import ro.uaic.info.graph.model.VertexQueue;
import ro.uaic.info.graph.model.VertexSet;

/**
 * The Edmonds–Karp algorithm is an implementation of the Ford–Fulkerson method
 * for computing the maximum flow in a flow network.
 *
 * It's time complexity is min(O(nmU),O(n m<sup>2</sup>), where <em>n</em> is
 * the number of vertices and <em>m</em> is the number of edges and <em>U</em>
 * is an ubber bound of edge capacities.
 *
 * BFS.
 *
 * @author Cristian Frăsinaru
 */
public class EdmondsKarpMaximumFlow extends DirectedGraphAlgorithm
        implements MaximumFlowAlgorithm {

    protected int source, sink;
    protected boolean[] visited;
    protected boolean[] forward;
    protected int[] parent;
    protected double[] residual;
    protected VertexQueue queue;
    protected double[][] flowData;
    //
    protected VertexSet sourcePartition;
    protected VertexSet sinkPartition;
    protected EdgeSet cutEdges;

    /**
     *
     * @param graph the input network.
     * @param source the source vertex.
     * @param sink the sink vertex.
     */
    public EdmondsKarpMaximumFlow(Digraph graph, int source, int sink) {
        this(graph, source, sink, null);

    }

    /**
     *
     * @param graph the input network.
     * @param source the source vertex.
     * @param sink the sink vertex.
     * @param flow the initial flow.
     */
    public EdmondsKarpMaximumFlow(Digraph graph, int source, int sink, NetworkFlow flow) {
        super(graph);
        if (source == sink) {
            throw new IllegalArgumentException("Source and sink must be different.");
        }
        if (graph.isAllowingMultipleEdges() || graph.isAllowingSelfLoops()) {
            throw new IllegalArgumentException("Multigraphs and pseudographs are not supported.");
        }
        this.source = source;
        this.sink = sink;
        if (flow != null) {
            initFlowData();
            for (var e : flow.edges()) {
                int v = e.source();
                int u = e.target();
                int pos = graph.adjListPos(v, u);
                flowData[graph.indexOf(v)][pos] = flow.get(e);
            }
        }
    }

    @Override
    public double getValue() {
        if (flowData == null) {
            compute();
        }
        int si = graph.indexOf(source);
        double value = 0.0;
        for (var it = graph.succesorIterator(source); it.hasNext();) {
            int u = it.next();
            value += flowData[si][graph.adjListPos(source, u)];
        }
        return value;
    }

    @Override
    public double getValue(int v, int u) {
        if (flowData == null) {
            compute();
        }
        int pos = graph.adjListPos(v, u);
        if (pos < 0) {
            return 0;
        }
        return flowData[graph.indexOf(v)][pos];
    }

    @Override
    public NetworkFlow getFlow() {
        NetworkFlow flow = new NetworkFlow(graph, source, sink);
        for (var it = graph.edgeIterator(); it.hasNext();) {
            Edge e = it.next();
            flow.put(e, getValue(e));
        }
        return flow;
    }

    @Override
    public VertexSet getSourcePartition() {
        if (sourcePartition != null) {
            return sourcePartition;
        }
        if (flowData == null) {
            compute();
        }
        sourcePartition = new VertexSet(graph);
        for (int v : graph.vertices()) {
            if (visited[graph.indexOf(v)]) {
                sourcePartition.add(v);
            }
        }
        return sourcePartition;
    }

    @Override
    public VertexSet getSinkPartition() {
        if (sinkPartition != null) {
            return sinkPartition;
        }
        if (flowData == null) {
            compute();
        }
        sinkPartition = new VertexSet(graph);
        for (int v : graph.vertices()) {
            if (!visited[graph.indexOf(v)]) {
                sinkPartition.add(v);
            }
        }
        return sinkPartition;
    }

    @Override
    public EdgeSet getCutEdges() {
        if (cutEdges != null) {
            return cutEdges;
        }
        getSourcePartition();
        getSinkPartition();
        cutEdges = new EdgeSet(graph);
        for (int v : sourcePartition) {
            for (var it = graph.neighborIterator(v); it.hasNext();) {
                int u = it.next();
                if (sinkPartition.contains(u)) {
                    cutEdges.add(v, u);
                }
            }
        }
        return cutEdges;
    }

    private boolean scan(int v) {
        //forward edges
        int vi = graph.indexOf(v);
        for (var it = graph.succesorIterator(v); it.hasNext();) {
            int u = it.next(); //v -> u
            int ui = graph.indexOf(u);
            if (visited[ui]) {
                continue;
            }
            double flow = flowData[vi][it.adjListPos()];
            double capacity = it.getEdgeWeight();
            if (capacity > flow) {
                visited[ui] = true;
                forward[ui] = true;
                parent[ui] = vi;
                residual[ui] = Math.min(capacity - flow, residual[vi]);
                queue.offer(u);
                if (u == sink) {
                    return true;
                }
            }
        }
        //backward edges
        for (var it = graph.predecessorIterator(v); it.hasNext();) {
            int u = it.next(); //v <- u
            int ui = graph.indexOf(u);
            if (visited[ui]) {
                continue;
            }
            double flow = flowData[ui][it.adjListPos()];
            if (flow > 0) {
                visited[ui] = true;
                forward[ui] = false;
                parent[ui] = vi;
                residual[ui] = Math.min(flow, residual[vi]);
                queue.offer(u);
            }
        }
        return false;
    }

    private void initFlowData() {
        flowData = new double[graph.numVertices()][];
        for (int v : graph.vertices()) {
            flowData[graph.indexOf(v)] = new double[graph.degree(v)];
        }
    }

    protected void reset() {
        Arrays.fill(visited, false);
        queue.clear();
    }

    protected void compute() {
        int n = graph.numVertices();
        visited = new boolean[n];
        forward = new boolean[n];
        parent = new int[n];
        residual = new double[n];
        queue = new VertexQueue(graph, n);
        if (flowData == null) {
            initFlowData();
        }

        int si = graph.indexOf(source);
        int ti = graph.indexOf(sink);
        boolean hasAugmentingPath;
        do {
            visited[si] = true;
            residual[si] = Double.POSITIVE_INFINITY;
            parent[si] = -1;
            queue.offer(source);
            hasAugmentingPath = false;
            while (!queue.isEmpty()) {
                int v = queue.poll();
                if (scan(v)) {
                    hasAugmentingPath = true;
                    break;
                }
            }
            if (hasAugmentingPath) {
                //increase flow with the residual capacity in the sink
                double r = residual[ti];
                int ui = ti;
                do {
                    int vi = parent[ui];
                    if (forward[ui]) {
                        //v -> u
                        int v = graph.vertexAt(vi);
                        int pos = graph.adjListPos(v, graph.vertexAt(ui));
                        flowData[vi][pos] += r;
                    } else {
                        //v <- u
                        int u = graph.vertexAt(ui);
                        int pos = graph.adjListPos(u, graph.vertexAt(vi));
                        flowData[ui][pos] -= r;
                    }
                    ui = vi;
                } while (ui != si);
                reset();
            }
        } while (hasAugmentingPath);
    }

}
