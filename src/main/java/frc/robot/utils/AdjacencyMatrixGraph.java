package frc.robot.utils;

import java.util.*;

public class AdjacencyMatrixGraph<K, V> {
    private final Map<K, Map<K, V>> graph = new HashMap<>();

    public void addEdge(K from, K to, V valueFromTo, V valueToFrom) {
        graph.computeIfAbsent(from, k -> new HashMap<>()).put(to, valueFromTo);

        graph.computeIfAbsent(to, k -> new HashMap<>()).put(from, valueToFrom);
    }

    public void removeEdge(K from, K to) {
        graph.computeIfAbsent(from, k -> new HashMap<>()).put(to, null);
        graph.computeIfAbsent(to, k -> new HashMap<>()).put(from, null);
    }

    public V hasEdge(K from, K to) {
        return graph.getOrDefault(from, Map.of()).getOrDefault(to, null);
    }

    public V getEdge(K from, K to) {
        return graph.getOrDefault(from, Map.of()).get(to);
    }

    public Map<K, V> getEdges(K from) {
        return graph.getOrDefault(from, Map.of());
    }

    public List<List<K>> getPaths(K from, K to) {
        List<List<K>> paths = new ArrayList<>();
        List<K> path = new ArrayList<>();
        path.add(from);
        getPaths(from, to, path, paths);
        return paths;
    }

    private void getPaths(K from, K to, List<K> path, List<List<K>> paths) {
        if (from.equals(to)) {
            paths.add(path);
            return;
        }
        for (K next : getEdges(from).keySet()) {
            if (!path.contains(next)) {
                List<K> newPath = new ArrayList<>(path);
                newPath.add(next);
                getPaths(next, to, newPath, paths);
            }
        }
    }
}
