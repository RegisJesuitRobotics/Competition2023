package frc.robot.utils;

import java.util.*;

public class AdjacencyMatrixGraphWithReversableList<K, V> {
    private final Map<K, Map<K, List<V>>> graph = new HashMap<>();

    public void addEdge(K from, K to, List<V> value) {
        graph.computeIfAbsent(from, k -> new HashMap<>()).put(to, value);

        List<V> reversed = new ArrayList<>(List.copyOf(value));
        Collections.reverse(reversed);
        graph.computeIfAbsent(to, k -> new HashMap<>()).put(from, reversed);
    }

    public void removeEdge(K from, K to) {
        graph.computeIfAbsent(from, k -> new HashMap<>()).put(to, null);
        graph.computeIfAbsent(to, k -> new HashMap<>()).put(from, null);
    }

    public List<V> hasEdge(K from, K to) {
        return graph.getOrDefault(from, Map.of()).getOrDefault(to, Collections.emptyList());
    }

    public List<V> getEdge(K from, K to) {
        return graph.getOrDefault(from, Map.of()).get(to);
    }

    public Map<K, List<V>> getEdges(K from) {
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
