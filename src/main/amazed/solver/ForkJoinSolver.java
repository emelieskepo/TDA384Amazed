package amazed.solver;

import amazed.maze.Maze;

import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.Stack;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.RecursiveTask;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * <code>ForkJoinSolver</code> implements a solver for
 * <code>Maze</code> objects using a fork/join multi-thread
 * depth-first search.
 */
public class ForkJoinSolver extends SequentialSolver {

    // Shared flag to indicate that the goal has been found
    private static final AtomicBoolean goalFound = new AtomicBoolean(false);

    // List of sub-solvers for forked tasks
    private final List<ForkJoinSolver> subSolvers = new ArrayList<>();

    /**
     * Creates a solver that searches in <code>maze</code> from the
     * start node to a goal.
     */
    public ForkJoinSolver(Maze maze) {
        super(maze);
        this.visited = new ConcurrentSkipListSet<>();
        this.predecessor = new ConcurrentHashMap<>();
    }

    /**
     * Creates a solver that forks after a number of steps.
     */
    public ForkJoinSolver(Maze maze, int forkAfter) {
        this(maze);
        this.forkAfter = forkAfter;
    }

    /**
     * Creates a solver starting at a specific node, sharing visited nodes and predecessors.
     */
    public ForkJoinSolver(Maze maze, int forkAfter, int startNode, Set<Integer> visitedSet,
                          Map<Integer, Integer> predecessorMap) {
        this(maze, forkAfter);
        this.start = startNode;
        this.visited = visitedSet;
        this.predecessor = predecessorMap;
    }

    @Override
    public List<Integer> compute() {
        return parallelSearch();
    }

    /**
     * Performs parallel depth-first search with optional task forking.
     */
    private List<Integer> parallelSearch() {

        int stepsSinceFork = 0;
        int player = maze.newPlayer(start);

        // Stack for DFS
        Stack<Integer> frontier = new Stack<>();
        frontier.push(start);

        while (!frontier.isEmpty() && !goalFound.get()) {

            int currentNode = frontier.pop();

            // Only process if not already visited
            if (visited.add(currentNode) || currentNode == start) {

                // Move the player to the current node
                maze.move(player, currentNode);

                // Check if current node is a goal
                if (maze.hasGoal(currentNode)) {
                    goalFound.set(true);
                    return reconstructPath(currentNode);
                }

                stepsSinceFork++;
                boolean isFirstNeighbor = true;

                // Explore neighbors
                for (int neighbor : maze.neighbors(currentNode)) {

                    if (!visited.contains(neighbor)) {

                        // Set predecessor for path reconstruction
                        predecessor.putIfAbsent(neighbor, currentNode);

                        // Either continue DFS locally or fork a new task
                        if (isFirstNeighbor || stepsSinceFork < forkAfter) {
                            frontier.push(neighbor);
                            isFirstNeighbor = false;
                        } else {
                            if (visited.add(neighbor)) {
                                stepsSinceFork = 0;

                                // Fork new solver for this neighbor
                                ForkJoinSolver forkedSolver = new ForkJoinSolver(
                                        maze, forkAfter, neighbor, visited, predecessor);

                                subSolvers.add(forkedSolver);
                                forkedSolver.fork();
                            }
                        }
                    }
                }
            }
        }

        // Join forked solvers
        for (ForkJoinSolver solver : subSolvers) {
            List<Integer> result = solver.join();

            if (result != null) {
                List<Integer> pathToChild = pathFromTo(start, solver.start);
                pathToChild.addAll(result.subList(1, result.size()));
                return pathToChild;
            }
        }

        return null;
    }

    /**
     * Reconstructs the path from start to the given node.
     */
    private List<Integer> reconstructPath(int node) {
        List<Integer> path = new ArrayList<>();
        int current = node;

        while (current != start && predecessor.containsKey(current)) {
            path.add(0, current); // insert at front
            current = predecessor.get(current);
        }
        path.add(0, start);
        return path;
    }
}