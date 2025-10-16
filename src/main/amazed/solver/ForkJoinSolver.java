package amazed.solver;

import amazed.maze.Maze;

import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.Stack;
import java.util.concurrent.*;

/**
 * <code>ForkJoinSolver</code> implements a solver for
 * <code>Maze</code> objects using a fork/join multi-thread
 * depth-first search.
 * <p>
 * Instances of <code>ForkJoinSolver</code> should be run by a
 * <code>ForkJoinPool</code> object.
 */

public class ForkJoinSolver
        extends SequentialSolver {

    // Shared flag to indicate that the goal has been found
    private static final AtomicBoolean goalFound = new AtomicBoolean(false);

    // List of sub-solvers that will run in parallel threads
    private final List<ForkJoinSolver> subSolvers = new ArrayList<>();

    /**
     * Creates a solver that searches in <code>maze</code> from the
     * start node to a goal.
     * Initializes visited and predecessor
     * @param maze   the maze to be searched
     */
    public ForkJoinSolver(Maze maze) {
        super(maze);
        this.visited = new ConcurrentSkipListSet<>();
        this.predecessor = new ConcurrentHashMap<>();
    }

    /**
     * Creates a solver that searches in <code>maze</code> from the
     * start node to a goal, forking after a given number of visited
     * nodes.
     *
     * @param maze        the maze to be searched
     * @param forkAfter   the number of steps (visited nodes) after
     *                    which a parallel task is forked; if
     *                    <code>forkAfter &lt;= 0</code> the solver never
     *                    forks new tasks
     */
    public ForkJoinSolver(Maze maze, int forkAfter) {
        this(maze);
        this.forkAfter = forkAfter;
    }

    /**
     * Creates a solver starting at a specific node, sharing visited nodes and predecessors.
     *
     * @param maze           the maze to be searched
     * @param forkAfter      the number of steps (visited nodes) after which a parallel task is forked;
     *                       if <code>forkAfter &lt;= 0</code> the solver never forks new tasks
     * @param startNode      the node from which this solver starts searching
     * @param visitedSet     the set of nodes already visited, shared with other solvers
     * @param predecessorMap the map of predecessors for each visited node, shared with other solvers
     */
    public ForkJoinSolver(Maze maze, int forkAfter, int startNode, Set<Integer> visitedSet,
                          Map<Integer, Integer> predecessorMap) {
        this(maze, forkAfter);
        this.start = startNode;
        this.visited = visitedSet;
        this.predecessor = predecessorMap;
    }


    /**
     * Searches for and returns the path, as a list of node
     * identifiers, that goes from the start node to a goal node in
     * the maze. If such a path cannot be found (because there are no
     * goals, or all goals are unreacheable), the method returns
     * <code>null</code>.
     *
     * @return   the list of node identifiers from the start node to a
     *           goal node in the maze; <code>null</code> if such a path cannot
     *           be found.
     */
    @Override
    public List<Integer> compute() {
        return parallelSearch();
    }

    /**
     * Performs parallel depth-first search with optional child forking.
     */
    private List<Integer> parallelSearch() {

        int stepsSinceFork = 0;
        int player = maze.newPlayer(start);

        // Stack for DFS
        Stack<Integer> frontier = new Stack<>();
        frontier.push(start); // Adds starting node to stack

        // Loops until stack is empty or goal is found
        while (!frontier.isEmpty() && !goalFound.get()) {

            int currentNode = frontier.pop(); // Removes latest node added to stack

            // Only process if not already visited
            if (visited.add(currentNode) || currentNode == start) {
                maze.move(player, currentNode); // Move the player to the current node

                // Check if current node is a goal
                if (maze.hasGoal(currentNode)) {
                    goalFound.set(true); // Stop other parallel threads
                    return reconstructPath(currentNode); // Return path
                }

                stepsSinceFork++;
                boolean isFirstNeighbor = true;

                // Explore neighbors to currentnode
                for (int neighbor : maze.neighbors(currentNode)) {
                    if (!visited.contains(neighbor)) { // If the neighbour isnt already visited
                        predecessor.putIfAbsent(neighbor, currentNode);  // Set predecessor for path reconstruction

                        // Either continue DFS locally or fork a new child
                        if (isFirstNeighbor || stepsSinceFork < forkAfter) {
                            frontier.push(neighbor);
                            isFirstNeighbor = false;
                        } else {
                            if (visited.add(neighbor)) {
                                stepsSinceFork = 0;

                                // Fork new solver for this neighbor
                                ForkJoinSolver forkedSolver = new ForkJoinSolver(
                                        maze, forkAfter, neighbor, visited, predecessor); // Share visited nodes and predecessor with parent
                                subSolvers.add(forkedSolver);
                                forkedSolver.fork();
                            }
                        }
                    }
                }
            }
        }

        /**
         * Waits for all forked sub-solvers to finish and returns the first found path.
         * Each sub-solver is joined to get its result. If a path is found, it is
         * combined with the path to the sub-solver's start node and returned.
         * Returns null if no sub-solver finds a path.
         */
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
     * Reconstructs the path from the start node to the specified node.
     *
     * The method follows the 'predecessor' map from the given node backwards
     * to the start node, inserting each node at the beginning of the list
     * to build the path in the correct order.
     *
     * @param node the target node to which the path should be reconstructed
     * @return a list of node identifiers representing the path from start to the given node
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
