package amazed.solver;

import amazed.maze.Maze;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * ForkJoinSolver — parallell DFS med rekursiv forking vid korsningar.
 * Föräldern spawnar barn i korsningar (fler än 1 väg).
 * Barnen kan i sin tur spawn:a egna barn längre fram.
 */
public class ForkJoinSolver extends SequentialSolver {

    private final Set<Integer> sharedVisited;
    private final Map<Integer, Integer> sharedPredecessor;
    private final AtomicBoolean goalFound;
    private final int taskStart;

    public ForkJoinSolver(Maze maze, int forkAfter) {
        super(maze);
        this.forkAfter = forkAfter;
        this.sharedVisited = ConcurrentHashMap.newKeySet();
        this.sharedPredecessor = new ConcurrentHashMap<>();
        this.goalFound = new AtomicBoolean(false);
        this.taskStart = maze.start();
    }

    private ForkJoinSolver(Maze maze, int start,
                           int forkAfter,
                           Set<Integer> visited,
                           Map<Integer, Integer> predecessor,
                           AtomicBoolean goalFound) {
        super(maze);
        this.forkAfter = forkAfter;
        this.sharedVisited = visited;
        this.sharedPredecessor = predecessor;
        this.goalFound = goalFound;
        this.taskStart = start;
    }

    @Override
    public List<Integer> compute() {
        return parallelSearch();
    }

    private List<Integer> parallelSearch() {
        int playerId = maze.newPlayer(taskStart);
        Deque<Integer> stack = new ArrayDeque<>();
        stack.push(taskStart);

        while (!stack.isEmpty() && !goalFound.get()) {
            int current = stack.pop();

            // tillåt att startnoden alltid utforskas
            if (current != taskStart && !sharedVisited.add(current))
                continue;

            maze.move(playerId, current);

            // mål hittat
            if (maze.hasGoal(current)) {
                goalFound.set(true);
                return pathFromTo(maze.start(), current);
            }

            // samla alla grannar som inte är besökta
            List<Integer> neighbors = new ArrayList<>();
            for (int nb : maze.neighbors(current)) {
                if (!sharedVisited.contains(nb)) {
                    sharedPredecessor.putIfAbsent(nb, current);
                    neighbors.add(nb);
                }
            }

            // vid korsning (>1 möjlig väg): forka
            if (neighbors.size() > 1) {
                List<ForkJoinSolver> children = new ArrayList<>();

                // parent tar första vägen, barn tar resten
                int first = neighbors.remove(0);
                stack.push(first);

                for (int nb : neighbors) {
                    if (sharedVisited.add(nb)) {
                        ForkJoinSolver child = new ForkJoinSolver(
                                maze, nb, forkAfter, sharedVisited, sharedPredecessor, goalFound);
                        children.add(child);
                        child.fork();
                    }
                }

                // vänta på barn — om någon hittar mål, returnera vägen
                for (ForkJoinSolver child : children) {
                    List<Integer> res = child.join();
                    if (res != null) {
                        return res;
                    }
                }

            } else if (neighbors.size() == 1) {
                // bara en väg — fortsätt sekventiellt
                stack.push(neighbors.get(0));
            }
            // annars: dead end → fortsätt poppa
        }

        // 🔧 FIX: Om någon annan tråd hittade målet, försök återskapa vägen
        if (goalFound.get()) {
            List<Integer> path = reconstructIfGoalExists();
            if (path != null) return path;
        }

        return null;
    }

    /**
     * Hjälpmetod som försöker rekonstruera vägen när en annan tråd hittat mål.
     */
    private List<Integer> reconstructIfGoalExists() {
        // hitta någon nod som har en väg till mål
        for (Integer id : sharedVisited) {
            if (maze.hasGoal(id)) {
                return pathFromTo(maze.start(), id);
            }
        }
        return null;
    }
}
