# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def create_path(end, start, graph):
    path = []
    current = end
    while True:
        current, direction = graph[current]
        path.append(direction)
        if current == start:
            break
    return path[::-1]

def traversal(problem, datastruct):
    start = problem.startState
    if problem.isGoalState(start):
        return []
    path = {}
    frontier = datastruct()
    frontier.push(start)
    explored = set()
    while not frontier.isEmpty():
        node = frontier.pop()
        explored.add(node)
        for child, direction, cost in problem.getSuccessors(node):
            if child not in explored and child not in frontier.list:
                path[child] = node, direction
                frontier.push(child)
    raise Exception('Not found')


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    start = problem.getStartState()
    explored = set()
    frontier = util.Stack()
    frontier.push((start, []))
    while not frontier.isEmpty():
        node, path = frontier.pop()
        explored.add(node)
        if problem.isGoalState(node):
            return path
        for child, direction, cost in problem.getSuccessors(node):
            if child in explored:
                continue
            frontier.push((child, path + [direction]))
    return None


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from searchAgents import CornersProblem

    if isinstance(problem, CornersProblem):
        trail = []
        start = problem.startingPosition
        for corner in problem.corners:
            child, path = _path_to_node(problem, start, corner)
            start = child
            trail.extend(path)
        return trail
    else:
        start = problem.getStartState()
        explored = set()
        frontier = util.Queue()
        frontier.push((start, []))
        while not frontier.isEmpty():
            node, path = frontier.pop()
            if node in explored:
                continue
            if problem.isGoalState(node):
                return path
            explored.add(node)
            for child, direction, cost in problem.getSuccessors(node):
                frontier.push((child, path + [direction]))
        return None


def _path_to_node(problem, start, target):
    path = {}
    frontier = util.Queue()
    frontier.push(start)
    explored = set()
    while not frontier.isEmpty():
        node = frontier.pop()
        explored.add(node)
        for child, direction, cost in problem.getSuccessors(node):
            if child not in explored and child not in frontier.list:
                path[child] = node, direction
                if child == target:
                    return child, create_path(child, start, path)
                frontier.push(child)
    raise Exception('Not found')



def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []
    frontier = util.PriorityQueue()
    frontier.push((start, [], 0), priority=0)
    explored = set()
    while not frontier.isEmpty():
        node, path, cost = frontier.pop()
        if node in explored:
            continue
        if problem.isGoalState(node):
            return path
        explored.add(node)
        for child, direction, child_cost in problem.getSuccessors(node):
            cum_cost = child_cost + cost
            frontier.push((child, path + [direction], cum_cost), priority=cum_cost)
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def _path_to_node_priority(problem, start, target, heuristic):
    path = {}
    frontier = util.PriorityQueue()
    frontier.push(start, priority=1)
    explored = set()
    while not frontier.isEmpty():
        node = frontier.pop()
        explored.add(node)
        for child, direction, cost in problem.getSuccessors(node):
            if child not in explored and child not in frontier.list:
                path[child] = node, direction
                if child == target:
                    return child, create_path(child, start, path)
                frontier.push(child, priority=heuristic(child, problem))
    raise Exception('Not found')


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    start = problem.getStartState()
    visited = set()
    frontier = util.PriorityQueue()
    frontier.push((start, [], 0), priority=0)
    while not frontier.isEmpty():
        node, path, node_cost = frontier.pop()
        if problem.isGoalState(node):
            return path
        if node in visited:
            continue
        for child, direction, child_cost in problem.getSuccessors(node):
            expand_cost = node_cost + child_cost
            total_cost = expand_cost + heuristic(child, problem)
            frontier.push((child, path + [direction], expand_cost), priority=total_cost)
        visited.add(node)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
