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
from game import Directions

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
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    visited = set()
    fringe = util.Stack()
    node_parent = {}
    return_path = []
    start_node = [problem.getStartState(), "", 0]

    fringe.push(start_node)
    visited.add(start_node[0])
    while not fringe.isEmpty():
        node = fringe.pop()
        visited.add(node[0])
        if problem.isGoalState(node[0]):
            cur_node = node
            while cur_node[0] != start_node[0]:
                return_path.append(cur_node)
                cur_node = node_parent[cur_node]
            reverse = return_path[::-1]
            return [a[1] for a in reverse]
        candidates = problem.getSuccessors(node[0])
        for candidate in candidates:
            if candidate[0] not in visited:
                node_parent[candidate] = node
                fringe.push(candidate)
    return None

def breadthFirstSearch(problem):
    visited = set()
    fringe = util.Queue()
    node_parent = {}
    return_path = []
    start_node = [problem.getStartState(), "", 0]

    fringe.push(start_node)
    visited.add(start_node[0])
    while not fringe.isEmpty():
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            cur_node = node
            while cur_node[0] != start_node[0]:
                return_path.append(cur_node)
                cur_node = node_parent[cur_node]
            reverse = return_path[::-1]
            return [a[1] for a in reverse]

        candidates = problem.getSuccessors(node[0])
        for candidate in candidates:
            if candidate[0] not in visited:
                node_parent[candidate] = node
                fringe.push(candidate)
                visited.add(candidate[0])
    return None

def uniformCostSearch(problem):
    visited = set()
    fringe = util.PriorityQueue()
    node_parent = {}
    return_path = []
    start_node = [problem.getStartState(), "", 0]

    fringe.push(start_node, 0)
    while not fringe.isEmpty():
        node = fringe.pop()
        if node[0] in visited:
            continue
        print(node[0])
        visited.add(node[0])
        print(visited)
        if problem.isGoalState(node[0]):
            cur_node = node
            while cur_node[0] != start_node[0]:
                return_path.append(cur_node)
                cur_node = node_parent[cur_node]
            reverse = return_path[::-1]
            return [a[1] for a in reverse]
        candidates = problem.getSuccessors(node[0])
        for candidate in candidates:
            candidate = list(candidate)
            if candidate[0] not in visited:
                candidate[2] += node[2]
                node_parent[tuple(candidate)] = node
                fringe.push(tuple(candidate), candidate[2])
    return None

def nullHeuristic(state, problem=None):

    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    visited = set()
    fringe = util.PriorityQueue()
    node_parent = {}
    return_path = []
    start_node = [problem.getStartState(), "", 0]
    fringe.push(start_node, 0)
    while not fringe.isEmpty():
        node = fringe.pop()
        if node[0] in visited:
            continue
        visited.add(node[0])
        if problem.isGoalState(node[0]):
            cur_node = node
            while cur_node[0] != start_node[0]:
                return_path.append(cur_node)
                cur_node = node_parent[cur_node]
            reverse = return_path[::-1]
            return [a[1] for a in reverse]
        candidates = problem.getSuccessors(node[0])
        for candidate in candidates:
            candidate = list(candidate)
            if candidate[0] not in visited:
                candidate[2] += node[2]
                node_parent[tuple(candidate)] = node
                fringe.push(tuple(candidate), candidate[2]+heuristic(candidate[0], problem))
    return None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
