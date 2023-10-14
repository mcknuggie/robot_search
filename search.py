# -*- coding: utf-8 -*-

### Package Imports ###
import heapq
import abc
from typing import List, Optional, Tuple
### Package Imports ###


class Stack:
    """A container with a last-in-first-out (LIFO) queuing policy."""
    def __init__(self):
        self.list = []

    def push(self,item):
        """Push 'item' onto the stack"""
        self.list.append(item)

    def pop(self):
        """Pop the most recently pushed item from the stack"""
        return self.list.pop()

    def isEmpty(self):
        """Returns true if the stack is empty"""
        return len(self.list) == 0

class Queue:
    """A container with a first-in-first-out (FIFO) queuing policy."""
    def __init__(self):
        self.list = []

    def push(self,item):
        """Enqueue the 'item' into the queue"""
        self.list.insert(0,item)

    def pop(self):
        """
          Dequeue the earliest enqueued item still in the queue. This
          operation removes the item from the queue.
        """
        return self.list.pop()

    def isEmpty(self):
        """Returns true if the queue is empty"""
        return len(self.list) == 0

class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

class SearchProblem(abc.ABC):
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """

    @abc.abstractmethod
    def getStartState(self) -> "State":
        """
        Returns the start state for the search problem.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def isGoalState(self, state: "State") -> bool:
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def getSuccessors(self, state: "State") -> List[Tuple["State", str, int]]:
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def getCostOfActions(self, actions) -> int:
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        raise NotImplementedError


ACTION_LIST = ["UP", "DOWN", "LEFT", "RIGHT"]

class GridworldSearchProblem(SearchProblem):
    def __init__(self, file):
        """
        Read the text file and initialize all necessary variables for the search problem
        The first line of two numbers (r,c) are the number of rows and columns of the given matrix.
        The following r rows contain the matrix itself.
        The final row contains the location (row, col) of the starting position of the robot.
        """
        try:
            with open(file, 'r') as file:
                # Read the entire content of the file into a string
                i = 0
                # get dimensions of grid
                first_line = file.readline().split(" ")
                dimensions = (int(first_line[0]), int(first_line[1]))

                # get grid itself
                grid = []
                for i in range(dimensions[0] + 1):
                    if i == 1:
                        continue
                    string_list = file.readline().strip().split(" ")
                    row = [int(x) for x in string_list]
                    grid.append(row)

                # get starting coordinates
                last_line = file.readlines()[-1].strip().split(" ")
                starting_location = (int(last_line[1]), int(last_line[0]))
            
            self.dimensions = dimensions
            self.grid = grid
            self.starting_location = starting_location

            residence_locations = set()
            for i in range(len(grid)):
                for j in range(len(grid[0])):
                    if grid[i][j] == 1:
                        residence_locations.add((j, i))
            
            self.residence_locations = residence_locations

        except FileNotFoundError:
            print(f"The file '{file}' was not found.")
        except Exception as e:
            print(f"An error occurred: {str(e)}")

    def getStartState(self) -> "State":
        return (self.starting_location, [], [])

    def isGoalState(self, state: "State") -> bool:
        # create a set of all residences in the grid and compare it to the current state's tuple (converted to a set) of residences
        if set(state[2]) == self.residence_locations:
            return True

        return False

    def getSuccessors(self, state: "State") -> List[Tuple["State", str, int]]:
        # given a state, return a list of all the possible valid states to visit next
        # successor: a state containing the new location, the new path, and a possibly updated list of visited residences
        # action: direction in which we travel to get to the new state
        # stepCost: always 1

        # only exclude a possible successor if it's out of bounds or an obstacle
        current_location = state[0]
        current_path = state[1]
        current_visited_residences = state[2]

        directions = {
            "UP" : True,
            "RIGHT": True,
            "DOWN": True,   
            "LEFT": True
        }

        # first plainly check if we're currently on a boundary
        if current_location[0] == 0:
            directions["LEFT"] = False
        if current_location[0] == self.dimensions[1]-1:
            directions["RIGHT"] = False
        if current_location[1] == 0:
            directions["UP"] = False
        if current_location[1] == self.dimensions[0]-1:
            directions["DOWN"] = False

        # then, check if any of the neighboring grid squares are obstacles

        # if in top left corner
        if not directions["LEFT"] and not directions["UP"]:
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
        # if in top right corner
        elif not directions["RIGHT"] and not directions["UP"]:
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
        # if in bottom right corner
        elif not directions["DOWN"] and not directions["RIGHT"]:
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False
        # if in bottom left corner
        elif not directions["DOWN"] and not directions["LEFT"]:
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False
        # if ONLY on left boundary
        elif not directions["LEFT"]:
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
        # if ONLY on right boundary       
        elif not directions["RIGHT"]:
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
        # if ONLY on top boundary       
        elif not directions["UP"]:
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
        # if ONLY on bottom boundary       
        elif not directions["DOWN"]:
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False 
        # if not on ANY boundary
        elif directions["UP"] and directions["RIGHT"] and directions["DOWN"] and directions["LEFT"]:
            if self.grid[current_location[1]][current_location[0]-1] == -1:
                directions["LEFT"] = False
            if self.grid[current_location[1]][current_location[0]+1] == -1:
                directions["RIGHT"] = False
            if self.grid[current_location[1]-1][current_location[0]] == -1:
                directions["UP"] = False
            if self.grid[current_location[1]+1][current_location[0]] == -1:
                directions["DOWN"] = False
            
        allowed_directions = []
        for direction, allowed in directions.items():
            if allowed:
                allowed_directions.append(direction)

        successor_list = []
        movement_direction = ""
        for allowed_direction in allowed_directions:
            if allowed_direction == "UP":
                new_location = (current_location[0], current_location[1]-1)
                movement_direction = "UP"
            elif allowed_direction == "RIGHT":
                new_location = (current_location[0]+1, current_location[1])
                movement_direction = "RIGHT"
            elif allowed_direction == "DOWN":
                new_location = (current_location[0], current_location[1]+1)
                movement_direction = "DOWN"
            elif allowed_direction == "LEFT":
                new_location = (current_location[0]-1, current_location[1])
                movement_direction = "LEFT"

            new_path = current_path.copy()
            new_path.append(movement_direction)

            successor_list.append((new_location, new_path, current_visited_residences))

        output_list = []
        for successor in successor_list:
            output_list.append((successor, successor[1][-1], 1))

        return output_list

    def getCostOfActions(self, actions: List[str]) -> int:       
        return len(actions)


def depthFirstSearch(problem: SearchProblem) -> List[str]:
    """ Search the deepest nodes in the search tree first. """

    stack = Stack()
    visited_set = set()

    stack.push(problem.getStartState())
    # store the locations of the squares already visited AND store the set of visited residences at the time of the visit to the square
    visited_set.add((problem.getStartState()[0], ()))

    while not stack.isEmpty():
        state = stack.pop()
        # if we reached a goal state!
        if problem.isGoalState(state):
            return state[1]
        else:
            successors = problem.getSuccessors(state)
            successors = successors[::-1] # reverse list so that it searches in correct order
            for successor in successors:
                if (successor[0][0], tuple(successor[0][2])) not in visited_set:
                    # if we found a residence
                    if successor[0][0] in problem.residence_locations:
                        new_visited_residence_set = state[2].copy()
                        # only append to list of found residences if havent been to this residence before
                        if successor[0][0] not in state[2]:
                            new_visited_residence_set.append(successor[0][0])
                        new_state = (successor[0][0], successor[0][1], new_visited_residence_set)
                    else: # if the valid successor is not a residence
                        new_state = (successor[0][0], successor[0][1], successor[0][2])                       
                    stack.push(new_state)
                    visited_set.add((new_state[0], tuple(new_state[2])))

    print("Didn't find a valid path")
    return -1


def breadthFirstSearch(problem: SearchProblem) -> List[str]:
    """Search the shallowest nodes in the search tree first."""
    
    queue = Queue()
    visited_set = set()

    if problem.getStartState()[0] in problem.residence_locations:
        start_state = (problem.getStartState()[0], problem.getStartState()[1], [problem.getStartState()[0]])
    else:
        start_state = problem.getStartState()

    queue.push(start_state)

    # store the locations of the squares already visited AND store the set of visited residences at the time of the visit to the square
    visited_set.add((start_state[0], frozenset(start_state[2])))

    while not queue.isEmpty():
        state = queue.pop()
        # if we reached a goal state!
        if problem.isGoalState(state):
            # print("reached goal state!")
            return state[1]
        else:
            successors = problem.getSuccessors(state)
            for successor in successors:
                if (successor[0][0], tuple(successor[0][2])) not in visited_set:
                    # if we found a residence
                    if successor[0][0] in problem.residence_locations:
                        new_visited_residence_set = state[2].copy()
                        # only append to list of found residences if havent been to this residence before
                        if successor[0][0] not in state[2]:
                            new_visited_residence_set.append(successor[0][0])
                        new_state = (successor[0][0], successor[0][1], new_visited_residence_set)
                    else: # if the valid successor is not a residence
                        new_state = (successor[0][0], successor[0][1], successor[0][2])                       
                    queue.push(new_state)
                    visited_set.add((new_state[0], tuple(new_state[2])))

    print("Didn't find a valid path")
    return -1


def nullHeuristic(state: "State", problem: Optional[GridworldSearchProblem] = None) -> int:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def simpleHeuristic(state: "State", problem: Optional[GridworldSearchProblem] = None) -> int:
    """
    This heuristic returns the number of residences that you have not yet visited.
    """
    return len(problem.residence_locations) - len(state[2])


def customHeuristic(state: "State", problem: Optional[GridworldSearchProblem] = None) -> int:
    """
    h(n) = Manhattan Distance to the nearest unvisited residence
    """

    visited_residences = set(state[2])
    current_location = state[0]
    all_residences = problem.residence_locations
    # do set subtraction to get unvisted residence locations
    unvisited_residences = all_residences - visited_residences

    if len(unvisited_residences) == 0:
        return 0

    manhattan_distances = []
    for unvisited_residence in unvisited_residences:
        manhattan_distances.append(abs(unvisited_residence[0] - current_location[0]) + abs(unvisited_residence[1] - current_location[1]))

    return min(manhattan_distances)


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[str]:
    """Search the node that has the lowest combined cost and heuristic first.
    This function takes in an arbitrary heuristic (which itself is a function) as an input."""
    
    priority_queue = PriorityQueue()
    visited_set = set()

    # we start on a residence
    if problem.getStartState()[0] in problem.residence_locations:
        start_state = (problem.getStartState()[0], problem.getStartState()[1], [problem.getStartState()[0]])
    else:
        start_state = problem.getStartState()

    priority_queue.push(start_state, heuristic(start_state, problem))

    # store the locations of the squares already visited AND store the set of visited residences at the time of the visit to the square
    visited_set.add((start_state[0], frozenset(start_state[2])))

    while not priority_queue.isEmpty():
        state = priority_queue.pop()
        # if we reached a goal state!
        if problem.isGoalState(state):
            # print("reached goal state!")
            return state[1]
        else:
            successors = problem.getSuccessors(state)
            for successor in successors:
                # only revisit a square if you have visited additional residences since you were last there
                if (successor[0][0], tuple(successor[0][2])) not in visited_set:
                    new_state = ()
                    # if we found a residence
                    if successor[0][0] in problem.residence_locations:
                        new_visited_residence_set = state[2].copy()
                        # only append to list of found residences if havent been to this residence before
                        if successor[0][0] not in state[2]:
                            new_visited_residence_set.append(successor[0][0])
                        new_state = (successor[0][0], successor[0][1], new_visited_residence_set)
                    else: # if the valid successor is not a residence 
                        new_state = (successor[0][0], successor[0][1], successor[0][2])
                    # len(new_state[1]) : g(n) (the total distance from starting state until now); heuristic(new_state, problem) : h(n)
                    visited_set.add((new_state[0], tuple(new_state[2])))
                    priority_queue.update(new_state, len(new_state[1]) + heuristic(new_state, problem))
                    

    print("Didn't find a valid path")
    return -1


if __name__ == "__main__":
    ### Test Cases ###
    gridworld_search_problem = GridworldSearchProblem("sample_test_case1.txt") # Test Case 1
    print(depthFirstSearch(gridworld_search_problem))
    print(breadthFirstSearch(gridworld_search_problem))
    print(aStarSearch(gridworld_search_problem, nullHeuristic))
    print(aStarSearch(gridworld_search_problem, simpleHeuristic))
    print(aStarSearch(gridworld_search_problem, customHeuristic))
    
    gridworld_search_problem = GridworldSearchProblem("sample_test_case2.txt") # Test Case 2
    print(depthFirstSearch(gridworld_search_problem))
    print(breadthFirstSearch(gridworld_search_problem))
    print(aStarSearch(gridworld_search_problem, nullHeuristic))
    print(aStarSearch(gridworld_search_problem, simpleHeuristic))
    print(aStarSearch(gridworld_search_problem, customHeuristic))
    
    gridworld_search_problem = GridworldSearchProblem("sample_test_case3.txt") # Test Case 3
    print(depthFirstSearch(gridworld_search_problem))
    print(breadthFirstSearch(gridworld_search_problem))
    print(aStarSearch(gridworld_search_problem, nullHeuristic))
    print(aStarSearch(gridworld_search_problem, simpleHeuristic))
    print(aStarSearch(gridworld_search_problem, customHeuristic))
