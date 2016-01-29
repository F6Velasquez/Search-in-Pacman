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

    #Nodos a expandir
    OPEN = util.Stack()

    #Nodos expandidos
    CLOSE = []

    #initial position
    #(coordenada,direccion,coste,padre)
    startNode = (problem.getStartState(), "", 0,None)

    #Nodo iniicial --> pila
    OPEN.push(startNode )

    #Indices
    COORD = 0   #State
    DIR = 1     #Action
    COST = 2    #Cost
    FATHER = 3  #Father node

    while not OPEN.isEmpty():

        #Obtenemos elemento de la lista
        vertex = OPEN.pop()

        #Comprobamos si es un goal
        if problem.isGoalState(vertex[COORD]):
            path_to_Goal = []
            #Construimos camino al path
            while vertex[FATHER]!= None:
                path_to_Goal.append(vertex[DIR])
                vertex = vertex[FATHER]

            #Necesario para path= Raiz ----> Goal
            path_to_Goal.reverse()
            return path_to_Goal

        #Comprobar si no se expandio antes
        if not vertex[COORD] in CLOSE:

            #Expandemos el nodo
            sucesors = problem.getSuccessors(vertex[COORD])
            #recorremos los hijos del nodo
            for tempNode in sucesors:
                if not tempNode[COORD] in CLOSE:


                    state=tempNode[COORD]
                    action = tempNode[DIR]
                    cost = tempNode[COST]
                    fath = vertex
                    #creamos un nuevo nodo
                    newNode= (state,action,cost,fath)
                    #Agregamos a nodos a expandir
                    OPEN.push(newNode)

        #Cerramos el nodo expandido
        CLOSE.append(vertex[COORD])
    return []


def breadthFirstSearch(problem):
    #Nodos a expandir
    OPEN = util.Queue()

    #Nodos expandidos
    CLOSE = []

    #initial position
    #(coordenada,direccion,coste,padre)
    startNode = (problem.getStartState(), "", 0,None)

    #Nodo iniicial --> Cola
    OPEN.push(startNode )

    #Indices
    COORD = 0   #State
    DIR = 1     #Action
    COST = 2    #Cost
    FATHER = 3  #Father node

    while not OPEN.isEmpty():

        #Obtenemos elemento de la lista
        vertex = OPEN.pop()

        #Comprobamos si es un goal
        if problem.isGoalState(vertex[COORD]):
            path_to_Goal = []
            #Construimos camino al path
            while vertex[FATHER]!= None:
                path_to_Goal.append(vertex[DIR])
                vertex = vertex[FATHER]

            #Necesario para path= Raiz ----> Goal
            path_to_Goal.reverse()
            return path_to_Goal

        #Comprobar si no se expandio antes
        if not vertex[COORD] in CLOSE:
            #Expandemos el nodo
            sucesors = problem.getSuccessors(vertex[COORD])
            #recorremos los hijos del nodo
            for tempNode in sucesors:
                if not tempNode[COORD] in CLOSE:


                    state=tempNode[COORD]
                    action = tempNode[DIR]
                    cost = tempNode[COST]
                    fath = vertex
                    #creamos un nuevo nodo
                    newNode= (state,action,cost,fath)
                    #Agregamos a nodos a expandir
                    OPEN.push(newNode)

        #Cerramos el nodo expandido
        CLOSE.append(vertex[COORD])
    return []



def uniformCostSearch(problem):
    #teoria: dijkstra es un caso de A* con heuristica 0
    solution = aStarSearch(problem, heuristic=nullHeuristic)
    return solution



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    #Nodos a expandir
    OPEN = util.PriorityQueue()

    #Nodos expandidos
    CLOSE = []

    #initial position
    #(coordenada,direccion,coste,padre)
    startNode = (problem.getStartState(), "", 0,None)

    #Nodo iniicial --> Heap
    OPEN.push(startNode,0 )

    #Indices
    COORD = 0   #State
    DIR = 1     #Action
    COST = 2    #Cost
    FATHER = 3  #Father node

    while not OPEN.isEmpty():

        #Obtenemos elemento de la lista
        vertex = OPEN.pop()

        #Comprobamos si es un goal
        if problem.isGoalState(vertex[COORD]):
            path_to_Goal = []
            #Construimos camino al path
            while vertex[FATHER]!= None:
                path_to_Goal.append(vertex[DIR])
                vertex = vertex[FATHER]

            #Necesario para path= Raiz ----> Goal
            path_to_Goal.reverse()
            return path_to_Goal

        #Comprobar si no se expandio antes
        if not vertex[COORD] in CLOSE:

            #Expandemos el nodo
            sucesors = problem.getSuccessors(vertex[COORD])
            #recorremos los hijos del nodo
            for tempNode in sucesors:
                if not tempNode[COORD] in CLOSE:

                    fath = vertex
                    state=tempNode[COORD]
                    action = tempNode[DIR]

                    #Obtener camino
                    path = []
                    path = path_to_x(vertex, path)
                    path.reverse()
                    path.append(tempNode[DIR])

                    #Coste acumulado
                    gn = problem.getCostOfActions(path)

                    #Coste total fn = gn + h(n)@heuristica
                    fn = gn + heuristic(tempNode[COORD],problem)

                    #creamos un nuevo nodo
                    newNode= (state,action,gn,fath)
                    #Agregamos a nodos a expandir
                    OPEN.push(newNode,fn)

        #Cerramos el nodo expandido
        CLOSE.append(vertex[COORD])
    return []

#Otener camino de raiz a nodo especifico en A*
#Debido a error: sys is not defined en A*
#similar a bucle de Goal
def path_to_x( vertex, path ):
    while vertex[3]!= None:
        path.append(vertex[1])
        vertex = vertex[3]
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
