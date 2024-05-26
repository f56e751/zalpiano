from punchcost import PunchCost
from graphnode import Node
import numpy as np
import matplotlib.pyplot as plt
from point import Point

class Graph():
    def __init__(self, PunchCost: PunchCost, criticalCost):
        self.adjacentDistance = None
        self.nodes = {}  # Using a dictionary to store nodes
        self.PunchCost = PunchCost
        self.criticalCost = criticalCost
        self.maxCost = 0

    def initializeAdjacentDistance(self, adjacentDistance):
        self.adjacentDistance = adjacentDistance

    # def insertNode(self, node: Node):
    #     point_key = node.getPoint().getPosition()  # Use the position as the key
    #     if point_key not in self.nodes:
    #         self.nodes[point_key] = node

    #     for other_key, other_node in self.nodes.items():
    #         if other_key != point_key:
    #             distance = self.getNodeDistance(node, other_node)
    #             if distance < self.adjacentDistance:
    #                 node.updateAdjacentNode(other_node)
    #                 other_node.updateAdjacentNode(node)


    def insertNode(self, node: Node):
        point_key = node.getPoint()  # Use the position as the key
        if point_key not in self.nodes:
            self.nodes[point_key] = node
            # print(f"graph.py -> node point is {node.getPoint().getPosition()}")

        for other_key, other_node in self.nodes.items():
            if other_key != point_key:
                distance = self.getNodeDistance(node, other_node)
                if distance < self.adjacentDistance:
                    node.updateAdjacentNode(other_node)
                    other_node.updateAdjacentNode(node)

    # def moveNextNode(self, node: Node):
    #     # 펀치가 날라와서 근처 코스트가 높아지는 경우
    #         # 현재 노드에서 인접한 노드 중에서 코스트가 제일 낮은 곳으로 감, 대신 이동 시 해당 노드의 코스트가 criticalCost를 넘으면 안됨 
    #     # 펀치가 빠지면서 근처 코스트가 낮아져서 
    #         # 현재 노드의 코스트가 criticalCost보다 낮아지면 현재 노드와 인접한 노드 중에서 가장 코스트가 높은 곳으로 복귀함
    #         # 이동하는 노드의 코스트가 criticalCost보다 높으면 안됨
    #             # 만약 모든 노드의 코스트가 criticalCost보다 낮다먄 원점으로 복귀
    #                 # 이거는 어떻게 알까?
    #                 # 노드의 max cost를 따로 관리하기 -> 노드 전체 코스트 계산 시 저장하면 될듯
    #     if self.maxCost < 0.2:
    #         # TODO self.getNode((0,0)) 이거 잘 되는지 디버깅
    #         return self.getNode((0,0))
        
    #     adjacentNodes = node.getAdjacentNode()
    #     # 재귀로 작성하기
    #     if node.getCost() > self.criticalCost:
    #         for adjacentNode in adjacentNodes:


    # def moveNextNode_case_currentNodeCost_higher_than_criticalCost(self, node: Node):
    #     if self.maxCost < 0.2:
    #         # TODO self.getNode((0,0)) 이거 잘 되는지 디버깅
    #         return self.getNode((0,0))

        


    # def moveNextNode_case_currentNodeCost_lower_than_criticalCost(self, node: Node):
    #     if self.maxCost < 0.2:
    #         # TODO self.getNode((0,0)) 이거 잘 되는지 디버깅
    #         return self.getNode((0,0))
        

    # def moveNextNode_case_currentNodeCost_higher_than_criticalCost(self, node: Node, visited=None):
    #     if visited is None:
    #         visited = set()

    #     visited.add(node)
    #     if node.getCost() <= self.criticalCost:
    #         return node  # Found a node with cost lower than critical

    #     minNode = None
    #     minCost = float('inf')
    #     for adjNode in node.getAdjacentNode():
    #         print(adjNode)
    #         if adjNode not in visited and adjNode.getCost() > self.criticalCost:
    #             if adjNode.getCost() < minCost:
    #                 candidate = self.moveNextNode_case_currentNodeCost_higher_than_criticalCost(adjNode, visited)
    #                 if candidate and candidate.getCost() < minCost:
    #                     minCost = candidate.getCost()
    #                     minNode = candidate

    #     return minNode if minNode else None  # Return the node found or None if no suitable node exists

    # def moveNextNode_case_currentNodeCost_lower_than_criticalCost(self, node: Node, visited=None):
    #     if visited is None:
    #         visited = set()

    #     visited.add(node)
    #     if node.getCost() > self.criticalCost:
    #         return node  # Found a node with cost higher than critical

    #     maxNode = None
    #     maxCost = float('-inf')
    #     for adjNode in node.getAdjacentNode():
    #         print(adjNode)
    #         if adjNode not in visited and adjNode.getCost() < self.criticalCost:
    #             if adjNode.getCost() > maxCost:
    #                 candidate = self.moveNextNode_case_currentNodeCost_lower_than_criticalCost(adjNode, visited)
    #                 if candidate and candidate.getCost() > maxCost:
    #                     maxCost = candidate.getCost()
    #                     maxNode = candidate

    #     return maxNode if maxNode else None  # Return the node found or None if no suitable node exists

    def moveNextNode_case_currentNodeCost_higher_than_criticalCost(self, node: Node, visited=None):
        # 전부 다 critical값을 넘으면 주변으로 피하는 로직 추가
        if visited is None:
            visited = set()

        visited.add(node)
        if node.getCost() <= self.criticalCost:
            return node.getPoint()  # Found a node with cost lower than critical

        minNode = None
        minNode = self.findMin(node.getCost(), node.getAdjacentNode())
        # print(f"minNode point is {minNode.getPoint().getPosition()}")
        if minNode == None:
            # print(f"node point is {node.getPoint().getPosition()}")
            return node.getPoint()
        
        while minNode.getCost() > self.criticalCost:
            currentNode = minNode
            minNode = self.findMin(currentNode.getCost(), currentNode.getAdjacentNode())
            if minNode == None:
                return currentNode.getPoint()
            

        # print(f"minNode point is {minNode.getPoint().getPosition()}")
        return minNode.getPoint()


    def findMin(self, currentCost, adjNodes):
        minCost = float('inf')
        minNode = None
        for adjNode in adjNodes:
            adjCost = adjNode.getCost()
            if adjCost < minCost and adjCost < currentCost:
                minCost = adjCost
                minNode = adjNode
        return minNode
    

    def findMax(self, currentCost, adjNodes):
        maxCost = - float('inf')
        maxNode = None
        for adjNode in adjNodes:
            adjCost = adjNode.getCost()
            if adjCost > maxCost and adjCost > currentCost:
                maxCost = adjCost
                maxNode = adjNode
        return maxNode


    def moveNextNode_case_currentNodeCost_lower_than_criticalCost(self, node: Node, visited=None):
        if visited is None:
            visited = set()

        visited.add(node)
        if node.getCost() >= self.criticalCost:
            return node.getPoint()  # Found a node with cost lower than critical

        maxNode = None
        maxNode = self.findMax(node.getCost(), node.getAdjacentNode())
        if maxNode == None:
            return node.getPoint()
        
        while maxNode.getCost() < self.criticalCost:
            currentNode = maxNode
            maxNode = self.findMax(currentNode.getCost(), currentNode.getAdjacentNode())
            if maxNode == None:
                return currentNode.getPoint()

        return maxNode.getPoint()



    def getNode(self, pointKey: Point):
        return self.nodes[pointKey]
    
    def updateCostSingle(self, node, cost):
        point_key = node.getPoint()
        assert point_key in self.nodes, 'Node not in Graph'
        self.nodes[point_key].updateCost(cost)
    
    def updateCost(self):
        self.maxCost = 0
        for point_key, node in self.nodes.items():
            cost = self.PunchCost.get_cost_at_point(point_key) 
            node.updateCost(cost)
            if cost > self.maxCost:
                self.maxCost = cost

    def getCostAtPoint(self, point):
        # Return the cost of the node at a specific point (constant time lookup)
        assert point in self.nodes, 'point not in nodes'

        if point in self.nodes:
            return self.nodes[point].cost
        return None  # If the point does not exist in the graph

    def getNodeDistance(self, node1: Node, node2: Node):
        x_node1, y_node1 = node1.getPoint().getPosition()
        x_node2, y_node2 = node2.getPoint().getPosition()
        return np.sqrt((x_node1 - x_node2)**2 + (y_node1 - y_node2)**2)
    
    def display_graph(self):
        for key, node in self.nodes.items():
            adjacents = [n.getPoint().getPosition() for n in node.getAdjacentNode()]
            print(f"Node {key} has adjacent nodes at {adjacents}")

    def plot_graph(self):
        fig, ax = plt.subplots()
        for key, node in self.nodes.items():
            x, y = key.getPosition()
            ax.scatter(x, y, color='blue')
            for adj in node.getAdjacentNode():
                adj_x, adj_y = adj.getPoint().getPosition()
                ax.plot([x, adj_x], [y, adj_y], 'r-')
        ax.set_aspect('equal')
        plt.grid(True)
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Graph Nodes and Connections')
        plt.show()

    def getSize(self):
        return len(self.nodes)
    
    def getMaxCost(self):
        return self.maxCost


# class Graph():
#     def __init__(self, PunchCost: PunchCost):
#         self.adjacentDistance = None
#         self.nodes = [] ######################
#         # dict로 바꾸기?
#         self.PunchCost = PunchCost

#     def initializeAdjacentDistance(self, adjacentDistance):
#         self.adjacentDistance = adjacentDistance

#     def insertNode(self, node):
#         if node not in self.nodes:
#             self.nodes.append(node)

#         for otherNode in self.nodes:
#             if otherNode != node:
#                 distance = self.getNodeDistance(otherNode, node)
#                 if distance < self.adjacentDistance:
#                     otherNode.updateAdjacentNode(node)
#                     node.updateAdjacentNode(otherNode)
    
#     def updateCostSingle(self, node, cost):
#         assert node in self.nodes, 'node not in Graph'
#         if node in self.nodes:
#             node.updateCost(cost)
    
#     def updateCost(self):
#         for node in self.nodes:
#             position = node.getPoint().getPosition()
#             cost = self.PunchCost.get_cost_at_point(position[0], position[1])

#     def getNodeDistance(self, node1: Node, node2: Node):
#         x_node1, y_node1 = node1.getPoint().getPosition()
#         x_node2, y_node2 = node2.getPoint().getPosition()
#         return np.sqrt((x_node1 - x_node2)**2 + (y_node1 - y_node2)**2)
    
#     def display_graph(self):
#         print("display graph")
#         print(self.nodes)
#         for node in self.nodes:
#             adjacents = [n.getPoint().getPosition() for n in node.getAdjacentNode()]
#             print(f"Node {node.getPoint().getPosition()} has adjacent nodes at {adjacents}")

#     def plot_graph(self):
#         fig, ax = plt.subplots()
#         for node in self.nodes:
#             x, y = node.getPoint().getPosition()
#             ax.scatter(x, y, color='blue')
#             for adj in node.getAdjacentNode():
#                 adj_x, adj_y = adj.getPoint().getPosition()
#                 ax.plot([x, adj_x], [y, adj_y], 'r-')
#         ax.set_aspect('equal')
#         plt.grid(True)
#         plt.xlabel('X Coordinate')
#         plt.ylabel('Y Coordinate')
#         plt.title('Graph Nodes and Connections')
#         plt.show()


#     def getSize(self):
#         return len(self.nodes)

if __name__ == "__main__":
    graph = Graph()
    graph.initializeAdjacentDistance(100)  # Set adjacent distance threshold to 5 units

    # Create and insert nodes
    point1 = Point(1, 2)
    point2 = Point(3, 4)
    point3 = Point(5, 6)

    node1 = Node(point1)
    node2 = Node(point2)
    node3 = Node(point3)

    graph.insertNode(node1)
    graph.insertNode(node2)
    graph.insertNode(node3)

    # Update costs and print results
    graph.updateCost(node1, 10)
    graph.updateCost(node2, 20)
    graph.updateCost(node3, 30)

    print(f"Node1 Cost: {node1.cost}")
    print(f"Node2 Cost: {node2.cost}")
    print(f"Node3 Cost: {node3.cost}")

    # Display adjacent nodes
    print(f"Node1 Adjacent Nodes: {[node.getPoint().getPosition() for node in node1.getAdjacentNode()]}")
    print(f"Node2 Adjacent Nodes: {[node.getPoint().getPosition() for node in node2.getAdjacentNode()]}")
    print(f"Node3 Adjacent Nodes: {[node.getPoint().getPosition() for node in node3.getAdjacentNode()]}")

