from point import Point
import numpy as np
import matplotlib.pyplot as plt
from punchcost import PunchCost

class Node():
    def __init__(self, point: Point):
        if not isinstance(point, Point):
            raise TypeError("point is not an instance of SandbagPosition")
        self.point = point
        self.adjacent = []
        self.cost = 0

    def getPoint(self):
        return self.point

    def getAdjacentNode(self):
        return self.adjacent
    
    def isCostHigh(self, criticalCost):
        return self.cost > criticalCost
    
    def updateCost(self, cost):
        self.cost = cost
    
    def updateAdjacentNode(self, node):
        if not isinstance(node, Node):
            raise TypeError("The provided object is not an instance of Node.")
        
        if node not in self.adjacent:
            self.adjacent.append(node)

class Graph():
    def __init__(self, PunchCost: PunchCost):
        self.adjacentDistance = None
        self.nodes = {}  # Using a dictionary to store nodes
        self.PunchCost = PunchCost

    def initializeAdjacentDistance(self, adjacentDistance):
        self.adjacentDistance = adjacentDistance

    def insertNode(self, node):
        point_key = node.getPoint().getPosition()  # Use the position as the key
        if point_key not in self.nodes:
            self.nodes[point_key] = node

        for other_key, other_node in self.nodes.items():
            if other_key != point_key:
                distance = self.getNodeDistance(node, other_node)
                if distance < self.adjacentDistance:
                    node.updateAdjacentNode(other_node)
                    other_node.updateAdjacentNode(node)
    
    def updateCostSingle(self, node, cost):
        point_key = node.getPoint().getPosition()
        assert point_key in self.nodes, 'Node not in Graph'
        self.nodes[point_key].updateCost(cost)
    
    def updateCost(self):
        for point_key, node in self.nodes.items():
            cost = self.PunchCost.get_cost_at_point(*point_key)  # Unpack the position tuple
            node.updateCost(cost)

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
            x, y = key
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

