import unittest
from point import Point
from punchcost import PunchCost
from boxing_2_0_0.boxing_2_0_0.graphnode import Graph, Node
from optimalAction import OptimalAction
import numpy as np

class TestOptimalAction(unittest.TestCase):
    def setUp(self):
        # Set up a basic environment for the tests
        self.punch_cost = PunchCost(sigma=100.0, grid_size=11)
        self.sandbag_position = Point(5, 5)  # Example position
        self.optimal_action = OptimalAction(self.punch_cost, self.sandbag_position, 100, 200)
        
        # Add nodes to the graph based on points from PunchCost
        for point in self.punch_cost.getPoints():
            node = Node(point)
            self.optimal_action.Graph.insertNode(node)
        
        # Assuming some costs are already calculated, let's update the cost for test purposes
        for node in self.optimal_action.Graph.nodes.values():
            # Randomly assigning costs for the sake of testing
            node.updateCost(np.random.uniform(0.1, 0.5))

    def test_getOptimalAction_noMovementNeeded(self):
        # Test case where no movement is needed
        left_coordinate = Point(10, 10)
        right_coordinate = Point(-10, -10)
        center_coordinate = Point(0, 0)
        result = self.optimal_action.getOptimalAction(left_coordinate.getPosition(), right_coordinate.getPosition(), center_coordinate.getPosition())
        self.assertEqual(result, (0,0), "Expected no movement")

    def test_getOptimalAction_moveNeeded(self):
        # Test case where movement is needed
        self.optimal_action.Graph.updateCost()  # Ensure all costs are updated
        left_coordinate = Point(10, 10)
        right_coordinate = Point(-10, -10)
        center_coordinate = Point(0, 0)
        result = self.optimal_action.getOptimalAction(left_coordinate.getPosition(), right_coordinate.getPosition(), center_coordinate.getPosition())
        self.assertNotEqual(result, (0,0), "Expected movement to a new position")

    def test_GraphInitialization(self):
        # Check if graph initialization works as expected
        self.assertTrue(self.optimal_action.Graph.nodes, "Graph should have nodes initialized from PunchCost points")
        self.assertTrue(self.optimal_action.Graph.adjacentDistance is not None, "Graph should have an adjacent distance set")

    def test_NodeAdjacency(self):
        # Verify that nodes have been correctly set as adjacent
        some_node = next(iter(self.optimal_action.Graph.nodes.values()))
        self.assertTrue(some_node.getAdjacentNode(), "Nodes should have adjacent nodes due to defined adjacent distance")

    # Add more tests as needed to cover all methods and cases

if __name__ == '__main__':
    unittest.main()
