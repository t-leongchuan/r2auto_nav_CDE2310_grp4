"""
======================================================================
This file is based on original code and ideas by:
    Kai Nakamura
    https://kainakamura.com/project/slam-robot

Description:
    A Utility-Based class that provides BFS-Based Search to identify frontiers
    in the Occupancy Grid Given.
    
    Note: Not a ROS2 Node.

Modifications:
    - Minimal Changes were made.s
======================================================================
"""
from .path_planner import PathPlanner
from nav_msgs.msg import OccupancyGrid
from cde2310_interfaces.msg import Frontier, FrontierList



# ==============================================================================
# Constants for filtering and validating frontier cells
# ==============================================================================

# Maximum value to consider a cell walkable (lower = more cautious)
# Effect: Helps ignore semi-transparent obstacles or fuzzy edges
WALKABLE_THRESHOLD = 50

# Minimum frontier size in number of cells
# Effect: Filters out tiny, unreachable gaps
MIN_FRONTIER_SIZE = 4


class FrontierSearch:
    
    '''
    Performs a BFS search starting from a coordinate and ripples out from the start
    to identify frontiers.

    @param mapdata Occupany Grid
    @param start Starting Coordinates to perform search
    @param include_frontier_cells Optional boolean 
    '''
    @staticmethod
    def search(
        mapdata: OccupancyGrid,
        start: "tuple[int, int]",
        include_frontier_cells: bool = False):

        # Create Queue for BFS
        queue = []
        queue.append(start)

        # Initialize Dictionary for keeping track of visited and Frontier Cells
        visited = {}
        is_frontier = {}
        visited[start] = True

        # Initialize list of frontier cells
        # Recall that a frontier consists of a size and a centroid (Point)
        frontiers = []

        # Initialize list of frontier cells
        frontier_cells = []

        while queue:
            current = queue.pop(0)

            # A neighbour is a tuple(int, int) and represents a cell coordinate.
            # TODO: neighbors_of_4 method
            for neighbor in PathPlanner.neighbors_of_4(mapdata, current):

                # Value is the probability of occupancy.
                # 0 indicates a free cell, while 100 indicates occupied cell (obstacle)
                # Note that a value of -1 indicates unknown cell.
                # TODO: PathPlanner get_cell_value method
                neighbor_value = PathPlanner.get_cell_value(mapdata, neighbor)
                
                # If this neighbor cell is known, add it to the frontier.
                if neighbor_value >= 0 and not neighbor in visited:
                    visited[neighbor] = True
                    queue.append(neighbor)

                # Else the value is -1, and current neighbor likely a frontier. 
                elif FrontierSearch.is_new_frontier_cell(mapdata, neighbor, is_frontier):
                    is_frontier[neighbor] = True

                    '''
                    Since we have identified one frontier cell, we can perform
                    another BFS using this frontier cell as the starting point
                    to identify the entire stretch of frontier that this frontier
                    cell belongs to.

                    This elif block will only be triggered if it is a new frontier
                    that has yet to been identified, thus the need for a dictionary
                    to keep track of which neighbor is a frontier_cell to avoid
                    excessive calls to this new BFS search.
                    '''

                    # TODO: To be annotated
                    #
                    #
                    new_frontier, new_frontier_cells = (
                        FrontierSearch.build_new_frontier(mapdata, 
                            neighbor, 
                            is_frontier, 
                            include_frontier_cells
                        )
                    )

                    # TODO: To be annotated
                    if new_frontier.size >= MIN_FRONTIER_SIZE:
                        frontiers.append(new_frontier)
                        if include_frontier_cells:
                            frontier_cells.extend(new_frontier_cells)
                            
        return (FrontierList(frontiers=frontiers), frontier_cells)

    '''
    Performs a BFS search starting from a coordinate and ripples out from the start
    to map out frontier.

    @param mapdata Occupany Grid
    @param initial_cell Starting frontier cell Coordinates to perform search
    @param is_frontier: Dictionary of (coordinate, Visited).
    @param include_frontier_cells Optional boolean 
    '''
    @staticmethod
    def build_new_frontier(mapdata: OccupancyGrid, 
        initial_cell: "tuple[int, int]", 
        is_frontier: dict, 
        include_frontier_cells: bool = False
    ):

        # Initialize Frontier Fields
        size = 1
        centroid_x = initial_cell[0]
        centroid_y = initial_cell[1]

        # Create queue for BFS
        queue = []
        queue.append(initial_cell)

        # Initialize list of frontier cells
        frontier_cells = []

        # BFS for Frontier Cells
        while queue:
            current = queue.pop(0)

            # TODO: to be annotated
            if include_frontier_cells:
                frontier_cells.append(current)

            # Neighbor consists of tuple(int, int) and represents a cell coordinate.
            # TODO: neighbors_of_8 method
            for neighbor in PathPlanner.neighbors_of_8(mapdata, current):
                if FrontierSearch.is_new_frontier_cell(mapdata, neighbor, is_frontier):

                    # Mark as Frontier
                    is_frontier[neighbor] = True

                    # Update Size and Centroid
                    size += 1
                    centroid_x += neighbor[0]
                    centroid_y += neighbor[1]
                    queue.append(neighbor)


        # Calculate Centroid by taking the average
        # For simplicity sake, just treat centroid as center point of this frontier.
        centroid_x /= size
        centroid_y /= size

        # Make and return new frontier 
        # TODO: grid_to_world() method
        # TODO: Annotate
        centroid = PathPlanner.grid_to_world(
            mapdata, (int(centroid_x), int(centroid_y))
        )

        return (
            Frontier(size=size, centroid=centroid),
            frontier_cells
        )
    
    '''
    Checks if the Given Cell is part of a new frontier or has already been identified
    in a known frontier.
    '''
    @staticmethod
    def is_new_frontier_cell(
        mapdata: OccupancyGrid, cell: "tuple[int, int]", is_frontier: dict
    ):

        # Cell must be unknown and not already a frontier
        if PathPlanner.get_cell_value(mapdata, cell) != -1 or cell in is_frontier:
            return False
        
        # Cell should have at least one connected cell that is free
        for neighbor in PathPlanner.neighbors_of_4(mapdata, cell):
            neighbor_value = PathPlanner.get_cell_value(mapdata, neighbor)

            if neighbor_value >= 0 and neighbor_value < WALKABLE_THRESHOLD:
                return True
        
        return False
