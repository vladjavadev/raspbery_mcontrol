from dstar.grid import OccupancyGridMap 
from threading import Lock
class GridDto:
    def __init__(self,
                 viewing_range=3):
        
        self.cell_unit = 200 #mm
        self.x_dim = None
        self.y_dim = None
        self.start = None
        self.current = self.start
        self.observation = {"pos": None, "type": None}
        self.goal = None
        self.viewing_range = viewing_range
        self.path = None
        self.totalDistance = 0
        self.pred_time = 0
        self.pred_distance = 0
        self._lock = Lock()
        self._lock_dist = Lock()
        self.world = None

    def set_predict_time_distance(self, time, distance):
        self.pred_time = time
        self.pred_distance = distance

    def get_predict_time_distance(self):
        return (self.pred_time, self.pred_distance)

    def build_world(self):
        self.world = OccupancyGridMap(x_dim=self.x_dim,
                                      y_dim=self.y_dim,
                                      exploration_setting='8N')

    def set_start(self, start):
        self.start=tuple(start[0],start[1])

    def set_goal(self, goal):
        self.goal=tuple(goal[0],goal[1])
    def set_distance(self, dist):
        with self._lock_dist:
            self.totalDistance = dist

    def get_total_distance(self):
        return self.totalDistance

    def set_unit(self, cell_unit):
        self.cell_unit = cell_unit
        
    def get_unit(self):
        return self.cell_unit

    def set_dim(self, dim_tuple):
        self.x_dim=dim_tuple[0]
        self.y_dim=dim_tuple[1]
        self.world = OccupancyGridMap(x_dim=self.x_dim,
                                      y_dim=self.y_dim,
                                      exploration_setting='8N')

    def set_path(self, path=None):
        self.path = path
    def get_path(self):
        with self._lock:
            return self.path
    
    def get_position(self):
        return self.current

    def set_position(self, pos: (int, int)):
        with self._lock:
            self.current = pos

    def get_goal(self):
        return self.goal

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_start(self, start: (int, int)):
        self.start = start

    def set_obs(self, grid_cell: (int, int)):
        with self._lock:
            if self.world.is_unoccupied(grid_cell):
                self.world.set_obstacle(grid_cell)
                self.observation = {"pos": grid_cell, "type": "OBSTACLE"}

    def rem_obs(self, grid_cell: (int, int)):
        with self._lock:
            if not self.world.is_unoccupied(grid_cell):
                print("grid cell: ".format(grid_cell))
                self.world.remove_obstacle(grid_cell)
                self.observation = {"pos": grid_cell, "type": "UNOCCUPIED"}