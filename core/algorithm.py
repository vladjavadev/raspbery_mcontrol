
import threading
from dstar.d_star_lite import DStarLite
from dstar.grid import OccupancyGridMap, SLAM
import data as srv
import robot.move_logic as lgc
import time
from data.grid_dto import GridDto 

OBSTACLE = 255
UNOCCUPIED = 0


def run_algorithm(dto: GridDto, stop_event: threading.Event):

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    while dto.x_dim is None or dto.y_dim is None:
        print("Waiting for grid dimensions...")
        time.sleep(0.2)

    dto.build_world()
    start_pos = tuple(dto.start)
    dto.set_position(start_pos)
    goal = tuple(dto.goal)
    view_range = 5


    new_map = dto.world
    
    # Add obstacles
    # obstacles = [
    #     (4, 4), (3, 4), (3, 5),  # Horizontal wall
    #      (7, 3), (7, 4),  # Another wall
    #     (5, 7), (6, 7), (7, 7),  # Vertical wall
    # ]
    
    # # Place obstacles
    # for obs in obstacles:
    #     new_map.set_obstacle(obs)
    
    old_map = new_map

    new_position = start_pos
    last_position = start_pos

    # new_observation = None
    # type = OBSTACLE

    # D* Lite (optimized)
    dstar = DStarLite(map=new_map,
                      s_start=start_pos,
                      s_goal=goal)

    # SLAM to detect vertices
    slam = SLAM(map=new_map,
                view_range=view_range)

    # Initial path planning
    path, g, rhs = dstar.move_and_replan(robot_position=new_position)
    
    if path is None:
        print("No valid path found from start to goal!")
        exit(1)
        
    print(f"Initial path found: {path}")
    
    # logic = lgc.Logic(pos=new_position, dir=(0,1), vMode=2)
    # for obs in obstacles:
    #     new_map.set_obstacle(obs)
    # Only proceed if we have a valid path
    if path:
        dto.set_path(path)
    try:

        while True:
            if( stop_event.is_set()):
                print("Stopping algorithm as stop_event is set.")
                break
            
            time.sleep(0.1)
            start = time.time()

            
            # update the map
            # print(path)
            # drive gui
            # if path[0]==dto.get_goal():
            #     logic.move_robot(path,dto.get_position())
            #     print("Reached goal!")
            #     break

            new_position = dto.get_position()
            new_observation = dto.observation
            new_map = dto.world

            # logic.move_robot(path,new_position)
            if new_observation is not None:
                old_map = new_map
                slam.set_ground_truth_map(gt_map=new_map)

            # print("new_pos and last_pos",new_position,last_position)
            if new_position != last_position:
                dto._lock.acquire()
                last_position = new_position

                # slam
                new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)

                dstar.new_edges_and_old_costs = new_edges_and_old_costs
                dstar.sensed_map = slam_map

                # d star

                path, g, rhs = dstar.move_and_replan(robot_position=new_position)

                end = time.time()
                dto._lock.release()
            dto.set_path(path)
                
    except TypeError as e:
        print(e)
    finally:
        stop_event.set()



if __name__ == "__main__":
    run_algorithm(srv.g_dt)

