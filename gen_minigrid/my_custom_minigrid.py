import gymnasium as gym
from minigrid.core.grid import Grid
from minigrid.core.world_object import Wall, Goal, Door, Key
from minigrid.core.mission import MissionSpace
from minigrid.minigrid_env import MiniGridEnv
from gymnasium.envs.registration import register
import time
import numpy as np
import os

from get_points_from_pgm import get_points_from_pgm

filename = os.path.join('pgm','my_map_200.pgm')

class CustomMiniGridEnv(MiniGridEnv):
    def __init__(self, grid_size = 200, max_steps=1000000000):
        mission_space = MissionSpace(mission_func=lambda: "reach the goal")

        coordinates, width, height = get_points_from_pgm(filename, 225)
        #np.savetxt('output1.txt', coordinates, fmt='%d')
        self.width = width
        self.height = height    

        grid_size = max(grid_size, width + 2)
        
        self.object_positions = []

        object_types = ['wall'] * len(coordinates)
        for coord, obj_type in zip(coordinates, object_types):
            new_coord = (coord[0] + 1, coord[1] + 1)
            self.object_positions.append((obj_type, new_coord))

        super().__init__(
            mission_space = mission_space,
            grid_size = grid_size,
            max_steps = max_steps
            , render_mode = 'human'
        )

    def _gen_grid(self, width, height):
        self.grid = Grid(width, height)
        # Generate border
        #wall_rect takes up 1 width and 1 height on the grid
        self.grid.wall_rect(0, 0, width, height)

        # Place objects according to object_positions
        
        agent_set = False
        for obj, pos in self.object_positions:
            #print(f"Placing {obj} at {pos}")
            if obj == 'wall':
                self.put_obj(Wall(), *pos)
            elif obj == 'goal':
                self.put_obj(Goal(), *pos)
            elif obj == 'key':
                self.put_obj(Key('yellow'), *pos)
            elif obj == 'door':
                self.put_obj(Door('yellow', is_locked=False), *pos)
            elif obj == 'agent':
                self.agent_pos = pos
                self.agent_dir = 0
                agent_set = True
            
        
        print(f"Agent Position: {self.agent_pos}")
        print(f"Agent Direction: {self.agent_dir}") 
        if agent_set:
            assert self.agent_pos is not None, "Agent position must be set in the environment!"
            assert self.agent_dir in [0, 1, 2, 3], "Agent direction must be 0, 1, 2, or 3!"
        else:
            # Handle the case where no agent is placed
            self.agent_pos = (8,8)
            self.agent_dir = 0

        print(f"Agent Position: {self.agent_pos}")
        print(f"Agent Direction: {self.agent_dir}") 

register(
    id='MiniGrid-Custom-v0',
    entry_point='__main__:CustomMiniGridEnv'
)

if __name__ == "__main__":
    # Create the environment
    print("Creating environment")
    env = gym.make('MiniGrid-Custom-v0')

    # Reset the environment
    print("Resetting environment")
    obs = env.reset()

    # Render the environment
    print("Rendering environment")
    while True:
        print("rendering")
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())  # Take a random action to see movement
        env.render()  # Render the environment
        time.sleep(5.0)  # Sleep for a short period to slow down the rendering

        if terminated or truncated:
            obs, info = env.reset()  # Reset the environment if done

    env.close()
