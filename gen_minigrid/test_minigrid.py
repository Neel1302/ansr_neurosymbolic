import gym
import my_custom_minigrid  # Import your custom environment file

# Define object positions (e.g., 'goal', 'key', 'door', 'wall')
object_positions = {
    'goal': (5, 5),
    'key': (2, 2),
    'door': (4, 3),
    'wall': (3, 3)
}

# Create the custom MiniGrid environment with the specified object positions
env = gym.make('MiniGrid-Custom-v0', object_positions=object_positions)

# Reset the environment to get the initial observation
obs = env.reset()

done = False
while not done:
    # Render the environment
    env.render()

    # Get a random action
    action = env.action_space.sample()

    # Step the environment with the chosen action
    obs, reward, done, info = env.step(action)

    # Print the observation, reward, and info (optional)
    print("Observation:", obs)
    print("Reward:", reward)
    print("Info:", info)

# Close the environment
env.close()
