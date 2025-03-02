#!/usr/bin/env python3
# RRT with 3D spherical obstacles
# RRT creates path from randomly generated points until tree connects to goal

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import argparse
from matplotlib.animation import FuncAnimation

def plot_workspace(ax, bndry, spheres, r):
    """Plot the 3D workspace with obstacles"""
    # Create a figure and 3D axis
    for i in range(spheres.shape[0]):
        # Create a sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = spheres[i, 0] + r * np.outer(np.cos(u), np.sin(v))
        y = spheres[i, 1] + r * np.outer(np.sin(u), np.sin(v))
        z = spheres[i, 2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # Plot the surface
        ax.plot_surface(x, y, z, color='b', alpha=0.3, edgecolor=None)
    
    # Set axis limits
    ax.set_xlim(bndry[0], bndry[1])
    ax.set_ylim(bndry[2], bndry[3])
    ax.set_zlim(bndry[4], bndry[5])
    
    # Set labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title('RRT Path Through Obstacle Field')
    
    # Set view angle
    ax.view_init(elev=10, azim=5)
    
    # Make the plot interactive
    plt.tight_layout()
    plt.draw()

def is_collided(q, spheres, r):
    """Check if point q collides with any sphere"""
    # Calculate distances from q to all sphere centers
    distances = np.sqrt(np.sum((spheres - q)**2, axis=1))
    # Return True if any distance is less than or equal to radius
    return np.any(distances <= r)

def find_nearest_node(q_rand, V):
    """Find the nearest node in the tree to q_rand"""
    # Calculate distances from q_rand to all nodes in V
    distances = np.sqrt(np.sum((V - q_rand)**2, axis=1))
    # Return the index of the closest node
    return np.argmin(distances)

def can_it_extend(rand_q, q, spheres, r, branch_length):
    """Check if the tree can extend from q toward rand_q"""
    # Calculate direction vector
    dir_vector = (rand_q - q) / np.linalg.norm(rand_q - q)
    # Calculate potential new node
    potential_q = q + dir_vector * branch_length
    
    # Track the last valid point (start with the new potential point)
    last_valid_point = potential_q
    collision_detected = False
    
    # Interpolate path and check for collision at waypoints
    # To interpolate properly, we need to go from q to potential_q
    # Create interpolation points from q to potential_q
    interp_count = 5  # Number of interpolation points
    
    # Create array of interpolation points from q to potential_q
    interp_points = []
    for t in np.linspace(0, 1, interp_count):
        interp_point = q + t * (potential_q - q)
        interp_points.append(interp_point)
    
    # Check each interpolation point for collision
    for i, point in enumerate(interp_points):
        if is_collided(point, spheres, r):
            collision_detected = True
            # If we're at the first point, we can't extend at all
            if i == 0:
                return False, q
            # Otherwise, use the last valid point
            last_valid_point = interp_points[i-1]
            break
        
        # Check if the point is very close to the random sample
        if np.all(np.round(point) == np.round(rand_q)):
            # If we've reached the random sample without collision
            return True, rand_q
    
    # If we've detected a collision, return the last valid point
    if collision_detected:
        print(f"Collision detected - extending to last valid point")
        return True, last_valid_point
    
    # If we've reached here, the entire extension is valid
    return True, potential_q

def pick_best_path(Estart, Vstart, goal):
    """Pick the best path from the tree"""
    start_path = []
    
    # If tree reached goal
    if np.all(np.round(Vstart[-1], 1) == goal):
        row = Vstart.shape[0] - 1
        for z in range(Estart.shape[0]):
            start_path.append(Vstart[row])
            next_row = Estart[row-1, 0]
            
            if next_row == 0:  # 0-indexed in Python
                break
            else:
                row = next_row
        
        # Add start point and flip
        start_path.append(Vstart[0])
        best_path = np.array(start_path[::-1])
        
    else:  # Path did not connect to goal
        best_path = np.array([])
        print('Path did not reach goal, try increasing iterations')
    
    return best_path

def generate_rrt(fig, ax, start, goal, spheres, r, bndry, branch_length, visualize_tree=True, goal_sample_freq=5, dynamic_sampling=False):
    """Generate RRT path from start to goal"""
    # Check if start or goal is in collision
    if is_collided(start, spheres, r) or is_collided(goal, spheres, r):
        print('Start or goal must not be in obstacle')
        return np.array([])
    
    N = 1500  # Max number of samples
    
    # Parameters for dynamic goal sampling
    initial_sample_freq = 10  # Sample goal every 10 iterations initially
    final_sample_freq = 2     # Sample goal every 2 iterations at the end
    
    # Initialize tree with start point
    Vstart = np.array([start])
    Estart = np.array([])
    
    # Display initial setup
    if visualize_tree:
        plot_workspace(ax, bndry, spheres, r)
        
        # Plot start and goal
        ax.plot([start[0]], [start[1]], [start[2]], 'r*', markersize=10)
        ax.plot([goal[0]], [goal[1]], [goal[2]], 'g*', markersize=10)
        
        # Make the plot visible and interactive
        fig.canvas.draw()
        plt.pause(0.1)
    
    for i in range(N):
        # Determine the current goal sampling frequency
        if dynamic_sampling:
            # Dynamic goal sampling frequency - decreases with iterations
            # Linear interpolation between initial and final frequency based on progress
            progress = i / N  # 0 at start, approaches 1 at end
            current_freq = int(initial_sample_freq - (initial_sample_freq - final_sample_freq) * progress)
            current_freq = max(current_freq, final_sample_freq)  # Ensure it doesn't go below final frequency
        else:
            # Use static sampling frequency
            current_freq = goal_sample_freq
        
        # Search through config space
        if i % current_freq == 0:
            # Sample goal with appropriate frequency
            rand_q = goal
            # Print when sampling goal for debugging
            if dynamic_sampling:
                print(f"Iteration {i}: Sampling goal point (current freq: {current_freq})")
            else:
                print(f"Iteration {i}: Sampling goal point")
        else:
            # Sample random point
            rand_q = np.random.rand(3) * (bndry[1::2] - bndry[::2]) + bndry[::2]
        
        # Find nearest node in tree
        index1 = find_nearest_node(rand_q, Vstart)
        q = Vstart[index1]
        
        # Check if tree can extend
        can_extend, new_node = can_it_extend(rand_q, q, spheres, r, branch_length)
        
        if can_extend:
            # Add new_node to start tree
            Vstart = np.vstack((Vstart, new_node))
            if Estart.size == 0:
                Estart = np.array([[index1, Vstart.shape[0] - 1]])
            else:
                Estart = np.vstack((Estart, [index1, Vstart.shape[0] - 1]))
            
            # Add line to plot
            if visualize_tree:
                # Use different color for lines extending toward goal
                if i % current_freq == 0 and can_extend:
                    line_color = 'g'  # Green for goal-directed extensions
                    line_width = 1.2
                else:
                    line_color = 'k'  # Black for random extensions
                    line_width = 0.8
                
                # Plot tree incrementally with appropriate color
                ax.plot([q[0], new_node[0]], [q[1], new_node[1]], [q[2], new_node[2]], 
                        color=line_color, linewidth=line_width)
                
                # Update the plot to show progress and allow interaction
                if i % 10 == 0:  # Only update display periodically for performance
                    fig.canvas.draw_idle()
                    plt.pause(0.001)  # Small pause to allow for interaction
            
            # Check if reached goal
            if np.all(np.round(new_node, 1) == goal):
                print(f"Goal reached at iteration {i}!")
                break
    
    # Pick the best path
    PATH = pick_best_path(Estart, Vstart, goal)
    
    if PATH.size > 0 and visualize_tree:
        # Plot final path
        for i in range(PATH.shape[0] - 1):
            ax.plot([PATH[i, 0], PATH[i+1, 0]], 
                    [PATH[i, 1], PATH[i+1, 1]], 
                    [PATH[i, 2], PATH[i+1, 2]], 'r-', linewidth=2)
            
        # Ensure the path is visible
        fig.canvas.draw()
        plt.pause(0.1)
    
    return PATH

def animate_view(fig, ax):
    """Animate the view of the 3D plot"""
    for angle in range(5, 365, 5):
        ax.view_init(elev=10, azim=angle)
        fig.canvas.draw()
        plt.pause(0.1)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='3D RRT Path Planning')
    parser.add_argument('--no-vis', action='store_true', help='Disable real-time visualization during search')
    parser.add_argument('--goal-freq', type=int, default=5, help='Base goal sampling frequency (default: 5)')
    parser.add_argument('--obstacles', type=int, default=50, help='Number of obstacles (default: 50)')
    parser.add_argument('--dynamic-sampling', action='store_true', help='Enable dynamic goal sampling (starts at 10, ends at 2)')
    args = parser.parse_args()
    
    # Enable interactive mode for matplotlib
    plt.ion()
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Define workspace boundaries [xmin, xmax, ymin, ymax, zmin, zmax]
    bndry = np.array([0, 40, 0, 40, 0, 40])
    
    # Number and radius of spherical obstacles
    S = args.obstacles
    r = 1.5
    
    # Generate random sphere locations
    spheres = np.random.randint(10, 34, size=(S, 3))
    
    # Branch length for RRT
    branch_length = 1
    
    # Start and goal positions
    start = np.array([3, 5, 10])
    goal = np.array([35, 35, 35])
    
    # Create figure - we'll use this same figure throughout
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Make figure interactive with mouse control
    plt.tight_layout()
    
    # Set visualization flag based on command line argument
    visualize_during_search = not args.no_vis
    
    print(f"Configuration:")
    print(f"- Number of obstacles: {S}")
    if args.dynamic_sampling:
        print(f"- Dynamic goal sampling: Enabled (starts at 10, decreases to 2)")
    else:
        print(f"- Goal sampling frequency: Every {args.goal_freq} iterations")
    print(f"- Real-time visualization: {'Disabled' if args.no_vis else 'Enabled'}")
    print(f"- Interactive mode: Enabled (you can rotate the plot during execution)")
    
    # Generate RRT path using the same figure and axis
    PATH = generate_rrt(fig, ax, start, goal, spheres, r, bndry, branch_length, 
                        visualize_tree=visualize_during_search,
                        goal_sample_freq=args.goal_freq,
                        dynamic_sampling=args.dynamic_sampling)
    
    if PATH.size > 0:
        # Plot the final path if it wasn't visualized during generation
        if not visualize_during_search:
            # We need to set up the workspace and plot for final visualization
            plot_workspace(ax, bndry, spheres, r)
            ax.plot([start[0]], [start[1]], [start[2]], 'r*', markersize=10)
            ax.plot([goal[0]], [goal[1]], [goal[2]], 'g*', markersize=10)
            
            # Now plot the final path
            for i in range(PATH.shape[0] - 1):
                ax.plot([PATH[i, 0], PATH[i+1, 0]], 
                        [PATH[i, 1], PATH[i+1, 1]], 
                        [PATH[i, 2], PATH[i+1, 2]], 'r-', linewidth=2)
            
            # Ensure the figure is drawn
            fig.canvas.draw()
            plt.pause(0.1)
        
        print(f"Path found with {PATH.shape[0]} waypoints!")
        print("Starting view animation - Close the plot window when done")
        
        # Animate view if path found - using the same figure and axis
        animate_view(fig, ax)
    else:
        print("No path found. Try increasing iterations or reducing obstacles.")
    
    # Switch to non-interactive mode for final display
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main() 