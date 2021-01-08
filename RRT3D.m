% RRT with 3D spherical obstacles
% RRT creates path from randomly generated points until tree connects to
% goal

clc; clear;
clf
bndry = [0, 40, 0, 40, 0, 40]; % x limits, y limits, z limits
S = 200; % number of spheres
r = 1.5; % radius of spheres
spheres = randi([10 33], [S, 3]); % randomly pick sphere locations
branch_length = 1;
plot_w_space(bndry, spheres, r)

start = [3, 5, 10]; % start point
hold on
plot3(start(1), start(2), start(3), 'r*')
goal = [35, 35, 35]; % goal point
plot3(goal(1), goal(2), goal(3), 'r*')

PATH = generateRRT(start, goal, spheres, r, bndry, branch_length);

if ~isempty(PATH)
    % spin
    for i = 5:200
        view([i 10])
        pause(0.09)
    end
end


function plot_w_space(bndry, spheres, r)
figure(1)

for i=1:size(spheres, 1)
    [x, y, z] = ellipsoid(spheres(i,1), spheres(i, 2), spheres(i, 3),r,r,r,20);
    surf(x, y, z, 'EdgeColor', 'none', 'FaceAlpha',0.3)
    hold on
end

axis equal
axis(bndry)
set(gcf, 'color', 'white')
xlabel('x')
ylabel('y')
zlabel('z')
title('RRT Path Through Obstacle Field')
view([5 10])

end

function PATH = generateRRT(start, goal, spheres, r, bndry, branch_length)

if ~(is_collided(start, spheres, r) || is_collided(goal, spheres, r))
    N = 1500; % max number of samples
    
    Vstart = start; % each tree vertex is a point in configuration space (q)
    Estart = [];
    
    for i=1:N
        % search through config space
        if mod( i , 10) == 0 
            % sample goal every 10 points
            rand_q = goal;
        else
            % sample random point
            rand_q = rand(3,1)'.*(bndry(:,2)- bndry(:,1)) + bndry(:,1);
        end
        
        % find nearest node in tree
        index1 = find_nearest_node(rand_q, Vstart);
        q = Vstart(index1, :);
        
        [can_extend, new_node] = can_it_extend(rand_q, q, spheres, r, branch_length);
          
        if can_extend  % can it extend
            % add new_node to start tree
            Vstart = vertcat(Vstart, new_node);
            Estart = vertcat(Estart, [index1, length(Vstart)]);
            
            % plotting tree
            StartLines = animatedline('color', 'k', 'linewidth',0.8);
            hold on
            
            addpoints(StartLines, q(1), q(2), q(3));
            addpoints(StartLines, new_node(1), new_node(2), new_node(3));
            drawnow limitrate
        
            if all(round(new_node,1) == goal) % if reached goal
                break
            end
            
        end
    end
    
else
    PATH = [];
    disp('start or goal must not be in obstacle')
    return
end

% pick the best path in V and E
PATH = pick_best_path(Estart, Vstart, goal);

% Plotting final path
h = animatedline('color', 'r', 'LineWidth', 2);

for i = 1:size(PATH, 1)
    addpoints(h, PATH(i,1), PATH(i,2), PATH(i,3));
    drawnow limitrate
end
drawnow

end

function bool = is_collided(q, spheres, r)
%Spheres is S by 3
q = repmat(q, size(spheres, 1), 1);
d = sqrt(sum((spheres - q).^2, 2)); % distances
bool = any(d <= r);
end

function closest_node_idx = find_nearest_node(q_rand, V)
q = repmat(q_rand, size(V, 1), 1);
d = sqrt(sum((V - q).^2, 2)); % distances
[~, closest_node_idx] = min(d);
end

function [bool, new_node] = can_it_extend(rand_q, q, spheres, r, branch_length)
% find new point fixed length away in direction of random point
dir_vector = (rand_q - q) / norm(rand_q - q);
potential_q = q + dir_vector * branch_length;

bool = true;
flag = 0;
new_node = potential_q;

% interpolate path and check for collision in each config waypoint
for x = linspace(potential_q(1), q(1), 5)
    for y = linspace(potential_q(2), q(2), 5)
        for z = linspace(potential_q(3), q(3), 5)
            if is_collided([x y z], spheres, r)
                flag = 1;
                bool = false;
                break
            end

        if (flag == 1)
            break
        end
        
    if (flag == 1)
        break
    end

            if all(round([x, y, z]) == rand_q) % if branch passes through point
                new_node = rand_q;
                flag = 1;
                break
            end
            
        end
    end
end
end

function best_path = pick_best_path(Estart, Vstart, goal)
start_path = [];

if all(round(Vstart(end, :), 1) == goal) % if tree reached goal
    row = size(Vstart, 1);
    for z = 1:size(Estart, 1)
        start_path = vertcat(start_path, Vstart(row, :));
        next_row = Estart(row-1, 1);

        if next_row == 1
            break
        else
            row = next_row;
        end
    end

    % Combine and reorient
    start_path = vertcat(start_path, Vstart(1, :));
    best_path = flip(start_path);
    
else  % the path did not connect to goal
    best_path = [];
    disp('path did not reach goal, try increasing iterations')
end
end




