#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <chrono>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkLineSource.h>
#include <vtkCubeSource.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <thread>

// Structure to represent a color
struct Color
{
    double r, g, b;
    Color(double r = 0, double g = 0, double b = 0) : r(r), g(g), b(b) {}
};

// Structure to represent a 3D point
struct Point3D
{
    double x, y, z;
    Color color; // Add color property

    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z), color(0.3, 0.3, 0.8) {}

    double distanceTo(const Point3D &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Point3D operator-(const Point3D &other) const
    {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D operator+(const Point3D &other) const
    {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D operator*(double scalar) const
    {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    Point3D operator/(double scalar) const
    {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    double norm() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Point3D normalize() const
    {
        double n = norm();
        if (n > 0)
        {
            return Point3D(x / n, y / n, z / n);
        }
        return *this;
    }

    bool operator==(const Point3D &other) const
    {
        // More lenient equality check - points are equal if they are within 0.5 units of each other
        return distanceTo(other) < 0.5;
    }

    friend std::ostream &operator<<(std::ostream &os, const Point3D &p)
    {
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        return os;
    }
};

// Structure to represent an edge in the RRT graph
struct Edge
{
    int from, to;
    Edge(int from = 0, int to = 0) : from(from), to(to) {}
};

// Structure to represent a node in the RRT* tree
struct Node
{
    Point3D point;
    int parent;
    double cost; // Cost from start to this node

    Node(const Point3D &p, int parent = -1, double cost = 0.0)
        : point(p), parent(parent), cost(cost) {}
};

class RRTStar3D
{
private:
    // Configuration parameters
    double boundary[6]; // xmin, xmax, ymin, ymax, zmin, zmax
    double branchLength;
    int maxIterations;
    std::vector<Point3D> spheres; // Obstacles represented as spheres
    double sphereRadius;

    // Planning parameters
    Point3D startPoint;
    Point3D goalPoint;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    int goalSampleFreq;
    bool dynamicSampling;
    int initialSampleFreq;
    int finalSampleFreq;

    // RRT* specific parameters
    double neighborhoodRadius;

    // Visualization
    bool visualize;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;

    // Random number generation
    std::mt19937 rng;

    // Helper function to generate random colors
    Color getRandomColor()
    {
        std::uniform_real_distribution<double> dist(0.1, 0.9);
        return Color(dist(rng), dist(rng), dist(rng));
    }

public:
    RRTStar3D() : branchLength(1.0),
                  maxIterations(500),
                  sphereRadius(1.5),
                  goalSampleFreq(5),
                  dynamicSampling(false),
                  initialSampleFreq(10),
                  finalSampleFreq(2),
                  neighborhoodRadius(5.0),
                  visualize(true)
    {
        // Initialize random number generator with a time-based seed
        rng.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());

        // Set default boundary
        setBoundary(-20, 20, -20, 20, -20, 20);

        // Initialize visualization if enabled
        if (visualize)
        {
            initVisualization();
        }
    }

    // Setters for configuration
    void setStartPoint(const Point3D &start) { startPoint = start; }
    void setGoalPoint(const Point3D &goal) { goalPoint = goal; }
    void setBoundary(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
    {
        boundary[0] = xmin;
        boundary[1] = xmax;
        boundary[2] = ymin;
        boundary[3] = ymax;
        boundary[4] = zmin;
        boundary[5] = zmax;
    }

    void setVisualization(bool vis) { visualize = vis; }
    void setBranchLength(double length) { branchLength = length; }
    void setMaxIterations(int iterations) { maxIterations = iterations; }
    void setSphereRadius(double radius) { sphereRadius = radius; }
    void setGoalSampleFreq(int freq) { goalSampleFreq = freq; }
    void setDynamicSampling(bool dynamic) { dynamicSampling = dynamic; }
    void setNeighborhoodRadius(double radius) { neighborhoodRadius = radius; }

    // Generate random obstacles
    void generateRandomObstacles(int numObstacles)
    {
        spheres.clear();

        std::uniform_real_distribution<double> xDist(boundary[0] + sphereRadius, boundary[1] - sphereRadius);
        std::uniform_real_distribution<double> yDist(boundary[2] + sphereRadius, boundary[3] - sphereRadius);
        std::uniform_real_distribution<double> zDist(boundary[4] + sphereRadius, boundary[5] - sphereRadius);

        // Create specified number of random obstacles
        for (int i = 0; i < numObstacles; i++)
        {
            Point3D sphere(xDist(rng), yDist(rng), zDist(rng));

            // Ensure obstacles don't overlap with start or goal
            if (sphere.distanceTo(startPoint) > 2 * sphereRadius &&
                sphere.distanceTo(goalPoint) > 2 * sphereRadius)
            {
                sphere.color = getRandomColor();
                spheres.push_back(sphere);
            }
            else
            {
                i--; // Try again
            }
        }
    }

    // Set obstacles manually
    void setObstacles(const std::vector<Point3D> &obstacles)
    {
        spheres = obstacles;
    }

    // Check if a point collides with any obstacle
    bool isCollided(const Point3D &point) const
    {
        // Check if point is outside boundary
        if (point.x < boundary[0] || point.x > boundary[1] ||
            point.y < boundary[2] || point.y > boundary[3] ||
            point.z < boundary[4] || point.z > boundary[5])
        {
            return true;
        }

        // Check collision with obstacles
        for (const auto &sphere : spheres)
        {
            if (point.distanceTo(sphere) < sphereRadius)
            {
                return true;
            }
        }

        return false;
    }

    // Find nearest node index to a given point
    int findNearestNode(const Point3D &randPoint) const
    {
        int nearestIndex = -1;
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < nodes.size(); i++)
        {
            double dist = nodes[i].point.distanceTo(randPoint);
            if (dist < minDist)
            {
                minDist = dist;
                nearestIndex = i;
            }
        }

        return nearestIndex;
    }

    // Find all nodes within a neighborhood radius
    std::vector<int> findNeighborNodes(const Point3D &point, double radius) const
    {
        std::vector<int> neighbors;

        for (size_t i = 0; i < nodes.size(); i++)
        {
            double dist = nodes[i].point.distanceTo(point);
            if (dist < radius)
            {
                neighbors.push_back(i);
            }
        }

        return neighbors;
    }

    // Calculate path cost from start to a given node
    double getCost(int nodeIndex) const
    {
        return nodes[nodeIndex].cost;
    }

    // Calculate cost of a potential new path
    double getNewPathCost(int fromIndex, const Point3D &to) const
    {
        return nodes[fromIndex].cost + nodes[fromIndex].point.distanceTo(to);
    }

    // Extend tree toward random point
    std::pair<bool, Point3D> extendToward(const Point3D &randPoint, const Point3D &nearestPoint)
    {
        // Get direction vector
        Point3D dir = randPoint - nearestPoint;
        double dist = dir.norm();

        // If distance is less than branch length, return random point
        if (dist <= branchLength)
        {
            // Check for collision
            if (!isCollided(randPoint))
            {
                return {true, randPoint};
            }
            return {false, Point3D()};
        }

        // Normalize direction and scale by branch length
        dir = dir.normalize() * branchLength;
        Point3D newPoint = nearestPoint + dir;

        // Check for collision
        if (!isCollided(newPoint))
        {
            return {true, newPoint};
        }

        return {false, Point3D()};
    }

    // Generate random point in configuration space
    Point3D generateRandomPoint()
    {
        std::uniform_real_distribution<double> xDist(boundary[0], boundary[1]);
        std::uniform_real_distribution<double> yDist(boundary[2], boundary[3]);
        std::uniform_real_distribution<double> zDist(boundary[4], boundary[5]);

        // Occasionally sample the goal point to bias search toward it
        std::uniform_int_distribution<int> goalDist(1, 100);

        // Dynamic sampling: adjust goal sampling frequency based on iterations
        int currentGoalFreq = goalSampleFreq;
        if (dynamicSampling)
        {
            double progress = static_cast<double>(nodes.size()) / maxIterations;
            currentGoalFreq = initialSampleFreq -
                              (initialSampleFreq - finalSampleFreq) * progress;
            currentGoalFreq = std::max(1, currentGoalFreq);
        }

        if (goalDist(rng) <= currentGoalFreq)
        {
            return goalPoint;
        }

        return Point3D(xDist(rng), yDist(rng), zDist(rng));
    }

    // Initialize visualization components
    void initVisualization()
    {
        // Create a renderer, render window, and interactor
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);
        interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(renderWindow);

        // Set interactor style
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        interactor->SetInteractorStyle(style);

        // Set background color
        renderer->SetBackground(0.1, 0.1, 0.1);

        // Set up camera
        renderer->ResetCamera();
        vtkCamera *camera = renderer->GetActiveCamera();
        camera->SetPosition(0, 0, 100);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        // Add bounding box for reference
        addBoundingBox();

        // Add start and goal points
        addSphere(startPoint, 1.0, 0.0, 1.0, 0.0); // Green for start
        addSphere(goalPoint, 1.0, 1.0, 0.0, 0.0);  // Red for goal
    }

    // Add a sphere to the visualization
    void addSphere(const Point3D &center, double radius, double r, double g, double b)
    {
        if (!visualize)
            return;

        // Create a sphere
        vtkSmartPointer<vtkSphereSource> sphereSource =
            vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(center.x, center.y, center.z);
        sphereSource->SetRadius(radius);
        sphereSource->SetPhiResolution(20);
        sphereSource->SetThetaResolution(20);

        // Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphereSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);

        // Add the actor to the scene
        renderer->AddActor(actor);
        renderWindow->Render();
    }

    // Add a line to the visualization
    void addLine(const Point3D &p1, const Point3D &p2, double r, double g, double b, double width = 1.0)
    {
        if (!visualize)
            return;

        // Create a line
        vtkSmartPointer<vtkLineSource> lineSource =
            vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(p1.x, p1.y, p1.z);
        lineSource->SetPoint2(p2.x, p2.y, p2.z);

        // Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);

        // Add the actor to the scene
        renderer->AddActor(actor);
        renderWindow->Render();
    }

    // Add bounding box to visualize the configuration space
    void addBoundingBox()
    {
        if (!visualize)
            return;

        double xmin = boundary[0], xmax = boundary[1];
        double ymin = boundary[2], ymax = boundary[3];
        double zmin = boundary[4], zmax = boundary[5];

        // Create cube source for the bounding box
        vtkSmartPointer<vtkCubeSource> cubeSource =
            vtkSmartPointer<vtkCubeSource>::New();
        cubeSource->SetBounds(xmin, xmax, ymin, ymax, zmin, zmax);

        // Create mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(cubeSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
        actor->GetProperty()->SetOpacity(0.1);
        actor->GetProperty()->SetRepresentationToWireframe();

        // Add the actor to the scene
        renderer->AddActor(actor);
    }

    // Update visualization with current RRT* tree
    void updateVisualization()
    {
        if (!visualize)
            return;

        // Clear previous visualization
        renderer->RemoveAllViewProps();

        // Add bounding box
        addBoundingBox();

        // Add obstacles
        for (const auto &sphere : spheres)
        {
            addSphere(sphere, sphereRadius, sphere.color.r, sphere.color.g, sphere.color.b);
        }

        // Add start and goal points
        addSphere(startPoint, 1.0, 0.0, 1.0, 0.0); // Green for start
        addSphere(goalPoint, 1.0, 1.0, 0.0, 0.0);  // Red for goal

        // Add tree edges
        for (const auto &edge : edges)
        {
            addLine(nodes[edge.from].point, nodes[edge.to].point, 0.7, 0.7, 0.7);
        }

        // Render the scene
        renderWindow->Render();
    }

    // Generate RRT* tree
    std::vector<Point3D> generateRRTStar()
    {
        nodes.clear();
        edges.clear();

        // Add start point as the root node
        nodes.push_back(Node(startPoint, -1, 0.0));

        // Main RRT* loop
        int iteration = 0;
        int goalNodeIndex = -1;

        while (iteration < maxIterations)
        {
            // Generate random point
            Point3D randPoint = generateRandomPoint();

            // Find nearest node
            int nearestIndex = findNearestNode(randPoint);
            if (nearestIndex == -1)
                continue;

            // Extend toward random point
            auto [success, newPoint] = extendToward(randPoint, nodes[nearestIndex].point);

            if (success)
            {
                // Find all nodes within neighborhood radius
                double searchRadius = std::min(neighborhoodRadius,
                                               std::pow(log(nodes.size() + 1) / (nodes.size() + 1), 1.0 / 3.0) *
                                                   std::sqrt(2 * (boundary[1] - boundary[0]) * (boundary[3] - boundary[2]) * (boundary[5] - boundary[4])));

                std::vector<int> neighbors = findNeighborNodes(newPoint, searchRadius);

                // Find the node that would provide the minimal cost path to new point
                int minCostIndex = nearestIndex;
                double minCost = getNewPathCost(nearestIndex, newPoint);

                for (int neighborIdx : neighbors)
                {
                    double newCost = getNewPathCost(neighborIdx, newPoint);
                    if (newCost < minCost && !isCollided(newPoint))
                    {
                        minCost = newCost;
                        minCostIndex = neighborIdx;
                    }
                }

                // Add new node and edge
                int newNodeIndex = nodes.size();
                nodes.push_back(Node(newPoint, minCostIndex, minCost));
                edges.push_back(Edge(minCostIndex, newNodeIndex));

                // Rewire the tree
                for (int neighborIdx : neighbors)
                {
                    if (neighborIdx != minCostIndex)
                    {
                        double costThroughNew = minCost + newPoint.distanceTo(nodes[neighborIdx].point);

                        if (costThroughNew < nodes[neighborIdx].cost && !isCollided(newPoint))
                        {
                            // Update parent and cost
                            int oldParent = nodes[neighborIdx].parent;
                            nodes[neighborIdx].parent = newNodeIndex;
                            nodes[neighborIdx].cost = costThroughNew;

                            // Update edge
                            for (size_t i = 0; i < edges.size(); i++)
                            {
                                if (edges[i].from == oldParent && edges[i].to == neighborIdx)
                                {
                                    edges[i].from = newNodeIndex;
                                    break;
                                }
                            }
                        }
                    }
                }

                // Check if we've reached the goal
                if (newPoint.distanceTo(goalPoint) < branchLength)
                {
                    goalNodeIndex = newNodeIndex;

                    // Add goal node with direct connection
                    int finalNodeIndex = nodes.size();
                    double finalCost = nodes[newNodeIndex].cost + newPoint.distanceTo(goalPoint);
                    nodes.push_back(Node(goalPoint, newNodeIndex, finalCost));
                    edges.push_back(Edge(newNodeIndex, finalNodeIndex));

                    // Continue to optimize after finding a path
                    // In RRT*, we don't break immediately upon finding a path
                }
            }

            // // Update visualization occasionally
            // if (visualize && iteration % 50 == 0)
            // {
            //     updateVisualization();
            // }

            iteration++;
        }

        // Update visualization one last time
        if (visualize)
        {
            updateVisualization();
        }

        // Find the best path to goal if one exists
        std::vector<Point3D> path;
        if (goalNodeIndex != -1)
        {
            int currentIdx = -1;

            // Find the node closest to the goal
            double minDistToGoal = std::numeric_limits<double>::max();
            for (size_t i = 0; i < nodes.size(); i++)
            {
                double dist = nodes[i].point.distanceTo(goalPoint);
                if (dist < minDistToGoal)
                {
                    minDistToGoal = dist;
                    currentIdx = i;
                }
            }

            // Trace path back to start
            while (currentIdx != -1)
            {
                path.push_back(nodes[currentIdx].point);
                currentIdx = nodes[currentIdx].parent;
            }

            // Reverse path to get start-to-goal order
            std::reverse(path.begin(), path.end());

            // Visualize the final path
            if (visualize)
            {
                for (size_t i = 0; i < path.size() - 1; i++)
                {
                    addLine(path[i], path[i + 1], 1.0, 0.8, 0.0, 3.0); // Thick yellow line for the path
                }
                renderWindow->Render();
            }
        }

        return path;
    }

    // Find the optimal path from start to goal
    std::vector<Point3D> findPath()
    {
        auto path = generateRRTStar();

        if (path.empty())
        {
            std::cout << "No path found!" << std::endl;
        }
        else
        {
            std::cout << "Path found with " << path.size() << " waypoints!" << std::endl;

            // Calculate total path length
            double pathLength = 0.0;
            for (size_t i = 0; i < path.size() - 1; i++)
            {
                pathLength += path[i].distanceTo(path[i + 1]);
            }
            std::cout << "Path length: " << pathLength << std::endl;
        }

        if (visualize)
        {
            startInteractor();
        }

        return path;
    }

    // Start the visualization interactor
    void startInteractor()
    {
        if (visualize)
        {
            interactor->Initialize();
            interactor->Start();
        }
    }
};

// Animation callback for rotating the camera
class vtkAnimationCallback : public vtkCommand
{
public:
    static vtkAnimationCallback *New()
    {
        return new vtkAnimationCallback;
    }

    vtkAnimationCallback() : Camera(nullptr), Renderer(nullptr), Angle(0) {}

    void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData)) override
    {
        if (eventId != vtkCommand::TimerEvent || !Camera || !Renderer)
            return;

        // Increment the angle
        Angle = (Angle + 1) % 360;

        // Position camera at a fixed distance from the focal point
        double distance = 100.0;
        double angle = Angle * vtkMath::Pi() / 180.0;
        double camX = distance * cos(angle);
        double camY = distance * sin(angle);

        Camera->SetPosition(camX, camY, 50.0);
        Camera->SetViewUp(0, 0, 1);

        // Render the scene
        Renderer->ResetCameraClippingRange();
        Renderer->GetRenderWindow()->Render();
    }

    void SetCamera(vtkCamera *camera) { Camera = camera; }
    void SetRenderer(vtkRenderer *renderer) { Renderer = renderer; }

private:
    vtkCamera *Camera;
    vtkRenderer *Renderer;
    int Angle;
};

void startRotationAnimation(vtkSmartPointer<vtkRenderer> renderer,
                            vtkSmartPointer<vtkRenderWindowInteractor> interactor)
{
    // Create the animation callback
    vtkSmartPointer<vtkAnimationCallback> animationCallback =
        vtkSmartPointer<vtkAnimationCallback>::New();
    animationCallback->SetCamera(renderer->GetActiveCamera());
    animationCallback->SetRenderer(renderer);

    // Create a timer for animation
    interactor->Initialize();
    interactor->CreateRepeatingTimer(30); // 30ms timer (approximately 33 fps)
    interactor->AddObserver(vtkCommand::TimerEvent, animationCallback);

    // Start the interaction
    interactor->Start();
}

int main(int argc, char *argv[])
{
    // Parse command line arguments (simplified)
    bool visualize = true;
    int goalFreq = 5;
    int obstacles = 50;
    bool dynamicSampling = false;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--no-vis")
        {
            visualize = false;
        }
        else if (arg == "--goal-freq" && i + 1 < argc)
        {
            goalFreq = std::stoi(argv[++i]);
        }
        else if (arg == "--obstacles" && i + 1 < argc)
        {
            obstacles = std::stoi(argv[++i]);
        }
        else if (arg == "--dynamic-sampling")
        {
            dynamicSampling = true;
        }
    }

    // Create and configure RRT* planner
    RRTStar3D rrtstar;
    rrtstar.setVisualization(visualize);
    rrtstar.setGoalSampleFreq(goalFreq);
    rrtstar.setDynamicSampling(dynamicSampling);
    rrtstar.setNeighborhoodRadius(10.0); // Set radius for neighbor search in RRT*

    // Set start and goal points
    rrtstar.setStartPoint(Point3D(-15, -15, -15));
    rrtstar.setGoalPoint(Point3D(15, 15, 15));

    // Generate obstacles
    rrtstar.generateRandomObstacles(obstacles);

    // Find path
    std::cout << "Running RRT* algorithm..." << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();

    auto path = rrtstar.findPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "Computation time: " << duration.count() << " ms" << std::endl;

    return 0;
}