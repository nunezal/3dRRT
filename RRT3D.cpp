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

// Structure to represent a 3D point with color
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

// Structure to represent an edge in the RRT tree
struct Edge
{
    int from, to;
    Edge(int from = 0, int to = 0) : from(from), to(to) {}
};

// List of predefined colors
std::vector<Color> obstacleColors = {
    Color(0.3, 0.3, 0.8), // Blue
    Color(0.8, 0.3, 0.3), // Red
    Color(0.3, 0.8, 0.3), // Green
    Color(0.8, 0.8, 0.3), // Yellow
    Color(0.8, 0.3, 0.8), // Purple
    Color(0.3, 0.8, 0.8), // Cyan
    Color(0.6, 0.4, 0.2), // Brown
    Color(0.5, 0.5, 0.5), // Gray
};

// Class for RRT algorithm
class RRT3D
{
private:
    // Boundary [xmin, xmax, ymin, ymax, zmin, zmax]
    double boundary[6] = {0, 40, 0, 40, 0, 40};

    double branchLength;
    int maxIterations;
    std::vector<Point3D> spheres;
    double sphereRadius;

    // Start and goal
    Point3D startPoint;
    Point3D goalPoint;
    std::vector<Point3D> vertices;
    std::vector<Edge> edges;
    int goalSampleFreq;
    bool dynamicSampling;
    int initialSampleFreq;
    int finalSampleFreq;

    // Visualization
    bool visualize;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;

    // Random number generator
    std::mt19937 rng;

    // Get a random color from the list
    Color getRandomColor()
    {
        std::uniform_int_distribution<int> colorDist(0, obstacleColors.size() - 1);
        return obstacleColors[colorDist(rng)];
    }

public:
    RRT3D() : branchLength(1.0),
              maxIterations(1500),
              sphereRadius(1.5),
              goalSampleFreq(5),
              dynamicSampling(false),
              initialSampleFreq(10),
              finalSampleFreq(2),
              visualize(true)
    {
        // Initialize random number generator with time-based seed
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng = std::mt19937(seed);
    }

    // Setup methods
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

    // Generate random spherical obstacles
    void generateRandomObstacles(int numObstacles)
    {
        spheres.clear();

        // Set random seed based on current time
        rng.seed(std::chrono::system_clock::now().time_since_epoch().count());

        std::uniform_real_distribution<double> xDist(boundary[0] + 5, boundary[1] - 5);
        std::uniform_real_distribution<double> yDist(boundary[2] + 5, boundary[3] - 5);
        std::uniform_real_distribution<double> zDist(boundary[4] + 5, boundary[5] - 5);

        for (int i = 0; i < numObstacles; i++)
        {
            Point3D sphere(xDist(rng), yDist(rng), zDist(rng));

            // Assign a random color to the obstacle
            sphere.color = getRandomColor();

            // Ensure obstacles are not too close to start or goal
            if (sphere.distanceTo(startPoint) > 5 && sphere.distanceTo(goalPoint) > 5)
            {
                spheres.push_back(sphere);
            }
        }
    }

    // Set obstacles manually
    void setObstacles(const std::vector<Point3D> &obstacles)
    {
        spheres = obstacles;
    }

    // Check if a point collides with any sphere
    bool isCollided(const Point3D &point) const
    {
        for (const auto &sphere : spheres)
        {
            if (point.distanceTo(sphere) <= sphereRadius)
            {
                return true;
            }
        }
        return false;
    }

    // Find the nearest node in the tree to a random point
    int findNearestNode(const Point3D &randPoint) const
    {
        int nearest = 0;
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < vertices.size(); i++)
        {
            double dist = randPoint.distanceTo(vertices[i]);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = i;
            }
        }

        return nearest;
    }

    // Check if the tree can extend and return the new node
    std::pair<bool, Point3D> canItExtend(const Point3D &randPoint, const Point3D &nearestPoint)
    {
        // Calculate direction vector
        Point3D dir = (randPoint - nearestPoint).normalize();
        // Calculate potential new node
        Point3D potentialNode = nearestPoint + (dir * branchLength);

        // Track the last valid point (start with the new potential point)
        Point3D lastValidPoint = potentialNode;

        // Interpolate path and check for collision at waypoints
        int interpCount = 5; // Number of interpolation points

        for (int i = 0; i <= interpCount; i++)
        {
            double t = static_cast<double>(i) / interpCount;
            Point3D interpPoint = nearestPoint + (dir * branchLength * t);

            if (isCollided(interpPoint))
            {
                // If we're at the first point, we can't extend at all
                if (i == 0)
                {
                    return std::make_pair(false, nearestPoint);
                }

                // Otherwise, use the last valid point
                return std::make_pair(true, lastValidPoint);
            }

            // Update last valid point
            lastValidPoint = interpPoint;
        }

        // No collision detected, can extend to the full branch length
        return std::make_pair(true, potentialNode);
    }

    // Generate random point in the workspace
    Point3D generateRandomPoint()
    {
        std::uniform_real_distribution<double> xDist(boundary[0], boundary[1]);
        std::uniform_real_distribution<double> yDist(boundary[2], boundary[3]);
        std::uniform_real_distribution<double> zDist(boundary[4], boundary[5]);

        return Point3D(xDist(rng), yDist(rng), zDist(rng));
    }

    // Initialize visualization
    void initVisualization()
    {
        if (!visualize)
            return;

        std::cout << "Initializing visualization..." << std::endl;

        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->AddRenderer(renderer);
        renderWindow->SetSize(1200, 900);
        renderWindow->SetWindowName("3D RRT Path Planning - Interactive Visualization");

        interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(renderWindow);

        // Set trackball interaction style for easy rotation
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        interactor->SetInteractorStyle(style);

        // Setup the scene with a gradient background
        renderer->SetBackground(0.1, 0.2, 0.3);  // Dark blue background
        renderer->SetBackground2(0.2, 0.3, 0.4); // Lighter blue for gradient
        renderer->SetGradientBackground(true);

        // Add bounding box
        addBoundingBox();

        // Add obstacles with their assigned colors
        for (const auto &sphere : spheres)
        {
            addSphere(sphere, sphereRadius, sphere.color.r, sphere.color.g, sphere.color.b);
        }

        // Add start and goal points with larger size and brighter colors
        addSphere(startPoint, 1.0, 1.0, 0.0, 0.0); // Red start
        addSphere(goalPoint, 1.0, 0.0, 1.0, 0.0);  // Green goal

        // Setup camera
        vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
        camera->SetPosition(boundary[1] * 1.5, boundary[3] * 1.5, boundary[5] * 1.5);
        camera->SetFocalPoint((boundary[0] + boundary[1]) / 2,
                              (boundary[2] + boundary[3]) / 2,
                              (boundary[4] + boundary[5]) / 2);
        camera->SetViewUp(0, 0, 1);

        // Initialize interactor
        interactor->Initialize();
        renderWindow->Render();

        std::cout << "Visualization window should be visible now" << std::endl;
    }

    // Add a sphere to the visualization
    void addSphere(const Point3D &center, double radius, double r, double g, double b)
    {
        if (!visualize)
            return;

        vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(center.x, center.y, center.z);
        sphereSource->SetRadius(radius);
        sphereSource->SetPhiResolution(20); // Higher resolution for smoother spheres
        sphereSource->SetThetaResolution(20);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphereSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // Set color and material properties
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetAmbient(0.2);        // Ambient light reflection
        actor->GetProperty()->SetDiffuse(0.7);        // Diffuse light reflection
        actor->GetProperty()->SetSpecular(0.5);       // Specular light reflection
        actor->GetProperty()->SetSpecularPower(20.0); // Specular power

        // Set opacity - lower for obstacles, full for start/goal points
        // Check if this is a start or goal point (which should be fully opaque)
        bool isStartOrGoal = (center == startPoint || center == goalPoint);
        if (!isStartOrGoal)
        {
            actor->GetProperty()->SetOpacity(0.4); // Semi-transparent obstacles
        }
        else
        {
            actor->GetProperty()->SetOpacity(1.0); // Fully opaque start/goal
        }

        renderer->AddActor(actor);
    }

    // Add a line to the visualization
    void addLine(const Point3D &p1, const Point3D &p2, double r, double g, double b, double width = 1.0)
    {
        if (!visualize)
            return;

        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(p1.x, p1.y, p1.z);
        lineSource->SetPoint2(p2.x, p2.y, p2.z);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetProperty()->SetLineWidth(width);

        // Make lines more visible with lighting effects
        actor->GetProperty()->SetAmbient(0.6); // Higher ambient for better visibility
        actor->GetProperty()->SetDiffuse(0.8);

        // Check if this is a path line (usually colored differently than tree branches)
        bool isPathLine = (r > 0.5 && g < 0.5 && b < 0.5); // Red lines are typically path lines
        if (isPathLine)
        {
            actor->GetProperty()->SetLineWidth(width * 2.0); // Make path lines thicker
            actor->GetProperty()->SetOpacity(1.0);           // Fully opaque
        }
        else
        {
            actor->GetProperty()->SetOpacity(0.8); // Slightly transparent for tree branches
        }

        renderer->AddActor(actor);
    }

    // Add a bounding box to the visualization
    void addBoundingBox()
    {
        if (!visualize)
            return;

        double xmin = boundary[0], xmax = boundary[1];
        double ymin = boundary[2], ymax = boundary[3];
        double zmin = boundary[4], zmax = boundary[5];

        // Create wireframe box
        addLine(Point3D(xmin, ymin, zmin), Point3D(xmax, ymin, zmin), 0.5, 0.5, 0.5);
        addLine(Point3D(xmin, ymax, zmin), Point3D(xmax, ymax, zmin), 0.5, 0.5, 0.5);
        addLine(Point3D(xmin, ymin, zmax), Point3D(xmax, ymin, zmax), 0.5, 0.5, 0.5);
        addLine(Point3D(xmin, ymax, zmax), Point3D(xmax, ymax, zmax), 0.5, 0.5, 0.5);

        addLine(Point3D(xmin, ymin, zmin), Point3D(xmin, ymax, zmin), 0.5, 0.5, 0.5);
        addLine(Point3D(xmax, ymin, zmin), Point3D(xmax, ymax, zmin), 0.5, 0.5, 0.5);
        addLine(Point3D(xmin, ymin, zmax), Point3D(xmin, ymax, zmax), 0.5, 0.5, 0.5);
        addLine(Point3D(xmax, ymin, zmax), Point3D(xmax, ymax, zmax), 0.5, 0.5, 0.5);

        addLine(Point3D(xmin, ymin, zmin), Point3D(xmin, ymin, zmax), 0.5, 0.5, 0.5);
        addLine(Point3D(xmax, ymin, zmin), Point3D(xmax, ymin, zmax), 0.5, 0.5, 0.5);
        addLine(Point3D(xmin, ymax, zmin), Point3D(xmin, ymax, zmax), 0.5, 0.5, 0.5);
        addLine(Point3D(xmax, ymax, zmin), Point3D(xmax, ymax, zmax), 0.5, 0.5, 0.5);
    }

    // Update visualization
    void updateVisualization()
    {
        if (!visualize)
            return;

        // Make sure the window is visible
        renderWindow->SetWindowName("3D RRT Path Planning - In Progress");
        renderWindow->Render();

        // Process events to keep UI responsive
        interactor->ProcessEvents();

        // Small delay to allow visualization to be seen
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Generate the RRT path
    std::vector<Point3D> generateRRT()
    {
        // Initialize visualization
        if (visualize)
        {
            initVisualization();
        }

        // Clear previous data
        vertices.clear();
        edges.clear();

        // Add start point as the first vertex
        vertices.push_back(startPoint);

        // Main RRT loop
        for (int i = 0; i < maxIterations; i++)
        {
            // Generate random point (with goal biasing)
            Point3D randPoint;

            // Dynamic goal sampling frequency
            int currentGoalFreq = goalSampleFreq;
            if (dynamicSampling)
            {
                // Linearly interpolate between initial and final frequencies
                double progress = static_cast<double>(i) / maxIterations;
                currentGoalFreq = initialSampleFreq + static_cast<int>((finalSampleFreq - initialSampleFreq) * progress);
                currentGoalFreq = std::max(1, currentGoalFreq); // Ensure it's at least 1
            }

            // Goal biasing
            if (i % currentGoalFreq == 0)
            {
                randPoint = goalPoint;
            }
            else
            {
                randPoint = generateRandomPoint();
            }

            // Find nearest node in the tree
            int nearestIndex = findNearestNode(randPoint);
            Point3D nearestPoint = vertices[nearestIndex];

            // Try to extend the tree
            auto [canExtend, newNode] = canItExtend(randPoint, nearestPoint);

            if (canExtend)
            {
                // Add new node and edge
                vertices.push_back(newNode);
                edges.push_back(Edge(nearestIndex, vertices.size() - 1));

                // Visualize the new edge with color based on depth
                if (visualize)
                {
                    // Calculate depth (distance from root) to determine color
                    double depth = static_cast<double>(edges.size()) / maxIterations;

                    // Gradient from blue (shallow) to green (deep)
                    double r = 0.0;
                    double g = 0.5 + depth * 0.5; // Increase green with depth
                    double b = 0.8 - depth * 0.6; // Decrease blue with depth

                    addLine(nearestPoint, newNode, r, g, b, 1.5);
                }

                // Check if reached goal
                if (newNode == goalPoint || newNode.distanceTo(goalPoint) < 1.0)
                {
                    std::cout << "Goal reached at iteration " << i << "!" << std::endl;
                    // Add a direct connection to the goal for visualization
                    if (newNode.distanceTo(goalPoint) < 1.0 && !(newNode == goalPoint))
                    {
                        vertices.push_back(goalPoint);
                        edges.push_back(Edge(vertices.size() - 2, vertices.size() - 1));
                        if (visualize)
                        {
                            addLine(newNode, goalPoint, 1.0, 0.0, 0.0, 2.0); // Red final connection
                        }
                    }
                    break;
                }

                // Update visualization more frequently
                if (i % 5 == 0)
                {
                    updateVisualization();
                }
            }
        }

        // Find the path
        return findPath();
    }

    // Find the path from the tree
    std::vector<Point3D> findPath()
    {
        std::vector<Point3D> path;
        bool goalFound = false;
        int goalIndex = -1;

        // Check if the goal was reached or if we got close enough
        for (size_t i = 0; i < vertices.size(); i++)
        {
            if (vertices[i] == goalPoint || vertices[i].distanceTo(goalPoint) < 1.0)
            {
                // Goal was reached or we got close enough
                goalFound = true;
                goalIndex = i;
                break;
            }
        }

        if (goalFound)
        {
            // Trace back from goal to start
            int currentIndex = goalIndex;

            while (currentIndex != 0)
            {
                path.push_back(vertices[currentIndex]);

                // Find the parent node
                for (const auto &edge : edges)
                {
                    if (edge.to == currentIndex)
                    {
                        currentIndex = edge.from;
                        break;
                    }
                }
            }

            // Add start point
            path.push_back(startPoint);

            // Reverse to get path from start to goal
            std::reverse(path.begin(), path.end());

            std::cout << "Path found with " << path.size() << " waypoints!" << std::endl;

            // Visualize the final path with a distinct color
            if (visualize && !path.empty())
            {
                for (size_t i = 0; i < path.size() - 1; i++)
                {
                    // Use a bright red color with increased width for the final path
                    addLine(path[i], path[i + 1], 1.0, 0.0, 0.0, 3.0);
                }

                // Add small spheres at each waypoint for better visibility
                for (size_t i = 1; i < path.size() - 1; i++) // Skip start and goal which already have spheres
                {
                    // Use orange color for waypoints
                    addSphere(path[i], 0.3, 1.0, 0.5, 0.0);
                }

                updateVisualization();
                std::cout << "Path successfully found!" << std::endl;
            }

            return path;
        }

        std::cout << "Path did not reach goal, try increasing iterations" << std::endl;
        return path;
    }

    // Start the interactor for final visualization
    void startInteractor()
    {
        if (visualize)
        {
            std::cout << "Starting interactive view - click and drag to rotate" << std::endl;
            std::cout << "Press 'e' to exit the application" << std::endl;
            interactor->Start();
        }
    }
};

// Callback for rotating animation
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
        if (vtkCommand::TimerEvent == eventId)
        {
            Angle = (Angle + 1) % 360;

            if (Camera && Renderer)
            {
                double elevation = 10.0;
                Camera->SetPosition(
                    150.0 * cos(vtkMath::RadiansFromDegrees(static_cast<double>(Angle))),
                    150.0 * sin(vtkMath::RadiansFromDegrees(static_cast<double>(Angle))),
                    50.0);
                Camera->SetViewUp(0, 0, 1);

                Renderer->ResetCameraClippingRange();
                Renderer->GetRenderWindow()->Render();
            }
        }
    }

    void SetCamera(vtkCamera *camera) { Camera = camera; }
    void SetRenderer(vtkRenderer *renderer) { Renderer = renderer; }

private:
    vtkCamera *Camera;
    vtkRenderer *Renderer;
    int Angle;
};

// Helper function to start the rotation animation
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

    // Create RRT planner
    RRT3D rrt;

    // Set configuration
    rrt.setStartPoint(Point3D(3, 5, 10));
    rrt.setGoalPoint(Point3D(35, 35, 35));
    rrt.setBranchLength(1.0);
    rrt.setSphereRadius(1.5);
    rrt.setMaxIterations(5000);
    rrt.setGoalSampleFreq(goalFreq);
    rrt.setDynamicSampling(dynamicSampling);
    rrt.setVisualization(visualize);

    // Generate random obstacles
    rrt.generateRandomObstacles(obstacles);

    // Print configuration
    std::cout << "Configuration:" << std::endl;
    std::cout << "- Number of obstacles: " << obstacles << std::endl;
    std::cout << "- Maximum iterations: 5000" << std::endl;
    if (dynamicSampling)
    {
        std::cout << "- Dynamic goal sampling: Enabled (starts at 10, decreases to 2)" << std::endl;
    }
    else
    {
        std::cout << "- Goal sampling frequency: Every " << goalFreq << " iterations" << std::endl;
    }
    std::cout << "- Real-time visualization: " << (visualize ? "Enabled" : "Disabled") << std::endl;
    std::cout << "- Interactive mode: Enabled (you can rotate the plot during execution)" << std::endl;

    // Generate RRT path
    std::vector<Point3D> path = rrt.generateRRT();

    if (!path.empty())
    {
        std::cout << "Path successfully found!" << std::endl;

        // Start the interactive visualization or animate the view
        rrt.startInteractor();
    }
    else
    {
        std::cout << "Failed to find a path. Try increasing iterations or reducing obstacles." << std::endl;
    }

    return 0;
}