#include <SFML/Graphics.hpp>
#include <iostream>
#include <unordered_map>
#include <stdio.h>
#include <math.h>
#include "stdc++.h"


//
// Created by Arnab Datta on 14/03/22.Game
//


using namespace std;

class Position{
public:
    sf::Vector2f value;
};

class Velocity{
public:
    sf::Vector2f value;
    float length();

    void normalize();

};

class LinearAcceleration
{
public:
    sf::Vector2f value;
    float mag=sqrt(value.x*value.x + value.y*value.y);
    void normalize();

    float length();

};

class SteeringOutput
{
public:
    LinearAcceleration linear;
    float angular;
};

class Kinematic
{
public:
    Position position;
    Velocity velocity;
    float rotation;
    float orientation=0;
    float maxSpeed=10000;
    float maxRotation=2;

    void update(SteeringOutput steering, float time);
};

class SteeringBehaviour
{
public:
    Kinematic Character;
    Kinematic target;
    virtual SteeringOutput getSteering()=0;

};

class VelocityMatch : public SteeringBehaviour
{
public:
    Kinematic character;
    Kinematic target;

    float maxAcceleration=1000000000000;

    //The time over which to achieve target speed
    float timeToTarget=0.01;

    SteeringOutput getSteering();
};

class RotationMatch : public SteeringBehaviour
{
public:
    Kinematic character;
    Kinematic target;

    float maxRotation=1000000000000;

    //The time over which to achieve target speed
    float timeToTarget=0.01;

    SteeringOutput getSteering();
};

class Arrive : public SteeringBehaviour
{
public:
    Kinematic character;
    Kinematic target;

    float targetSpeed;
    Velocity targetVelocity;
    float maxAcceleration;
    float maxSpeed;

    //The radius of satisfaction
    float radiusSat;

    //The radius of deceleration
    float radiusDecel;

    //The time over which to achieve target speed
    float timeToTarget=0.25;

    //Arrive(Kinematic character, Kinematic target, float targetSpeed, Velocity targetVelocity, float maxAcceleration, float maxSpeed, float radiusSat, float radiusDecel);
    SteeringOutput getSteering();
};

class Align : public SteeringBehaviour
{
public:
    Kinematic character;
    Kinematic target;

    float targetRotation;

    float maxAngularAcceleration;
    float maxRotation;

    //The radius for arriving at target
    float radiusSat;

    //The radius for beginning to slow down
    float radiusDecel;

    //The time over which to achieve target speed
    float timeToTarget = 0.25;

    SteeringOutput getSteering();
};

class Wander
{
public:
    Kinematic character;
    Kinematic target;
    float maxRotation;

    float circleOffset;
    float circleRadius;

    float targetOrientation;



    sf::Vector2f getWanderLocation(float r);
};

class Separation
{
public:
    Kinematic character;
    float maxAcceleration;

    std::vector<Kinematic> targets; // a list of potential targets
    float radius;
    float inverseCoefficient;//coefficient of decay for inverse sqaure law

    SteeringOutput getSteering(int x, int y);



};

class FindCenterOfMass
{
public:
    sf::Vector2f findCenter(const std::vector<Kinematic> &targets);
};



    float Velocity::length()
    {
        float mag=sqrt(value.x*value.x + value.y*value.y);
        return mag;
    }
    void Velocity::normalize()
    {
        float mag=sqrt(value.x*value.x + value.y*value.y);
        value.x/=mag;
        value.y/=mag;
    }


    void LinearAcceleration::normalize()
    {
        value.x/=mag;
        value.y/=mag;
    }
    float LinearAcceleration::length()
    {
        return mag;
    }

    void Kinematic::update(SteeringOutput steering, float time)
    {
        //Update the position and orientation
//        std::cout<<"Velocity X is "<<velocity.value.x<<"\n";
        position.value+=velocity.value*time;
//        std::cout<<"New position is "<<position.value.x<<"\n";
        orientation+=rotation*time;

        // if(velocity.length()>0)
        // {
        //     orientation=atan2(velocity.value.y, velocity.value.x);
        // }

        //and the velocity and rotation
        velocity.value+=steering.linear.value*time;
        rotation+=steering.angular*time;

        // std::cout<<"\n";
        // std::cout<<"Sprite X location is "<<position.value.x<<"\n";
        // std::cout<<"Sprite Y location is "<<position.value.y<<"\n";
        // std::cout<<"Sprite X velocity is "<<velocity.value.x<<"\n";
        // std::cout<<"Sprite Y velocity is "<<velocity.value.y<<"\n";
        // std::cout<<"Sprite Linear Acceleration X is "<<steering.linear.value.x<<"\n";
        // std::cout<<"Sprite Linear Acceleration Y is "<<steering.linear.value.y<<"\n";
        // std::cout<<"Sprite Rotation is "<<rotation<<"\n";
        // std::cout<<"Sprite Orientation is "<<orientation<<"\n";
        // std::cout<<"Angular Acceleration is "<<steering.angular<<"\n";
        // std::cout<<"\n";

        if(rotation>100)
            rotation=100;

        if(velocity.length()>1000)
        {
            velocity.normalize();
            velocity.value.x*=1000;
            velocity.value.y*=1000;
        }

    }


    SteeringOutput VelocityMatch::getSteering()
    {
        SteeringOutput result;

        //Acceleration tries to get to the target velocity
        result.linear.value=target.velocity.value-character.velocity.value;
        result.linear.value/=timeToTarget;

        return result;
    }

    SteeringOutput RotationMatch::getSteering()
    {
        SteeringOutput result;

        //Acceleration tries to get to the target velocity
        result.angular=target.rotation-character.rotation;
        result.angular/=timeToTarget;

        return result;
    }

    SteeringOutput Arrive::getSteering()
    {
        SteeringOutput result;

        //Get the direction to the target
        sf::Vector2f direction = target.position.value-character.position.value;

        float distance = sqrt(direction.x*direction.x + direction.y*direction.y);


        //Check if we are there, return no steering
        if(distance<radiusSat)
        {
            targetSpeed=0;
        }

            //If we are outside the slowRadius, then move at max speed
        else if(distance>radiusDecel)
        {
            targetSpeed = maxSpeed;
        }
            //Otherwise calculate a scaled speed
        else
        {
            targetSpeed=maxSpeed*distance/radiusDecel;
        }

        //The target velocity combines speed and direction
        targetVelocity.value=direction;
        targetVelocity.normalize();
        targetVelocity.value*=targetSpeed;

        //Acceleration tries to get to the target velocity
        result.linear.value = targetVelocity.value-character.velocity.value;
        result.linear.value /= timeToTarget;


        //Check if acceleration is too fast
        if(result.linear.length()>maxAcceleration)
        {
            result.linear.normalize();
            result.linear.value*=maxAcceleration;
        }
        return result;
    }

//Arrive::Arrive(Kinematic character, Kinematic target, float targetSpeed, Velocity targetVelocity, float maxAcceleration,
//               float maxSpeed, float radiusSat, float radiusDecel) {
//    this->character=character;
//    this->target=target;
//    this->targetSpeed=targetSpeed;
//    this->targetVelocity=targetVelocity;
//    this->maxAcceleration=maxAcceleration;
//    this->maxSpeed=maxSpeed;
//    this->radiusSat;
//    this->radiusDecel=radiusDecel;
//}

SteeringOutput Align::getSteering()
    {
        SteeringOutput result;

        //Get the naive direction to the target
        // std::cout<<"Desired Orientation is "<<target.orientation<<"\n";

        float rotation = target.orientation-character.orientation;

        if(abs(rotation)>6.28)
            rotation=fmod(rotation,6.28);
        if(rotation>3.14)
            rotation-=6.28;
        if(rotation<-3.14)
            rotation+=6.28;

        float rotationSize = abs(rotation);

        //Check if we are there, return no steering.
        if(rotationSize<radiusSat)
        {
            // std::cout<<"Rotation less that radius of Satisfaction"<<"\n";
            targetRotation=0;
        }

            //If we are outside the slowRadius, then use maximum rotation
        else if(rotationSize>radiusDecel)
        {
            // std::cout<<"Rotation more than the Radius of Deceleration"<<"\n";
            targetRotation=maxRotation;
        }
            //Otherwise calculate a scaled rotation
        else
        {
            // std::cout<<"Rotation between radiusSat and radiusDecel"<<"\n";
            targetRotation=maxRotation*rotationSize/radiusDecel;
        }

        //The final target rotation combines speed (already in the variable) and direction
        targetRotation*=rotation/rotationSize;

        //Acceleration tries to get to the target rotation
        result.angular=targetRotation-character.rotation;
        result.angular/=timeToTarget;

        //Check if the acceleration is too great
        float angularAcceleration = abs(result.angular);
        if(angularAcceleration>maxAngularAcceleration)
        {
            result.angular/=angularAcceleration;
            result.angular*=maxAngularAcceleration;
        }

        result.linear.value.x=0;
        result.linear.value.y=0;
        return result;
    }

    sf::Vector2f Wander::getWanderLocation(float r)
    {
        targetOrientation=r*maxRotation;
        sf::Vector2f wanderLocation(circleOffset*cos(character.orientation) + circleRadius*cos(targetOrientation),circleOffset*sin(character.orientation) + circleRadius*sin(targetOrientation));

        return wanderLocation;
    }

    SteeringOutput Separation::getSteering(int x, int y)
    {

        SteeringOutput result;
        for(const Kinematic target : targets)
        {
            float x=target.position.value.x;
            float y=target.position.value.y;


            float separationAcceleration=0;
            float distance_x=character.position.value.x-x;
            float distance_y=character.position.value.y-y;
            float distance=sqrt(distance_x*distance_x+distance_y*distance_y);

            if(distance==0)
                continue;

            if(distance<radius)
            {
                separationAcceleration= maxAcceleration;
            }

            result.linear.value.x+=separationAcceleration*distance_x/distance;
            result.linear.value.y+=separationAcceleration*distance_y/distance;
        }

        return result;

    }

    sf::Vector2f FindCenterOfMass::findCenter(const std::vector<Kinematic> &targets)
    {
        sf::Vector2f center(0,0);
        for(Kinematic target : targets)
        {
//            std::cout<<"Sprite X and Y is "<<target.position.value.x <<" and "<<target.position.value.y<<"\n";
            center.x+=target.position.value.x;
            center.y+=target.position.value.y;
        }
        center.x/=2;
        center.y/=2;
//        std::cout<<"Center of mass X and Y is "<<center.x<<" and "<<center.y<<"\n";
        return center;

    }




// Program to find Dijkstra's shortest path using
// priority_queue in STL
using namespace std;
using namespace std::chrono;
# define INF 0x3f3f3f3f

// iPair ==> Integer Pair
typedef pair<int, double> iPair;

class Node
{
public:
    double x;
    double y;

public:
    Node(int x, int y);

};

// This class represents a directed graph using
// adjacency list representation
class Graph
{
public:
    int V; // No. of vertices

    // In a weighted graph, we need to store vertex
    // and weight pair for every edge
    list< pair<int, double> > *adj;
    std::unordered_map<int, Node > position;
    std::unordered_map<int, string> name;



public:
    Graph(int V); // Constructor

    // function to add an edge to graph
    void addEdge(int u, int v, double w);

    double euclidianHeuristic(int x1, int y1, int x2, int y2);

    double manhattanHeuristic(int x1, int y1, int x2, int y2);

    // prints shortest path from s
    void djikstra(int src, int goal);

    list<Node> astar(int src, int goal);

    pair<int, Node> closestPoint(int x, int y);
};



// Allocates memory for adjacency list

Node::Node(int x, int y) {
    this->x=x;
    this->y=y;
}

Graph::Graph(int V)
{
    this->V = V;
    adj = new list<iPair> [V];
}

void Graph::addEdge(int u, int v, double w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

double Graph::euclidianHeuristic(int x1, int y1, int x2, int y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

double Graph::manhattanHeuristic(int x1, int y1, int x2, int y2)
{
    return abs(x1-x2) + abs(y1-y2);
}

// Prints shortest paths from src to all other vertices
void Graph::djikstra(int src, int goal)
{
    // Get starting timepoint
    auto start = high_resolution_clock::now();

    int closedListFill=0;
    int openListFill=0;
    bool foundGoal=false;
    // Create a priority queue to store vertices that
    // are being preprocessed. This is weird syntax in C++.
    // Refer below link for details of this syntax
    // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
    priority_queue< iPair, vector <iPair> , greater<iPair> > pq;

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<double> dist(V, INF);
    vector<int> parent(V, INF);

    // Insert source itself in priority queue and initialize
    // its distance as 0.
    pq.push(make_pair(0, src));
    dist[src] = 0;
    parent[src] = -1;

    /* Looping till priority queue becomes empty (or all
    distances are not finalized) */
    while (!pq.empty())
    {
        // The first vertex in pair is the minimum distance
        // vertex, extract it from priority queue.
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = pq.top().second;
        pq.pop();
        closedListFill++;

        if(u==goal)
        {
            foundGoal= true;
            break;
        }
        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, double> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            double weight = (*i).second;

            // If there is shorted path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
                parent[v]=u;
                openListFill++;
            }
        }
    }
    // Get ending timepoint
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // Print shortest distances stored in dist[]
    printf("DJIKSTRA'S ALGORITHM Goal Distance from Source\n");
    printf("Closed List Fill is %d and Open List Fill is %d\n",closedListFill,openListFill);
    printf("Time taken is %d microseconds\n",duration.count());
    if(foundGoal) {
        printf("%d \t\t %f    ", goal, dist[goal]);
        printf("Came here from %d", parent[goal]);
        printf("  Path is %d <- ", goal);
        int current = goal;
        while (current != -1) {
            printf("%d <- ", parent[current]);
            current = parent[current];
        }
        printf("\n");
    }
    else
    {
        printf("No path to Goal\n");
    }


}

list<Node> Graph::astar(int src, int goal)
{
    auto start = high_resolution_clock::now();
    int closedListFill=0;
    int openListFill=0;
    bool foundGoal= false;
    list<Node> result;
    // Create a priority queue to store vertices that
    // are being preprocessed. This is weird syntax in C++.
    // Refer below link for details of this syntax
    // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
    priority_queue< iPair, vector <iPair> , greater<iPair> > pq;

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    vector<double> csf(V, INF);
    vector<double> etc(V, INF);
    vector<int> parent(V, INF);

    // Insert source itself in priority queue and initialize
    // its distance as 0.
    pq.push(make_pair(INF, src));
    csf[src] = 0;
    parent[src] = -1;

    /* Looping till priority queue becomes empty (or all
    distances are not finalized) */
    printf("Starting A*\n");
    while (!pq.empty())
    {
        // The first vertex in pair is the minimum distance
        // vertex, extract it from priority queue.
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = pq.top().second;
        pq.pop();
        closedListFill++;
        if(u==goal)
        {
            foundGoal= true;
            break;
        }
        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, double> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            double weight = (*i).second;
            double hVToGoal= euclidianHeuristic(position.at(v).x, position.at(v).y, position.at(goal).x, position.at(goal).y);

            // If there is shorted path to v through u.
            if (csf[v] > csf[u] + weight)
            {
                // Updating distance of v
                csf[v] = csf[u] + weight;
                etc[v]=csf[v]+hVToGoal;
                pq.push(make_pair(etc[v], v));
                openListFill++;
                parent[v]=u;
            }
        }
    }

    // Get ending timepoint
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    // Print shortest distances stored in dist[]
    printf("A-STAR ALGORITHM Goal Distance from Source\n");
    printf("Closed List Fill is %d and Open List Fill is %d\n",closedListFill,openListFill);
    printf("Time taken is %d microseconds\n",duration.count());
    if(foundGoal) {
        printf("%d \t\t %f    ", goal, csf[goal]);
        printf("Came here from %d", parent[goal]);
        printf("  Path is %d <- ", goal);
        int current = goal;
        result.push_back(position.at(goal));
        while (parent[current] != -1) {
            printf("%d <- ", parent[current]);
            current = parent[current];
            result.push_back(position.at(current));
        }
        printf("\n");
        list<Node>::iterator i;
        for(i=result.begin();i!=result.end();++i )
        {
            printf("X is %f and y is %f\n",(double)(*i).x, (double)(*i).y);
        }
    }
    else
    {
        printf("No path to goal\n");
    }

    result.reverse();
    return  result;

}

pair<int,Node> Graph::closestPoint(int x, int y) {
    double closest_x=0;
    double closest_y=0;
    int closest_index=0;
    double min_distance=10000000000;
    unordered_map<int, Node>::iterator i;
    for(i=position.begin();i!=position.end();++i)
    {
        printf("Searching map at index %d\n", (*i).first);
        double point_x=(*i).second.x;
        double point_y=(*i).second.y;
        double distance=sqrt((x-point_x)*(x-point_x) + (y-point_y)*(y-point_y));
        if(distance<min_distance)
        {
            min_distance=distance;
            closest_x=point_x;
            closest_y=point_y;
            closest_index=(*i).first;
        }
    }

    Node node(closest_x,closest_y);
    //printf("Closest point is %d with %f and %f", closest_index,node.x,node.y);

    return pair<int, Node> (closest_index,node);
}



//Code for Sprite Decision Tree
struct DTNode {
    string data;
    struct DTNode *left;
    struct DTNode *right;
    DTNode(string data)
    {
        this->data=data;
    }
};
struct DTNode *newDTNode(string data) {
    struct DTNode *newNode = new struct DTNode(data);
    std::cout << "Malloc complete"<<"\n";
    newNode->left = NULL;
    newNode->right = NULL;
    return (newNode);
}
DTNode createSpriteDecisionTree()
{
    struct DTNode *root = newDTNode("enteredNewRoom?");
    root->left = newDTNode("circleAroundObstacle");
    root->right = newDTNode("monsterInSameRoom?");
    root->right->left = newDTNode("evadeToNextRoom");
    root->right->right = newDTNode("circlingObstacleComplete?");
    root->right->right->left = newDTNode("hide");
    root->right->right->right = newDTNode("circleAroundObstacle");
    return *root;
}
DTNode traverseSpriteDecisionTree(struct DTNode *root, bool sameRoomAsMonster, bool circlingObstacleComplete, bool roomChanged)
{
    if(root->data=="enteredNewRoom?")
    {
        if(roomChanged)
        {
            return traverseSpriteDecisionTree(root->left,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
        else
        {
            return traverseSpriteDecisionTree(root->right,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
    }

    if(root->data=="monsterInSameRoom?")
    {
        if(sameRoomAsMonster)
        {
            return traverseSpriteDecisionTree(root->left,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
        else
        {
            return traverseSpriteDecisionTree(root->right,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
    }
    else if(root->data=="circlingObstacleComplete?")
    {
        if(circlingObstacleComplete)
        {
            return traverseSpriteDecisionTree(root->left,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
        else
        {
            return traverseSpriteDecisionTree(root->right,sameRoomAsMonster,circlingObstacleComplete, roomChanged);
        }
    }
    else
    {
        return *root;
    }
}
list<Node> cycleAroundObstacle1Path()
{
    list<Node> result;

    //Nodes around obstacle1
    Node node0(100,400);
    Node node1(300,400);
    Node node2(100,600);
    Node node3(300,600);

    result.push_back(node0);
    result.push_back(node1);
    result.push_back(node3);
    result.push_back(node2);

    return result;
}
list<Node> cycleAroundObstacle2Path()
{
    list<Node> result;

    //Nodes around obstacle2
    Node node4(500,400);
    Node node5(700,400);
    Node node6(500,600);
    Node node7(700,600);

    result.push_back(node4);
    result.push_back(node5);
    result.push_back(node7);
    result.push_back(node6);

    return result;
}
list<Node> cycleAroundObstacle3Path()
{
    list<Node> result;

    //Nodes around obstacle3
    Node node8(900,400);
    Node node9(1100,400);
    Node node10(900,600);
    Node node11(1100,600);

    result.push_back(node8);
    result.push_back(node9);
    result.push_back(node11);
    result.push_back(node10);

    return result;
}
int hideAtNode(int roomNumber)
{
    if(roomNumber==1)
    {
        return 18;
    }
    if(roomNumber==2)
    {
        return 22;
    }
    if(roomNumber==3)
    {
        return 26;
    }
    return 1;
}
int evadeToNode(int roomNumber)
{
    if(roomNumber==1)
    {
        return 0;
    }
    if(roomNumber==2)
    {
        return 4;
    }
    if(roomNumber==3)
    {
        return 8;
    }
    return 0;
}
list<Node> cycleAroundObstacle(int roomNumber)
{
    list<Node> result;
    if(roomNumber==1)
    {
        result=cycleAroundObstacle1Path();
    }
    if(roomNumber==2)
    {
        result=cycleAroundObstacle2Path();
    }
    if(roomNumber==3)
    {
        result=cycleAroundObstacle3Path();
    }
    return result;
}
int findNextRoom(int roomNumber)
{
    int nextRoomNumber = roomNumber+1;
    if(nextRoomNumber==4)
    {
        nextRoomNumber=1;
    }
    return nextRoomNumber;
}

//Code for Monster Behaviour Tree
struct EnvironmentState {
    bool foundSprite;
    bool sameRoomAsSprite;
    bool closeToSprite;
    bool moveToNextRoomComplete;
    bool allNodesInRoomChecked;
};
EnvironmentState *environmentState = new EnvironmentState{false, false, false, false, false};
class BTNode {  // This class represents each BTNode in the behaviour tree.
public:
    int state=0;
    string value="root";
    virtual int run() = 0;
};
deque<BTNode*> runningNodes;
class CompositeBTNode : public BTNode {  //  This type of BTNode follows the Composite Pattern, containing a list of other BTNodes.
private:
    std::list<BTNode*> children;
public:
    const std::list<BTNode*>& getChildren() const {return children;}
    void addChild (BTNode* child) {children.emplace_back(child);}
};
class Selector : public CompositeBTNode {
public:
    list<BTNode *> children;
    list<BTNode *>::iterator currentChild;
    int currentChildState;
    Selector()
    {
        value="Selector";
        state=0;
    }
    virtual int run() override {
        children=getChildren();
        std::cout << "Running Selector." << std::endl;  // will return true
        std::cout << "There are " <<children.size()<<" children of the selector."<< std::endl;  // will return true
        for (BTNode* currentChild:children)
        {
            std::cout << (currentChild) << std::endl;
            currentChildState=(currentChild)->run();
            if(currentChildState==1)
            {
                std::cout << "Child passed." << std::endl;  // will return true
                state=1;
                return state;
            }
            else if(currentChildState==0)
            {
                std::cout << "Child failed. Going to next child." << std::endl;  // will return true
            }
            else if(currentChildState==2)
            {
                std::cout << "Found a running child." << std::endl;  // will return true
                runningNodes.push_front(this);
                state=2;
                break;
            }
        }
        return state;
    }
};
class Sequence : public CompositeBTNode {
public:
    list<BTNode *> children;
    list<BTNode *>::iterator currentChild;
    int currentChildState;
    Sequence()
    {
        value="Sequence";
        state=1;
    }
    virtual int run() override {
        children=getChildren();
        std::cout << "Inside Sequence." << std::endl;  // will return true
        std::cout << "There are " <<children.size()<<" children of the selector."<< std::endl;  // will return true
        for(BTNode* currentChild : children)
        {
            currentChildState=(currentChild)->run();
            if(currentChildState==0)
            {
                state=0;
                return state;
            }
            else if(currentChildState==1)
            {
            }
            else if(currentChildState==2)
            {
                std::cout << "Found a running child." << std::endl;  // will return true
                runningNodes.push_front(this);
                state=2;
                break;
            }
        }
        return state;
    }
};
class CheckIfFoundSpriteTask : public BTNode {  // Each task will be a class (derived from BTNode of course).
public:
    CheckIfFoundSpriteTask () {
        value="CheckIfFoundSpriteTask";
        state=0;
    }
    virtual int run() override {
        if (environmentState->foundSprite) {
            std::cout << "The monster has found the sprite." << std::endl;  // will return true
            state=1;
        }else {
            std::cout << "The monster has not found for the sprite." << std::endl;  // will return false
            state = 0;
        }
        return state;

    }
};
class CheckIfSameRoomTask : public BTNode {  // Each task will be a class (derived from BTNode of course).
public:
    CheckIfSameRoomTask () {
        value="CheckIfSameRoomTask";
        state=0;
    }
    virtual int run() override {
        if (environmentState->sameRoomAsSprite) {
            std::cout << "The monster is in the same room as the sprite." << std::endl;  // will return true
            state = 1;
        }
        else {
            std::cout << "The monster is in a different room than the sprite." << std::endl;  // will return false
            state=0;
        }
        return state;
    }
};
class CheckIfCloseToSpriteTask : public BTNode {  // Each task will be a class (derived from BTNode of course).
public:
    CheckIfCloseToSpriteTask () {
        value="CheckIfCloseToSpriteTask";
        state=0;
    }
    virtual int run() override {
        if (environmentState->closeToSprite) {
            std::cout << "The monster is close to the target." << std::endl;  // will return true
            state=1;
        } else {
            std::cout << "The monster is not close to the target." << std::endl;  // will return false
            state=0;
        }
        return state;
    }
};
class MoveToNextRoomTask : public BTNode {
public:
    MoveToNextRoomTask () {
        value="MoveToNextRoomTask";
        state=0;
    }
    virtual int run() override {
        if(environmentState->moveToNextRoomComplete)
        {
            //std::cout << "The monster has moved to the next room." << std::endl;  // will return true
            state=1;
        }else {
            //std::cout << "The monster is moving to the next room." << std::endl;  // will return running
            state= 2;
            if(runningNodes.empty())
            {
                runningNodes.push_back(this);
            }
            else if(runningNodes.back()!=this)
            {
                runningNodes.push_back(this);
            }
        }
        return state;
    }
};
class CheckAllNodesInRoomTask : public BTNode {
public:
    CheckAllNodesInRoomTask () {
        value="CheckAllNodesInRoomTask";
        state=0;
    }
    virtual int run() override {
        if(environmentState->allNodesInRoomChecked)
        {
            std::cout << "The monster has checked all nodes in the same room." << std::endl;  // will return true
            state=1;
        }
        else {
            std::cout << "The monster is checking all nodes in the same room." << std::endl;  // will return running
            state=2;
            if(runningNodes.empty())
            {
                runningNodes.push_back(this);
            }
            else if(runningNodes.back()!=this)
            {
                runningNodes.push_back(this);
            }
        }
        return state;
    }
};
class SeekSpriteLocationTask : public BTNode {
public:
    SeekSpriteLocationTask () {
        value="SeekSpriteLocationTask";
        state=0;
    }
    virtual int run() override {
        if(environmentState->foundSprite)
        {
            std::cout << "The monster found the Sprite." << std::endl;  // will return true
            state=1;
        }else {
            std::cout << "The monster is seeking the Sprite location." << std::endl;  // will return true
            state= 2;
            if(runningNodes.empty())
            {
                runningNodes.push_back(this);
            }
            else if(runningNodes.back()!=this)
            {
                runningNodes.push_back(this);
            }
        }
        return state;
    }
};
Selector createMonsterBehaviourTree()
{
    Selector *root = new Selector;
    Sequence *sequence1 = new Sequence;
    Selector *selector21 = new Selector;
    Selector *selector22 = new Selector;


    CheckIfFoundSpriteTask *checkIfFoundSpriteTask = new CheckIfFoundSpriteTask();
    CheckIfSameRoomTask *checkIfSameRoomTask = new CheckIfSameRoomTask();
    MoveToNextRoomTask *moveToNextRoomTask = new MoveToNextRoomTask();
    CheckIfCloseToSpriteTask *checkIfCloseToSpriteTask =new CheckIfCloseToSpriteTask();
    CheckAllNodesInRoomTask *checkAllNodesInRoomTask = new CheckAllNodesInRoomTask();
    SeekSpriteLocationTask *seekSpriteLocationTask = new SeekSpriteLocationTask();



    root->addChild(checkIfFoundSpriteTask);
    root->addChild(sequence1);
    sequence1->addChild(selector21);
    sequence1->addChild(selector22);
    sequence1->addChild(seekSpriteLocationTask);
    selector21->addChild(checkIfSameRoomTask);
    selector21->addChild(moveToNextRoomTask);
    selector22->addChild(checkIfCloseToSpriteTask);
    selector22->addChild(checkAllNodesInRoomTask);
    std::cout << "Tree and Environment State created and initialized." << std::endl;  // will return true

    return *root;
}
list<Node> checkAllNodesInRoom1Path()
{
    list<Node> result;

    //Nodes in Room 1
    Node node3(300,600);
    Node node21(300,800);
    Node node20(100,800);
    Node node2(100,600);
    Node node0(100,400);
    Node node18(100,200);
    Node node19(300,200);
    Node node1(300,400);

    result.push_back(node3);
    result.push_back(node21);
    result.push_back(node20);
    result.push_back(node2);
    result.push_back(node0);
    result.push_back(node18);
    result.push_back(node19);
    result.push_back(node1);

    return result;
}
list<Node> checkAllNodesInRoom2Path()
{
    list<Node> result;

    //Nodes in Room 2
    Node node7(700,600);
    Node node25(700,800);
    Node node24(500,800);
    Node node6(500,600);
    Node node4(500,400);
    Node node22(500,200);
    Node node23(700,200);
    Node node5(700,400);

    result.push_back(node7);
    result.push_back(node25);
    result.push_back(node24);
    result.push_back(node6);
    result.push_back(node4);
    result.push_back(node22);
    result.push_back(node23);
    result.push_back(node5);

    return result;
}
list<Node> checkAllNodesInRoom3Path()
{
    list<Node> result;

    //Nodes in Room 3
    Node node11(1100,800);
    Node node29(1100,800);
    Node node28(900,600);
    Node node10(900,400);
    Node node8(900,200);
    Node node26(1100,200);
    Node node27(1100,400);
    Node node9(1100,600);

    result.push_back(node11);
    result.push_back(node29);
    result.push_back(node28);
    result.push_back(node10);
    result.push_back(node8);
    result.push_back(node26);
    result.push_back(node27);
    result.push_back(node9);

    return result;
}
list<Node> checkAllNodesInRoom(int roomNumber)
{
    list<Node> result;
    if(roomNumber==1)
    {
        result=checkAllNodesInRoom1Path();
    }
    if(roomNumber==2)
    {
        result=checkAllNodesInRoom2Path();
    }
    if(roomNumber==3)
    {
        result=checkAllNodesInRoom3Path();
    }
    return result;
}


//Code for Environment Variables
int currentRoom(float x, float y) {
    int roomNumber=0;

    float distanceFromRoom1=sqrt((x-200)*(x-200) + (y-500)*(y-500));
    float distanceFromRoom2=sqrt((x-600)*(x-600) + (y-500)*(y-500));
    float distanceFromRoom3=sqrt((x-1000)*(x-1000) + (y-500)*(y-500));

    if(distanceFromRoom1<=distanceFromRoom2 && distanceFromRoom1<=distanceFromRoom3)
    {
        return 1;
    }
    else if(distanceFromRoom2<=distanceFromRoom3 && distanceFromRoom2<=distanceFromRoom1)
    {
        return 2;
    }
    else
    {
        return 3;
    }

    return 0;
}
bool checkCloseToSprite(float sprite_x, float sprite_y, float monster_x, float monster_y)
{
    float distance=sqrt((monster_x-sprite_x)*(monster_x-sprite_x)+(monster_y-sprite_y)*(monster_y-sprite_y));
    if(distance<=100)
    {
        return true;
    }
    return false;
}
bool checkFoundSprite(float sprite_x, float sprite_y, float monster_x, float monster_y)
{
    float distance=sqrt((monster_x-sprite_x)*(monster_x-sprite_x)+(monster_y-sprite_y)*(monster_y-sprite_y));
    if(distance<=10)
    {
        return true;
    }
    return false;
}

//Code for Game Graph generation
std::unordered_map<int, Node > createGraphNodeDictionary()
{
    std::unordered_map<int, Node > pathPosition;

    //Nodes around obstacle1
    Node node0(100,400);
    Node node1(300,400);
    Node node2(100,600);
    Node node3(300,600);

    //Nodes around obstacle2
    Node node4(500,400);
    Node node5(700,400);
    Node node6(500,600);
    Node node7(700,600);

    //Nodes around obstacle3
    Node node8(900,400);
    Node node9(1100,400);
    Node node10(900,600);
    Node node11(1100,600);


    //Nodes in the doorway 1
    Node node14(350,500);
    Node node12(400,500);
    Node node15(450,500);

    //Nodes in the doorway 2
    Node node16(750,500);
    Node node13(800,500);
    Node node17(850,500);

    //Nodes in Room 1
    Node node18(100,200);
    Node node19(300,200);
    Node node20(100,800);
    Node node21(300,800);

    //Nodes in Room 2
    Node node22(500,200);
    Node node23(700,200);
    Node node24(500,800);
    Node node25(700,800);

    //Nodes in Room 3
    Node node26(900,200);
    Node node27(1100,200);
    Node node28(900,800);
    Node node29(1100,800);


    //Inserting to position hashmap
    pathPosition.insert(std::pair<int, Node>(0, node0));
    pathPosition.insert(std::pair<int, Node>(1, node1));
    pathPosition.insert(std::pair<int, Node>(2, node2));
    pathPosition.insert(std::pair<int, Node>(3, node3));
    pathPosition.insert(std::pair<int, Node>(4, node4));
    pathPosition.insert(std::pair<int, Node>(5, node5));
    pathPosition.insert(std::pair<int, Node>(6, node6));
    pathPosition.insert(std::pair<int, Node>(7, node7));
    pathPosition.insert(std::pair<int, Node>(8, node8));
    pathPosition.insert(std::pair<int, Node>(9, node9));
    pathPosition.insert(std::pair<int, Node>(10, node10));
    pathPosition.insert(std::pair<int, Node>(11, node11));
    pathPosition.insert(std::pair<int, Node>(12, node12));
    pathPosition.insert(std::pair<int, Node>(13, node13));
    pathPosition.insert(std::pair<int, Node>(14, node14));
    pathPosition.insert(std::pair<int, Node>(15, node15));
    pathPosition.insert(std::pair<int, Node>(16, node16));
    pathPosition.insert(std::pair<int, Node>(17, node17));
    pathPosition.insert(std::pair<int, Node>(18, node18));
    pathPosition.insert(std::pair<int, Node>(19, node19));
    pathPosition.insert(std::pair<int, Node>(20, node20));
    pathPosition.insert(std::pair<int, Node>(21, node21));
    pathPosition.insert(std::pair<int, Node>(22, node22));
    pathPosition.insert(std::pair<int, Node>(23, node23));
    pathPosition.insert(std::pair<int, Node>(24, node24));
    pathPosition.insert(std::pair<int, Node>(25, node25));
    pathPosition.insert(std::pair<int, Node>(26, node26));
    pathPosition.insert(std::pair<int, Node>(27, node27));
    pathPosition.insert(std::pair<int, Node>(28, node28));
    pathPosition.insert(std::pair<int, Node>(29, node29));

    return pathPosition;

}
Graph addEdgesBetweenGraphNodes(Graph graph)
{

    //Connecting vertices around obstacle1
    graph.addEdge(0,1,1);
    graph.addEdge(1,3,1);
    graph.addEdge(2,3,1);
    graph.addEdge(0,2,1);

    //Connecting vertices around obstacle2
    graph.addEdge(4,5,1);
    graph.addEdge(5,7,1);
    graph.addEdge(6,7,1);
    graph.addEdge(4,6,1);

    //Connecting vertices around obstacle3
    graph.addEdge(8,9,1);
    graph.addEdge(9,11,1);
    graph.addEdge(10,11,1);
    graph.addEdge(8,10,1);

    //Connecting vertices through doorway 1
    graph.addEdge(1,14,1);
    graph.addEdge(3,14,1);
    graph.addEdge(14,12,1);
    graph.addEdge(12,15,1);
    graph.addEdge(15,4,1);
    graph.addEdge(15,6,1);

    //Connecting vertices through doorway 2
    graph.addEdge(5,16,1);
    graph.addEdge(7,16,1);
    graph.addEdge(16,13,1);
    graph.addEdge(13,17,1);
    graph.addEdge(17,8,1);
    graph.addEdge(17,10,1);

    //Connecting vertices in Room 1
    graph.addEdge(0,18,1);
    graph.addEdge(0,19,1);
    graph.addEdge(18,19,1);
    graph.addEdge(19,1,1);
    graph.addEdge(18,1,1);

    graph.addEdge(2,20,1);
    graph.addEdge(2,21,1);
    graph.addEdge(3,21,1);
    graph.addEdge(20,21,1);
    graph.addEdge(3,20,1);

    //Connecting vertices in Room 2
    graph.addEdge(4,22,1);
    graph.addEdge(22,23,1);
    graph.addEdge(4,23,1);
    graph.addEdge(5,23,1);
    graph.addEdge(5,22,1);

    graph.addEdge(6,24,1);
    graph.addEdge(7,25,1);
    graph.addEdge(24,25,1);
    graph.addEdge(6,25,1);
    graph.addEdge(7,24,1);

    //Connecting vertices in Room 1
    graph.addEdge(8,26,1);
    graph.addEdge(8,27,1);
    graph.addEdge(9,27,1);
    graph.addEdge(9,26,1);
    graph.addEdge(26,27,1);

    graph.addEdge(10,28,1);
    graph.addEdge(10,29,1);
    graph.addEdge(11,28,1);
    graph.addEdge(11,29,1);
    graph.addEdge(28,29,1);

    return graph;
}


//SFML Graphics for Wall Generation
struct wall {
    float x1, y1, x2, y2;

    wall(float x1, float y1, float x2, float y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}
};

int main(int, char const**)
{
    int window_x_limit=1280;
    int window_y_limit=960;

    // Just defining some walls
    // You'd read this from a file
    std::vector<wall> walldata;
    // Let's just add four walls around our room


    // Some wall pointing inwards
    walldata.push_back(wall(400, 0, 400, 400));
    walldata.push_back(wall(400, window_y_limit, 400, window_y_limit-400));
    walldata.push_back(wall(800, 0, 800, 400));
    walldata.push_back(wall(800, window_y_limit, 800, window_y_limit-400));

    sf::Color outlinecolor(sf::Color::Black);
    sf::Color wallcolor(sf::Color::Black);

    sf::VertexArray walls(sf::Quads);
    // Let's first add the outlines, since those have to be drawn first
    for (unsigned int i = 0; i < walldata.size(); ++i) {
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x1 - 4, walldata[i].y1 - 4), outlinecolor)); // top left corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x2 + 4, walldata[i].y1 - 4), outlinecolor)); // top right corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x2 + 4, walldata[i].y2 + 4), outlinecolor)); // bottom right corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x1 - 4, walldata[i].y2 + 4), outlinecolor)); // bottom left corner
    }
    // And now the walls again, this time with their actual color
    for (unsigned int i = 0; i < walldata.size(); ++i) {
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x1 - 2, walldata[i].y1 - 2), wallcolor)); // top left corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x2 + 2, walldata[i].y1 - 2), wallcolor)); // top right corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x2 + 2, walldata[i].y2 + 2), wallcolor)); // bottom right corner
        walls.append(sf::Vertex(sf::Vector2f(walldata[i].x1 - 2, walldata[i].y2 + 2), wallcolor)); // bottom left corner
    }



    sf::RectangleShape obstacle1(sf::Vector2f(100,100));
    obstacle1.setPosition(150,450);
    obstacle1.setFillColor(sf::Color::Black);

    sf::RectangleShape obstacle2(sf::Vector2f(100,100));
    obstacle2.setPosition(550,450);
    obstacle2.setFillColor(sf::Color::Black);

    sf::RectangleShape obstacle3(sf::Vector2f(100,100));
    obstacle3.setPosition(950,450);
    obstacle3.setFillColor(sf::Color::Black);

    bool leftClick=false;
    int V=30;
    Graph graph(V);

    graph.position=createGraphNodeDictionary();
    graph=addEdgesBetweenGraphNodes(graph);
    list<Node> path=graph.astar(0,1);
    list<Node> monsterPath=graph.astar(8,9);
    list<Node>::iterator  currentPathIndex;
    list<Node>::iterator  currentMonsterPathIndex;
    currentPathIndex=path.begin();
    currentMonsterPathIndex=monsterPath.begin();



    sf::Clock clock;
    // Create the first window
    sf::RenderWindow window(sf::VideoMode(window_x_limit, window_y_limit), "SFML window for Task 1");

    sf::Text textVelocityX;
    sf::Text textVelocityY;
    sf::Text textLinearAccelerationX;
    sf::Text textLinearAccelerationY;
    sf::Text textRotation;
    sf::Text textAngularAcceleration;


    sf::Font font;
    if(!font.loadFromFile("OpenSans-Regular.ttf"))
    {
        std::cout << "Could not load font";
        return 0;
    }
    // select the font
    textVelocityX.setFont(font); // font is a sf::Font
    textVelocityY.setFont(font); // font is a sf::Font
    textLinearAccelerationX.setFont(font); // font is a sf::Font
    textLinearAccelerationY.setFont(font); // font is a sf::Font
    textRotation.setFont(font); // font is a sf::Font
    textAngularAcceleration.setFont(font); // font is a sf::Font

    // set the character size
    textVelocityX.setCharacterSize(35); // in pixels, not points!
    textVelocityY.setCharacterSize(35); // in pixels, not points!
    textLinearAccelerationX.setCharacterSize(35); // in pixels, not points!
    textLinearAccelerationY.setCharacterSize(35); // in pixels, not points!
    textRotation.setCharacterSize(35); // in pixels, not points!
    textAngularAcceleration.setCharacterSize(35); // in pixels, not points!

    // set the color
    textVelocityX.setFillColor(sf::Color::Black);
    textVelocityY.setFillColor(sf::Color::Black);
    textLinearAccelerationX.setFillColor(sf::Color::Black);
    textLinearAccelerationY.setFillColor(sf::Color::Black);
    textRotation.setFillColor(sf::Color::Black);
    textAngularAcceleration.setFillColor(sf::Color::Black);

    // set the text style
    textVelocityX.setStyle(sf::Text::Bold | sf::Text::Underlined);
    textVelocityY.setStyle(sf::Text::Bold | sf::Text::Underlined);
    textLinearAccelerationX.setStyle(sf::Text::Bold | sf::Text::Underlined);
    textLinearAccelerationY.setStyle(sf::Text::Bold | sf::Text::Underlined);
    textRotation.setStyle(sf::Text::Bold | sf::Text::Underlined);
    textAngularAcceleration.setStyle(sf::Text::Bold | sf::Text::Underlined);


    // Load a sprite to display
    sf::Texture textureSprite;
    if (!textureSprite.loadFromFile("lowResSprite.png")) {
        std::cout << "Could not load sprite texture";
        return 0;
    }
    sf::Sprite sprite(textureSprite);
    sprite.scale(2,2);

    sf::Texture textureMonster;
    if (!textureMonster.loadFromFile("monster.png")) {
        std::cout << "Could not load sprite texture";
        return 0;
    }
    sf::Sprite monster(textureMonster);
    monster.scale(0.05,0.05);
    std::cout << "Loaded sprite and monster"<<"\n";

    std::vector <Kinematic> sprites;
    int drop_timer=0;
    int drop_timer_monster=0;

    std::vector <sf::Vector2f> breadcrumbs;
    std::vector <sf::Vector2f> breadcrumbsMonster;


    //Create Kinematic structure for Sprite
    Position spritePosition;
    spritePosition.value.x=0;
    spritePosition.value.y=0;
    Velocity spriteVelocity;
    spriteVelocity.value.x=0;
    spriteVelocity.value.y=0;
    Kinematic spriteKinematic;
    spriteKinematic.maxSpeed=10000;
    spriteKinematic.position=spritePosition;
    spriteKinematic.velocity=spriteVelocity;


    //Create Kinematic structure for Monster
    Position monsterPosition;
    monsterPosition.value.x=1000;
    monsterPosition.value.y=0;
    Velocity monsterVelocity;
    monsterVelocity.value.x=0;
    monsterVelocity.value.y=0;
    Kinematic monsterKinematic;
    monsterKinematic.maxSpeed=10000;
    monsterKinematic.position=monsterPosition;
    monsterKinematic.velocity=monsterVelocity;



    sf::Vector2f targetPosition (640,480);
    sf::Vector2f targetMonsterPosition (640,480);
    sf::Vector2i  mousePosition (640,480);

    //Initialize SteeringBehaviour
    Arrive arrive;
    Align align;

    bool foundSprite= false;
    bool closeToSprite=false;
    string prevDecision="";
    string prevBehaviour="";
    int prevRoomNumber=0;
    int roomNumber=1;
    int prevMonsterRoomNumber=0;
    int monsterRoomNumber=0;
    bool roomChanged=false;
    bool monsterRoomChanged=false;
    bool cyclesAroundObstacleComplete = false;
    bool allNodesInRoomChecked = false;
    bool moveToNextRoomComplete=true;

    int cycleAroundObstacleNumber=0;
    bool sameRoomAsMonster= false;

    std::cout << "Initialized environment variables"<<"\n";

    struct DTNode root = createSpriteDecisionTree();
    path=cycleAroundObstacle1Path();
    currentPathIndex=path.begin();
    std::cout << "Created DT"<<"\n";

    Selector BTRoot = createMonsterBehaviourTree();
    int rootState=BTRoot.run();


    // Start the game loop
    while (window.isOpen())
    {
        sf::sleep(sf::milliseconds(1));
        clock.restart();

        // Process events
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window: exit
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            // Escape pressed: exit
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
        }


        //Getting environment variables which will be used to traverse Sprite Decision Tree and Monster Behaviour Tree
        roomNumber=currentRoom(spriteKinematic.position.value.x,spriteKinematic.position.value.y);
        monsterRoomNumber= currentRoom(monsterKinematic.position.value.x, monsterKinematic.position.value.y);
        closeToSprite=checkCloseToSprite(spriteKinematic.position.value.x, spriteKinematic.position.value.y, monsterKinematic.position.value.x, monsterKinematic.position.value.y);
        foundSprite=checkFoundSprite(spriteKinematic.position.value.x, spriteKinematic.position.value.y, monsterKinematic.position.value.x, monsterKinematic.position.value.y);
        if(roomNumber!=prevRoomNumber){
            roomChanged=true;
            prevRoomNumber=roomNumber;
        }
        if(monsterRoomNumber!=prevMonsterRoomNumber)
        {
            monsterRoomChanged= true;
            prevMonsterRoomNumber=monsterRoomNumber;
        }
        if(monsterRoomNumber==roomNumber)
        {
            sameRoomAsMonster= true;
        }
        else{
            sameRoomAsMonster= false;
        }


        //Traversing Sprite Decision Tree
        struct DTNode result = traverseSpriteDecisionTree(&root, sameRoomAsMonster,cyclesAroundObstacleComplete, roomChanged);
        string decision=result.data;
        if(decision=="circleAroundObstacle" && prevDecision!="circleAroundObstacle")
        {
            path= cycleAroundObstacle(roomNumber);
            currentPathIndex=path.begin();
            roomChanged= false;
            cycleAroundObstacleNumber=0;
            cyclesAroundObstacleComplete= false;
            prevDecision=decision;
        }
        if(decision=="evadeToNextRoom" && prevDecision!="evadeToNextRoom")
        {
            int nextRoomNumber=findNextRoom(roomNumber);
            pair<int, Node> closestPositionToSprite= graph.closestPoint(spriteKinematic.position.value.x, spriteKinematic.position.value.y);
            path=graph.astar(closestPositionToSprite.first, evadeToNode(nextRoomNumber));
            currentPathIndex=path.begin();
            prevDecision=decision;
        }
        if(decision=="hide" && prevDecision!="hide")
        {
            pair<int, Node> closestPositionToSprite= graph.closestPoint(spriteKinematic.position.value.x, spriteKinematic.position.value.y);
            path=graph.astar(closestPositionToSprite.first, hideAtNode(roomNumber));
            currentPathIndex=path.begin();
            prevDecision=decision;
        }

        //Traversing Monster Behaviour Tree
        environmentState->foundSprite=foundSprite;
        environmentState->closeToSprite=closeToSprite;
        environmentState->sameRoomAsSprite=sameRoomAsMonster;
        environmentState->allNodesInRoomChecked=allNodesInRoomChecked;
        environmentState->moveToNextRoomComplete=moveToNextRoomComplete;
//        for (auto it = runningNodes.begin(); it != runningNodes.end(); ++it)
//            cout << ' ' << (*it)->value<<"\n";
        BTNode* currentBTNode = runningNodes.back();
        string behaviour=(currentBTNode->value);
        int currentStatus=(*currentBTNode).run();
        if(currentStatus!=2)
        {
            runningNodes.pop_back();
        }
        if(behaviour=="MoveToNextRoomTask" && moveToNextRoomComplete)
        {
            moveToNextRoomComplete= false;
            int nextRoomNumber= findNextRoom(monsterRoomNumber);
            pair<int, Node> closestPositionToMonster= graph.closestPoint(monsterKinematic.position.value.x, monsterKinematic.position.value.y);
            monsterPath=graph.astar(closestPositionToMonster.first, evadeToNode(nextRoomNumber));
            currentMonsterPathIndex=monsterPath.begin();
            prevBehaviour=behaviour;
        }
        if(behaviour=="CheckAllNodesInRoomTask" && allNodesInRoomChecked)
        {
            allNodesInRoomChecked= false;
            monsterPath= checkAllNodesInRoom(monsterRoomNumber);
            currentMonsterPathIndex=monsterPath.begin();
        }

        //Pathfollowing for Sprite
        targetPosition={(float)(*currentPathIndex).x, (float)(*currentPathIndex).y};
        Node closestPoint=(graph.closestPoint(spriteKinematic.position.value.x, spriteKinematic.position.value.y)).second;
        if((float)closestPoint.x==(float)(*currentPathIndex).x && (float)closestPoint.y==(float)(*currentPathIndex).y)
        {
            if((currentPathIndex)!=(--path.end()))
            {
                ++currentPathIndex;
            }
            //In case there are multiple cycles around obstacle and path needs to be repeated
            else if(!cyclesAroundObstacleComplete)
            {
                currentPathIndex=path.begin();
                cycleAroundObstacleNumber++;
                if(cycleAroundObstacleNumber==1)
                {
                    cyclesAroundObstacleComplete= true;
                }

            }
        }

        //Pathfollowing for Monster
        targetMonsterPosition={(float)(*currentMonsterPathIndex).x, (float)(*currentMonsterPathIndex).y};
        Node closestPointToMonster=(graph.closestPoint(monsterKinematic.position.value.x, monsterKinematic.position.value.y)).second;
        if((float)closestPointToMonster.x==(float)(*currentMonsterPathIndex).x && (float)closestPointToMonster.y==(float)(*currentMonsterPathIndex).y)
        {
            if((currentMonsterPathIndex)!=(--monsterPath.end()))
            {
                ++currentMonsterPathIndex;
                std::cout<<"Moving to next node in the path "<<"\n";
            }
            else if(behaviour=="MoveToNextRoomTask")
            {
                moveToNextRoomComplete= true;
                std::cout<<"Path complete "<<"\n";
            }
            else if(behaviour=="CheckAllNodesInRoomTask")
            {
                allNodesInRoomChecked= true;
            }
        }
        //Override Pathfollowing for Monster if it is close to Sprite
        if(behaviour=="SeekSpriteLocationTask" && prevBehaviour!="SeekSpriteLocationTask")
        {
            targetMonsterPosition=spriteKinematic.position.value;
        }


        //Behaviour Control
        if(sameRoomAsMonster)
        {
            if(behaviour=="MoveToNextRoomTask")
            {
                (*currentBTNode).state=1;
                runningNodes.pop_back();
            }
        }



        //Getting the value of elapsed time in this cycle
        sf::Time ElapsedTime = clock.getElapsedTime();
        //float time=ElapsedTime.asSeconds();
        float time=0.001;

        //Updating sprite with linear and angular acceleration to reach velocity and rotation goal
        arrive.maxAcceleration=25000000;
        arrive.maxSpeed=200000;
        arrive.radiusDecel=150000;
        arrive.radiusSat=50;
        arrive.character.position=spriteKinematic.position;
        arrive.character.velocity=spriteKinematic.velocity;
        arrive.target.position.value.x=targetPosition.x;
        arrive.target.position.value.y=targetPosition.y;
        SteeringOutput steeringOutputArrive = arrive.getSteering();
        spriteKinematic.update(steeringOutputArrive, time);


        arrive.maxAcceleration=25000000;
        arrive.maxSpeed=200000;
        arrive.radiusDecel=150000;
        arrive.radiusSat=50;
        arrive.character.position=monsterKinematic.position;
        arrive.character.velocity=monsterKinematic.velocity;
        arrive.target.position.value.x=targetMonsterPosition.x;
        arrive.target.position.value.y=targetMonsterPosition.y;
        SteeringOutput steeringOutputArriveMonster = arrive.getSteering();
        monsterKinematic.update(steeringOutputArriveMonster, time);


        align.maxRotation=15;
        align.maxAngularAcceleration=500;
        align.radiusDecel=6;
        align.radiusSat=0.5;
        align.character.orientation=spriteKinematic.orientation;
        align.character.rotation=spriteKinematic.rotation;
        align.target.orientation=atan2(targetPosition.y-spriteKinematic.position.value.y, targetPosition.x-spriteKinematic.position.value.x);
//        std::cout<<"Target orientation is "<<align.target.orientation<<"\n";
//        std::cout<<"Sprite position X and Y position is "<<spriteKinematic.position.value.x<<" and "<<spriteKinematic.position.value.y<<"\n";
//        std::cout<<"Sprite orientation is "<<spriteKinematic.orientation<<" \n";

        SteeringOutput steeringOutputAlign = align.getSteering();
        spriteKinematic.update(steeringOutputAlign, time);

        align.maxRotation=15;
        align.maxAngularAcceleration=500;
        align.radiusDecel=6;
        align.radiusSat=0.5;
        align.character.orientation=monsterKinematic.orientation;
        align.character.rotation=monsterKinematic.rotation;
        align.target.orientation=atan2(targetMonsterPosition.y-monsterKinematic.position.value.y, targetMonsterPosition.x-monsterKinematic.position.value.x);
        SteeringOutput steeringOutputAlignMonster = align.getSteering();
        monsterKinematic.update(steeringOutputAlignMonster, time);


        //Make sure sprite and monster is within window boundary
        if (spriteKinematic.position.value.x < 30)
        {
            spriteKinematic.position.value.x = 30;
        }
        if (spriteKinematic.position.value.x >= 1250)
        {
            spriteKinematic.position.value.x = 1250;
        }
        if (spriteKinematic.position.value.y < 30)
        {
            spriteKinematic.position.value.y = 30;
        }
        if (spriteKinematic.position.value.y >= 930)
        {
            spriteKinematic.position.value.y = 930;
        }
        if (monsterKinematic.position.value.x < 30)
        {
            monsterKinematic.position.value.x = 30;
        }
        if (monsterKinematic.position.value.x >= 1250)
        {
            monsterKinematic.position.value.x = 1250;
        }
        if (monsterKinematic.position.value.y < 30)
        {
            monsterKinematic.position.value.y = 30;
        }
        if (monsterKinematic.position.value.y >= 930)
        {
            monsterKinematic.position.value.y = 930;
        }



        //Check if game over
        if(foundSprite)
        {
            spriteKinematic.position.value.x=0;
            spriteKinematic.position.value.y=0;
            monsterKinematic.position.value.x=1000;
            monsterKinematic.position.value.y=0;
            spriteKinematic.velocity.value.x=0;
            spriteKinematic.velocity.value.y=0;
            monsterKinematic.velocity.value.x=0;
            monsterKinematic.velocity.value.y=0;
            foundSprite= false;
        }

        //set position of sprite and moster
        sprite.setPosition(spriteKinematic.position.value);
        monster.setPosition(monsterKinematic.position.value);


        //set orientation of sprite in the direction of motion
        if(spriteKinematic.velocity.length()>0)
            sprite.setRotation(spriteKinematic.orientation*180/3.14);
        //set orientation of monster in the direction of motion
//        if(monsterKinematic.velocity.length()>0)
//            monster.setRotation(monsterKinematic.orientation*180/3.14 );

        // Clear screen
        window.clear(sf::Color::White);

        // Draw the sprite and monster
        window.draw(sprite);
        window.draw(monster);
        drop_timer++;
        drop_timer_monster++;
        if(drop_timer==50)
        {
            drop_timer=0;
            breadcrumbs.push_back(spriteKinematic.position.value);
        }
        for(sf::Vector2f breadcrumb:breadcrumbs)
        {
            sf::CircleShape circle(5);
            circle.setFillColor(sf::Color(255,100,100,255));
            circle.setPosition(breadcrumb);
            window.draw(circle);
        }
        if(drop_timer_monster==50)
        {
            drop_timer_monster=0;
            breadcrumbsMonster.push_back(monsterKinematic.position.value);
        }
        for(sf::Vector2f breadcrumbMonster:breadcrumbsMonster)
        {
            sf::CircleShape circle(5);
            circle.setFillColor(sf::Color(sf::Color::Green));
            circle.setPosition(breadcrumbMonster);
            window.draw(circle);
        }

        // Update the window
        // inside the main loop, between window.clear() and window.display()

        //Display graph nodes
        for(int i=0;i<graph.position.size();i++)
        {
            sf::CircleShape circle(5);
            circle.setFillColor(sf::Color::Blue);
            circle.setPosition(graph.position.at(i).x,graph.position.at(i).y);
            window.draw(circle);
        }


        textVelocityX.setString("DT - "+result.data);
        textVelocityY.setString("BT - "+behaviour);
        textVelocityY.setPosition(0,50);
        textLinearAccelerationX.setString("Sprite Room - "+std::to_string(roomNumber));
        textLinearAccelerationX.setPosition(0,100);
        textLinearAccelerationY.setString("Monster Room - "+std::to_string(monsterRoomNumber));
        textLinearAccelerationY.setPosition(0,150);
//        textRotation.setString(std::to_string(spriteKinematic.rotation));
//        textRotation.setPosition(0,200);
//        textAngularAcceleration.setString(std::to_string(steeringOutputAlign.angular));
//        textAngularAcceleration.setPosition(0,250);
//
        window.draw(textVelocityX);
        window.draw(textVelocityY);
        window.draw(textLinearAccelerationX);
        window.draw(textLinearAccelerationY);
//        window.draw(textRotation);
//        window.draw(textAngularAcceleration);


        window.draw(walls);
        window.draw(obstacle1);
        window.draw(obstacle2);
        window.draw(obstacle3);

        window.display();
    }

    return EXIT_SUCCESS;
}