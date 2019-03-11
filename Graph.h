#ifndef GRAPH_HEADER
#define GRAPH_HEADER
#include <vector>
#include <algorithm> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <stack>
#include <map>
#include <set>
//map labels za vsichki varhove
//map labela edgevete 

struct Edge {
    int to;
    double weight;
    Edge(int,int);
};

template <class T>
class Graph {
    
    public:
    Graph();
    Graph(std::string); 
    void addVertex(T value);
    void addEdge(T value, T otherValue, int weight);
    bool isReacheable(T from, T to);
    bool isCyclic(T from);
    void shortestPaths(T source, T target, const std::vector<T>& closedPaths = std::vector<T>());
    bool isEulerian();
    void printEulerianCircuit();
    void printDeadEnds();
    bool isMotherVertex(T);
    
    private:
    int number;
    std::map<int,size_t> inEdges;
    bool isSC();
    Graph<T> transpose();
    void DFSHelper(int n, std::map<int,bool>& visited);
    void parse(std::string);
    std::map<int,T> verticesR;
    std::map<T,int> vertices;
    std::map<int, std::vector<Edge>> verticesEdges;
    
};

Edge::Edge(int to,int weight) {
    this->to = to;
    this->weight = weight;
}
template <class T>
Graph<T>::Graph() {
    
}
template <class T>
Graph<T>::Graph(std::string filename) {
    this->parse(filename);    
}
template <class T>
void Graph<T>::parse(std::string filename) {
    std::ifstream readFile(filename);

    std::string line, from, to;
    int weight;
    while (std::getline(readFile, line))
    {
        std::istringstream iss(line);
        iss >> from;
        this->addVertex(from);
        while (iss >> to >> weight) {
            this->addEdge(from, to, weight);
        }
    }
}
template <class T>
void Graph<T>::addVertex(T value) {
    typename std::map<T,int>::iterator it = this->vertices.find(value);
    if (it == this->vertices.end()) {
        vertices[value] = this->number;
        verticesR[this->number] = value;
        std::vector<Edge> newEdges;
        verticesEdges[this->number] = newEdges;
        this->number++;
    }
}
template <class T>
void Graph<T>::addEdge(T value, T otherValue, int weight) {
    this->addVertex(value);
    this->addVertex(otherValue);
    int keyFrom = this->vertices[value];
    int keyTo = this->vertices[otherValue];
    Edge newEdge(keyTo, weight);
    this->verticesEdges[keyFrom].push_back(newEdge);
    this->inEdges[keyTo]++;
}
/*
* Check if path from the two supplied vertices exists
*/
template <class T>
bool Graph<T>::isReacheable(T from, T to){

    typename std::map<T, int>::iterator it 
        = this->vertices.find(from);
    if (it == this->vertices.end()) {
        
        std::cout << "No such source node" << std::endl;
        return false;
    }
    typename std::map<T, int>::iterator itDest 
        = this->vertices.find(to);
    if (itDest == this->vertices.end()) {
        
        std::cout << "No such target node" << std::endl;
        return false;
    }  
    
    if (from == to) return true;
    
    std::queue<int> q;
    q.push(it->second);
    std::map<int, bool> visited;
    while (!q.empty()) {
        int v = q.front();
        q.pop();
        // if the node itself
        if (this->verticesR[v] == to) return true;

        for (auto edge : this->verticesEdges[v])
        {
            if (edge.to == this->vertices[to]) return true;
            
            if (!visited[edge.to]) {
                visited[edge.to] = true;
                q.push(edge.to);
            }
        }
    }
    return false;
}
/*
* Check if given vertex is part of a cycle in the graph
*/
template <class T>
bool Graph<T>::isCyclic(T from) {

    typename std::map<T, int>::iterator it 
        = this->vertices.find(from);
    
    if (it == this->vertices.end()) {
        std::cout << "No vertex " << from << "exists in graph" << std::endl;
        return false;
    }
    std::stack<int> stack;
    stack.push(it->second);
    std::map<int, bool> visited;
    while (!stack.empty()) {
        int v = stack.top();
        stack.pop();

        for (auto edge : this->verticesEdges[v])
        {
            if (edge.to == this->vertices[from]) return true;
            
            if (!visited[edge.to]) {
                visited[edge.to] = true;
                stack.push(edge.to);
            }
        }
    }
    return false;

}

template <class T>
void Graph<T>::shortestPaths(T source, T target, const std::vector<T>& closedPaths) {

    typename std::map<T, int>::iterator itSource 
        = this->vertices.find(source);
    if (itSource == this->vertices.end()) {

        std::cout << "No such source vertex" << std::endl;
        return;
    }
    typename std::map<T, int>::iterator itTarget 
        = this->vertices.find(target);
    if (itTarget == this->vertices.end()) {
        
        std::cout << "No such target vertex" << std::endl;
        return;
    }
    int k = 3;
    std::pair<std::vector<T>,int> path;
    std::vector<T> pathVector;
    pathVector.push_back(source);
    path = make_pair(pathVector, 0);
    std::set<std::pair<std::vector<T>,int>> paths;
    std::map<int,int> numOfPaths;
    for (auto pair : this->verticesR)
    {
        numOfPaths[pair.first] = 0;
    }
    // handle closed crossroads
    std::map<int,bool> closedCrossroads;
    for (auto pair : this->verticesR)
    {
        closedCrossroads[pair.first] = false;
    }
    for (auto v : closedPaths)
    {
        closedCrossroads[this->vertices[v]] = true;
    }


    // modify priority queue to become min heap
    std::priority_queue< std::pair<std::vector<T>,int >, 
    std::vector<std::pair<std::vector<T>,int>>,
    std::greater<std::pair<std::vector<T>,int>>> B;

    B.push(path);

    while (!B.empty() && numOfPaths[this->vertices[target]] < k)
    {
        // namira Pu shortest cost path in B with cost C
        // B.pop();
        std::vector<T> vectorPth = B.top().first;
        int cost =  B.top().second;
        T uValue = vectorPth[vectorPth.size() - 1];
        int u = this->vertices[uValue];
        numOfPaths[u]++;
        B.pop();
        if (uValue == target) {
            paths.insert(make_pair(vectorPth, cost));
            k++;
        }
        if (numOfPaths[u] <= k)
        {
            for (auto edge : this->verticesEdges[u])
            {
                std::vector<T> tmpVectorPth = vectorPth;
                tmpVectorPth.push_back(this->verticesR[edge.to]);
                B.push(make_pair(tmpVectorPth, cost+ edge.weight));      
            }
        }
    }
    // Printing part
    std::cout << "--- Трите най-кратки пътища от " << source << " до " << target << " ---" << std::endl;
    int realPaths = 0;
    for (auto path : paths)
    {
        bool hasClosedPath = false;
        // handle closed crossroads
        for (auto cl : closedPaths)
        {
            if (std::find(path.first.begin(), path.first.end(), cl) != path.first.end())
            {
                hasClosedPath = true;
                break;
            }
        }
        if (!hasClosedPath)
        {
            realPaths++;
            for (size_t i = 0; i < path.first.size(); i++)
            {
                std::cout << path.first[i];
                if (i < path.first.size() - 1) std::cout << " -->  "; 
            }
            std::cout << std::endl;
        }
    }
    if (realPaths == 0) 
    {
        std::cout << "Няма намерени пътища" << std::endl;
    }
}
template <class T>
void Graph<T>::DFSHelper(int n, std::map<int,bool>& visited)
{
    typename std::map<int, std::vector<Edge>>::iterator it
        = this->verticesEdges.find(n);
    std::stack<int> stack;
    stack.push(it->first);
    
    visited[it->first] = true;

    while (!stack.empty()) {
        int v = stack.top();
        stack.pop();

        for (auto edge : this->verticesEdges[v])
        {
            if (!visited[edge.to]) {
                visited[edge.to] = true;
                stack.push(edge.to);
            }
        }
    }
}
/*
* Check if graph is strongly connected
*/
template <class T>
bool Graph<T>::isSC() 
{ 
    // Mark all the vertices as not visited
    std::map<int,bool> visited; 
    for (int i = 0; i < this->vertices.size(); i++)
    { 
        visited[i] = false; 
    }
    // Find the first vertex with non-zero degree 
    int n;
    for (n = 0; n < this->vertices.size(); n++)
    { 
        if (this->verticesEdges[n].size() > 0) 
          break; 
    }
    // Do DFS traversal starting from first non zero degree vertex. 
    this->DFSHelper(n, visited);
    // If DFS traversal doesn't visit all vertices, then return false. 
    for (int i = 0; i < this->vertices.size(); i++)
    { 
        if (this->verticesEdges[i].size() > 0 && visited[i] == false) 
        {
            return false; 
        }
    }
    // Create a reversed graph 
    Graph<T> gr = transpose(); 
  
    // Mark all the vertices as not visited (For second DFS) 
    for (int i = 0; i < this->vertices.size(); i++)
    { 
        visited[i] = false; 
    }
    // Do DFS for reversed graph starting from first vertex. 
    // Staring Vertex must be same starting point of first DFS 
    gr.DFSHelper(n, visited);
  
    // If all vertices are not visited in second DFS, then 
    // return false
    for (int i = 0; i < this->vertices.size(); i++) 
    {
        if (this->verticesEdges[i].size() > 0 && visited[i] == false) 
             return false; 
    }
    return true; 
} 
/*
* Transpose graph
*/
template <class T>
Graph<T> Graph<T>::transpose()
{ 
    Graph<T> g; 
    for (int v = 0; v < this->verticesR.size(); v++) 
    { 
        // Recur for all the vertices adjacent to this vertex 
        typename std::vector<Edge>::iterator i; 
        for(i = this->verticesEdges[v].begin(); i != this->verticesEdges[v].end(); ++i) 
        {
            Edge newEdge(v,i->weight);
            g.verticesEdges[i->to].push_back(newEdge);
            (g.inEdges[v])++;   
        } 
    } 
    return g; 
} 
/*
* Check if graph has an eulerian cycle
*/
template <class T>
bool Graph<T>::isEulerian() 
{
    // Check if all non-zero degree vertices are connected 
    if (!this->isSC()) return false; 
  
    // Check if in degree and out degree of every vertex is same 
    for (int i = 0; i < this->vertices.size(); i++) 
    {
        if (this->verticesEdges[i].size() != this->inEdges[i])
            return false;
    }
    return true; 
}
/*
* Print Eulerian Circuit
*/
template <class T>
void Graph<T>::printEulerianCircuit() 
{ 
    // Check if it has eulerian cycle
    if (!this->isEulerian()) return;

    std::map<int,int> edge_count; 
    std::map<int, std::vector<Edge>> localVertexEdges = this->verticesEdges;
    for (int i=0; i< localVertexEdges.size(); i++) 
    {
        edge_count[i] = localVertexEdges[i].size();
    }
    //empty graph
    if (!this->verticesEdges.size()) 
        return;  
  
    std::stack<int> curr_path; 
    std::vector<T> circuit;
  
    // start from any vertex 
    curr_path.push(0);
    int curr_v = 0; // Current vertex 
    
    while (!curr_path.empty())
    { 
        // If there's remaining edge 
        if (edge_count[curr_v]) 
        {
            // Push the vertex 
            curr_path.push(curr_v); 
  
            // Find the next vertex using an edge
            int next_v = localVertexEdges[curr_v].back().to;
            // and remove that edge 
            edge_count[curr_v]--; 
            localVertexEdges[curr_v].pop_back();
            
            curr_v = next_v; 
        }
        // back-track to find remaining circuit 
        else
        {
            circuit.push_back(this->verticesR[curr_v]);
            // Back-tracking 
            curr_v = curr_path.top(); 
            curr_path.pop(); 
        }
    }
    // we've got the circuit, now print it in reverse
    std::cout << "Ойлеровият цикъл е :" << std::endl;

    for (int i=circuit.size()-1; i>=0; i--) 
    { 
        std::cout << circuit[i]; 
        if (i) 
           std::cout<<" -> "; 
    }
    std::cout << std::endl;
}
/*
* Checks if a vertex is a mother vertex meaning it is possible to reach all other vertices from it
* Time Complexity : O(V + E)
*/
template <class T>
bool Graph<T>::isMotherVertex(T value)
{ 
    // check if it exists
    if (this->vertices.find(value) == this->vertices.end()) return false;
    
    // Now check if v is actually a mother vertex 
  
    std::map<int,bool> visited;
    for (auto pair : this->verticesR) {
        visited[pair.first] = false;
    }
    this->DFSHelper(this->vertices[value], visited);  
    
    for (int i = 0; i < this->vertices.size(); i++)
    { 
        if (visited[i] == false) return false;
    }
    return true; 
}
/*
* Print dead-end streets
*/
template <class T>
void Graph<T>::printDeadEnds()
{
    // check if empty
    if (this->verticesEdges.size() == 0) return;

    std::queue<int> q;
    for (auto pair : this->verticesEdges) {
        q.push(pair.first);
        std::map<int, bool> visited;
        // check if with this line ev works correctly
        visited[pair.first] = true;

        while (!q.empty()) {
            int v = q.front();
            q.pop();
            for (auto edge : this->verticesEdges[v])
            {
                if (!visited[edge.to]) {
                    visited[edge.to] = true;
                    if(this->verticesEdges[edge.to].size() == 0) {
                        // its one way
                        std::cout << this->verticesR[v] << " --> " << this->verticesR[edge.to] << std::endl; 
                    } else {
                        q.push(edge.to);
                    }
                }
            }
        }
    }
}
#endif