#include <iostream>
#include "Graph.h"

int main() {
    
    Graph<std::string> graph("file.txt");
    // tests for existence of path
    std::cout << "Има път от Попа до НДК : " << graph.isReacheable("Попа", "НДК") << std::endl;
    std::cout << "Има път от Тест до БСФС : " << graph.isReacheable("Тест", "БСФС") << std::endl;
    // tests for existence of cycle with starting vertex given
    std::cout << "Попа участва в цикъл : " << graph.isCyclic("Попа") << std::endl;
    std::cout << "Тест участва в цикъл : " << graph.isCyclic("Тест") << std::endl;
    // tests for one ways
    std::cout << "--- Задънени улици  ---" << std::endl;
    graph.printDeadEnds();
    graph.shortestPaths("Попа", "БСФС");
    graph.shortestPaths("Попа", "Тест");
    std::vector<std::string> closed;
    closed.push_back("НДК");
    graph.shortestPaths("Попа", "БСФС", closed);
    
    /*
    Test for eulerian path
    Graph<int> g; 
    g.addEdge(1, 0,1); 
    g.addEdge(0, 2,1); 
    g.addEdge(2, 1,1); 
    g.addEdge(0, 3,1); 
    g.addEdge(3, 4,1); 
    g.addEdge(4, 0,1); 
    g.printEulerianCircuit();
    */
    /*
    Test for mother vertex
    Graph<int> g;
    g.addEdge(0, 1, 1); 
    g.addEdge(0, 2, 1); 
    g.addEdge(1, 3, 1); 
    g.addEdge(4, 1, 1);
    g.addEdge(5, 6, 1);
    g.addEdge(6, 4, 1); 
    g.addEdge(5, 2, 1); 
    g.addEdge(6, 0, 1);
    
    std::cout << "Възможно ли е да стигнем до всички върхове от 5: " << g.isMotherVertex(5) << std::endl; 
    std::cout << "Възможно ли е да стигнем до всички върхове от 4: " << g.isMotherVertex(4) << std::endl;
    */
    return 0;
}