#include <fstream>
#include <vector>
#include <utility>
#include <queue>

#define INF 999999

using namespace std;

class Graph{
public:
    // arcul
    struct Edge{
        int source, dest, weight;
    };
    // numarul de varfuri, muchii si lista de adiacenta a fiecarui varf
    int V, E;
    vector<vector<pair<int, int>>>lstAdj;

    // un vector de arce
    vector<Edge> edges;

    Graph(int VC, int EC): V(VC), E(EC){
        lstAdj.resize(V);
    }

    ~Graph() = default;

    // BELLMAN_FORD - algoritm pentru determinarea drumurilor de cost minim intre
    //                un nod sursa si toate celelalte noduri
    //              - va fi folosit pentru a reponderare si verificare daca exista
    //              - cicluri "negative" in graf (ciclu cu ponderi negative)
    [[nodiscard]] vector<int> BELLMAN_FORD(const vector<Edge>& newEdges, int source) const {
        // initializem un vector de distante
        vector<int> dist(V + 1, INF);

        // distanta pana la sursa, din sursa, va fi 0
        dist[source] = 0;

        // parcurgem lista de adiacenta de V - 1 ori, deoarece un drum maxim intre 2 varfuri
        // va avea lungimea maxima V - 1
        for(int i = 0; i < V; i++)
            for(const auto& edge: newEdges) {
                int src = edge.source;
                int dest = edge.dest;
                int weight = edge.weight;
                // RELAX
                if (dist[dest] >= dist[src] + weight)
                  dist[dest] = dist[src] + weight;
            }
        // verificam daca exista cicluri negative
        // daca exista cicluri negative, returnam un array gol
        for(const auto& edge: newEdges) {
            int src = edge.source;
            int dest = edge.dest;
            int weight = edge.weight;
            if (dist[dest] > dist[src] + weight)
                return {};
        }

        // daca nu, inseamna ca am putut calcula corect distantele, fara a avea cicluri negative
        // returnam vectorul de distante
        return dist;
    }

    // DIJKSTRA - algoritm pentru determinarea drumurilor de cost minim intre un nod sursa si toate celelalte noduri
    //          - poate fi folosit daca ponderea arcelor este > 0, fiind un algoritm care Greedy
    //            care ofera optimul global pentru aceasta restrictie
    [[nodiscard]] vector<int> DIJKSTRA(int source) const {
        // initializam un vector de distante
        vector<int> dist(V + 1, INF);

        dist[source] = 0;

        // initializam o coada cu prioritati, care va contine varfurile ce nu au fost inca prelucrate de DIJKSTRA
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

        // introducem nodul sursa in coada
        // distanta de la sursa la sursa este 0
        pq.emplace(source, 0);

        // iteram prin coada cat timp nu este goala
        while(!pq.empty()) {
            // extragem varful curent de procesat
            pair<int, int> current = pq.top();
            pq.pop();

            // iteram prin vecinii sai
            for(const auto& neighbor: lstAdj[current.first]) {
                int dest = neighbor.first;
                int weight = neighbor.second;
                // RELAX
                if (dist[dest] > current.second + weight) {
                    dist[dest] = current.second + weight;
                    // adaugam si varful cu distanta modificat in coada
                    pq.emplace(dest, dist[dest]);
                }
            }
        }
        return dist;
    }

    // JOHNSON - algoritm care determina drumul de cost minim intre oricare doua varfuri, indiferent de pondere
    //         - se foloseste de BELLMAN_FORD pentru a 'pozitiva' ponderea arcelor
    //         - se foloseste de DIJKSTRA pentru a determina drumul de cost minim intre arce
    bool JOHNSON(ofstream& fout) {
        // recalculam distantele de la un nod sursa nou adaugat pana la celalte noduri
        // adaugam in vectorul de arce noul varf, cu pondere 0 pe fiecare arc
        vector<Edge> newEdges = edges;

        for (int i = 0; i < V; i++) {
            Edge edge{};
            edge.source = V;
            edge.dest = i;
            edge.weight = 0;
            newEdges.emplace_back(edge);
        }

        // recuperam noile distante
        vector<int> newDist = BELLMAN_FORD(newEdges, V);
        if (newDist.empty())
            return false;

        for (int i = 0; i < V; i++)
            for (auto &vertex: lstAdj[i])
                vertex.second = vertex.second + newDist[i] - newDist[vertex.first];

        for(int i = 0; i < V; i++)
            for(const auto& neighbor: lstAdj[i]) {
                fout << i << " " << neighbor.first << " " << neighbor.second << endl;
            }

        // calculam distanta noua folosind graful cu ponderile noi pt toate nodurile
        for (int i = 0; i < V; i++) {
            vector<int> finalDist = DIJKSTRA(i);

            // in sfarsit, putem recalcula vechile distante
            for (unsigned int j = 0; j < finalDist.size() - 1; j++) {
                if (finalDist[j] >= INF)
                    fout <<"INF ";
                else {
                    int dist = finalDist[j] - newDist[i] + newDist[j];
                    fout << dist << " ";
                }
            }
            fout << endl;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    // initializam fisierele
    ifstream fin(argv[1]);
    ofstream fout(argv[2]);

    int V, E;

    // citim numarul de varfuri si numarul de arce
    fin >> V >> E;

    // intializam graful
    Graph graph(V, E);

    // citim arcele
    int extr1, extr2, weight;
    while(fin >> extr1 >> extr2 >> weight) {
        Graph::Edge edge{};
        edge.source = extr1;
        edge.dest = extr2;
        edge.weight = weight;
        graph.edges.emplace_back(edge);
        graph.lstAdj[extr1].emplace_back(extr2, weight);
    }

    if(!graph.JOHNSON(fout))
        fout << -1;
    return 0;
}