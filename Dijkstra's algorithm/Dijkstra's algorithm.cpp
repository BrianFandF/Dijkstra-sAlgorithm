#include <iostream>
#include <vector>
#include <queue>
#include <limits>
using namespace std;

const double INF = numeric_limits<double>::infinity();

void Dijkstra(const vector<vector<double>>& graph, int source, int target) {
    int n = graph.size();
    vector<double> distance(n, INF);
    vector<int> parent(n, -1);
    vector<bool> visited(n, false);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    distance[source] = 0;
    pq.push(make_pair(0, source));

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) {
            continue;
        }
        visited[u] = true;

        for (int v = 0; v < n; v++) {
            double weight = graph[u][v];

            if (weight > 0 && distance[u] + weight < distance[v]) {
                distance[v] = distance[u] + weight;
                parent[v] = u;
                pq.push(make_pair(distance[v], v));
            }
        }
    }

    if (distance[target] == INF) {
        cout << "No path from " << source + 1 << " to " << target + 1 << "\n";
    }
    else {
        cout << "(" << source + 1 << ", " << target + 1 << ") ";
        cout << "Shortest path is: ";
        vector<int> path;
        int node = target;
        while (node != -1) {
            path.push_back(node + 1);
            node = parent[node];
        }
        for (int i = path.size() - 1; i >= 0; i--) {
            cout << path[i] << " ";
        }
        cout << "with length " << distance[target] << "\n";
    }
}

int main() {
    vector<vector<double>> graph = {
        {0, 5, 3, 7, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 2, 8, 0, 0, 0, 0, 0, 0, 0},
        {0, 5, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 4, 9, 8, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 3, 6, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 3, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 8, 0, 1, 2, 0, 4, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 4, 9, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 4},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    for (int i = 0; i < graph.size(); i++)
    {
        for (int j = 0; j < graph.size(); j++)
        {
            Dijkstra(graph, i, j);
        }

        cout << "///////////////////////" << endl;
    }
}