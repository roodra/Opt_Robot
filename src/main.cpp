#include <iostream>
#include <vector>
#include <climits>

using namespace std;

const int V = 4; // Number of vertices

// Function to find the minimum cost path
int tsp(vector<vector<int>>& graph, vector<bool>& visited, int pos, int count, int cost, int& ans) {
    if (count == V && graph[pos][0]) {
        ans = min(ans, cost + graph[pos][0]);
        return ans;
    }

    for (int i = 0; i < V; i++) {
        if (!visited[i] && graph[pos][i]) {
            visited[i] = true;
            tsp(graph, visited, i, count + 1, cost + graph[pos][i], ans);
            visited[i] = false;
        }
    }
    return ans;
}

int main() {
    vector<vector<int>> graph = {
        { 0, 10, 15, 20 },
        { 10, 0, 35, 25 },
        { 15, 35, 0, 30 },
        { 20, 25, 30, 0 }
    };

    vector<bool> visited(V, false);
    visited[0] = true;
    int ans = INT_MAX;

    tsp(graph, visited, 0, 1, 0, ans);

    cout << "The minimum cost of visiting all cities: " << ans << endl;
    return 0;
}
