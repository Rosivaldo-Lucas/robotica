#include "grid.hpp"

void printPath(int currentVertex, vector<int> parents, vector<int>& finalPath) {
	if (currentVertex == NO_PARENT) {
		return;
	}
	printPath(parents[currentVertex], parents, finalPath);
	finalPath.push_back(currentVertex);
	cout << currentVertex << " ";
}

// A utility function to print
// the constructed distances
// array and shortest paths
vector<int> printSolution(int startVertex, vector<double> distances, vector<int> parents) {
	int nVertices = distances.size();
	cout << "Vertex\t Distance\tPath\n";
	vector<int> finalPath;

	int finalIndex = distances.size()-1;

	cout << startVertex << " -> ";
	cout << finalIndex << " \t";
	cout << distances[finalIndex] << "\t";
	printPath(finalIndex, parents, finalPath);
	cout << endl;


	cout << "final path: ";
	for (auto p : finalPath) {
		cout << p << "(" << distances[p] << ") ";
	}
	cout << endl;

	finalPath[0] = 0;

	return finalPath;

	// for (int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++) {
	// 	if (vertexIndex != startVertex) {
	// 		cout << "\n"
	// 			 << startVertex << " -> ";
	// 		cout << vertexIndex << " \t\t\t ";
	// 		cout << distances[vertexIndex] << "\t\t\t";
	// 		printPath(vertexIndex, parents, finalPath);
	// 	}
	// }
}

// this method returns a minimum distance for the
// vertex which is not included in Tset.
int minimumDist(vector<double> dist, vector<bool> Tset)
{
	double min = DBL_MAX, index, minLenght = 100;

	for (int i = 0; i < dist.size(); i++)
	{
		if (Tset[i] == false && dist[i] <= min && dist[i] <= minLenght)
		{
			min = dist[i];
			index = i;
		}
	}
	return index;
}

bool TrajPlanner::Dijkstra(vector<vector<double>> graph, int src, vector<int>& shortestPath) {
	vector<double> dist = vector<double>(graph.size(), DBL_MAX); // integer array to calculate minimum distance for each node.
	vector<bool> Tset = vector<bool>(graph.size(), false);		 // boolean array to mark visted/unvisted for each node.
	vector<int> parents = vector<int>(graph.size());

	int finalIndex = graph.size()-1;

	parents[src] = NO_PARENT;

	dist[src] = 0; // Source vertex distance is set to zero.

	for (int i = 0; i < graph.size(); i++) {
		int m = minimumDist(dist, Tset); // vertex not yet included.
		Tset[m] = true;					 // m with minimum distance included in Tset.
		for (int j = 0; j < graph.size(); j++) {
			// Updating the minimum distance for the particular node.
			if (!Tset[j] && graph[m][j] && dist[m] != DBL_MAX && dist[m] + graph[m][j] < dist[j]) {
				parents[j] = m;
				dist[j] = dist[m] + graph[m][j];
			}
		}
	}

	cout << "shortest path found? " << dist[finalIndex] << " => " << (dist[finalIndex] >= DBL_MAX) << endl;

	shortestPath = printSolution(src, dist, parents);

	bool found_solution = dist[finalIndex] < DBL_MAX;

	return found_solution;
}
