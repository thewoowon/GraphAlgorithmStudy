#include <string>
#include <vector>
#include <iostream>
#include <set>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;
template <typename T>
struct Edge
{
    unsigned src;
    unsigned dst;
    T weight;
};

template <typename T>
class  Graph
{
public:
    Graph(unsigned N) : V(N) {}

    auto vertices() const { return V; }

    auto& edges() const { return edge_list; }

    auto edges(unsigned v) const
    {
        vector<Edge<T>> edges_from_v;
        for (auto& e : edge_list)
        {
            if (e.src == v)
                edges_from_v.emplace_back(e);
        }

        return edges_from_v;
    }
    void add_edge(Edge<T>&& e)
    {
        // 에지 양 끝 정점 ID가 유효한지 검사
        if (e.src >= 1 && e.src <= V && e.dst >= 1 && e.dst <= V)
            edge_list.emplace_back(e);
        else
            cerr << "에러 : 유효 범위를 벗어난 정점!" << endl;
    }

    template <typename U>
    friend ostream& operator<< (ostream& os, const Graph<U>& G);

private:
    unsigned V;
    vector<Edge<T>> edge_list;

};

template <typename U>
ostream& operator<< (ostream& os, const Graph<U>& G)
{
    for (unsigned i = 1; i < G.vertices(); i++)
    {
        os << i << ":\t";
        auto edges = G.edges(i);
        for (auto& e : edges)
            os << "{" << e.dst << " : " << e.weight << "}, ";

        os << endl;
    }

    return os;
}

template <typename T>
auto create_reference_graph()
{
    Graph<T> G(9);

    map<unsigned, vector<pair<unsigned, T>>> edge_map;
    edge_map[1] = { {2,2},{5,3} };
    edge_map[2] = { {1,2},{5,5},{4,1} };
    edge_map[3] = { {4,2},{7,3} };
    edge_map[4] = { {2,1},{3,2},{5,2},{6,4},{8,5} };
    edge_map[5] = { {1,3},{2,5},{4,2},{8,3} };
    edge_map[6] = { {4,4},{7,4},{8,1} };
    edge_map[7] = { {3,3},{6,4} };
    edge_map[8] = { {4,5},{5,3},{6,1} };

    for (auto& i : edge_map)
        for (auto& j : i.second)
            G.add_edge(Edge<T>{i.first, j.first, j.second});

    return G;
}

template <typename T>
struct Label {
    unsigned ID;
    T distance;

    inline bool operator> (const Label<T>& l) const
    {
        return this->distance > l.distance;
    }
};


template <typename T>
auto dijkstra_shortest_path(const Graph<T>& G, unsigned src, unsigned dst)
{
    priority_queue<Label<T>, vector<Label<T>>, greater<Label<T>>> heap;


    vector<T> distance(G.vertices(), numeric_limits<T>::max());

    set<unsigned> visited;
    vector<unsigned> parent(G.vertices());

    heap.emplace(Label<T>{src, 0});
    parent[src] = src;


    while (!heap.empty())
    {
        auto current_vertax = heap.top();
        heap.pop();

        if (current_vertax.ID == dst)
        {
            cout << current_vertax.ID << "번 정점(목적 정점)에 도착!" << endl;
            break;
        }
        if (visited.find(current_vertax.ID) == visited.end())
        {
            cout << current_vertax.ID << "번 정점에 안착!" << endl;

            for (auto& e : G.edges(current_vertax.ID))
            {
                auto neighbor = e.dst;
                auto new_distance = current_vertax.distance + e.weight;

                if (new_distance < distance[neighbor])
                {
                    heap.emplace(Label<T>{neighbor, new_distance});

                    parent[neighbor] = current_vertax.ID;
                    distance[neighbor] = new_distance;
                }
            }
            visited.insert(current_vertax.ID);

        }
    }

    vector<unsigned> shortest_path;
    auto current_vertax = dst;

    while (current_vertax != src)
    {
        shortest_path.push_back(current_vertax);
        current_vertax = parent[current_vertax];
    }


    shortest_path.push_back(src);
    reverse(shortest_path.begin(), shortest_path.end());

    return shortest_path;
}
int main()
{
    using T = unsigned;

    auto G = create_reference_graph<T>();

    cout << "[입력 그래프]" << endl;
    cout << G << endl;

    auto shortest_path = dijkstra_shortest_path<T>(G, 1, 6);


    cout << endl << "[1번과 6번 정점 사이의 최단 경로]" << endl;
    for (auto v : shortest_path)
        cout << v << " ";
    cout << endl;
}

