#include <string>
#include <vector>
#include <iostream>
#include <set>
#include <map>
#include <queue>
#include <limits>

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
auto prim_MST(const Graph<T>& G, unsigned src)
{
    priority_queue<Label<T>, vector<Label<T>>, greater<Label<T>>> heap;

    vector<T> distance(G.vertices(), numeric_limits<T>::max());

    set<unsigned> visited;
    vector<unsigned> MST;

    heap.emplace(Label<T>{src, 0});

    while (!heap.empty())
    {
        auto current_vertax = heap.top();
        heap.pop();

        if (visited.find(current_vertax.ID) == visited.end())
        {
            MST.push_back(current_vertax.ID);

            for (auto& e : G.edges(current_vertax.ID))
            {
                auto neighbor = e.dst;
                auto new_distance = e.weight;

                if (new_distance < distance[neighbor])
                {
                    heap.emplace(Label<T>{neighbor, new_distance});
                    distance[neighbor] = new_distance;
                }
            }
            visited.insert(current_vertax.ID);
        }
    }
    return MST;
}

int main()
{
    using T = unsigned;

    auto G = create_reference_graph<T>();
    cout << "" << endl;
    cout << G << endl;

    auto MST = prim_MST<T>(G, 1);

    cout << "" << endl;
    for (auto v : MST)
        cout << v << endl;
    cout << endl;
}