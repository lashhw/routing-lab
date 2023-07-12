#ifndef ROUTER_HPP
#define ROUTER_HPP

#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <sstream>
#include <algorithm>

class Router {
private:
    enum edge_t {
        H, V, A
    };

    struct coor_t {
        int x;
        int y;
        int layer;
        bool operator<(const coor_t &other) const {
            return std::make_tuple(x, y, layer) < std::make_tuple(other.x, other.y, other.layer);
        }
    };

    struct literal_t {
        bool negate;
        int var_id;
    };

    struct hard_clause_t {
        int num_literals;
        literal_t *literals;
    };

    int max_layer;
    int x_min, y_min, x_max, y_max;
    int min_pitch_size;
    int via_cost;
    int num_nets;
    int num_bits;
    std::vector<std::string> nets_name;
    std::vector<std::pair<coor_t, coor_t>> nets_coor;
    std::vector<int> x_coors, y_coors;
    int num_x, num_y;
    int *eh_cost;
    int *ev_cost;

    int ***eh_g;
    int ***ev_g;
    int ***ea_g;

    int num_vars = 0;
    int ****eh_id;
    int ****ev_id;
    int ****ea_id;
    int ****n_id;
    std::vector<hard_clause_t> hard_clauses;

    std::vector<int> var_assign;

    std::vector<coor_t> *paths;
    bool ***occupied;

    template<typename T>
    void create_3d_arr(T ***&arr, T init_value) {
        arr = new T **[num_x];
        for (int i = 0; i < num_x; i++) {
            arr[i] = new T *[num_y];
            for (int j = 0; j < num_y; j++) {
                arr[i][j] = new T[max_layer];
                for (int k = 0; k < max_layer; k++)
                    arr[i][j][k] = init_value;
            }
        }
    };

    void dfs(coor_t curr, int net, bool ***visited) {
        const auto &[x, y, layer] = curr;
        visited[x][y][layer] = true;
        paths[net].push_back({x, y, layer});

        if (x != 0 && eh_g[x - 1][y][layer] == net && !visited[x - 1][y][layer])
            return dfs({x - 1, y, layer}, net, visited);
        if (x != num_x - 1 && eh_g[x][y][layer] == net && !visited[x + 1][y][layer])
            return dfs({x + 1, y, layer}, net, visited);
        if (y != 0 && ev_g[x][y - 1][layer] == net && !visited[x][y - 1][layer])
            return dfs({x, y - 1, layer}, net, visited);
        if (y != num_y - 1 && ev_g[x][y][layer] == net && !visited[x][y + 1][layer])
            return dfs({x, y + 1, layer}, net, visited);
        if (layer != 0 && ea_g[x][y][layer - 1] == net && !visited[x][y][layer - 1])
            return dfs({x, y, layer - 1}, net, visited);
        if (layer != max_layer - 1 && ea_g[x][y][layer] == net && !visited[x][y][layer + 1])
            return dfs({x, y, layer + 1}, net, visited);
    }

    void rip_up(int net) {
        for (const auto &[x, y, layer] : paths[net])
            occupied[x][y][layer] = false;
        paths[net].clear();
    }

    void dijkstra(int net, bool ***visited, coor_t ***prev) {
        const auto &[source, target] = nets_coor[net];

        for (int i = 0; i < num_x; i++)
            for (int j = 0; j < num_y; j++)
                for (int k = 0; k < max_layer; k++)
                    visited[i][j][k] = false;

        typedef std::tuple<int, int, int, int, int, int, int> icc;
        std::priority_queue<icc, std::vector<icc>, std::greater<>> pq;
        pq.emplace(0, target.x, target.y, target.layer, -1, -1, -1);

        while (!pq.empty()) {
            auto [cost, x, y, layer, x_, y_, layer_] = pq.top();
            pq.pop();

            if (visited[x][y][layer])
                continue;
            visited[x][y][layer] = true;

            prev[x][y][layer] = {x_, y_, layer_};
            if (x == source.x && y == source.y && layer == source.layer)
                break;

            if (x != 0 && !visited[x - 1][y][layer] && !occupied[x - 1][y][layer])
                pq.emplace(cost + eh_cost[x - 1], x - 1, y, layer, x, y, layer);
            if (x != num_x - 1 && !visited[x + 1][y][layer] && !occupied[x + 1][y][layer])
                pq.emplace(cost + eh_cost[x], x + 1, y, layer, x, y, layer);
            if (y != 0 && !visited[x][y - 1][layer] && !occupied[x][y - 1][layer])
                pq.emplace(cost + ev_cost[y - 1], x, y - 1, layer, x, y, layer);
            if (y != num_y - 1 && !visited[x][y + 1][layer] && !occupied[x][y + 1][layer])
                pq.emplace(cost + ev_cost[y], x, y + 1, layer, x, y, layer);
            if (layer != 0 && !visited[x][y][layer - 1] && !occupied[x][y][layer - 1])
                pq.emplace(cost + via_cost, x, y, layer - 1, x, y, layer);
            if (layer != max_layer - 1 && !visited[x][y][layer + 1] && !occupied[x][y][layer + 1])
                pq.emplace(cost + via_cost, x, y, layer + 1, x, y, layer);
        }

        // backtrack
        assert(visited[source.x][source.y][source.layer]);
        coor_t curr = source;
        while (true) {
            paths[net].push_back(curr);
            occupied[curr.x][curr.y][curr.layer] = true;
            if (curr.x == target.x && curr.y == target.y && curr.layer == target.layer)
                break;
            curr = prev[curr.x][curr.y][curr.layer];
        }
    }

    int path_cost(int net) {
        int cost = 0;
        for (auto it = paths[net].begin(); it != paths[net].end() - 1; it++) {
            const auto &[x, y, layer] = std::min(*it, *(it + 1));
            const auto &[x_, y_, layer_] = std::max(*it, *(it + 1));
            if (x + 1 == x_ && y == y_ && layer == layer_)
                cost += eh_cost[x];
            else if (x == x_ && y + 1 == y_ && layer == layer_)
                cost += ev_cost[y];
            else if (x == x_ && y == y_ && layer + 1 == layer_)
                cost += via_cost;
            else
                assert(false);
        }
        return cost;
    }

    void fill_id(int ****&arr, int dim_0, int dim_1, int dim_2, int dim_3) {
        arr = new int ***[dim_0];
        for (int i = 0; i < dim_0; i++) {
            arr[i] = new int **[dim_1];
            for (int j = 0; j < dim_1; j++) {
                arr[i][j] = new int *[dim_2];
                for (int k = 0; k < dim_2; k++) {
                    arr[i][j][k] = new int[dim_3];
                    for (int l = 0; l < dim_3; l++) {
                        arr[i][j][k][l] = num_vars;
                        num_vars++;
                    }
                }
            }
        }
    }

    template<edge_t edge>
    bool is_legal_edge(int x, int y, int layer, int net, int ***pin_net) {
        int first_pin = pin_net[x][y][layer];
        int second_pin;
        if constexpr (edge == H)
            second_pin = pin_net[x + 1][y][layer];
        else if constexpr (edge == V)
            second_pin = pin_net[x][y + 1][layer];
        else if constexpr (edge == A)
            second_pin = pin_net[x][y][layer + 1];

        if (first_pin != -1 && net != first_pin)
            return false;
        if (second_pin != -1 && net != second_pin)
            return false;
        return true;
    }

    template <edge_t edge>
    void short_rule(int ****id, int dim_0, int dim_1, int dim_2, int ***pin_net) {
        for (int i = 0; i < dim_0; i++) {
            for (int j = 0; j < dim_1; j++) {
                for (int k = 0; k < dim_2; k++) {
                    for (int l = 0; l < num_nets; l++) {
                        if (is_legal_edge<edge>(i, j, k, l, pin_net)) {
                            for (int m = 0; m < num_bits; m++) {
                                hard_clauses.emplace_back();
                                hard_clauses.back().num_literals = 2;
                                hard_clauses.back().literals = new literal_t[2];
                                hard_clauses.back().literals[0].negate = true;
                                hard_clauses.back().literals[0].var_id = id[i][j][k][l];
                                hard_clauses.back().literals[1].negate = !((l >> m) & 1);
                                hard_clauses.back().literals[1].var_id = n_id[i][j][k][m];

                                int alt_i = i;
                                int alt_j = j;
                                int alt_k = k;
                                if constexpr (edge == H)
                                    alt_i++;
                                else if constexpr (edge == V)
                                    alt_j++;
                                else if constexpr (edge == A)
                                    alt_k++;

                                hard_clauses.emplace_back();
                                hard_clauses.back().num_literals = 2;
                                hard_clauses.back().literals = new literal_t[2];
                                hard_clauses.back().literals[0].negate = true;
                                hard_clauses.back().literals[0].var_id = id[i][j][k][l];
                                hard_clauses.back().literals[1].negate = !((l >> m) & 1);
                                hard_clauses.back().literals[1].var_id = n_id[alt_i][alt_j][alt_k][m];
                            }
                        } else {
                            hard_clauses.emplace_back();
                            hard_clauses.back().num_literals = 1;
                            hard_clauses.back().literals = new literal_t[1];
                            hard_clauses.back().literals[0].negate = true;
                            hard_clauses.back().literals[0].var_id = id[i][j][k][l];
                        }
                    }
                }
            }
        }
    };

public:
    void readCircuitSpec(std::ifstream& inputFile) {
        std::string str;

        inputFile >> str;
        assert(str == "max_layer");
        inputFile >> max_layer;

        inputFile >> str;
        assert(str == "boundary");
        inputFile >> x_min >> y_min >> x_max >> y_max;

        inputFile >> str;
        assert(str == "min_pitch_size");
        inputFile >> min_pitch_size;

        inputFile >> str;
        assert(str == "via_cost");
        inputFile >> via_cost;

        inputFile >> str;
        assert(str == "nets");
        inputFile >> num_nets;

        // assume sizeof(int) is 32 bits
        num_bits = (num_nets <= 1) ? 0 : (32 - __builtin_clz(num_nets - 1));

        for (int i = 0; i < num_nets; i++) {
            nets_name.emplace_back();
            nets_coor.emplace_back();
            inputFile >> nets_name[i];
            inputFile >> nets_coor[i].first.x;
            inputFile >> nets_coor[i].first.y;
            inputFile >> nets_coor[i].first.layer;
            inputFile >> nets_coor[i].second.x;
            inputFile >> nets_coor[i].second.y;
            inputFile >> nets_coor[i].second.layer;
            x_coors.push_back(nets_coor[i].first.x);
            x_coors.push_back(nets_coor[i].second.x);
            y_coors.push_back(nets_coor[i].first.y);
            y_coors.push_back(nets_coor[i].second.y);
        }

        auto fill_gap = [&](std::vector<int> &coors, int c_min, int c_max, std::unordered_map<int, int> &c_map) {
            std::sort(coors.begin(), coors.end());
            coors.erase(std::unique(coors.begin(), coors.end()), coors.end());

            assert(!coors.empty());

            auto last = coors.end();
            for (int c = c_min; c <= coors.front() - min_pitch_size; c += min_pitch_size)
                coors.push_back(c);
            for (int c = *(last - 1) + min_pitch_size; c <= c_max; c += min_pitch_size)
                coors.push_back(c);

            for (auto it = coors.begin(); it != last - 1; it++)
                for (auto c = *it + min_pitch_size; c <= *(it + 1) - min_pitch_size; c += min_pitch_size)
                    coors.push_back(c);

            std::sort(coors.begin(), coors.end());

            // check uniqueness
            for (auto it = coors.begin(); it != coors.end() - 1; it++)
                assert(*it != *(it + 1));

            int count = 0;
            for (const auto &c : coors) {
                c_map[c] = count;
                count++;
            }
        };

        std::unordered_map<int, int> x_map;
        fill_gap(x_coors, x_min, x_max, x_map);
        std::unordered_map<int, int> y_map;
        fill_gap(y_coors, y_min, y_max, y_map);

        num_x = (int)x_coors.size();
        num_y = (int)y_coors.size();

        for (auto &[source, target] : nets_coor) {
            source.x = x_map[source.x];
            source.y = y_map[source.y];
            target.x = x_map[target.x];
            target.y = y_map[target.y];
        }

        eh_cost = new int[num_x - 1];
        ev_cost = new int[num_y - 1];
        for (int i = 0; i < num_x - 1; i++)
            eh_cost[i] = x_coors[i + 1] - x_coors[i];
        for (int i = 0; i < num_y - 1; i++)
            ev_cost[i] = y_coors[i + 1] - y_coors[i];
    }

    void generateGraph() {
        auto init_graph = [](int ***&g, int dim_0, int dim_1, int dim_2) {
            g = new int **[dim_0];
            for (int i = 0; i < dim_0; i++) {
                g[i] = new int *[dim_1];
                for (int j = 0; j < dim_1; j++) {
                    g[i][j] = new int[dim_2];
                    for (int k = 0; k < dim_2; k++)
                        g[i][j][k] = -1;
                }
            }
        };
        init_graph(eh_g, num_x - 1, num_y, max_layer);
        init_graph(ev_g, num_x, num_y - 1, max_layer);
        init_graph(ea_g, num_x, num_y, max_layer - 1);
    }

    std::string getSysCommand(int) {
        return "./open-wbo clause.sat > sat_result.txt";
    }

    void generateClauses(std::ofstream& outputFile) {
        fill_id(eh_id, num_x - 1, num_y, max_layer, num_nets);
        fill_id(ev_id, num_x, num_y - 1, max_layer, num_nets);
        fill_id(ea_id, num_x, num_y, max_layer - 1, num_nets);
        fill_id(n_id, num_x, num_y, max_layer, num_bits);

        int ***pin_net;
        create_3d_arr(pin_net, -1);
        for (int i = 0; i < num_nets; i++) {
            const auto &[source, target] = nets_coor[i];
            assert(pin_net[source.x][source.y][source.layer] == -1);
            assert(pin_net[target.x][target.y][target.layer] == -1);
            pin_net[source.x][source.y][source.layer] = i;
            pin_net[target.x][target.y][target.layer] = i;
        }

        short_rule<H>(eh_id, num_x - 1, num_y, max_layer, pin_net);
        short_rule<V>(ev_id, num_x, num_y - 1, max_layer, pin_net);
        short_rule<A>(ea_id, num_x, num_y, max_layer - 1, pin_net);

        int num_neighbors;
        std::pair<int ****, coor_t> neighbors[6];
        bool comb[6];
        for (int i = 0; i < num_x; i++) {
            for (int j = 0; j < num_y; j++) {
                for (int k = 0; k < max_layer; k++) {
                    num_neighbors = 0;
                    if (i != 0)
                        neighbors[num_neighbors++] = {eh_id, {i - 1, j, k}};
                    if (i != num_x - 1)
                        neighbors[num_neighbors++] = {eh_id, {i, j, k}};
                    if (j != 0)
                        neighbors[num_neighbors++] = {ev_id, {i, j - 1, k}};
                    if (j != num_y - 1)
                        neighbors[num_neighbors++] = {ev_id, {i, j, k}};
                    if (k != 0)
                        neighbors[num_neighbors++] = {ea_id, {i, j, k - 1}};
                    if (k != max_layer - 1)
                        neighbors[num_neighbors++] = {ea_id, {i, j, k}};

                    int net = pin_net[i][j][k];
                    if (net != -1) {
                        // pin node should have at least one incident edge
                        hard_clauses.emplace_back();
                        hard_clauses.back().num_literals = num_neighbors;
                        hard_clauses.back().literals = new literal_t[num_neighbors];
                        for (int m = 0; m < num_neighbors; m++) {
                            hard_clauses.back().literals[m].negate = false;
                            auto id = neighbors[m].first;
                            auto [x, y, z] = neighbors[m].second;
                            hard_clauses.back().literals[m].var_id = id[x][y][z][net];
                        }

                        // pin node cannot have more than one incident edge
                        assert(num_neighbors >= 2);
                        std::fill(comb, comb + 2, true);
                        std::fill(comb + 2, comb + num_neighbors, false);
                        do {
                            hard_clauses.emplace_back();
                            hard_clauses.back().num_literals = 2;
                            hard_clauses.back().literals = new literal_t[2];
                            for (int l = 0, m = 0; l < num_neighbors; l++) {
                                if (comb[l]) {
                                    auto id = neighbors[l].first;
                                    auto [x, y, z] = neighbors[l].second;
                                    hard_clauses.back().literals[m].negate = true;
                                    hard_clauses.back().literals[m].var_id = id[x][y][z][net];
                                    m++;
                                }
                            }
                        } while (std::prev_permutation(comb, comb + num_neighbors));
                    } else {
                        for (int l = 0; l < num_nets; l++) {
                            // non-pin node cannot have only one incident edge
                            for (int m = 0; m < num_neighbors; m++) {
                                hard_clauses.emplace_back();
                                hard_clauses.back().num_literals = num_neighbors;
                                hard_clauses.back().literals = new literal_t[num_neighbors];
                                for (int n = 0; n < num_neighbors; n++) {
                                    hard_clauses.back().literals[n].negate = (m == n);
                                    auto id = neighbors[n].first;
                                    auto [x, y, z] = neighbors[n].second;
                                    hard_clauses.back().literals[n].var_id = id[x][y][z][l];
                                }
                            }

                            // non-pin node cannot have more than two incident edge
                            if (num_neighbors >= 3) {
                                std::fill(comb, comb + 3, true);
                                std::fill(comb + 3, comb + num_neighbors, false);
                                do {
                                    hard_clauses.emplace_back();
                                    hard_clauses.back().num_literals = 3;
                                    hard_clauses.back().literals = new literal_t[3];
                                    for (int m = 0, n = 0; m < num_neighbors; m++) {
                                        if (comb[m]) {
                                            auto id = neighbors[m].first;
                                            auto [x, y, z] = neighbors[m].second;
                                            hard_clauses.back().literals[n].negate = true;
                                            hard_clauses.back().literals[n].var_id = id[x][y][z][l];
                                            n++;
                                        }
                                    }
                                } while (std::prev_permutation(comb, comb + num_neighbors));
                            }
                        }
                    }
                }
            }
        }

#ifndef NDEBUG
        std::ofstream debug_file("sat_debug.txt");
        std::map<int, std::tuple<char, int, int, int, int>> var_map;
        auto fill_map = [&](int ****id, char c, int dim_0, int dim_1, int dim_2, int dim_3) {
            for (int i = 0; i < dim_0; i++)
                for (int j = 0; j < dim_1; j++)
                    for (int k = 0; k < dim_2; k++)
                        for (int l = 0; l < dim_3; l++)
                            var_map[id[i][j][k][l]] = {c, i, j, k, l};
        };
        fill_map(eh_id, 'h', num_x - 1, num_y, max_layer, num_nets);
        fill_map(ev_id, 'v', num_x, num_y - 1, max_layer, num_nets);
        fill_map(ea_id, 'a', num_x, num_y, max_layer - 1, num_nets);
        fill_map(n_id, 'n', num_x, num_y, max_layer, num_bits);

        debug_file << "clauses:\n";
        for (const auto &hard_clause : hard_clauses) {
            for (int i = 0; i < hard_clause.num_literals; i++) {
                if (hard_clause.literals[i].negate)
                    debug_file << '~';
                const auto &[c, dim_0, dim_1, dim_2, dim_3] = var_map[hard_clause.literals[i].var_id];
                debug_file << c << ',' << dim_0 << ',' << dim_1 << ',' << dim_2 << ',' << dim_3 << ' ';
            }
            debug_file << '\n';
        }

        debug_file << "\nmapping:\n";
        for (const auto &[key, value] : var_map) {
            const auto &[c, dim_0, dim_1, dim_2, dim_3] = value;
            debug_file << c << ',' << dim_0 << ',' << dim_1 << ',' << dim_2 << ',' << dim_3 << " => " << key << '\n';
        }
#endif

        outputFile << "p cnf " << num_vars << ' ' << hard_clauses.size() << '\n';
        for (const auto &hard_clause : hard_clauses) {
            for (int i = 0; i < hard_clause.num_literals; i++) {
                if (hard_clause.literals[i].negate)
                    outputFile << '-';
                outputFile << hard_clause.literals[i].var_id + 1 << ' ';
            }
            outputFile << "0\n";
        }
    }

    bool readSolverResult(std::ifstream& inputFile, int) {
        std::string line;
        while (std::getline(inputFile, line)) {
            std::stringstream ss(line);
            std::string first;
            ss >> first;
            if (first == "v") {
                var_assign.resize(num_vars);
                for (int var_id; ss >> var_id; ) {
                    if (var_id < 0)
                        var_assign[-var_id - 1] = false;
                    else
                        var_assign[var_id - 1] = true;
                }
                return true;
            }
        }
        assert(false);
        return true;
    }

    void postProcess() {
        auto fill_graph = [&](int ***g, int ****id, int dim_0, int dim_1, int dim_2) {
            for (int i = 0; i < dim_0; i++)
                for (int j = 0; j < dim_1; j++)
                    for (int k = 0; k < dim_2; k++)
                        for (int l = 0; l < num_nets; l++)
                            if (var_assign[id[i][j][k][l]])
                                g[i][j][k] = l;
        };

        fill_graph(eh_g, eh_id, num_x - 1, num_y, max_layer);
        fill_graph(ev_g, ev_id, num_x, num_y - 1, max_layer);
        fill_graph(ea_g, ea_id, num_x, num_y, max_layer - 1);

        paths = new std::vector<coor_t>[num_nets];
        bool ***visited;
        create_3d_arr(visited, false);
        for (int i = 0; i < num_nets; i++) {
            for (int j = 0; j < num_x; j++)
                for (int k = 0; k < num_y; k++)
                    for (int l = 0; l < max_layer; l++)
                        visited[j][k][l] = false;
            dfs(nets_coor[i].first, i, visited);
        }

        create_3d_arr(occupied, false);
        for (int i = 0; i < num_nets; i++)
            for (const auto &[x, y, layer] : paths[i])
                occupied[x][y][layer] = true;

        int prev_cost = 0;
        for (int i = 0; i < num_nets; i++)
            prev_cost += path_cost(i);
        std::cout << "initial: " << prev_cost << std::endl;

        coor_t ***prev;
        create_3d_arr(prev, {});
        for (int i = 0; ; i++) {
            for (int j = 0; j < num_nets; j++) {
                rip_up(j);
                dijkstra(j, visited, prev);
            }

            int new_cost = 0;
            for (int j = 0; j < num_nets; j++)
                new_cost += path_cost(j);
            std::cout << "iter " << i << ": " << new_cost << std::endl;

            if (prev_cost == new_cost)
                break;
            prev_cost = new_cost;
        }
    }

    void printRoutingResult(std::ofstream& outputFile) {
#ifndef NDEBUG
        std::ofstream debug_file("route_debug.txt");
        debug_file << num_nets << '\n';
        auto print = [&](int ***g, int dim_0, int dim_1, int dim_2) {
            std::vector<std::tuple<int, int, int, int>> v;
            for (int i = 0; i < dim_0; i++)
                for (int j = 0; j < dim_1; j++)
                    for (int k = 0; k < dim_2; k++)
                        if (g[i][j][k] != -1)
                            v.emplace_back(i, j, k, g[i][j][k]);
            debug_file << v.size() << '\n';
            for (const auto &[x, y, layer, net] : v)
                debug_file << x << ' ' << y << ' ' << layer << ' ' << net << '\n';
        };
        print(eh_g, num_x - 1, num_y, max_layer);
        print(ev_g, num_x, num_y - 1, max_layer);
        print(ea_g, num_x, num_y, max_layer - 1);
#endif

        outputFile << "x_coors " << x_coors.size() << '\n';
        for (const auto &x : x_coors)
            outputFile << x << ' ';
        outputFile << '\n';

        outputFile << "y_coors " << y_coors.size() << '\n';
        for (const auto &y : y_coors)
            outputFile << y << ' ';
        outputFile << '\n';

        for (int i = 0; i < num_nets; i++) {
            outputFile << nets_name[i] << ' ' << paths[i].size() << '\n';
            for (const auto &[x, y, layer] : paths[i])
                outputFile << x << ' ' << y << ' ' << layer << '\n';
        }
    }
};

#endif