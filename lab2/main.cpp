#include "ispdData.h"
#include "LayerAssignment.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <cassert>
#include <queue>
#include <limits>
#include <numeric>
#include <cmath>
#include <unordered_set>
#include <random>
#include <csignal>
#include <cstring>
#include <unistd.h>

enum dir_t {
    LEFT, RIGHT, DOWN, UP
};
enum algo_t {
    L, M, HUM
};
constexpr int initial_expand = 10;
constexpr int expand_increment = 10;

// global data
ISPDParser::ispdData *ispdData;
int spacing;
std::vector<std::vector<int>> hcapacity;
std::vector<std::vector<int>> hdemand;
std::vector<std::vector<int>> hhistory;
std::vector<std::vector<double>> hcostarr;
std::vector<std::vector<double>> hnetcost;
std::vector<std::vector<int>> vcapacity;
std::vector<std::vector<int>> vdemand;
std::vector<std::vector<int>> vhistory;
std::vector<std::vector<double>> vcostarr;
std::vector<std::vector<double>> vnetcost;
std::vector<std::vector<double>> d;
std::vector<std::vector<double>> s_d_h;
std::vector<std::vector<double>> s_d_v;
std::vector<std::vector<double>> t_d_h;
std::vector<std::vector<double>> t_d_v;
std::vector<std::vector<dir_t>> pi;
std::vector<std::vector<dir_t>> s_pi_h;
std::vector<std::vector<dir_t>> s_pi_v;
std::vector<std::vector<dir_t>> t_pi_h;
std::vector<std::vector<dir_t>> t_pi_v;
volatile bool interrupted = false;

void interrupt_handler(int signum) {
    std::cout << "Caught " << strsignal(signum) << ". Stopping gracefully..." << std::endl;
    interrupted = true;
}

double cost(int demand, int capacity, double history) {
    constexpr double c1 = 1.0;
    constexpr double c2 = 5.0;
    constexpr double c3 = 0.3;
    double h = 1.0 + c1 * history * history;
    double p = c2 / (1.0 + std::exp(c3 * (capacity - demand)));
    return h * p + 1.0;
}

double hcost(int ax_1, int ax_2, int ay) {
    if (ax_1 > ax_2)
        std::swap(ax_1, ax_2);
    assert(ax_1 + 1 == ax_2);
    return hnetcost[ax_1][ay];
}

double vcost(int ax, int ay_1, int ay_2) {
    if (ay_1 > ay_2)
        std::swap(ay_1, ay_2);
    assert(ay_1 + 1 == ay_2);
    return vnetcost[ax][ay_1];
}

void update_hcost(int ax, int ay) {
    hcostarr[ax][ay] = cost(hdemand[ax][ay], hcapacity[ax][ay], hhistory[ax][ay]);
}

void update_vcost(int ax, int ay) {
    vcostarr[ax][ay] = cost(vdemand[ax][ay], vcapacity[ax][ay], vhistory[ax][ay]);
}

void reset_history() {
    for (int i = 0; i < ispdData->numXGrid; i++) {
        for (int j = 0; j < ispdData->numYGrid; j++) {
            if (i != ispdData->numXGrid - 1) {
                hhistory[i][j] = 0;
                update_hcost(i, j);
            }
            if (j != ispdData->numYGrid - 1) {
                vhistory[i][j] = 0;
                update_vcost(i, j);
            }
        }
    }
}

void update_netcost(ISPDParser::Net *net, int b_left_ax, int b_right_ax, int b_bottom_ay, int b_top_ay) {
    for (int i = b_left_ax; i <= b_right_ax; i++) {
        for (int j = b_bottom_ay; j <= b_top_ay; j++) {
            auto p = std::make_pair(i, j);
            if (i != b_right_ax)
                hnetcost[i][j] = net->hpath.count(p) != 0 ? 0.0 : hcostarr[i][j];
            if (j != b_top_ay)
                vnetcost[i][j] = net->vpath.count(p) != 0 ? 0.0 : vcostarr[i][j];
        }
    }
}

std::tuple<int, int, int, int, bool, bool> get_bbox(ISPDParser::TwoPin *two_pin) {
    // get boundary coordinate
    int left = two_pin->from.x;
    int right = two_pin->to.x;
    bool t_on_right;
    if (left > right) {
        std::swap(left, right);
        t_on_right = false;
    } else {
        t_on_right = true;
    }
    int bottom = two_pin->from.y;
    int top = two_pin->to.y;
    bool t_on_top;
    if (bottom > top) {
        std::swap(bottom, top);
        t_on_top = false;
    } else {
        t_on_top = true;
    }
    return {left, right, bottom, top, t_on_right, t_on_top};
}

void flip(int num_x, int num_y, bool t_on_right, bool t_on_top,
          std::vector<std::vector<double>> &cost,
          std::vector<std::vector<dir_t>> &policy,
          int &s_x, int &s_y, int &t_x, int &t_y) {
    if (!t_on_top) {
        for (int i = 0; i < num_x; i++) {
            for (int j = 0; j < num_y / 2; j++) {
                std::swap(cost[i][j], cost[i][num_y - 1 - j]);
                std::swap(policy[i][j], policy[i][num_y - 1 - j]);
            }
        }
        s_y = num_y - 1 - s_y;
        t_y = num_y - 1 - t_y;
    }
    if (!t_on_right) {
        for (int j = 0; j < num_y; j++) {
            for (int i = 0; i < num_x / 2; i++) {
                std::swap(cost[i][j], cost[num_x - 1 - i][j]);
                std::swap(policy[i][j], policy[num_x - 1 - i][j]);
            }
        }
        s_x = num_x - 1 - s_x;
        t_x = num_x - 1 - t_x;
    }
}

void backtrack(ISPDParser::TwoPin *two_pin,
               int t_x, int t_y, int s_x, int s_y,
               int b_left_ax, int b_bottom_ay,
               const std::vector<std::vector<dir_t>> &policy) {
    auto &net = two_pin->parNet;
    int curr_x = t_x;
    int curr_y = t_y;
    while (curr_x != s_x || curr_y != s_y) {
        int curr_ax = b_left_ax + curr_x;
        int curr_ay = b_bottom_ay + curr_y;
        if (policy[curr_x][curr_y] == LEFT || policy[curr_x][curr_y] == RIGHT) {
            // left/right
            bool left = policy[curr_x][curr_y] == LEFT;
            int curr_lr_ax = left ? curr_ax - 1 : curr_ax;
            auto p = std::make_pair(curr_lr_ax, curr_ay);
            two_pin->h_used.emplace_back(p);
            auto it = net->hpath.find(p);
            if (it == net->hpath.end()) {
                net->hpath.emplace(p, 1);
                hdemand[p.first][p.second] += net->minimumWidth + spacing;
                update_hcost(p.first, p.second);
            } else {
                it->second++;
            }
            curr_x = left ? curr_x - 1 : curr_x + 1;
        } else if (policy[curr_x][curr_y] == DOWN || policy[curr_x][curr_y] == UP){
            // down/up
            bool down = policy[curr_x][curr_y] == DOWN;
            int du_ay = down ? curr_ay - 1 : curr_ay;
            auto p = std::make_pair(curr_ax, du_ay);
            two_pin->v_used.emplace_back(p);
            auto it = net->vpath.find(p);
            if (it == net->vpath.end()) {
                net->vpath.emplace(p, 1);
                vdemand[p.first][p.second] += net->minimumWidth + spacing;
                update_vcost(p.first, p.second);
            } else {
                it->second++;
            }
            curr_y = down ? curr_y - 1 : curr_y + 1;
        }
    }
}

void l_shape_routing(ISPDParser::TwoPin *twopin) {
    auto [left_ax, right_ax, bottom_ay, top_ay, t_on_right, t_on_top] = get_bbox(twopin);
    int num_x = right_ax - left_ax + 1;
    int num_y = top_ay - bottom_ay + 1;
    int s_x = 0;
    int s_y = 0;
    int t_x = num_x - 1;
    int t_y = num_y - 1;
    update_netcost(twopin->parNet, left_ax, right_ax, bottom_ay, top_ay);

    if (left_ax == right_ax) {
        for (int j = 1; j < num_y; j++)
            pi[0][j] = t_on_top ? DOWN : UP;
    } else if (bottom_ay == top_ay) {
        for (int i = 1; i < num_x; i++)
            pi[i][0] = t_on_right ? LEFT : RIGHT;
    } else {
        double right_and_up = 0.0;
        double up_and_right = 0.0;
        for (int i = 1; i < num_x; i++) {
            int curr_ax = t_on_right ? left_ax + i : right_ax - i;
            int curr_ay = t_on_top ? bottom_ay : top_ay;
            int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
            right_and_up += hcost(curr_ax, curr_lr_ax, curr_ay);
            pi[i][0] = t_on_right ? LEFT : RIGHT;
        }
        for (int j = 1; j < num_y; j++) {
            int curr_ax = t_on_right ? right_ax : left_ax;
            int curr_ay = t_on_top ? bottom_ay + j : top_ay - j;
            int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
            right_and_up += vcost(curr_ax, curr_ay, curr_du_ay);
            pi[t_x][j] = t_on_top ? DOWN : UP;
        }
        for (int j = 1; j < num_y; j++) {
            int curr_ax = t_on_right ? left_ax : right_ax;
            int curr_ay = t_on_top ? bottom_ay + j : top_ay - j;
            int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
            up_and_right += vcost(curr_ax, curr_ay, curr_du_ay);
            pi[0][j] = t_on_top ? DOWN : UP;
        }
        for (int i = 1; i < num_x; i++) {
            int curr_ax = t_on_right ? left_ax + i : right_ax - i;
            int curr_ay = t_on_top ? top_ay : bottom_ay;
            int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
            up_and_right += hcost(curr_ax, curr_lr_ax, curr_ay);
            pi[i][t_y] = t_on_right ? LEFT : RIGHT;
        }
        if (up_and_right < right_and_up)
            pi[t_x][t_y] = t_on_right ? LEFT : RIGHT;
        else
            pi[t_x][t_y] = t_on_top ? DOWN : UP;
    }

    flip(num_x, num_y, t_on_right, t_on_top, d, pi, s_x, s_y, t_x, t_y);
    backtrack(twopin, t_x, t_y, s_x, s_y, left_ax, bottom_ay, pi);
}

void monotonic_routing(ISPDParser::TwoPin *two_pin) {
    auto [left_ax, right_ax, bottom_ay, top_ay, t_on_right, t_on_top] = get_bbox(two_pin);
    int num_x = right_ax - left_ax + 1;
    int num_y = top_ay - bottom_ay + 1;
    int s_x = 0;
    int s_y = 0;
    int t_x = num_x - 1;
    int t_y = num_y - 1;
    update_netcost(two_pin->parNet, left_ax, right_ax, bottom_ay, top_ay);

    d[0][0] = 0;
    for (int i = 0; i < num_x; i++) {
        for (int j = 0; j < num_y; j++) {
            if (i != 0 || j != 0) {
                int curr_ax = t_on_right ? left_ax + i : right_ax - i;
                int curr_ay = t_on_top ? bottom_ay + j : top_ay - j;
                double lr_cost = std::numeric_limits<double>::max();
                if (i != 0) {
                    int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
                    lr_cost = d[i - 1][j] + hcost(curr_ax, curr_lr_ax, curr_ay);
                }
                double du_cost = std::numeric_limits<double>::max();
                if (j != 0) {
                    int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
                    du_cost = d[i][j - 1] + vcost(curr_ax, curr_ay, curr_du_ay);
                }
                if (lr_cost < du_cost) {
                    d[i][j] = lr_cost;
                    pi[i][j] = t_on_right ? LEFT : RIGHT;
                } else {
                    d[i][j] = du_cost;
                    pi[i][j] = t_on_top ? DOWN : UP;
                }
            }
        }
    }

    flip(num_x, num_y, t_on_right, t_on_top, d, pi, s_x, s_y, t_x, t_y);
    backtrack(two_pin, t_x, t_y, s_x, s_y, left_ax, bottom_ay, pi);
}

void h_monotonic_routing(ISPDParser::TwoPin *two_pin, int bottom_extend, int top_extend, bool commit) {
    auto [left_ax, right_ax, bottom_ay, top_ay, t_on_right, t_on_top] = get_bbox(two_pin);
    int b_bottom_ay = std::max(0, bottom_ay - bottom_extend);
    int b_top_ay = std::min(ispdData->numYGrid - 1, top_ay + top_extend);
    int s_x = 0;
    int s_y = t_on_top ? bottom_ay - b_bottom_ay : b_top_ay - top_ay;
    int t_x = right_ax - left_ax;
    int t_y = t_on_top ? top_ay - b_bottom_ay : b_top_ay - bottom_ay;

    // dp
    int num_x = right_ax - left_ax + 1;
    int num_y = b_top_ay - b_bottom_ay + 1;
    d[0][s_y] = 0;
    for (int j = s_y - 1; j >= 0; j--) {
        int curr_ax = t_on_right ? left_ax : right_ax;
        int curr_ay = t_on_top ? b_bottom_ay + j : b_top_ay - j;
        int curr_ud_ay = t_on_top ? curr_ay + 1 : curr_ay - 1;
        d[0][j] = d[0][j + 1] + vcost(curr_ax, curr_ay, curr_ud_ay);
        pi[0][j] = t_on_top ? UP : DOWN;
    }
    for (int j = s_y + 1; j < num_y; j++) {
        int curr_ax = t_on_right ? left_ax : right_ax;
        int curr_ay = t_on_top ? b_bottom_ay + j : b_top_ay - j;
        int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
        d[0][j] = d[0][j - 1] + vcost(curr_ax, curr_ay, curr_du_ay);
        pi[0][j] = t_on_top ? DOWN : UP;
    }
    for (int i = 1; i < num_x; i++) {
        int curr_ax = t_on_right ? left_ax + i : right_ax - i;
        int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
        int boundary_ay = t_on_top ? b_bottom_ay : b_top_ay;
        d[i][0] = d[i - 1][0] + hcost(curr_ax, curr_lr_ax, boundary_ay);
        pi[i][0] = t_on_right ? LEFT : RIGHT;
        for (int j = 1; j < num_y; j++) {
            int curr_ay = t_on_top ? b_bottom_ay + j : b_top_ay - j;
            int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
            double cost_lr = d[i - 1][j] + hcost(curr_ax, curr_lr_ax, curr_ay);
            double cost_du = d[i][j - 1] + vcost(curr_ax, curr_ay, curr_du_ay);
            if (cost_lr < cost_du) {
                d[i][j] = cost_lr;
                pi[i][j] = t_on_right ? LEFT : RIGHT;
            } else {
                d[i][j] = cost_du;
                pi[i][j] = t_on_top ? DOWN : UP;
            }
        }
        for (int j = num_y - 2; j >= 0; j--) {
            int curr_ay = t_on_top ? b_bottom_ay + j : b_top_ay - j;
            int curr_ud_ay = t_on_top ? curr_ay + 1 : curr_ay - 1;
            double cost_ud = d[i][j + 1] + vcost(curr_ax, curr_ay, curr_ud_ay);
            if (cost_ud < d[i][j]) {
                d[i][j] = cost_ud;
                pi[i][j] = t_on_top ? UP : DOWN;
            }
        }
    }

    flip(num_x, num_y, t_on_right, t_on_top, d, pi, s_x, s_y, t_x, t_y);
    if (commit)
        backtrack(two_pin, t_x, t_y, s_x, s_y, left_ax, b_bottom_ay, pi);
}

void v_monotonic_routing(ISPDParser::TwoPin *two_pin, int left_extend, int right_extend, bool commit) {
    auto [left_ax, right_ax, bottom_ay, top_ay, t_on_right, t_on_top] = get_bbox(two_pin);
    int b_left_ax = std::max(0, left_ax - left_extend);
    int b_right_ax = std::min(ispdData->numXGrid - 1, right_ax + right_extend);
    int s_x = t_on_right ? left_ax - b_left_ax : b_right_ax - right_ax;
    int s_y = 0;
    int t_x = t_on_right ? right_ax - b_left_ax : b_right_ax - left_ax;
    int t_y = top_ay - bottom_ay;

    // dp
    int num_x = b_right_ax - b_left_ax + 1;
    int num_y = top_ay - bottom_ay + 1;
    d[s_x][0] = 0;
    for (int i = s_x - 1; i >= 0; i--) {
        int curr_ax = t_on_right ? b_left_ax + i : b_right_ax - i;
        int curr_rl_ax = t_on_right ? curr_ax + 1 : curr_ax - 1;
        int curr_ay = t_on_top ? bottom_ay : top_ay;
        d[i][0] = d[i + 1][0] + hcost(curr_ax, curr_rl_ax, curr_ay);
        pi[i][0] = t_on_right ? RIGHT : LEFT;
    }
    for (int i = s_x + 1; i < num_x; i++) {
        int curr_ax = t_on_right ? b_left_ax + i : b_right_ax - i;
        int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
        int curr_ay = t_on_top ? bottom_ay : top_ay;
        d[i][0] = d[i - 1][0] + hcost(curr_ax, curr_lr_ax, curr_ay);
        pi[i][0] = t_on_right ? LEFT : RIGHT;
    }
    for (int j = 1; j < num_y; j++) {
        int boundary_ax = t_on_right ? b_left_ax : b_right_ax;
        int curr_ay = t_on_top ? bottom_ay + j : top_ay - j;
        int curr_du_ay = t_on_top ? curr_ay - 1 : curr_ay + 1;
        d[0][j] = d[0][j - 1] + vcost(boundary_ax, curr_ay, curr_du_ay);
        pi[0][j] = t_on_top ? DOWN : UP;
        for (int i = 1; i < num_x; i++) {
            int curr_ax = t_on_right ? b_left_ax + i : b_right_ax - i;
            int curr_lr_ax = t_on_right ? curr_ax - 1 : curr_ax + 1;
            double cost_lr = d[i - 1][j] + hcost(curr_ax, curr_lr_ax, curr_ay);
            double cost_du = d[i][j - 1] + vcost(curr_ax, curr_ay, curr_du_ay);
            if (cost_lr < cost_du) {
                d[i][j] = cost_lr;
                pi[i][j] = t_on_right ? LEFT : RIGHT;
            } else {
                d[i][j] = cost_du;
                pi[i][j] = t_on_top ? DOWN : UP;
            }
        }
        for (int i = num_x - 2; i >= 0; i--) {
            int curr_ax = t_on_right ? b_left_ax + i : b_right_ax - i;
            int curr_rl_ax = t_on_right ? curr_ax + 1 : curr_ax - 1;
            double cost_rl = d[i + 1][j] + hcost(curr_ax, curr_rl_ax, curr_ay);
            if (cost_rl < d[i][j]) {
                d[i][j] = cost_rl;
                pi[i][j] = t_on_right ? RIGHT : LEFT;
            }
        }
    }

    flip(num_x, num_y, t_on_right, t_on_top, d, pi, s_x, s_y, t_x, t_y);
    if (commit)
        backtrack(two_pin, t_x, t_y, s_x, s_y, b_left_ax, bottom_ay, pi);
}

void fill_hv(ISPDParser::TwoPin *ltp,
             ISPDParser::TwoPin *rtp,
             ISPDParser::TwoPin *btp,
             ISPDParser::TwoPin *ttp,
             int x, int y,
             int left_extend, int right_extend, int bottom_extend, int top_extend,
             int b_left_ax, int b_right_ax, int b_bottom_ay, int b_top_ay,
             std::vector<std::vector<double>> &d_h,
             std::vector<std::vector<double>> &d_v,
             std::vector<std::vector<dir_t>> &pi_h,
             std::vector<std::vector<dir_t>> &pi_v) {
    int ax = b_left_ax + x;
    int ay = b_bottom_ay + y;

    // fill h
    h_monotonic_routing(ltp, bottom_extend, top_extend, false);
    for (int i = 0; i < ax - b_left_ax; i++) {
        for (int j = 0; j < b_top_ay - b_bottom_ay + 1; j++) {
            d_h[i][j] = d[i][j];
            pi_h[i][j] = pi[i][j];
        }
    }
    h_monotonic_routing(rtp, bottom_extend, top_extend, false);
    for (int i = 0; i < b_right_ax - ax + 1; i++) {
        for (int j = 0; j < b_top_ay - b_bottom_ay + 1; j++) {
            d_h[i + x][j] = d[i][j];
            pi_h[i + x][j] = pi[i][j];
        }
    }

    // fill v
    v_monotonic_routing(btp, left_extend, right_extend, false);
    for (int i = 0; i < b_right_ax - b_left_ax + 1; i++) {
        for (int j = 0; j < ay - b_bottom_ay; j++) {
            d_v[i][j] = d[i][j];
            pi_v[i][j] = pi[i][j];
        }
    }
    v_monotonic_routing(ttp, left_extend, right_extend, false);
    for (int i = 0; i < b_right_ax - b_left_ax + 1; i++) {
        for (int j = 0; j < b_top_ay - ay + 1; j++) {
            d_v[i][j + y] = d[i][j];
            pi_v[i][j + y] = pi[i][j];
        }
    }
}

void hum_routing(ISPDParser::TwoPin *twopin,
                 int left_extend, int right_extend, int bottom_extend, int top_extend) {
    // get boundary
    auto [left_ax, right_ax, bottom_ay, top_ay, t_on_right, t_on_top] = get_bbox(twopin);
    int b_left_ax = std::max(0, left_ax - left_extend);
    int b_right_ax = std::min(ispdData->numXGrid - 1, right_ax + right_extend);
    int b_bottom_ay = std::max(0, bottom_ay - bottom_extend);
    int b_top_ay = std::min(ispdData->numYGrid - 1, top_ay + top_extend);
    update_netcost(twopin->parNet, b_left_ax, b_right_ax, b_bottom_ay, b_top_ay);

    // invoke HM or VM directly
    if (left_extend == 0 && right_extend == 0)
        return h_monotonic_routing(twopin, bottom_extend, top_extend, true);
    if (bottom_extend == 0 && top_extend == 0)
        return v_monotonic_routing(twopin, left_extend, right_extend, true);

    // ltp
    ISPDParser::TwoPin s_ltp;
    s_ltp.from = twopin->from;
    s_ltp.to = {b_left_ax, twopin->to.y};
    ISPDParser::TwoPin t_ltp;
    t_ltp.from = twopin->to;
    t_ltp.to = {b_left_ax, twopin->from.y};

    // rtp
    ISPDParser::TwoPin s_rtp;
    s_rtp.from = twopin->from;
    s_rtp.to = {b_right_ax, twopin->to.y};
    ISPDParser::TwoPin t_rtp;
    t_rtp.from = twopin->to;
    t_rtp.to = {b_right_ax, twopin->from.y};

    // btp
    ISPDParser::TwoPin s_btp;
    s_btp.from = twopin->from;
    s_btp.to = {twopin->to.x, b_bottom_ay};
    ISPDParser::TwoPin t_btp;
    t_btp.from = twopin->to;
    t_btp.to = {twopin->from.x, b_bottom_ay};

    // ttp
    ISPDParser::TwoPin s_ttp;
    s_ttp.from = twopin->from;
    s_ttp.to = {twopin->to.x, b_top_ay};
    ISPDParser::TwoPin t_ttp;
    t_ttp.from = twopin->to;
    t_ttp.to = {twopin->from.x, b_top_ay};

    // s_x, s_y, t_x, t_y
    int s_x = twopin->from.x - b_left_ax;
    int s_y = twopin->from.y - b_bottom_ay;
    int t_x = twopin->to.x - b_left_ax;
    int t_y = twopin->to.y - b_bottom_ay;

    // fill in d_h, d_v, pi_h, pi_v
    fill_hv(&s_ltp, &s_rtp, &s_btp, &s_ttp, s_x, s_y,
            left_extend, right_extend, bottom_extend, top_extend,
            b_left_ax, b_right_ax, b_bottom_ay, b_top_ay,
            s_d_h, s_d_v, s_pi_h, s_pi_v);
    fill_hv(&t_ltp, &t_rtp, &t_btp, &t_ttp, t_x, t_y,
            left_extend, right_extend, bottom_extend, top_extend,
            b_left_ax, b_right_ax, b_bottom_ay, b_top_ay,
            t_d_h, t_d_v, t_pi_h, t_pi_v);

    // determine u's position
    double min_d = std::numeric_limits<double>::max();
    int u_x, u_y;
    for (int i = 0; i < b_right_ax - b_left_ax + 1; i++) {
        for (int j = 0; j < b_top_ay - b_bottom_ay + 1; j++) {
            double curr_d = std::min(s_d_h[i][j], s_d_v[i][j]) + std::min(t_d_h[i][j], t_d_v[i][j]);
            if (curr_d < min_d) {
                min_d = curr_d;
                u_x = i;
                u_y = j;
            }
        }
    }

    // route from s to u
    auto &policy = (s_d_h[u_x][u_y] < s_d_v[u_x][u_y]) ? s_pi_h : s_pi_v;
    backtrack(twopin, u_x, u_y, s_x, s_y, b_left_ax, b_bottom_ay, policy);

    // backtrack from u to t
    policy = (t_d_h[u_x][u_y] < t_d_v[u_x][u_y]) ? t_pi_h : t_pi_v;
    backtrack(twopin, u_x, u_y, t_x, t_y, b_left_ax, b_bottom_ay, policy);
}

bool overflow(ISPDParser::TwoPin *twopin) {
    for (const auto &[x, y]: twopin->h_used)
        if (hdemand[x][y] > hcapacity[x][y])
            return true;
    for (const auto &[x, y]: twopin->v_used)
        if (vdemand[x][y] > vcapacity[x][y])
            return true;
    return false;
}

void rip_up(ISPDParser::TwoPin *two_pin) {
    auto &net = two_pin->parNet;
    for (const auto &p : two_pin->h_used) {
        auto it = net->hpath.find(p);
        assert(it != net->hpath.end());
        if (it->second == 1) {
            hdemand[p.first][p.second] -= net->minimumWidth + spacing;
            net->hpath.erase(it);
            update_hcost(p.first, p.second);
        } else {
            it->second--;
        }
    }
    for (const auto &p : two_pin->v_used) {
        auto it = net->vpath.find(p);
        assert(it != net->vpath.end());
        if (it->second == 1) {
            vdemand[p.first][p.second] -= net->minimumWidth + spacing;
            net->vpath.erase(it);
            update_vcost(p.first, p.second);
        } else {
            it->second--;
        }
    }
    two_pin->h_used.clear();
    two_pin->v_used.clear();
}

template <algo_t algo>
bool rip_up_and_reroute() {
    static std::mt19937 g;
    static std::vector<std::vector<ISPDParser::TwoPin *>> ovf_twopins;

    for (auto &net : ispdData->nets)
        for (auto &twopin : net->twopin)
            twopin.ripup = false;

    while (true) {
        // determine which twopins to rip-up and reroute
        ovf_twopins.clear();
        bool has_ovf = false;
        for (auto &net : ispdData->nets) {
            bool has_ovf_twopin = false;
            for (auto &twopin : net->twopin) {
                if (overflow(&twopin)) {
                    has_ovf = true;
                    if (!twopin.ripup) {
                        twopin.ripup = true;
                        if (!has_ovf_twopin) {
                            has_ovf_twopin = true;
                            ovf_twopins.emplace_back();
                        }
                        ovf_twopins.back().push_back(&twopin);
                    }
                }
            }
        }
        if (!has_ovf)
            return false;
        if (ovf_twopins.empty())
            break;
        std::shuffle(ovf_twopins.begin(), ovf_twopins.end(), g);

        // start rip-up and reroute
        for (auto &twopins : ovf_twopins) {
            if (interrupted)
                return false;
            for (auto &twopin : twopins)
                rip_up(twopin);
            for (auto &twopin : twopins) {
                if constexpr (algo == L)
                    l_shape_routing(twopin);
                else if constexpr (algo == M)
                    monotonic_routing(twopin);
                else if constexpr (algo == HUM)
                    hum_routing(twopin, twopin->x_extend, twopin->x_extend, twopin->y_extend, twopin->y_extend);
            }
        }
    }

    // end of iteration
    for (int i = 0; i < ispdData->numXGrid - 1; i++) {
        for (int j = 0; j < ispdData->numYGrid; j++) {
            if (hdemand[i][j] > hcapacity[i][j]) {
                hhistory[i][j]++;
                update_hcost(i, j);
            }
        }
    }
    for (int i = 0; i < ispdData->numXGrid; i++) {
        for (int j = 0; j < ispdData->numYGrid - 1; j++) {
            if (vdemand[i][j] > vcapacity[i][j]) {
                vhistory[i][j]++;
                update_vcost(i, j);
            }
        }
    }

    return true;
}

void update_extend() {
    for (auto &net : ispdData->nets) {
        for (auto &twopin : net->twopin) {
            // calculate overflow
            int h_ovf = 0;
            int v_ovf = 0;
            for (auto &[x, y] : twopin.h_used)
                if (hdemand[x][y] > hcapacity[x][y])
                    h_ovf++;
            for (auto &[x, y] : twopin.v_used)
                if (vdemand[x][y] > vcapacity[x][y])
                    v_ovf++;
            // calculate increment
            int x_incr = 0;
            int y_incr = 0;
            if (h_ovf > 0) {
                if (v_ovf > 0) {
                    int quota = expand_increment - 2;
                    double v_ratio = double(v_ovf) / (h_ovf + v_ovf);
                    int x_add = (int)std::lround(quota * v_ratio);
                    int y_add = quota - x_add;
                    x_incr = x_add + 1;
                    y_incr = y_add + 1;
                } else {
                    y_incr = expand_increment;
                }
            } else if (v_ovf > 0) {
                x_incr = expand_increment;
            }
            twopin.x_extend += x_incr;
            twopin.y_extend += y_incr;
        }
    }
    std::cout << "bbox extended" << std::endl;
}

void reset_extend() {
    for (auto &net : ispdData->nets) {
        for (auto &twopin : net->twopin) {
            twopin.x_extend = initial_expand;
            twopin.y_extend = initial_expand;
        }
    }
}

void fill_path() {
    for (auto &net : ispdData->nets) {
        for (auto &twopin : net->twopin) {
            twopin.path.clear();
            for (auto &[x, y] : twopin.h_used)
                twopin.path.emplace_back(x, y, true);
            for (auto &[x, y] : twopin.v_used)
                twopin.path.emplace_back(x, y, false);
        }
    }
}

int get_overflow() {
    int estimated_of = 0;
    for (int i = 0; i < ispdData->numXGrid - 1; i++)
        for (int j = 0; j < ispdData->numYGrid; j++)
            estimated_of += std::max(0, hdemand[i][j] - hcapacity[i][j]);
    for (int i = 0; i < ispdData->numXGrid; i++)
        for (int j = 0; j < ispdData->numYGrid - 1; j++)
            estimated_of += std::max(0, vdemand[i][j] - vcapacity[i][j]);
    return estimated_of;
}

void print_overflow() {
#ifdef VERBOSE
    fill_path();
    LayerAssignment::Graph graph;
    graph.initialLA(*ispdData, 1);
    graph.convertGRtoLA(*ispdData, true);
#endif
    std:: cout << "estimated OF = " << get_overflow() << std::endl;
}

int main(int argc, char **argv) {
    // measure execution time (start)
    time_t start = time(nullptr);

    // register interrupt handler
    struct sigaction act{};
    act.sa_handler = interrupt_handler;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    if (sigaction(SIGTERM, &act, nullptr) == -1)
        std::cout << "sigaction failed for SIGTERM" << std::endl;
    if (sigaction(SIGINT, &act, nullptr) == -1)
        std::cout << "sigaction failed for SIGINT" << std::endl;
    if (sigaction(SIGQUIT, &act, nullptr) == -1)
        std::cout << "sigaction failed for SIGQUIT" << std::endl;
    if (sigaction(SIGALRM, &act, nullptr) == -1)
        std::cout << "sigaction failed for SIGALRM" << std::endl;

    // time limitation
    if (argc >= 4) {
        int sec_limit = int(strtol(argv[3], nullptr, 10)) - 60;
        if (sec_limit >= 1) {
            alarm(sec_limit);
            std::cout << "timer set to " << sec_limit << " seconds" << std::endl;
        }
    }

    // parse
    std::cout << "parse..." << std::endl;
    assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile>");
    std::ifstream fp(argv[1]);
    assert(fp.is_open() && "Failed to open input file");
    ispdData = ISPDParser::parse(fp);
    fp.close();

    // Convert XY coordinates to grid coordinates
    // Delete nets that have more than 1000 sinks
    // Delete nets that have all pins inside the same tile
    std::cout << "remove..." << std::endl;
    ispdData->nets.erase(std::remove_if(ispdData->nets.begin(), ispdData->nets.end(), [&](ISPDParser::Net *net) {

        for (auto &pin : net->pins) {

            int x = (std::get<0>(pin) - ispdData->lowerLeftX) / ispdData->tileWidth;
            int y = (std::get<1>(pin) - ispdData->lowerLeftY) / ispdData->tileHeight;
            int z = std::get<2>(pin) - 1;

            if (std::any_of(net->pin3D.begin(), net->pin3D.end(), [x, y, z](const auto &pin) {
                return pin.x == x && pin.y == y && pin.z == z;
            })) continue;
            net->pin3D.emplace_back(x, y, z);

            if (std::any_of(net->pin2D.begin(), net->pin2D.end(), [x, y](const auto &pin) { 
                return pin.x == x && pin.y == y;
            })) continue;
            net->pin2D.emplace_back(x, y);

        }

        return net->pin3D.size() > 1000 || net->pin2D.size() <= 1;

    }), ispdData->nets.end());
    ispdData->numNet = int(ispdData->nets.size());

    // get minimum spacing in 2d graph
    {
        bool diff = false;
        bool first = true;
        for (const auto &x : ispdData->minimumSpacing) {
            if (!first && x != spacing)
                diff = true;
            spacing = x;
            first = false;
        }
        if (diff)
            std::cout << "WARNING: different minimum spacing between different layers" << std::endl;
    }

    // create grid map
    int default_hcapacity = std::accumulate(ispdData->horizontalCapacity.begin(), ispdData->horizontalCapacity.end(), 0);
    int default_vcapacity = std::accumulate(ispdData->verticalCapacity.begin(), ispdData->verticalCapacity.end(), 0);
    std::cout << "default_hcapacity = " << default_hcapacity << std::endl;
    std::cout << "default_vcapacity = " << default_vcapacity << std::endl;
    hcapacity = std::vector<std::vector<int>>(ispdData->numXGrid - 1, std::vector<int>(ispdData->numYGrid, default_hcapacity));
    hdemand = std::vector<std::vector<int>>(ispdData->numXGrid - 1, std::vector<int>(ispdData->numYGrid, 0));
    hhistory = std::vector<std::vector<int>>(ispdData->numXGrid - 1, std::vector<int>(ispdData->numYGrid, 0));
    hcostarr = std::vector<std::vector<double>>(ispdData->numXGrid - 1, std::vector<double>(ispdData->numYGrid, 0.0));
    hnetcost = std::vector<std::vector<double>>(ispdData->numXGrid - 1, std::vector<double>(ispdData->numYGrid, 0.0));
    vcapacity = std::vector<std::vector<int>>(ispdData->numXGrid, std::vector<int>(ispdData->numYGrid - 1, default_vcapacity));
    vdemand = std::vector<std::vector<int>>(ispdData->numXGrid, std::vector<int>(ispdData->numYGrid - 1, 0));
    vhistory = std::vector<std::vector<int>>(ispdData->numXGrid, std::vector<int>(ispdData->numYGrid - 1, 0));
    vcostarr = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid - 1, 0.0));
    vnetcost = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid - 1, 0.0));

    // initialize d & pi
    d = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid));
    s_d_h = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid));
    s_d_v = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid));
    t_d_h = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid));
    t_d_v = std::vector<std::vector<double>>(ispdData->numXGrid, std::vector<double>(ispdData->numYGrid));
    pi = std::vector<std::vector<dir_t>>(ispdData->numXGrid, std::vector<dir_t>(ispdData->numYGrid));
    s_pi_h = std::vector<std::vector<dir_t>>(ispdData->numXGrid, std::vector<dir_t>(ispdData->numYGrid));
    s_pi_v = std::vector<std::vector<dir_t>>(ispdData->numXGrid, std::vector<dir_t>(ispdData->numYGrid));
    t_pi_h = std::vector<std::vector<dir_t>>(ispdData->numXGrid, std::vector<dir_t>(ispdData->numYGrid));
    t_pi_v = std::vector<std::vector<dir_t>>(ispdData->numXGrid, std::vector<dir_t>(ispdData->numYGrid));

    // capacity adjustment
    for (const auto &ca_ptr : ispdData->capacityAdjs) {
        const ISPDParser::CapacityAdj &ca = *ca_ptr;
        auto g1 = ca.grid1;
        auto g2 = ca.grid2;
        if (g1 > g2)
            std::swap(g1, g2);
        auto [x1, y1, z1] = g1;
        auto [x2, y2, z2] = g2;
        if (x1 + 1 == x2 && y1 == y2 && z1 == z2) {
            hcapacity[x1][y1] -= ispdData->horizontalCapacity[z1 - 1];
            hcapacity[x1][y1] += ca.reducedCapacityLevel;
        } else if (x1 == x2 && y1 + 1 == y2 && z1 == z2) {
            vcapacity[x1][y1] -= ispdData->verticalCapacity[z1 - 1];
            vcapacity[x1][y1] += ca.reducedCapacityLevel;
        } else if (x1 == x2 && y1 == y2 && z2 != z1) {
            // ignore (is this correct?)
        } else {
            std::cout << "WARNING: invalid capacity adjustment" << std::endl;
        }
    }

    // initialize hcostarr & vcostarr
    for (int i = 0; i < ispdData->numXGrid; i++) {
        for (int j = 0; j < ispdData->numYGrid; j++) {
            if (i != ispdData->numXGrid - 1)
                update_hcost(i, j);
            if (j != ispdData->numYGrid - 1)
                update_vcost(i, j);
        }
    }

    // decompose into two-pin nets
    std::cout << "decompose two-pin nets..." << std::endl;
    for (auto &net : ispdData->nets) {
        typedef std::tuple<int, int, int> tiii;

        // construct complete graph
        int num_2d_pins = int(net->pin2D.size());
        int dist[num_2d_pins][num_2d_pins];
        for (int i = 0; i < num_2d_pins; i++) {
            for (int j = i + 1; j < num_2d_pins; j++) {
                int x = net->pin2D[i].x - net->pin2D[j].x;
                int y = net->pin2D[i].y - net->pin2D[j].y;
                int tmp = std::abs(x) + std::abs(y);
                dist[i][j] = tmp;
                dist[j][i] = tmp;
            }
        }

        std::priority_queue<tiii, std::vector<tiii>, std::greater<>> pq; // (cost, pin, ppin)
        std::vector<bool> visited(num_2d_pins, false);
        int visited_cnt = 0;

        // process pin 0
        for (int i = 1; i < num_2d_pins; i++)
            pq.emplace(dist[0][i], i, 0);
        visited[0] = true;
        visited_cnt++;

        // prim's algorithm
        while (visited_cnt < num_2d_pins) {
            assert(!pq.empty() && "pq is empty before MST finished construction");
            auto [cost, pin, ppin] = pq.top();
            pq.pop();
            if (!visited[pin]) {
                // update visit status
                visited[pin] = true;
                visited_cnt++;

                // add two-pin net
                net->twopin.emplace_back();
                ISPDParser::TwoPin &two_pin = net->twopin.back();
                two_pin.parNet = net;
                two_pin.from = net->pin3D[pin];
                two_pin.to = net->pin3D[ppin];

                // update pq
                for (int i = 0; i < num_2d_pins; i++)
                    if (!visited[i])
                        pq.emplace(dist[pin][i], i, pin);
            }
        }

        // sort
        std::sort(net->twopin.begin(), net->twopin.end(), [](const ISPDParser::TwoPin &tp1, const ISPDParser::TwoPin &tp2) {
            int len_1 = std::abs(tp1.from.x - tp1.to.x) + std::abs(tp1.from.y - tp1.to.y);
            int len_2 = std::abs(tp2.from.x - tp2.to.x) + std::abs(tp2.from.y - tp2.to.y);
            return len_1 < len_2;
        });
    }

    // pattern routing
    std::cout << "pattern routing..." << std::endl;
    {
        std::vector<std::tuple<int, ISPDParser::TwoPin *>> len_twopins;
        for (const auto &net : ispdData->nets) {
            for (auto &twopin : net->twopin) {
                int x = twopin.from.x - twopin.to.x;
                int y = twopin.from.y - twopin.to.y;
                int len = std::abs(x) + std::abs(y);
                len_twopins.emplace_back(len, &twopin);
            }
        }
        std::sort(len_twopins.begin(), len_twopins.end());
        for (auto &[len, twopin] : len_twopins)
            l_shape_routing(twopin);
        print_overflow();
    }

    // rip-up & reroute with L
    std::cout << "rip-up and reroute with pattern routing..." << std::endl;
    for (int i = 0; i < 2; i++) {
        if (!rip_up_and_reroute<L>())
            break;
        print_overflow();
    }

    // rip-up & reroute with M
    std::cout << "rip-up and reroute with monotonic routing..." << std::endl;
    reset_history();
    for (int i = 0; i < 2; i++) {
        if (!rip_up_and_reroute<M>())
            break;
        print_overflow();
    }

    // rip-up & reroute with HUM
    std::cout << "rip-up and reroute with HUM ..." << std::endl;
    reset_history();
    reset_extend();
    for (int i = 0; i < 100; i++) {
        if (!rip_up_and_reroute<HUM>())
            break;
        std::cout << "iter " << i << ": ";
        print_overflow();
        update_extend();
    }

    // fill path
    std::cout << "filling path..." << std::endl;
    fill_path();

#ifdef VERBOSE
    // output debug file
    std::cout << "output debug file..." << std::endl;
    std::ofstream horizontal("plot/horizontal.txt");
    for (int j = ispdData->numYGrid - 1; j >= 0; j--) {
        for (int i = 0; i < ispdData->numXGrid - 1; i++)
            horizontal << hdemand[i][j] - hcapacity[i][j] << " ";
        horizontal << std::endl;
    }
    std::ofstream vertical("plot/vertical.txt");
    for (int j = ispdData->numYGrid - 2; j >= 0; j--) {
        for (int i = 0; i < ispdData->numXGrid; i++)
            vertical << vdemand[i][j] - vcapacity[i][j] << " ";
        vertical << std::endl;
    }
    std::ofstream net_file("plot/net.txt");
    net_file << ispdData->nets.size() << std::endl;
    for (const auto &net : ispdData->nets) {
        net_file << net->name << " " << net->pin2D.size() << " " << net->hpath.size() << " " << net->vpath.size() << std::endl;
        for (const auto &[x, y, z] : net->pin2D)
            net_file << x << " " << y << std::endl;
        for (const auto &[key, value] : net->hpath)
            net_file << key.first << " " << key.second << std::endl;
        for (const auto &[key, value] : net->vpath)
            net_file << key.first << " " << key.second << std::endl;
    }
#endif

    // Assign routing layers to the two-pin net
    std::cout << "layer assignment..." << std::endl;
    LayerAssignment::Graph graph;
    graph.initialLA(*ispdData, 1);
    graph.convertGRtoLA(*ispdData, true);
    graph.COLA(true);

    // Output result
    graph.output3Dresult(argv[2]);

    // delete ispdData
    delete ispdData;

    // measure execution time (end)
    time_t end = time(nullptr);
    std::cout << "total execution time = " << end - start << "s" << std::endl;
    return 0;
}
