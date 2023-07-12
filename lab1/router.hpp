#ifndef AARP_LABS_ROUTER_HPP
#define AARP_LABS_ROUTER_HPP

#include <algorithm>
#include <tuple>
#include <deque>
#include <set>
#include <unordered_set>

struct Router {
    enum edge_t {
        TOP, BOTTOM, BOTH
    };

    int mjl;
    int snc;
    int ct;  // centralize threshold
    const std::vector<int> &top;
    const std::vector<int> &bottom;
    std::ofstream &out_file;

    int num_nets;
    std::vector<std::vector<int>> h;
    std::vector<std::set<std::tuple<int, int, int>>> v;  // col -> [(top_track, bottom_track, net_id), ...]
    std::vector<std::set<int>> dangle;
    std::vector<std::deque<std::pair<edge_t, int>>> next;

    Router(int init_cw,
           int mjl,
           int snc,
           int ct,
           const std::vector<int> &top,
           const std::vector<int> &bottom,
           std::ofstream &out_file) : mjl(mjl),
                                      snc(snc),
                                      top(top),
                                      ct(ct),
                                      bottom(bottom),
                                      out_file(out_file) {
        num_nets = std::max(*std::max_element(top.begin(), top.end()),
                            *std::max_element(bottom.begin(), bottom.end()));

        h.resize(init_cw, std::vector<int>(top.size(), 0));
        v.resize(top.size(), std::set<std::tuple<int, int, int>>());

        dangle.resize(num_nets + 1, std::set<int>());
        for (int i = 0; i < h.size(); i++)
            dangle[0].insert(i);

        next.resize(num_nets + 1);
        for (int i = 0; i < top.size(); i++) {
            if (top[i] == bottom[i]) {
                next[top[i]].emplace_back(BOTH, i);
            } else {
                next[top[i]].emplace_back(TOP, i);
                next[bottom[i]].emplace_back(BOTTOM, i);
            }
        }
    }

    int route() {
        for (int i = 0; i < top.size(); i++) {
            update_next(i);
            auto [top_resolved, bottom_resolved] = rule_1(i);
            rule_2(i);
            rule_3(i);
            rule_4(i);
            rule_4a(i);
            rule_5(i, top_resolved, bottom_resolved);
            rule_6(i);
        }

        for (int i = int(top.size()); ; i++) {
            v.emplace_back();
            if (!split_over(i))
                break;
            for (auto &track : h)
                track.push_back(0);
            rule_6(i);
        }

        std::cout << "routing finished. track count = " << h.size() << std::endl;
        return h.size();
    }

    void output_to_file() {
        std::vector<std::tuple<int, int, int>> h_lines[num_nets + 1];
        for (int i = 0; i < h.size(); i++) {
            int last_net = 0;
            int last_left = -1;
            for (int j = 0; j < h[0].size(); j++) {
                if (h[i][j] != last_net) {
                    if (last_net != 0)
                        h_lines[last_net].emplace_back(last_left, h.size() - i, j);
                    last_net = h[i][j];
                    last_left = j;
                }
            }
            if (last_net != 0)
                h_lines[last_net].emplace_back(last_left, h.size() - i, h[0].size());
        }

        std::vector<std::tuple<int, int, int>> v_lines[num_nets + 1];
        for (int i = 0; i < v.size(); i++)
            for (auto &v_line : v[i])
                v_lines[std::get<2>(v_line)].emplace_back(i, h.size() - std::get<0>(v_line), h.size() - std::get<1>(v_line));

        for (int i = 1; i <= num_nets; i++) {
            if (h_lines[i].empty() && v_lines[i].empty())
                continue;
            out_file << ".begin " << i << std::endl;
            for (auto [lef_x, lef_y, rig_x] : h_lines[i])
                out_file << ".H " << lef_x << " " << lef_y << " " << rig_x << std::endl;
            for (auto [bot_x, top_y, bot_y] : v_lines[i])
                out_file << ".V " << bot_x << " " << bot_y << " " << top_y << std::endl;
            out_file << ".end" << std::endl;
        }
    }

    void update_next(int col) {
        next[top[col]].pop_front();
        if (top[col] != bottom[col])
            next[bottom[col]].pop_front();
    }

    std::pair<bool, bool> rule_1(int col) {
        // link directly
        if (top[col] != 0 && top[col] == bottom[col] && next[top[col]].empty()) {
            v[col].emplace(-1, h.size(), top[col]);
            return {true, true};
        }

        int top_target = int(h.size());
        if (!dangle[0].empty())
            top_target = std::min(top_target, *dangle[0].begin());
        if (!dangle[top[col]].empty())
            top_target = std::min(top_target, *dangle[top[col]].begin());

        int bottom_target = -1;
        if (!dangle[0].empty())
            bottom_target = std::max(bottom_target, *dangle[0].rbegin());
        if (!dangle[bottom[col]].empty())
            bottom_target = std::max(bottom_target, *dangle[bottom[col]].rbegin());

        // conflict
        if (top_target != int(h.size()) && bottom_target != -1 &&
            top[col] != bottom[col] && top_target >= bottom_target) {
            if (top_target + 1 < h.size() - bottom_target)
                bottom_target = -1;
            else
                top_target = int(h.size());
        }

        bool top_resolved;
        if (top[col] == 0) {
            top_resolved = true;
        } else if (top_target != int(h.size())) {
            top_resolved = true;
            if (!dangle[0].empty() && top_target == *dangle[0].begin()) {
                dangle[0].erase(top_target);
                dangle[top[col]].insert(top_target);
            }
            v[col].emplace(-1, top_target, top[col]);
        } else {
            top_resolved = false;
        }

        bool bottom_resolved;
        if (bottom[col] == 0) {
            bottom_resolved = true;
        } else if (bottom_target != -1) {
            bottom_resolved = true;
            if (!dangle[0].empty() && bottom_target == *dangle[0].rbegin()) {
                dangle[0].erase(bottom_target);
                dangle[bottom[col]].insert(bottom_target);
            }
            v[col].emplace(bottom_target, h.size(), bottom[col]);
        } else {
            bottom_resolved = false;
        }

        return {top_resolved, bottom_resolved};
    }

    void rule_2(int col) {
        int merge_top_boundary = 0;
        int merge_bottom_boundary = int(h.size()) - 1;
        for (auto &v_line : v[col]) {
            if (std::get<0>(v_line) == -1)
                merge_top_boundary = std::get<1>(v_line);
            else if (std::get<1>(v_line) == h.size())
                merge_bottom_boundary = std::get<0>(v_line);
        }

        std::vector<std::tuple<int, int, int>> merge_candidates;  // (net_id, top_track, bottom_track)
        for (int i = 1; i <= num_nets; i++) {
            if (dangle[i].size() > 1) {
                int last = *dangle[i].begin();
                for (auto it = std::next(dangle[i].begin()); it != dangle[i].end(); it++) {
                    if (last >= merge_top_boundary && *it <= merge_bottom_boundary)
                        merge_candidates.emplace_back(i, last, *it);
                    last = *it;
                }
            }
        }

        std::vector<int> best_sels;
        for (int sel = 1; sel < (1 << merge_candidates.size()); sel++) {
            std::vector<std::tuple<int, int, int>> selected_lines;  // [(top_track, bottom_track, net_id), ...]
            for (int j = 0; j < merge_candidates.size(); j++) {
                if (sel & (1 << j)) {
                    auto [net_id, top_track, bottom_track] = merge_candidates[j];
                    selected_lines.emplace_back(top_track, bottom_track, net_id);
                }
            }
            std::sort(selected_lines.begin(), selected_lines.end());
            bool success = true;
            for (int j = 1; j < selected_lines.size(); j++) {
                auto [curr_top_track, curr_bottom_track, curr_net_id] = selected_lines[j];
                auto [prev_top_track, prev_bottom_track, prev_net_id] = selected_lines[j - 1];
                if (curr_net_id != prev_net_id && curr_top_track <= prev_bottom_track) {
                    success = false;
                    break;
                }
            }
            if (success)
                best_sels.push_back(sel);
        }

        std::vector<std::tuple<int, int, int>> stats;  // [(num_free_tracks, uncollapsed_edge_dist, jog_length_sum), ...]
        for (auto &best_sel : best_sels) {
            std::unordered_set<int> free_tracks;
            int uncollapsed_edge_dist = int(h.size());
            int jog_length_sum = 0;
            for (int i = 0; i < merge_candidates.size(); i++) {
                auto [net_id, top_track, bottom_track] = merge_candidates[i];
                if (best_sel & (1 << i)) {
                    free_tracks.insert(top_track);
                    if (next[net_id].empty())
                        free_tracks.insert(bottom_track);
                    jog_length_sum += bottom_track - top_track;
                } else {
                    uncollapsed_edge_dist = std::min(uncollapsed_edge_dist,
                                                     std::min(top_track + 1, int(h.size()) - bottom_track));
                }
            }
            stats.emplace_back(free_tracks.size(), uncollapsed_edge_dist, jog_length_sum);
        }

        if (!best_sels.empty()) {
            size_t best_permute_idx = std::max_element(stats.begin(), stats.end()) - stats.begin();
            for (int i = 0; i < merge_candidates.size(); i++) {
                if (best_sels[best_permute_idx] & (1 << i)) {
                    auto [net_id, top_track, bottom_track] = merge_candidates[i];
                    dangle[net_id].erase(top_track);
                    dangle[0].insert(top_track);
                    v[col].emplace(top_track, bottom_track, net_id);
                }
            }
        }
    }

    void lower_track(int col, int net_id, int old_track, int lowest_track) {
        for (auto [line_top_track, line_bottom_track, line_net_id] : v[col])
            if (line_net_id != net_id && line_bottom_track > old_track)
                lowest_track = std::min(lowest_track, line_top_track - 1);
        for (auto it = dangle[0].rbegin(); it != dangle[0].rend(); it++) {
            int target_track = *it;
            if (old_track < target_track && target_track <= lowest_track && target_track - old_track >= mjl) {
                v[col].emplace(old_track, target_track, net_id);
                dangle[0].erase(target_track);
                dangle[0].insert(old_track);
                dangle[net_id].erase(old_track);
                dangle[net_id].insert(target_track);
                break;
            }
        }
    }

    void raise_track(int col, int net_id, int old_track, int highest_track) {
        for (auto [line_top_track, line_bottom_track, line_net_id] : v[col])
            if (line_net_id != net_id && line_top_track < old_track)
                highest_track = std::max(highest_track, line_bottom_track + 1);
        for (int target_track : dangle[0]) {
            if (highest_track <= target_track && target_track < old_track && old_track - target_track >= mjl) {
                v[col].emplace(target_track, old_track, net_id);
                dangle[0].erase(target_track);
                dangle[0].insert(old_track);
                dangle[net_id].erase(old_track);
                dangle[net_id].insert(target_track);
                break;
            }
        }
    }

    void rule_3(int col) {
        for (int i = 1; i < num_nets; i++) {
            if (dangle[i].size() >= 2) {
                // lower highest track
                lower_track(col, i, *dangle[i].begin(), int(h.size()) - 1);
                // raise lowest track
                raise_track(col, i, *dangle[i].rbegin(), 0);
            }
        }
    }

    void rule_4(int col) {
        std::vector<std::tuple<int, int, int, bool>> candidates;  // (target_dist, net_id, old_track, is_lower)
        for (int i = 1; i < num_nets; i++) {
            if (dangle[i].size() == 1 && !next[i].empty()) {
                int old_track = *dangle[i].begin();
                if (next[i][0].first == TOP) {
                    bool rising = true;
                    for (int j = 1; j < next[i].size() && next[i][j].second - next[i][0].second <= snc; j++)
                        if (next[i][j].first != TOP)
                            rising = false;
                    if (rising && next[i][0].second - col <= ct)
                        candidates.emplace_back(next[i][0].second, i, old_track, false);
                } else if (next[i][0].first == BOTTOM) {
                    bool falling = true;
                    for (int j = 1; j < next[i].size() && next[i][j].second - next[i][0].second <= snc; j++)
                        if (next[i][j].first != BOTTOM)
                            falling = false;
                    if (falling && next[i][0].second - col <= ct)
                        candidates.emplace_back(next[i][0].second, i, old_track, true);
                }
            }
        }

        std::sort(candidates.begin(), candidates.end());
        for (auto [target_dist, net_id, old_track, is_lower] : candidates) {
            if (is_lower)
                lower_track(col, net_id, old_track, int(h.size()) - 1);
            else
                raise_track(col, net_id, old_track, 0);
        }
    }

    void rule_4a(int col) {
        for (int net_id = 1; net_id < num_nets; net_id++) {
            if (dangle[net_id].size() == 1 && !next[net_id].empty() && next[net_id][0].second - col > ct) {
                int old_track = *dangle[net_id].begin();
                int center_track_u = (int(h.size()) - 1) / 2;
                int center_track_l = int(h.size()) / 2;
                bool on_top_half = old_track <= center_track_u;
                if (on_top_half)
                    lower_track(col, net_id, old_track, center_track_u);
                else
                    raise_track(col, net_id, old_track, center_track_l);
            }
        }
    }

    void rule_5(int col, bool top_resolved, bool bottom_resolved) {
        auto fixup = [&](int new_track) {
            for (int i = 0; i <= col; i++) {
                std::vector<std::tuple<int, int, int>> new_lines;
                for (auto it = v[i].begin(); it != v[i].end(); ) {
                    bool updated = false;
                    auto [top_track, bottom_track, net_id] = *it;
                    if (top_track >= new_track) {
                        updated = true;
                        top_track++;
                    }
                    if (bottom_track >= new_track) {
                        updated = true;
                        bottom_track++;
                    }
                    if (updated) {
                        new_lines.emplace_back(top_track, bottom_track, net_id);
                        it = v[i].erase(it);
                    } else {
                        it++;
                    }
                }
                v[i].insert(new_lines.begin(), new_lines.end());
            }
            for (auto &d : dangle) {
                std::vector<int> new_d;
                for (auto it = d.begin(); it != d.end(); ) {
                    if (*it >= new_track) {
                        new_d.push_back(*it + 1);
                        it = d.erase(it);
                    } else {
                        it++;
                    }
                }
                d.insert(new_d.begin(), new_d.end());
            }
        };

        if (!top_resolved) {
            int best = int(h.size()) / 2;
            if (!v[col].empty() && std::get<0>(*v[col].begin()) < best)
                best = std::get<0>(*v[col].begin());
            fixup(best);
            h.insert(h.begin() + best, std::vector<int>(h[0].size(), 0));
            v[col].emplace(-1, best, top[col]);
            dangle[top[col]].insert(best);
        }

        if (!bottom_resolved) {
            int best = int(h.size() + 1) / 2;
            if (!v[col].empty() && std::get<1>(*v[col].rbegin()) >= best)
                best = std::get<1>(*v[col].rbegin()) + 1;
            fixup(best);
            h.insert(h.begin() + best, std::vector<int>(h[0].size(), 0));
            v[col].emplace(best, h.size(), bottom[col]);
            dangle[bottom[col]].insert(best);
        }
    }

    void rule_6(int col) {
        // update dangle status of complete nets
        for (int i = 1; i <= num_nets; i++) {
            if (dangle[i].size() == 1 && next[i].empty()) {
                dangle[0].insert(*dangle[i].begin());
                dangle[i].clear();
            }
        }

        for (int i = 1; i <= num_nets; i++)
            for (auto &track : dangle[i])
                h[track][col] = i;
    }

    // return false when no merge needed
    bool split_over(int col) {
        for (int i = 1; i <= num_nets; i++) {
            if (!dangle[i].empty()) {
                v[col].emplace(*dangle[i].begin(), *dangle[i].rbegin(), i);
                dangle[i].clear();
                return true;
            }
        }
        return false;
    }
};

#endif //AARP_LABS_ROUTER_HPP
