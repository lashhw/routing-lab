#include "ispdData.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <numeric>
#include <algorithm>

int main(int argc, char **argv) {
    std::cout << "parse..." << std::endl;
    assert(argc >= 3 && "Usage: ./router <inputFile> <outputFile>");
    std::ifstream fp(argv[1]);
    assert(fp.is_open() && "Failed to open input file");
    ISPDParser::ispdData *ispdData = ISPDParser::parse(fp);
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

    // get spacing in 2d graph
    int spacing;
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

    // get width
    int width;
    {
        bool diff = false;
        bool first = true;
        for (const auto &x : ispdData->nets) {
            if (!first && x->minimumWidth != width)
                diff = true;
            width = x->minimumWidth;
            first = false;
        }
        if (diff)
            std::cout << "WARNING: different width" << std::endl;
    }

    // calculate default capacity
    int default_hcapacity = std::accumulate(ispdData->horizontalCapacity.begin(), ispdData->horizontalCapacity.end(), 0);
    int default_vcapacity = std::accumulate(ispdData->verticalCapacity.begin(), ispdData->verticalCapacity.end(), 0);
    std::vector<std::vector<int>> hcapacity(ispdData->numXGrid - 1, std::vector<int>(ispdData->numYGrid, default_hcapacity));
    std::vector<std::vector<int>> hdemand(ispdData->numXGrid - 1, std::vector<int>(ispdData->numYGrid, 0));
    std::vector<std::vector<int>> vcapacity(ispdData->numXGrid, std::vector<int>(ispdData->numYGrid - 1, default_vcapacity));
    std::vector<std::vector<int>> vdemand(ispdData->numXGrid, std::vector<int>(ispdData->numYGrid - 1, 0));

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

    // parse
    std::ifstream f(argv[2]);
    std::string str;
    int wlen2d = 0;
    auto net_it = ispdData->nets.begin();
    while (std::getline(f, str)) {
        while (std::getline(f, str)) {
            if (str[0] == '!')
                break;
            int x1, y1, z1, x2, y2, z2;
            sscanf(str.c_str(), "(%d,%d,%d)-(%d,%d,%d)", &x1, &y1, &z1, &x2, &y2, &z2);
            x1 = (x1 - ispdData->lowerLeftX) / ispdData->tileWidth;
            x2 = (x2 - ispdData->lowerLeftX) / ispdData->tileWidth;
            y1 = (y1 - ispdData->lowerLeftY) / ispdData->tileHeight;
            y2 = (y2 - ispdData->lowerLeftY) / ispdData->tileHeight;
            if (x1 > x2)
                std::swap(x1, x2);
            if (y1 > y2)
                std::swap(y1, y2);
            if (y1 == y2 && z1 == z2) {
                for (int x = x1; x < x2; x++) {
                    auto p = std::make_pair(x, y1);
                    (*net_it)->hpath[p]++;
                    hdemand[x][y1] += width + spacing;
                    wlen2d += 1;
                }
            } else if (x1 == x2 && z1 == z2) {
                for (int y = y1; y < y2; y++) {
                    auto p = std::make_pair(x1, y);
                    (*net_it)->vpath[p]++;
                    vdemand[x1][y] += width + spacing;
                    wlen2d += 1;
                }
            } else if (x1 == x2 && y1 == y2 && z2 != z1) {
                // ignore
            } else {
                std::cout << "WARNING: invalid format" << std::endl;
            }
        }
        net_it++;
    }

    // print TOF/Wlen2D
    int overflow = 0;
    for (int i = 0; i < ispdData->numXGrid - 1; i++)
        for (int j = 0; j < ispdData->numYGrid; j++)
            overflow += std::max(0, hdemand[i][j] - hcapacity[i][j]);
    for (int i = 0; i < ispdData->numXGrid; i++)
        for (int j = 0; j < ispdData->numYGrid - 1; j++)
            overflow += std::max(0, vdemand[i][j] - vcapacity[i][j]);
    printf("overflow = %d\n", overflow);
    printf("Wlen2d = %d\n", wlen2d);

    // output
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
}