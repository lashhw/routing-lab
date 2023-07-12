#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "router.hpp"

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cout << "usage: ./Lab1 [input.txt] [output.txt]" << std::endl;
        return 1;
    }

    std::ifstream in_file(argv[1]);
    std::ofstream out_file(argv[2]);
    if (!in_file) {
        std::cout << "failed to open input file" << std::endl;
        return 1;
    }
    if (!out_file) {
        std::cout << "failed to open output file" << std::endl;
        return 1;
    }

    int init_cw = 20;
    int mjl = 6;
    int snc = 2;
    int ct = 35;
    if (argc == 7) {
        init_cw = std::stoi(argv[3]);
        mjl = std::stoi(argv[4]);
        snc = std::stoi(argv[5]);
        ct = std::stoi(argv[6]);
    }

    std::vector<int> top;
    {
        std::string line;
        std::getline(in_file, line);
        std::stringstream ss(line);
        for (int tmp; ss >> tmp; )
            top.push_back(tmp);
    }

    std::vector<int> bottom;
    {
        std::string line;
        std::getline(in_file, line);
        std::stringstream ss(line);
        for (int tmp; ss >> tmp; )
            bottom.push_back(tmp);
    }

    Router router(init_cw, mjl, snc, ct, top, bottom, out_file);
    router.route();
    router.output_to_file();
}
