#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <array>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        result += buffer.data();
    return result;
}

int main() {
    for (int mjl = 1; mjl <= 10; mjl++) {
        for (int snc = 0; snc <= 40; snc++) {
            for (int ct = 1; ct <= 70; ct++) {
                for (int init_cw = 1; init_cw <= 24; init_cw++) {
                    std::string cmd = "./Lab1 Deutsch_difficult.txt result.txt " +
                                      std::to_string(init_cw) + " " +
                                      std::to_string(mjl) + " " +
                                      std::to_string(snc) + " " +
                                      std::to_string(ct);
                    exec(cmd.c_str());
                    std::string result = exec("./verifier result.txt Deutsch_difficult.txt");
                    bool success = result.find("All signals are connected successfully.") != std::string::npos;
                    if (success) {
                        int track_count = 0;
                        for (size_t i = result.find("Track count: ") + 13; result[i] != '\n'; i++)
                            track_count = track_count * 10 + result[i] - '0';
                        if (track_count > 24)
                            continue;
                        int wl = 0;
                        for (size_t i = result.find("Total wirelength is ") + 20; result[i] != '\n'; i++)
                            wl = wl * 10 + result[i] - '0';
                        std::cout << init_cw << " " <<
                                     mjl << " " <<
                                     snc << " " <<
                                     ct << " " <<
                                     track_count << " " <<
                                     wl << std::endl;
                    }
                }
            }
        }
    }
}