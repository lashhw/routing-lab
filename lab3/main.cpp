#include "router.hpp"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>

int main(int argc, char **argv){
    assert(argc >= 2 && "Usage: ./Lab3 <inputFile> <ouptFile>");
    Router router;

    //Part 1: Parse the input file
    std::ifstream specStream(argv[1]);
    assert(specStream.is_open() && "Failed to open spec file");
    router.readCircuitSpec(specStream);
    specStream.close();

    //Part 2: initial the graph 
    router.generateGraph();

    //Part 3: Generate the clause file and put into the solver
    std::ofstream clauseStream("clause.sat");
    assert(clauseStream.is_open() && "Failed to open clause file");
    router.generateClauses(clauseStream);
    clauseStream.close();

    //Though the loop is not necessary, it can help you try different ways to solve the problem,
    //like monotoinc route or smaller routing space
    //The loop would be break when the readSolverResult return true, and you can decide when to break 
    for(int i = 0; ; i++){
        //Part 4: Run the solver 
        std::string command = router.getSysCommand(i);
        //Please use the following solvers
        if(command.find("./open-wbo") != 0 && command.find("glsol") != 0){
            std::cerr << "Error: solver not supported" << std::endl;
            return 1;
        }
        std::system(command.c_str());

        //Part 5: read the solver's result
        std::ifstream solverResultStream("sat_result.txt");
        assert(solverResultStream.is_open() && "Failed to open solver result file");
        bool flag = router.readSolverResult(solverResultStream, i);
        solverResultStream.close();
        if(flag) break;
    }

    //Part 6: Post-process you can refind the path or do something else
    //Or, you can just ignore this part
    router.postProcess();

    //Part 7: Print the result
    std::ofstream outFile(argv[2]);
    assert(outFile.is_open() && "Failed to open output file");
    router.printRoutingResult(outFile);
    outFile.close();

    return 0;
}