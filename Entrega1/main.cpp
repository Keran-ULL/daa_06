/**
 * @file   main.cpp
 * @brief  Programa cliente para el MS-CFLP-CI.
 *
 * Permite al usuario:
 *   1. Ejecutar el Algoritmo Voraz sobre una instancia individual.
 *   2. Ejecutar GRASP sobre una instancia individual.
 *   3. Lanzar el benchmark sobre wlp01-wlp08 (ambos algoritmos)
 *      y guardar los resultados en un fichero de texto tabulado.
 *
 * @author  Keran Miranda González
 * @version 1.0
 */

#include "Helper.h"
#include "MSCFLPInstance.h"
#include "MSCFLPSolution.h"
#include "Greedy.h"
#include "GRASP.h"
#include "BenchmarkRunner.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>

static void printSolution(const MSCFLPSolution& sol,
                           const std::string& algoName,
                           double cpuMs)
{
    Helper::printSeparator();
    std::cout << "  Algoritmo : " << algoName << "\n"
              << std::fixed << std::setprecision(2)
              << "  |Jopen|   : " << sol.countOpenFacilities() << "\n"
              << "  C. Fijo   : " << sol.getFixedCost()        << "\n"
              << "  C. Asig.  : " << sol.getTransportCost()    << "\n"
              << "  C. Total  : " << sol.getTotalCost()        << "\n"
              << "  Incomp.   : " << sol.countIncompatibilityViolations() << "\n"
              << "  Factible  : " << (sol.isFeasible() ? "Sí" : "No") << "\n"
              << std::setprecision(4)
              << "  CPU (s)   : " << cpuMs / 1000.0 << "\n";
    Helper::printSeparator();
}

static void runGreedySingle() {
    std::string path = Helper::askInstancePath();
    int slack        = Helper::askGreedySlack();

    MSCFLPInstance inst;
    try {
        inst.load(path);
    } catch (const std::exception& e) {
        std::cout << "\n  [ERROR] " << e.what() << "\n";
        return;
    }

    Helper::printRunning("Voraz (k=" + std::to_string(slack) + ")", path);

    Greedy greedy(inst, slack);
    auto sol = greedy.run();
    auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

    printSolution(mSol, greedy.getName(), greedy.getElapsedMs());
}

static void runGRASPSingle() {
    std::string path    = Helper::askInstancePath();
    Helper::GRASPParams p = Helper::askGRASPParams();

    MSCFLPInstance inst;
    try {
        inst.load(path);
    } catch (const std::exception& e) {
        std::cout << "\n  [ERROR] " << e.what() << "\n";
        return;
    }

    Helper::printRunning("GRASP", path);

    auto strategy = (p.strategy == Helper::ImprovStrategy::BEST_IMPROVEMENT)
                    ? ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT
                    : ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT;

    GRASP grasp(inst, p.iterations, p.alpha, p.beta, p.seed, strategy);
    auto sol = grasp.run();
    auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

    printSolution(mSol, grasp.getName(), grasp.getElapsedMs());
}

static void runBenchmark() {
    Helper::BenchmarkParams bp = Helper::askBenchmarkParams();

    BenchmarkRunner::Config cfg;
    cfg.instancesDir = bp.instancesDir;
    cfg.outputFile   = bp.outputFile;
    cfg.graspIter    = bp.graspIter;
    cfg.graspRuns    = bp.graspRuns;
    cfg.lrcSizes     = {2, 3};
    cfg.seed         = 42;

    BenchmarkRunner runner(cfg);
    try {
        runner.runAll();
    } catch (const std::exception& e) {
        std::cout << "\n  [ERROR] " << e.what() << "\n";
    }
}

int main() {
    Helper::printBanner();

    bool running = true;
    while (running) {
        Helper::Algorithm algo = Helper::menuAlgorithm();

        switch (algo) {

        case Helper::Algorithm::QUIT:
            running = false;
            std::cout << "\n  ¡Hasta luego!\n\n";
            break;

        case Helper::Algorithm::GREEDY: {
            Helper::Mode mode = Helper::menuMode("Voraz");
            switch (mode) {
            case Helper::Mode::SINGLE_INSTANCE:
                runGreedySingle();
                Helper::pressEnterToContinue();
                break;
            case Helper::Mode::BENCHMARK:
                runBenchmark();
                Helper::pressEnterToContinue();
                break;
            case Helper::Mode::BACK:
                break;
            }
            break;
        }

        case Helper::Algorithm::GRASP: {
            Helper::Mode mode = Helper::menuMode("GRASP");
            switch (mode) {
            case Helper::Mode::SINGLE_INSTANCE:
                runGRASPSingle();
                Helper::pressEnterToContinue();
                break;
            case Helper::Mode::BENCHMARK:
                runBenchmark();
                Helper::pressEnterToContinue();
                break;
            case Helper::Mode::BACK:
                break;
            }
            break;
        }
        }
    }

    return 0;
}
