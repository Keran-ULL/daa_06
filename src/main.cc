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
 * Compilación (C++17 requerido por std::filesystem):
 *   g++ -std=c++17 -O2 -o mscflpci main.cpp
 *
 * @author  DAA 2025-2026
 * @version 1.0
 */

#include "Entrega1/Helper.h"
#include "Instancia/MSCFLPInstance.h"
#include "Solucion/MSCFLPSolution.h"
#include "Algoritmo/Greedy.h"
#include "Entrega1/GRASP.h"
#include "Entrega1/Gvnsrl.h"
#include "Entrega1/BenchmarkRunner.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// Imprime en pantalla el resumen de una solución MS-CFLP-CI
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// Flujo: Voraz sobre instancia individual
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// Flujo: GRASP sobre instancia individual
// ─────────────────────────────────────────────────────────────────────────────
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

    auto lsChoice = static_cast<LocalSearchChoice>(static_cast<int>(p.lsChoice));

    GRASP grasp(inst, p.iterations, p.alpha, p.beta, p.seed, strategy,
                lsChoice, 3,
                p.rlAlpha, p.rlEpsilon, p.maxSinMejora, p.maxTotalIter);
    auto sol = grasp.run();
    auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

    printSolution(mSol, grasp.getName(), grasp.getElapsedMs());
}

// ─────────────────────────────────────────────────────────────────────────────
// Flujo: GVNS-RL sobre instancia individual
// ─────────────────────────────────────────────────────────────────────────────
static void runGVNSRLSingle() {
    std::string path = Helper::askInstancePath();
    Helper::GVNSRLParams p = Helper::askGVNSRLParams();

    MSCFLPInstance inst;
    try { inst.load(path); }
    catch (const std::exception& e) {
        std::cout << "\n  [ERROR] " << e.what() << "\n";
        return;
    }

    Helper::printRunning("GVNS-RL", path);

    GVNSRL gvns(inst, p.graspIter, p.alpha, p.beta, p.seed,
                p.maxGVNSIter, p.shakingK,
                p.rlAlpha, p.rlEpsilon, p.maxSinMejora, p.maxTotalRL,
                p.epsilonDecay, p.propReward, p.qLogFile,
                static_cast<GVNSImprovMode>(p.improvMode));
    auto sol = gvns.run();
    auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

    printSolution(mSol, gvns.getName(), gvns.getElapsedMs());

    // Análisis de selección de LS
    const auto& counts = gvns.getSelectionCounts();
    int total = counts[0]+counts[1]+counts[2]+counts[3];
    if (total > 0) {
        std::cout << "\n  --- Análisis de selección LS ---\n";
        const std::string nombres[] = {"IncompatElim","Shift","SwapInstalaciones","SwapClientes"};
        for (int k = 0; k < 4; ++k)
            std::cout << "  " << std::left << std::setw(20) << nombres[k]
                      << ": " << counts[k] << " veces ("
                      << std::fixed << std::setprecision(1)
                      << (counts[k]*100.0/total) << "%)\n";
    }
    if (!p.qLogFile.empty())
        std::cout << "\n  [OK] Evolución Q guardada en '" << p.qLogFile << "'\n";
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
    cfg.lsChoice     = static_cast<LocalSearchChoice>(static_cast<int>(bp.lsChoice));
    cfg.rlAlpha      = bp.rlAlpha;
    cfg.rlEpsilon    = bp.rlEpsilon;
    cfg.maxSinMejora = bp.maxSinMejora;
    cfg.maxTotalIter = bp.maxTotalIter;

    BenchmarkRunner runner(cfg);
    try {
        runner.runAll();
    } catch (const std::exception& e) {
        std::cout << "\n  [ERROR] " << e.what() << "\n";
    }
}

static void runBenchmarkGVNS() {
    std::cout << "\n  --- Parámetros Benchmark GVNS-RL ---\n";
    std::string dir  = Helper::readString("  Directorio de instancias (ej: instances/): ");
    if (!dir.empty() && dir.back() != '/') dir += '/';
    std::string out  = Helper::readString("  Fichero de salida (ej: resultados_gvns.txt): ");

    BenchmarkRunner::Config cfg;
    cfg.instancesDir = dir;
    cfg.outputFile   = out;
    cfg.graspIter    = Helper::readInt("  Iteraciones GRASP inicial (recomendado: 10): ", 1, 1000);
    cfg.gvnsIter     = Helper::readInt("  Iteraciones GVNS (recomendado: 200-500): ", 1, 100000);
    cfg.shakingK     = Helper::readInt("  Shaking k (recomendado: 3-12): ", 1, 50);
    cfg.maxSinMejora = Helper::readInt("  Pasos sin mejora VND-RL (recomendado: 20): ", 1, 1000);
    cfg.maxTotalIter = Helper::readInt("  Máximo pasos totales VND-RL (recomendado: 100): ", 1, 10000);
    std::cout << "  Decaimiento ε λ (100=sin decaimiento, 95=decay 5%): ";
    cfg.epsilonDecay = Helper::readInt("", 1, 100) / 100.0;
    cfg.propReward   = (Helper::readInt("  Recompensa (1=binaria, 2=proporcional): ", 1, 2) == 2);
    cfg.seed         = 42;

    // Dos configuraciones de α y ε
    std::cout << "\n  Configuración 1 (ej: α=0.1, ε=0.1):\n";
    std::cout << "    α (ej: 10 = 0.10): ";
    double a1 = Helper::readInt("", 1, 99) / 100.0;
    std::cout << "    ε (ej: 10 = 0.10): ";
    double e1 = Helper::readInt("", 1, 99) / 100.0;

    std::cout << "  Configuración 2 (ej: α=0.2, ε=0.2):\n";
    std::cout << "    α (ej: 20 = 0.20): ";
    double a2 = Helper::readInt("", 1, 99) / 100.0;
    std::cout << "    ε (ej: 20 = 0.20): ";
    double e2 = Helper::readInt("", 1, 99) / 100.0;

    cfg.rlConfigs = {{a1, e1}, {a2, e2}};

    BenchmarkRunner runner(cfg);
    try {
        runner.runAllGVNS();
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

        case Helper::Algorithm::GVNS_RL: {
            Helper::Mode mode = Helper::menuMode("GVNS-RL");
            switch (mode) {
            case Helper::Mode::SINGLE_INSTANCE:
                runGVNSRLSingle();
                Helper::pressEnterToContinue();
                break;
            case Helper::Mode::BENCHMARK:
                runBenchmarkGVNS();
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