/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   BenchmarkRunner.h
 * @author Keran Miranda González
 * @date   2025-06-01
 * @brief  Ejecuta los algoritmos sobre las instancias wlp01-wlp08 y escribe
 *         los resultados en tablas con el formato del guión.
 */

#pragma once

#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"
#include "../Algoritmo/Greedy.h"
#include "../Entrega1/GRASP.h"
#include "../Entrega1/Gvnsrl.h"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <chrono>

class BenchmarkRunner {
public:
    struct Config {
        std::string       instancesDir  = "instances/";
        std::string       outputFile    = "resultados.txt";
        int               graspIter     = 10;
        std::vector<int>  lrcSizes      = {2, 3};
        int               graspRuns     = 3;
        unsigned int      seed          = 42;
        LocalSearchChoice lsChoice      = LocalSearchChoice::ALL;
        double            rlAlpha       = 0.2;
        double            rlEpsilon     = 0.2;
        int               maxSinMejora  = 20;
        int               maxTotalIter  = 100;
        // GVNS-RL
        int               gvnsIter      = 300;
        int               shakingK      = 5;
        double            epsilonDecay  = 1.0;
        bool              propReward    = false;
        // Dos configuraciones de α y ε para comparar
        std::vector<std::pair<double,double>> rlConfigs = {{0.1,0.1},{0.2,0.2}};
    };

    // =========================================================================
    // Constructores
    // =========================================================================

    /** @brief Construcción con configuración completa. */
    explicit BenchmarkRunner(const Config& cfg)
        : cfg_(cfg)
    {}

    /** @brief Construcción con valores por defecto. */
    BenchmarkRunner()
        : cfg_{}
    {}

    /** @brief Construcción rápida con directorio y fichero de salida. */
    BenchmarkRunner(const std::string& instancesDir,
                    const std::string& outputFile)
        : cfg_{}
    {
        cfg_.instancesDir = instancesDir;
        cfg_.outputFile   = outputFile;
    }

    // =========================================================================
    // Punto de entrada principal
    // =========================================================================

    /**
     * @brief  Ejecuta todos los algoritmos sobre wlp01-wlp08 y escribe la
     *         salida en cfg_.outputFile.  También imprime el progreso por cout.
     */
    void runAll() {
        std::ofstream out(cfg_.outputFile);
        if (!out.is_open())
            throw std::runtime_error("BenchmarkRunner: no se puede crear '"
                                     + cfg_.outputFile + "'");

        const std::vector<std::string> instances = {
            "wlp01.dzn","wlp02.dzn","wlp03.dzn","wlp04.dzn",
            "wlp05.dzn","wlp06.dzn","wlp07.dzn","wlp08.dzn"
        };

        writeHeader(out);
        runGreedy(out, instances);
        runGRASP(out, instances);
        runGVNS(out, instances);
        writeFooter(out);

        out.close();
        std::cout << "\n[OK] Resultados escritos en '" << cfg_.outputFile << "'\n";
    }

    /// Solo ejecuta la tabla GVNS-RL (sin Voraz ni GRASP previos)
    void runAllGVNS() {
        std::ofstream out(cfg_.outputFile);
        if (!out.is_open())
            throw std::runtime_error("BenchmarkRunner: no se puede crear '"
                                     + cfg_.outputFile + "'");

        const std::vector<std::string> instances = {
            "wlp01.dzn","wlp02.dzn","wlp03.dzn","wlp04.dzn",
            "wlp05.dzn","wlp06.dzn","wlp07.dzn","wlp08.dzn"
        };

        writeHeader(out);
        runGVNS(out, instances);
        writeFooter(out);

        out.close();
        std::cout << "\n[OK] Resultados GVNS-RL escritos en '" << cfg_.outputFile << "'\n";
    }

private:
    Config cfg_;

    // =========================================================================
    // Estructura de una fila de resultados
    // =========================================================================

    struct Row {
        std::string instance;
        int         jOpen        = 0;
        double      fixedCost    = 0.0;
        double      transportCost= 0.0;
        double      totalCost    = 0.0;
        int         incompatViol = 0;
        double      cpuSeconds   = 0.0;
        bool        feasible     = false;
        // Solo GRASP
        int         lrcSize      = 0;
        int         run          = 0;
    };

    // =========================================================================
    // Tabla Voraz
    // =========================================================================

    void runGreedy(std::ostream& out,
                   const std::vector<std::string>& instances)
    {
        std::cout << "\n--- Algoritmo Voraz ---\n";
        printGreedyHeader(out);

        std::vector<Row> rows;

        for (const auto& name : instances) {
            std::string path = cfg_.instancesDir + name;
            std::cout << "  " << name << "... " << std::flush;

            MSCFLPInstance inst;
            try { inst.load(path); }
            catch (const std::exception& e) {
                std::cout << "ERROR: " << e.what() << "\n";
                continue;
            }

            Greedy greedy(inst);
            auto sol = greedy.run();
            auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

            Row row;
            row.instance      = name;
            row.jOpen         = mSol.countOpenFacilities();
            row.fixedCost     = mSol.getFixedCost();
            row.transportCost = mSol.getTransportCost();
            row.totalCost     = mSol.getTotalCost();
            row.incompatViol  = mSol.countIncompatibilityViolations();
            row.cpuSeconds    = greedy.getElapsedMs() / 1000.0;
            row.feasible      = mSol.isFeasible();
            rows.push_back(row);

            printGreedyRow(out, row);
            std::cout << "Coste=" << std::fixed << std::setprecision(0)
                      << row.totalCost << "  Incomp=" << row.incompatViol
                      << "  CPU=" << std::setprecision(4) << row.cpuSeconds << "s\n";
        }

        printGreedyAverage(out, rows);
        out << "\n";
    }

    // =========================================================================
    // Tabla GRASP
    // =========================================================================

    void runGRASP(std::ostream& out,
                  const std::vector<std::string>& instances)
    {
        std::cout << "\n--- Algoritmo GRASP ---\n";
        printGRASPHeader(out);

        std::vector<Row> allRows;

        for (const auto& name : instances) {
            std::string path = cfg_.instancesDir + name;

            MSCFLPInstance inst;
            try { inst.load(path); }
            catch (const std::exception& e) {
                std::cout << "  " << name << " ERROR: " << e.what() << "\n";
                continue;
            }

            for (int lrc : cfg_.lrcSizes) {
                for (int run = 1; run <= cfg_.graspRuns; ++run) {
                    std::cout << "  " << name
                              << "  LRC=" << lrc
                              << "  run=" << run << "... " << std::flush;

                    unsigned int runSeed = cfg_.seed + static_cast<unsigned int>(run * 1000);
                    GRASP grasp(inst, cfg_.graspIter, lrc, lrc, runSeed,
                               ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                               cfg_.lsChoice, 3,
                               cfg_.rlAlpha, cfg_.rlEpsilon,
                               cfg_.maxSinMejora, cfg_.maxTotalIter);
                    auto sol = grasp.run();
                    auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

                    Row row;
                    row.instance      = name;
                    row.lrcSize       = lrc;
                    row.run           = run;
                    row.jOpen         = mSol.countOpenFacilities();
                    row.fixedCost     = mSol.getFixedCost();
                    row.transportCost = mSol.getTransportCost();
                    row.totalCost     = mSol.getTotalCost();
                    row.incompatViol  = mSol.countIncompatibilityViolations();
                    row.cpuSeconds    = grasp.getElapsedMs() / 1000.0;
                    row.feasible      = mSol.isFeasible();
                    allRows.push_back(row);

                    printGRASPRow(out, row);
                    std::cout << "Coste=" << std::fixed << std::setprecision(0)
                              << row.totalCost << "  Incomp=" << row.incompatViol
                              << "  CPU=" << std::setprecision(4) << row.cpuSeconds << "s\n";
                }
            }
        }

        printGRASPAverage(out, allRows);
        out << "\n";
    }

    // =========================================================================
    // Formateo de tablas
    // =========================================================================

    // Anchuras de columna
    static constexpr int W_INST  = 12;
    static constexpr int W_NUM   =  8;
    static constexpr int W_COST  = 14;
    static constexpr int W_CPU   = 12;

    // =========================================================================
    // Tabla GVNS-RL: dos configuraciones de α/ε por instancia
    // =========================================================================

    void runGVNS(std::ostream& out,
                 const std::vector<std::string>& instances)
    {
        std::cout << "\n--- Algoritmo GVNS-RL ---\n";
        printGVNSHeader(out);

        std::vector<Row> allRows;

        for (const auto& name : instances) {
            std::string path = cfg_.instancesDir + name;

            MSCFLPInstance inst;
            try { inst.load(path); }
            catch (const std::exception& e) {
                std::cout << "  " << name << " ERROR: " << e.what() << "\n";
                continue;
            }

            for (const auto& [alpha, epsilon] : cfg_.rlConfigs) {
                std::cout << "  " << name
                          << "  α=" << alpha
                          << "  ε=" << epsilon
                          << "... " << std::flush;

                unsigned int runSeed = cfg_.seed;
                GVNSRL gvns(inst,
                            cfg_.graspIter, 3, 3, runSeed,
                            cfg_.gvnsIter, cfg_.shakingK,
                            alpha, epsilon,
                            cfg_.maxSinMejora, cfg_.maxTotalIter,
                            cfg_.epsilonDecay, cfg_.propReward, "");

                auto sol = gvns.run();
                auto& mSol = dynamic_cast<MSCFLPSolution&>(*sol);

                Row row;
                row.instance      = name;
                row.lrcSize       = 0;   // reutilizamos para α×100
                row.run           = 0;   // reutilizamos para ε×100
                row.jOpen         = mSol.countOpenFacilities();
                row.fixedCost     = mSol.getFixedCost();
                row.transportCost = mSol.getTransportCost();
                row.totalCost     = mSol.getTotalCost();
                row.incompatViol  = mSol.countIncompatibilityViolations();
                row.cpuSeconds    = gvns.getElapsedMs() / 1000.0;
                row.feasible      = mSol.isFeasible();
                allRows.push_back(row);

                printGVNSRow(out, row, alpha, epsilon);
                std::cout << "Coste=" << std::fixed << std::setprecision(0)
                          << row.totalCost << "  CPU="
                          << std::setprecision(4) << row.cpuSeconds << "s\n";
            }
        }

        printGVNSAverage(out, allRows);
        out << "\n";
    }

    void printGVNSHeader(std::ostream& out) const {
        out << "--------------------------------------------------------\n"
            << "  Algoritmo GVNS-RL (MS-CFLP-CI)\n"
            << "--------------------------------------------------------\n";
        out << std::left
            << std::setw(W_INST) << "Instancia"
            << std::right
            << std::setw(6)      << "α"
            << std::setw(6)      << "ε"
            << std::setw(W_NUM)  << "|Jopen|"
            << std::setw(W_COST) << "C.Fijo"
            << std::setw(W_COST) << "C.Asig."
            << std::setw(W_COST) << "C.Total"
            << std::setw(W_NUM)  << "Incomp."
            << std::setw(W_CPU)  << "CPU(s)"
            << "\n";
        out << std::string(W_INST+6+6+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";
    }

    void printGVNSRow(std::ostream& out, const Row& r,
                      double alpha, double epsilon) const {
        out << std::left  << std::setw(W_INST) << r.instance
            << std::right << std::fixed
            << std::setw(6) << std::setprecision(2) << alpha
            << std::setw(6) << std::setprecision(2) << epsilon
            << std::setprecision(0)
            << std::setw(W_NUM)  << r.jOpen
            << std::setw(W_COST) << r.fixedCost
            << std::setw(W_COST) << r.transportCost
            << std::setw(W_COST) << r.totalCost
            << std::setw(W_NUM)  << r.incompatViol
            << std::setprecision(4)
            << std::setw(W_CPU)  << r.cpuSeconds
            << (r.feasible ? "" : "  [INFEASIBLE]")
            << "\n";
    }

    void printGVNSAverage(std::ostream& out,
                           const std::vector<Row>& rows) const {
        if (rows.empty()) return;
        out << std::string(W_INST+6+6+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";
        double sumFixed=0,sumTrans=0,sumTotal=0,sumCpu=0,sumJ=0,sumInc=0;
        for (const auto& r : rows) {
            sumJ    += r.jOpen;       sumFixed += r.fixedCost;
            sumTrans+= r.transportCost; sumTotal += r.totalCost;
            sumInc  += r.incompatViol; sumCpu   += r.cpuSeconds;
        }
        double n = static_cast<double>(rows.size());
        out << std::left  << std::setw(W_INST) << "Promedio"
            << std::right << std::fixed
            << std::setw(6) << "-" << std::setw(6) << "-"
            << std::setprecision(1)
            << std::setw(W_NUM)  << (sumJ/n)
            << std::setw(W_COST) << (sumFixed/n)
            << std::setw(W_COST) << (sumTrans/n)
            << std::setw(W_COST) << (sumTotal/n)
            << std::setw(W_NUM)  << (sumInc/n)
            << std::setprecision(4)
            << std::setw(W_CPU)  << (sumCpu/n)
            << "\n";
    }

    void writeHeader(std::ostream& out) const {
        out << "========================================================\n"
            << "  MS-CFLP-CI — Resultados experimentales\n"
            << "  Instancias: wlp01-wlp08\n"
            << "========================================================\n\n";
    }

    void writeFooter(std::ostream& out) const {
        out << "========================================================\n"
            << "  Fin del informe\n"
            << "========================================================\n";
    }

    // ── Cabecera tabla Voraz ─────────────────────────────────────────────────
    void printGreedyHeader(std::ostream& out) const {
        out << "--------------------------------------------------------\n"
            << "  Algoritmo Voraz (MS-CFLP-CI)\n"
            << "--------------------------------------------------------\n";
        out << std::left
            << std::setw(W_INST) << "Instancia"
            << std::right
            << std::setw(W_NUM)  << "|Jopen|"
            << std::setw(W_COST) << "C.Fijo"
            << std::setw(W_COST) << "C.Asig."
            << std::setw(W_COST) << "C.Total"
            << std::setw(W_NUM)  << "Incomp."
            << std::setw(W_CPU)  << "CPU(s)"
            << "\n";
        out << std::string(W_INST+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";
    }

    void printGreedyRow(std::ostream& out, const Row& r) const {
        out << std::left  << std::setw(W_INST) << r.instance
            << std::right << std::fixed << std::setprecision(0)
            << std::setw(W_NUM)  << r.jOpen
            << std::setw(W_COST) << r.fixedCost
            << std::setw(W_COST) << r.transportCost
            << std::setw(W_COST) << r.totalCost
            << std::setw(W_NUM)  << r.incompatViol
            << std::setprecision(4)
            << std::setw(W_CPU)  << r.cpuSeconds
            << (r.feasible ? "" : "  [INFEASIBLE]")
            << "\n";
    }

    void printGreedyAverage(std::ostream& out,
                             const std::vector<Row>& rows) const
    {
        if (rows.empty()) return;
        out << std::string(W_INST+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";

        double sumFixed=0, sumTrans=0, sumTotal=0, sumCpu=0;
        double sumJopen=0, sumIncompat=0;
        for (const auto& r : rows) {
            sumJopen   += r.jOpen;
            sumFixed   += r.fixedCost;
            sumTrans   += r.transportCost;
            sumTotal   += r.totalCost;
            sumIncompat+= r.incompatViol;
            sumCpu     += r.cpuSeconds;
        }
        double n = static_cast<double>(rows.size());
        out << std::left  << std::setw(W_INST) << "Promedio"
            << std::right << std::fixed << std::setprecision(1)
            << std::setw(W_NUM)  << (sumJopen/n)
            << std::setw(W_COST) << (sumFixed/n)
            << std::setw(W_COST) << (sumTrans/n)
            << std::setw(W_COST) << (sumTotal/n)
            << std::setw(W_NUM)  << (sumIncompat/n)
            << std::setprecision(4)
            << std::setw(W_CPU)  << (sumCpu/n)
            << "\n";
    }

    // ── Cabecera tabla GRASP ─────────────────────────────────────────────────
    void printGRASPHeader(std::ostream& out) const {
        out << "--------------------------------------------------------\n"
            << "  Algoritmo GRASP (MS-CFLP-CI)\n"
            << "--------------------------------------------------------\n";
        out << std::left
            << std::setw(W_INST) << "Instancia"
            << std::right
            << std::setw(6)      << "|LRC|"
            << std::setw(6)      << "Ejec."
            << std::setw(W_NUM)  << "|Jopen|"
            << std::setw(W_COST) << "C.Fijo"
            << std::setw(W_COST) << "C.Asig."
            << std::setw(W_COST) << "C.Total"
            << std::setw(W_NUM)  << "Incomp."
            << std::setw(W_CPU)  << "CPU(s)"
            << "\n";
        out << std::string(W_INST+6+6+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";
    }

    void printGRASPRow(std::ostream& out, const Row& r) const {
        out << std::left  << std::setw(W_INST) << r.instance
            << std::right << std::fixed << std::setprecision(0)
            << std::setw(6)      << r.lrcSize
            << std::setw(6)      << r.run
            << std::setw(W_NUM)  << r.jOpen
            << std::setw(W_COST) << r.fixedCost
            << std::setw(W_COST) << r.transportCost
            << std::setw(W_COST) << r.totalCost
            << std::setw(W_NUM)  << r.incompatViol
            << std::setprecision(4)
            << std::setw(W_CPU)  << r.cpuSeconds
            << (r.feasible ? "" : "  [INFEASIBLE]")
            << "\n";
    }

    void printGRASPAverage(std::ostream& out,
                            const std::vector<Row>& rows) const
    {
        if (rows.empty()) return;
        out << std::string(W_INST+6+6+W_NUM+W_COST*3+W_NUM+W_CPU, '-') << "\n";

        double sumFixed=0, sumTrans=0, sumTotal=0, sumCpu=0;
        double sumJopen=0, sumIncompat=0;
        for (const auto& r : rows) {
            sumJopen   += r.jOpen;
            sumFixed   += r.fixedCost;
            sumTrans   += r.transportCost;
            sumTotal   += r.totalCost;
            sumIncompat+= r.incompatViol;
            sumCpu     += r.cpuSeconds;
        }
        double n = static_cast<double>(rows.size());
        out << std::left  << std::setw(W_INST) << "Promedio"
            << std::right << std::fixed
            << std::setw(6)      << "-"
            << std::setw(6)      << "-"
            << std::setprecision(1)
            << std::setw(W_NUM)  << (sumJopen/n)
            << std::setw(W_COST) << (sumFixed/n)
            << std::setw(W_COST) << (sumTrans/n)
            << std::setw(W_COST) << (sumTotal/n)
            << std::setw(W_NUM)  << (sumIncompat/n)
            << std::setprecision(4)
            << std::setw(W_CPU)  << (sumCpu/n)
            << "\n";
    }
};