/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   GRASP.h
 * @author Keran Miranda González
 * @date   2025-06-01
 * @brief  Implementación del algoritmo GRASP completo para el MS-CFLP-CI
 */

#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"
#include "../Algoritmo/GraspConstructive.h"
#include "../BusquedasLocales/ShiftLS.h"
#include "../BusquedasLocales/SwapClientesLS.h"
#include "../BusquedasLocales/SwapInstalacionesLS.h"
#include "../BusquedasLocales/IncompatElimLS.h"
#include <memory>
#include <string>
#include <limits>

/// Qué búsqueda(s) local(es) aplicar — debe coincidir con Helper::LocalSearchChoice
enum class LocalSearchChoice {
    SHIFT    = 1,
    SWAP_CLI = 2,
    SWAP_INST= 3,
    INCOMPAT = 4,
    ALL      = 5
};

class GRASP : public Metaheuristic {
public:
    explicit GRASP(const MSCFLPInstance& inst,
                   int               maxIterations = 30,
                   int               alpha         = 3,
                   int               beta          = 3,
                   unsigned int      seed          = 0,
                   ShiftLS::ImprovementStrategy strategy
                       = ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                   LocalSearchChoice lsChoice      = LocalSearchChoice::ALL)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
    {}

    std::string getName() const override {
        return "GRASP (iter=" + std::to_string(maxIterations_)
             + ", alpha=" + std::to_string(alpha_)
             + ", beta="  + std::to_string(beta_)
             + ", ls="    + lsName() + ")";
    }

    int getAlpha() const { return alpha_; }
    int getBeta()  const { return beta_;  }

protected:
    std::unique_ptr<Solution> solve() override {
        // Construir solo las LS necesarias
        ShiftLS             shiftLS  (inst_, lsStrategy_);
        SwapClientesLS      swapCliLS(inst_,
            static_cast<SwapClientesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        SwapInstalacionesLS swapInstLS(inst_,
            static_cast<SwapInstalacionesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        IncompatElimLS      incompLS (inst_,
            static_cast<IncompatElimLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));

        GRASPConstructive constructive(inst_, alpha_, beta_, seed_);

        std::unique_ptr<Solution> best = nullptr;
        double bestCost = std::numeric_limits<double>::infinity();

        for (iterationsRun_ = 0; iterationsRun_ < maxIterations_; ++iterationsRun_) {
            auto sol = constructive.run();

            // Fase de mejora según elección del usuario
            bool anyImproved = true;
            while (anyImproved) {
                switch (lsChoice_) {
                    case LocalSearchChoice::SHIFT:
                        anyImproved = shiftLS.improve(*sol);
                        break;
                    case LocalSearchChoice::SWAP_CLI:
                        anyImproved = swapCliLS.improve(*sol);
                        break;
                    case LocalSearchChoice::SWAP_INST:
                        anyImproved = swapInstLS.improve(*sol);
                        break;
                    case LocalSearchChoice::INCOMPAT:
                        anyImproved = incompLS.improve(*sol);
                        break;
                    case LocalSearchChoice::ALL:
                        anyImproved  = shiftLS.improve(*sol);
                        anyImproved |= swapCliLS.improve(*sol);
                        anyImproved |= swapInstLS.improve(*sol);
                        anyImproved |= incompLS.improve(*sol);
                        break;
                }
            }

            if (sol->getTotalCost() < bestCost) {
                bestCost = sol->getTotalCost();
                best     = sol->clone();
            }
        }

        return best;
    }

private:
    const MSCFLPInstance&        inst_;
    int                          alpha_;
    int                          beta_;
    ShiftLS::ImprovementStrategy lsStrategy_;
    LocalSearchChoice            lsChoice_;

    std::string lsName() const {
        switch (lsChoice_) {
            case LocalSearchChoice::SHIFT:     return "Shift";
            case LocalSearchChoice::SWAP_CLI:  return "SwapClientes";
            case LocalSearchChoice::SWAP_INST: return "SwapInstalaciones";
            case LocalSearchChoice::INCOMPAT:  return "IncompatElim";
            case LocalSearchChoice::ALL:       return "Todas";
        }
        return "?";
    }
};