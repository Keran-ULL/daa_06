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

enum class LocalSearchChoice {
    SHIFT     = 1,
    SWAP_CLI  = 2,
    SWAP_INST = 3,
    INCOMPAT  = 4,
    ALL       = 5
};

class GRASP : public Metaheuristic {
public:
    explicit GRASP(const MSCFLPInstance& inst,
                   int               maxIterations  = 30,
                   int               alpha          = 3,
                   int               beta           = 3,
                   unsigned int      seed           = 0,
                   ShiftLS::ImprovementStrategy strategy
                       = ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                   LocalSearchChoice lsChoice       = LocalSearchChoice::ALL,
                   int               swapInstFreq   = 3,
                   double            minImprovement = 0.0)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
        , swapInstFreq_(swapInstFreq)
        , minImprovement_(minImprovement)
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

        // LS construidas 
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

        // GRASPConstructive creado una vez: se reutiliza cambiando solo la semilla.
        GRASPConstructive constructive(inst_, alpha_, beta_, seed_);

        std::unique_ptr<Solution> best    = nullptr;
        double                    bestCost = std::numeric_limits<double>::infinity();

        for (iterationsRun_ = 0; iterationsRun_ < maxIterations_; ++iterationsRun_) {

            constructive.setSeed(seed_ + static_cast<unsigned int>(iterationsRun_));
            auto sol = constructive.run();

            const bool applySwapInst = (swapInstFreq_ <= 0) ||
                                       (iterationsRun_ % swapInstFreq_ == 0);

            bool anyImproved = true;
            while (anyImproved) {
                double costBefore = sol->getTotalCost();

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
                    case LocalSearchChoice::ALL: {
                        bool imp1 = incompLS.improve(*sol);
                        bool imp2 = shiftLS.improve(*sol);
                        bool imp3 = applySwapInst ? swapInstLS.improve(*sol) : false;
                        bool imp4 = (imp1||imp2||imp3) ? swapCliLS.improve(*sol) : false;
                        anyImproved = imp1 || imp2 || imp3 || imp4;
                        break;
                    }
                }

                // Parar si la mejora de esta vuelta es menor que el umbral
                if (anyImproved && (costBefore - sol->getTotalCost()) < minImprovement_)
                    anyImproved = false;

                if (sol->getTotalCost() < bestCost) {
                    bestCost = sol->getTotalCost();
                    best     = sol->clone();
                }
            }
        }
       if (best && !best->checkFeasibility()) {
            throw std::logic_error("GRASP::solve() — la mejor solución encontrada no es factible. "
                                   "Revise las implementaciones de las búsquedas locales.");
        }
        return best;
    }

private:
    const MSCFLPInstance&        inst_;
    int                          alpha_;
    int                          beta_;
    ShiftLS::ImprovementStrategy lsStrategy_;
    LocalSearchChoice            lsChoice_;
    int                          swapInstFreq_;
    double                       minImprovement_;  

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