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
#include <vector>
#include <random>
#include <numeric>
#include <chrono>

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
                   int               swapInstFreq   = 3)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
        , swapInstFreq_(swapInstFreq)
        , rvndRng_(seed == 0
                   ? static_cast<unsigned int>(
                         std::chrono::steady_clock::now()
                             .time_since_epoch().count())
                   : seed)
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

        // LS construidas una sola vez: sus cachés se pagan una sola vez
        // y se reutilizan en todas las iteraciones GRASP.
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

        std::unique_ptr<Solution> best    = nullptr;
        double                    bestCost = std::numeric_limits<double>::infinity();

        for (iterationsRun_ = 0; iterationsRun_ < maxIterations_; ++iterationsRun_) {

            unsigned int iterSeed = seed_ + static_cast<unsigned int>(iterationsRun_);
            GRASPConstructive constructive(inst_, alpha_, beta_, iterSeed);
            auto sol = constructive.run();

            const bool applySwapInst = (swapInstFreq_ <= 0) ||
                                       (iterationsRun_ % swapInstFreq_ == 0);

            const bool isBest      = (lsStrategy_ == ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT);
            const int  maxFirstIter = 10;
            int        localIter    = 0;
            bool       anyImproved  = true;

            while (anyImproved && (isBest || localIter < maxFirstIter)) {
                ++localIter;

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
                        anyImproved = rvnd(*sol, applySwapInst,
                                           shiftLS, swapCliLS, swapInstLS, incompLS);
                        break;
                }

                if (sol->getTotalCost() < bestCost) {
                    bestCost = sol->getTotalCost();
                    best     = sol->clone();
                }
            }
        }

        return best;
    }

private:
    /**
     * @brief  Random Variable Neighborhood Descent (RVND).
     *
     * Mantiene una lista de vecindarios disponibles. En cada paso:
     *   - Elige uno al azar de los disponibles.
     *   - Si mejora → reinicia la lista completa (todos disponibles de nuevo).
     *   - Si no mejora → elimina ese vecindario de la lista.
     * Termina cuando la lista queda vacía: ningún vecindario puede mejorar.
     *
     * @param  sol           Solución a mejorar (modificada in-place).
     * @param  applySwapInst Si false, SwapInstalaciones se omite en esta iter.
     * @return true si se realizó al menos una mejora.
     */
    bool rvnd(Solution&            sol,
              bool                 applySwapInst,
              ShiftLS&             shiftLS,
              SwapClientesLS&      swapCliLS,
              SwapInstalacionesLS& swapInstLS,
              IncompatElimLS&      incompLS)
    {
        // 0=Incompat, 1=Shift, 2=SwapInst, 3=SwapCli
        std::vector<int> available = {0, 1, 2, 3};
        bool anyImproved = false;

        while (!available.empty()) {
            std::uniform_int_distribution<int> dist(
                0, static_cast<int>(available.size()) - 1);
            int idx    = dist(rvndRng_);
            int chosen = available[idx];

            bool improved = false;
            switch (chosen) {
                case 0: improved = incompLS.improve(sol);                        break;
                case 1: improved = shiftLS.improve(sol);                         break;
                case 2: improved = applySwapInst ? swapInstLS.improve(sol)
                                                 : false;                        break;
                case 3: improved = swapCliLS.improve(sol);                       break;
            }

            if (improved) {
                available    = {0, 1, 2, 3};  // reiniciar lista completa
                anyImproved  = true;
            } else {
                available.erase(available.begin() + idx);  // eliminar vecindario
            }
        }
        return anyImproved;
    }

    const MSCFLPInstance&        inst_;
    int                          alpha_;
    int                          beta_;
    ShiftLS::ImprovementStrategy lsStrategy_;
    LocalSearchChoice            lsChoice_;
    int                          swapInstFreq_;
    std::mt19937                 rvndRng_;   ///< RNG exclusivo del RVND

    std::string lsName() const {
        switch (lsChoice_) {
            case LocalSearchChoice::SHIFT:     return "Shift";
            case LocalSearchChoice::SWAP_CLI:  return "SwapClientes";
            case LocalSearchChoice::SWAP_INST: return "SwapInstalaciones";
            case LocalSearchChoice::INCOMPAT:  return "IncompatElim";
            case LocalSearchChoice::ALL:       return "RVND";
        }
        return "?";
    }
};