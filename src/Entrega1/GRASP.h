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
                   int               maxIterations = 30,
                   int               alpha         = 3,
                   int               beta          = 3,
                   unsigned int      seed          = 0,
                   ShiftLS::ImprovementStrategy strategy
                       = ShiftLS::ImprovementStrategy::FIRST_IMPROVEMENT,
                   LocalSearchChoice lsChoice      = LocalSearchChoice::ALL,
                   int               swapInstFreq  = 3)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
        , swapInstFreq_(swapInstFreq)
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

        // OPT 1: construir LS una sola vez — los cachés se pagan una vez
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

            // OPT 2: semilla distinta por iteración → diversidad real
            unsigned int iterSeed = seed_ + static_cast<unsigned int>(iterationsRun_);
            GRASPConstructive constructive(inst_, alpha_, beta_, iterSeed);
            auto sol = constructive.run();

            // OPT 3: SwapInstalaciones solo cada swapInstFreq_ iteraciones
            bool applySwapInst = (swapInstFreq_ <= 0) ||
                                 (iterationsRun_ % swapInstFreq_ == 0);

            // Bucle de mejora: converge al óptimo local.
            //
            // BEST_IMPROVEMENT:  itera sin límite — cada llamada ya encontró
            //   el mejor movimiento posible, pocas vueltas hasta convergencia.
            //
            // FIRST_IMPROVEMENT: itera hasta maxFirstIter vueltas — cada llamada
            //   aplica el primer movimiento que mejora (barato por movimiento,
            //   pero más vueltas). Se limita porque las últimas vueltas aportan
            //   poco y cuestan igual que las primeras en exploración de vecindario.
            const bool  isBest      = (lsStrategy_ == ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT);
            const int   maxFirstIter = 10;   // límite solo para FIRST_IMPROVEMENT
            int  localIter  = 0;
            bool anyImproved = true;

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
                    case LocalSearchChoice::ALL: {
                        // LS ordenadas de menor a mayor coste computacional.
                        // SwapClientes (más costosa) solo si alguna de las
                        // anteriores mejoró: si el resto no puede mejorar,
                        // SwapClientes tampoco encontrará mejoras.
                        bool imp1 = incompLS.improve(*sol);
                        bool imp2 = shiftLS.improve(*sol);
                        bool imp3 = applySwapInst
                                    ? swapInstLS.improve(*sol)
                                    : false;
                        bool cheapImproved = imp1 || imp2 || imp3;
                        bool imp4 = cheapImproved
                                    ? swapCliLS.improve(*sol)
                                    : false;
                        anyImproved = imp1 || imp2 || imp3 || imp4;
                        break;
                    }
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
    int                          swapInstFreq_;

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