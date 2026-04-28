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
    ALL       = 5,   ///< RVND
    VND       = 6,   ///< VND orden fijo
    GVNS_RL   = 7    ///< GVNS con Reinforcement Learning
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
                   double            rlAlpha        = 0.2,
                   double            rlEpsilon      = 0.2,
                   int               maxSinMejora   = 20,
                   int               maxTotalIter   = 100)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
        , lsChoice_(lsChoice)
        , swapInstFreq_(swapInstFreq)
        , rlAlpha_(rlAlpha)
        , rlEpsilon_(rlEpsilon)
        , maxSinMejora_(maxSinMejora)
        , maxTotalIter_(maxTotalIter)
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
                    case LocalSearchChoice::VND:
                        anyImproved = vnd(*sol, applySwapInst,
                                          shiftLS, swapCliLS, swapInstLS, incompLS);
                        break;
                    case LocalSearchChoice::GVNS_RL:
                        anyImproved = vndRL(*sol, applySwapInst,
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
        std::vector<int> available   = {0, 1, 2};
        bool             anyImproved  = false;
        bool             cheapImproved = false;

        while (!available.empty()) {
            std::uniform_int_distribution<int> dist(
                0, static_cast<int>(available.size()) - 1);
            int idx    = dist(rvndRng_);
            int chosen = available[idx];

            bool improved = false;
            switch (chosen) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            if (improved) {
                anyImproved = true;
                if (chosen != 3) cheapImproved = true;
                available = cheapImproved
                            ? std::vector<int>{0, 1, 2, 3}
                            : std::vector<int>{0, 1, 2};
            } else {
                available.erase(available.begin() + idx);
                if (available.empty() && cheapImproved) {
                    available.push_back(3);
                    cheapImproved = false;  // una sola oportunidad
                }
            }
        }
        return anyImproved;
    }

    /**
     * @brief  Variable Neighborhood Descent (VND).
     *
     * Versión determinista del RVND: los vecindarios se prueban siempre
     * en el mismo orden fijo (Incompat → Shift → SwapInst → SwapCli).
     * Si el vecindario k mejora, se reinicia desde el primero (k=0).
     * Si no mejora, se avanza al siguiente (k+1).
     * Termina cuando ningún vecindario puede mejorar.
     *
     * SwapCli solo se prueba si alguno de los anteriores mejoró en esta
     * ronda (misma optimización que en el RVND).
     *
     * @param  sol           Solución a mejorar (modificada in-place).
     * @param  applySwapInst Si false, SwapInstalaciones se omite.
     * @return true si se realizó al menos una mejora.
     */
    bool vnd(Solution&            sol,
             bool                 applySwapInst,
             ShiftLS&             shiftLS,
             SwapClientesLS&      swapCliLS,
             SwapInstalacionesLS& swapInstLS,
             IncompatElimLS&      incompLS)
    {
        // Orden fijo: 0=Incompat, 1=Shift, 2=SwapInst, 3=SwapCli
        const int nNeighborhoods = 4;
        bool anyImproved  = false;
        bool cheapImproved = false;
        int  k = 0;

        while (k < nNeighborhoods) {
            if (k == 3 && !cheapImproved) break;

            bool improved = false;
            switch (k) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            if (improved) {
                anyImproved = true;
                if (k != 3) cheapImproved = true;
                k = 0;   // reiniciar desde el primer vecindario
            } else {
                ++k;     // avanzar al siguiente
            }
        }
        return anyImproved;
    }

    /**
     * @brief  VND con Reinforcement Learning (Q-Learning simplificado).
     *
     * Cada búsqueda local LSk tiene un valor Q(LSk) ∈ [0,1] que refleja
     * su historial de éxito. En cada paso se elige la LS con política
     * ε-greedy: con probabilidad ε se explora (LS aleatoria), con
     * probabilidad 1-ε se explota (LS con mayor Q).
     *
     * Tras ejecutar LSk:
     *   r = 1 si mejoró, 0 si no
     *   Q(LSk) ← Q(LSk) + α·[r - Q(LSk)]
     *
     * Criterio de parada: maxSinMejora_ pasos consecutivos sin mejora
     * o maxTotalIter_ pasos totales.
     *
     * Parámetros (guión recomienda α≈0.2, ε≈0.2):
     *   rlAlpha_    : tasa de aprendizaje α ∈ (0,1)
     *   rlEpsilon_  : probabilidad de exploración ε ∈ (0,1)
     *   maxSinMejora_: pasos sin mejora antes de parar
     *   maxTotalIter_: máximo de pasos totales
     */
    bool vndRL(Solution&            sol,
               bool                 applySwapInst,
               ShiftLS&             shiftLS,
               SwapClientesLS&      swapCliLS,
               SwapInstalacionesLS& swapInstLS,
               IncompatElimLS&      incompLS)
    {
        // Tabla Q: Q[0]=Incompat, Q[1]=Shift, Q[2]=SwapInst, Q[3]=SwapCli
        std::vector<double> Q(4, 0.5);
        std::uniform_real_distribution<double> realDist(0.0, 1.0);
        std::uniform_int_distribution<int>     idxDist(0, 3);

        bool anyImproved    = false;
        int  sinMejora      = 0;
        int  totalIter      = 0;

        while (sinMejora < maxSinMejora_ && totalIter < maxTotalIter_) {
            ++totalIter;

            // Selección ε-greedy
            int chosen;
            if (realDist(rvndRng_) < rlEpsilon_) {
                // Exploración: LS aleatoria
                chosen = idxDist(rvndRng_);
            } else {
                // Explotación: LS con mayor Q
                chosen = static_cast<int>(
                    std::max_element(Q.begin(), Q.end()) - Q.begin());
            }

            // Ejecutar LS elegida
            bool improved = false;
            switch (chosen) {
                case 0: improved = incompLS.improve(sol);                  break;
                case 1: improved = shiftLS.improve(sol);                   break;
                case 2: improved = applySwapInst
                                   ? swapInstLS.improve(sol) : false;     break;
                case 3: improved = swapCliLS.improve(sol);                 break;
            }

            // Recompensa binaria: r=1 si mejoró, r=0 si no
            double r = improved ? 1.0 : 0.0;

            // Actualizar Q: Q(LSk) ← Q(LSk) + α·[r - Q(LSk)]
            Q[chosen] += rlAlpha_ * (r - Q[chosen]);

            // Actualizar contadores
            if (improved) {
                anyImproved = true;
                sinMejora   = 0;
            } else {
                ++sinMejora;
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
    double                       rlAlpha_;       ///< Tasa de aprendizaje Q-Learning
    double                       rlEpsilon_;     ///< Probabilidad de exploración ε-greedy
    int                          maxSinMejora_;  ///< Pasos sin mejora antes de parar VND-RL
    int                          maxTotalIter_;  ///< Máximo de pasos totales VND-RL
    std::mt19937                 rvndRng_;

    std::string lsName() const {
        switch (lsChoice_) {
            case LocalSearchChoice::SHIFT:     return "Shift";
            case LocalSearchChoice::SWAP_CLI:  return "SwapClientes";
            case LocalSearchChoice::SWAP_INST: return "SwapInstalaciones";
            case LocalSearchChoice::INCOMPAT:  return "IncompatElim";
            case LocalSearchChoice::ALL:       return "RVND";
            case LocalSearchChoice::VND:       return "VND";
            case LocalSearchChoice::GVNS_RL:   return "GVNS-RL";
        }
        return "?";
    }
};