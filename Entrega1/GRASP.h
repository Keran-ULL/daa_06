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

#include "Algorithm.h"
#include "MSCFLPInstance.h"
#include "MSCFLPSolution.h"
#include "GRASPConstructive.h"
#include "ShiftLS.h"
#include "SwapClientesLS.h"
#include "SwapInstalacionesLS.h"
#include "IncompatElimLS.h"

#include <memory>
#include <string>
#include <limits>

class GRASP : public Metaheuristic {
public:

    /**
     * @param  inst          Instancia del problema.
     * @param  maxIterations Número de iteraciones del bucle GRASP.
     * @param  alpha         Tamaño LRC fase 1 del constructivo.
     * @param  beta          Tamaño LRC fase 2 del constructivo.
     * @param  seed          Semilla RNG (0 = basada en tiempo).
     * @param  strategy      Estrategia de mejora (FIRST o BEST).
     */
    explicit GRASP(const MSCFLPInstance& inst,
                   int          maxIterations = 30,
                   int          alpha         = 3,
                   int          beta          = 3,
                   unsigned int seed          = 0,
                   ShiftLS::ImprovementStrategy strategy
                       = ShiftLS::ImprovementStrategy::BEST_IMPROVEMENT)
        : Metaheuristic(inst, maxIterations, seed)
        , inst_(inst)
        , alpha_(alpha)
        , beta_(beta)
        , lsStrategy_(strategy)
    {}

    std::string getName() const override {
        return "GRASP (iter=" + std::to_string(maxIterations_)
             + ", alpha=" + std::to_string(alpha_)
             + ", beta="  + std::to_string(beta_) + ")";
    }

    // Getters para la tabla de resultados
    int getAlpha() const { return alpha_; }
    int getBeta()  const { return beta_;  }

protected:

    std::unique_ptr<Solution> solve() override {
        // Construir búsquedas locales (una sola vez, se reusan)
        ShiftLS             shiftLS(inst_, lsStrategy_);
        SwapClientesLS      swapCliLS(inst_,
            static_cast<SwapClientesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        SwapInstalacionesLS swapInstLS(inst_,
            static_cast<SwapInstalacionesLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));
        IncompatElimLS      incompLS(inst_,
            static_cast<IncompatElimLS::ImprovementStrategy>(
                static_cast<int>(lsStrategy_)));

        // Constructivo con semilla derivada de seed_ para reproducibilidad
        GRASPConstructive constructive(inst_, alpha_, beta_, seed_);

        std::unique_ptr<Solution> best = nullptr;
        double bestCost = std::numeric_limits<double>::infinity();

        for (iterationsRun_ = 0; iterationsRun_ < maxIterations_; ++iterationsRun_) {
            // 1. Fase constructiva
            auto sol = constructive.run();
            // 2. Fase de mejora: ciclo completo de las 4 LS
            bool globalImproved = true;
            while (globalImproved) {
                globalImproved  = shiftLS.improve(*sol);
                globalImproved |= swapCliLS.improve(*sol);
                globalImproved |= swapInstLS.improve(*sol);
                globalImproved |= incompLS.improve(*sol);
            }
            // 3. Actualizar mejor solución global
            if (sol->getTotalCost() < bestCost) {
                bestCost = sol->getTotalCost();
                best     = sol->clone();
            }
        }

        return best;
    }

private:
    const MSCFLPInstance& inst_;
    int    alpha_;
    int    beta_;
    ShiftLS::ImprovementStrategy lsStrategy_;
};
