/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   SwapClientesLS.h
 * @brief  Implementación de la clase SwapClientesLS, que representa una
 *        búsqueda local basada en el operador Swap-Clientes(i1, i2) para el problema MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */

#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <limits>
#include <string>

class SwapClientesLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit SwapClientesLS(const MSCFLPInstance& inst,
                            ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "SwapClientesLS (best)"
               : "SwapClientesLS (first)";
    }

    void setStrategy(ImprovementStrategy s) { strategy_ = s; }

    bool improve(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        bool anyImprovement = false;
        bool improved = true;
        while (improved) {
            improved = applyBestMove(sol);
            if (improved) anyImprovement = true;
        }
        return anyImprovement;
    }

    bool applyBestMove(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? applyBest(sol)
               : applyFirst(sol);
    }

private:
    const MSCFLPInstance& inst_;
    ImprovementStrategy   strategy_;
    struct SwapMove {
        int    i1, i2;          // clientes a intercambiar
        int    ja, jb;          // ja = instalación de i1, jb = instalación de i2
        double qa, qb;          // cantidades (absolutas) a intercambiar
        double delta;           // ganancia esperada (negativo = mejora)
    };

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        SwapMove best;
        best.delta = -1e-9;
        bool found = false;

        for (int ja = 0; ja < m; ++ja) {
            if (!sol.isOpen(ja)) continue;
            const auto& clientsA = sol.getClientsOf(ja);

            for (int jb = ja + 1; jb < m; ++jb) {
                if (!sol.isOpen(jb)) continue;
                const auto& clientsB = sol.getClientsOf(jb);

                for (int i1 : clientsA) {
                    const double d1 = inst_.getDemand(i1);
                    const double qa = sol.getX(i1, ja) * d1;

                    for (int i2 : clientsB) {
                        const double d2 = inst_.getDemand(i2);
                        const double qb = sol.getX(i2, jb) * d2;

                        double delta;
                        if (!isFeasibleSwap(sol, i1, i2, ja, jb, qa, qb, delta))
                            continue;
                        if (delta < best.delta) {
                            best = {i1, i2, ja, jb, qa, qb, delta};
                            found = true;
                        }
                    }
                }
            }
        }

        if (!found) return false;
        applySwap(sol, best);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        for (int ja = 0; ja < m; ++ja) {
            if (!sol.isOpen(ja)) continue;
            const auto& clientsA = sol.getClientsOf(ja);

            for (int jb = ja + 1; jb < m; ++jb) {
                if (!sol.isOpen(jb)) continue;
                const auto& clientsB = sol.getClientsOf(jb);

                for (int i1 : clientsA) {
                    const double d1 = inst_.getDemand(i1);
                    const double qa = sol.getX(i1, ja) * d1;

                    for (int i2 : clientsB) {
                        const double d2 = inst_.getDemand(i2);
                        const double qb = sol.getX(i2, jb) * d2;

                        double delta;
                        if (!isFeasibleSwap(sol, i1, i2, ja, jb, qa, qb, delta))
                            continue;
                        if (delta < -1e-9) {
                            applySwap(sol, {i1, i2, ja, jb, qa, qb, delta});
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    bool isFeasibleSwap(const MSCFLPSolution& sol,
                        int i1, int i2,
                        int ja, int jb,
                        double qa, double qb,
                        double& delta) const
    {
        const double d1 = inst_.getDemand(i1);
        const double d2 = inst_.getDemand(i2);

        // Capacidad: después de retirar i1 de ja, ¿cabe i2?
        double rja_after = sol.getResidualCap(ja) + qa;   // ja libera qa
        double rjb_after = sol.getResidualCap(jb) + qb;   // jb libera qb

        if (rja_after < qb - 1e-9) return false;  // ja no puede absorber i2
        if (rjb_after < qa - 1e-9) return false;  // jb no puede absorber i1

        // Incompatibilidades: i2 en ja (sin i1)
        // El incompCount[i2][ja] ya refleja la situación actual.
        // Si i1 e i2 son incompatibles entre sí, retirar i1 de ja reduce
        // incompCount[i2][ja] en 1.
        bool i1i2Incompat = isIncompatiblePair(i1, i2);
        int incompat_i2_ja = sol.getIncompCount(i2, ja) - (i1i2Incompat ? 1 : 0);
        int incompat_i1_jb = sol.getIncompCount(i1, jb) - (i1i2Incompat ? 1 : 0);

        if (incompat_i2_ja != 0) return false;
        if (incompat_i1_jb != 0) return false;

        // Cálculo de delta (solo transporte; coste fijo no cambia)
        delta = d1 * (inst_.getTransportCost(i1, jb) - inst_.getTransportCost(i1, ja))
              + d2 * (inst_.getTransportCost(i2, ja) - inst_.getTransportCost(i2, jb));

        return true;
    }

    void applySwap(MSCFLPSolution& sol, const SwapMove& mv) const {
        // Retirar i1 de ja e i2 de jb
        sol.removeAssignment(mv.i1, mv.ja);
        sol.removeAssignment(mv.i2, mv.jb);

        // Asignar i1 a jb e i2 a ja
        sol.assignDemand(mv.i1, mv.jb, mv.qa);
        sol.assignDemand(mv.i2, mv.ja, mv.qb);
    }

    bool isIncompatiblePair(int i1, int i2) const {
        for (int nb : inst_.getIncompatibleWith(i1))
            if (nb == i2) return true;
        return false;
    }
};