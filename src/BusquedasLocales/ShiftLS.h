/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   ShiftLS.h
 * @brief  Implementación de la clase ShiftLS, que representa una búsqueda
 *        local basada en el operador Shift(i, j1, j2) para el problema
 *        MS-CFLP-CI.
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

class ShiftLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit ShiftLS(const MSCFLPInstance& inst,
                     ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "ShiftLS (best)"
               : "ShiftLS (first)";
    }

    void setStrategy(ImprovementStrategy s) { strategy_ = s; }

    /**
     * @brief  Itera aplicando movimientos Shift hasta alcanzar un óptimo local.
     * @return true si se realizó al menos una mejora.
     */
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

    /**
     * @brief  Aplica un único movimiento Shift según la estrategia configurada.
     * @return true si el movimiento mejoró el coste total.
     */
    bool applyBestMove(Solution& solution) override {
        auto& sol = dynamic_cast<MSCFLPSolution&>(solution);
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? applyBest(sol)
               : applyFirst(sol);
    }

private:
    const MSCFLPInstance& inst_;
    ImprovementStrategy   strategy_;

    bool applyBest(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();

        int    bestI = -1, bestJ1 = -1, bestJ2 = -1;
        double bestDelta = -1e-9;   // solo aceptamos mejoras estrictas
        double bestQ     = 0.0;

        for (int i = 0; i < n; ++i) {
            const double di = inst_.getDemand(i);

            for (int j1 = 0; j1 < m; ++j1) {
                if (!sol.isServedBy(i, j1)) continue;

                const double assignedInJ1 = sol.getX(i, j1) * di;

                for (int j2 = 0; j2 < m; ++j2) {
                    if (j2 == j1) continue;
                    if (!sol.isOpen(j2) && !canOpenJ2BeCheaper(sol, j1, j2, assignedInJ1, di))
                        continue;
                    if (sol.getIncompCount(i, j2) != 0) continue;

                    const double rj2 = sol.getResidualCap(j2);
                    if (rj2 <= 1e-9) continue;

                    const double q = std::min(assignedInJ1, rj2);

                    double delta = q * (inst_.getTransportCost(i, j2)
                                      - inst_.getTransportCost(i, j1));

                    // ¿j1 quedará vacía tras el movimiento?
                    bool j1BecomesEmpty =
                        (sol.getClientsOf(j1).size() == 1)   // solo i
                        && (std::abs(assignedInJ1 - q) < 1e-9); // toda la asig.
                    if (j1BecomesEmpty)
                        delta -= inst_.getFixedCost(j1);

                    // ¿j2 estaba cerrada?
                    if (!sol.isOpen(j2))
                        delta += inst_.getFixedCost(j2);

                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestI = i; bestJ1 = j1; bestJ2 = j2;
                        bestQ = q;
                    }
                }
            }
        }

        if (bestI == -1) return false;
        applyShift(sol, bestI, bestJ1, bestJ2, bestQ);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int n = inst_.getN();
        const int m = inst_.getM();

        for (int i = 0; i < n; ++i) {
            const double di = inst_.getDemand(i);

            for (int j1 = 0; j1 < m; ++j1) {
                if (!sol.isServedBy(i, j1)) continue;

                const double assignedInJ1 = sol.getX(i, j1) * di;

                for (int j2 = 0; j2 < m; ++j2) {
                    if (j2 == j1) continue;
                    if (sol.getIncompCount(i, j2) != 0) continue;

                    const double rj2 = sol.getResidualCap(j2);
                    if (rj2 <= 1e-9) continue;

                    const double q = std::min(assignedInJ1, rj2);

                    double delta = q * (inst_.getTransportCost(i, j2)
                                      - inst_.getTransportCost(i, j1));

                    bool j1BecomesEmpty =
                        (sol.getClientsOf(j1).size() == 1)
                        && (std::abs(assignedInJ1 - q) < 1e-9);
                    if (j1BecomesEmpty)
                        delta -= inst_.getFixedCost(j1);

                    if (!sol.isOpen(j2))
                        delta += inst_.getFixedCost(j2);

                    if (delta < -1e-9) {
                        applyShift(sol, i, j1, j2, q);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void applyShift(MSCFLPSolution& sol,
                    int i, int j1, int j2, double q) const
    {
        // Abrir j2 si estaba cerrada
        if (!sol.isOpen(j2))
            sol.openFacility(j2);

        // Retirar q de j1
        const double di = inst_.getDemand(i);
        const double totalInJ1 = sol.getX(i, j1) * di;

        if (std::abs(totalInJ1 - q) < 1e-9) {
            // Quitar toda la asignación de i en j1
            sol.removeAssignment(i, j1);
            // Cerrar j1 si quedó vacía
            if (sol.getClientsOf(j1).empty())
                sol.closeFacility(j1);
        } else {
            // Quitar solo q (parcial): simular restando y reasignando
            // removeAssignment elimina todo; reponemos el resto
            sol.removeAssignment(i, j1);
            if (sol.getClientsOf(j1).empty())
                sol.closeFacility(j1);
            const double resto = totalInJ1 - q;
            if (resto > 1e-9) {
                if (!sol.isOpen(j1)) sol.openFacility(j1);
                sol.assignDemand(i, j1, resto);
            }
        }

        // Asignar q a j2
        sol.assignDemand(i, j2, q);
    }

    bool canOpenJ2BeCheaper(const MSCFLPSolution& sol,
                             int j1, int j2,
                             double assignedInJ1, double di) const
    {
        // Si j1 se vaciara y cerráramos: ahorro fijo j1 - coste fijo j2 > 0
        bool j1Solo = (sol.getClientsOf(j1).size() == 1);
        if (!j1Solo) return false;
        double saving = inst_.getFixedCost(j1) - inst_.getFixedCost(j2);
        return saving > 0;
    }
};
