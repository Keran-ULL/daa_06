/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   SwapInstalacionesLS.h
 * @brief  Implementación de la clase SwapInstalacionesLS, que representa una
 *        búsqueda local basada en el operador Swap-Instalaciones(jopen, jclosed)
 *        para el problema MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */
#pragma once

#include "../Plantillas/Algorithm.h"
#include "../Instancia/MSCFLPInstance.h"
#include "../Solucion/MSCFLPSolution.h"

#include <vector>
#include <algorithm>
#include <limits>
#include <string>

class SwapInstalacionesLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit SwapInstalacionesLS(const MSCFLPInstance& inst,
                                  ImprovementStrategy strategy = ImprovementStrategy::BEST_IMPROVEMENT)
        : LocalSearch(inst)
        , inst_(inst)
        , strategy_(strategy)
    {}

    std::string getName() const override {
        return strategy_ == ImprovementStrategy::BEST_IMPROVEMENT
               ? "SwapInstalacionesLS (best)"
               : "SwapInstalacionesLS (first)";
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
        int    jOpen;    // instalación a cerrar
        int    jClosed;  // instalación a abrir
        double delta;    // ganancia (negativo = mejora)
    };

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        // Listas de instalaciones abiertas y cerradas
        std::vector<int> opened, closed;
        for (int j = 0; j < m; ++j)
            (sol.isOpen(j) ? opened : closed).push_back(j);

        SwapMove best;
        best.delta = -1e-9;
        bool found = false;

        for (int jo : opened) {
            for (int jc : closed) {
                double delta;
                if (!evaluateSwap(sol, jo, jc, opened, delta)) continue;
                if (delta < best.delta) {
                    best = {jo, jc, delta};
                    found = true;
                }
            }
        }

        if (!found) return false;
        executeSwap(sol, best.jOpen, best.jClosed, collectOpenExcept(sol, best.jOpen));
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();

        std::vector<int> opened, closed;
        for (int j = 0; j < m; ++j)
            (sol.isOpen(j) ? opened : closed).push_back(j);

        for (int jo : opened) {
            for (int jc : closed) {
                double delta;
                if (!evaluateSwap(sol, jo, jc, opened, delta)) continue;
                if (delta < -1e-9) {
                    executeSwap(sol, jo, jc, collectOpenExcept(sol, jo));
                    return true;
                }
            }
        }
        return false;
    }

    bool evaluateSwap(const MSCFLPSolution& sol,
                      int jOpen, int jClosed,
                      const std::vector<int>& opened,
                      double& delta) const
    {
        // Clientes que hay que reasignar desde jOpen
        const std::vector<int>& clients = sol.getClientsOf(jOpen);
        if (clients.empty()) {
            // jOpen estaba vacía: simplemente cerrarla ahorra su coste fijo
            // pero abrir jClosed lo incrementa. Solo vale si f[jClosed] < f[jOpen].
            delta = inst_.getFixedCost(jClosed) - inst_.getFixedCost(jOpen);
            return true;
        }

        // Simular capacidades residuales de las instalaciones destino
        // (jClosed con su capacidad total; el resto con la actual)
        std::vector<int> targets;
        targets.push_back(jClosed);
        for (int j : opened)
            if (j != jOpen) targets.push_back(j);

        // residualCap simulado (indexado por j directamente)
        const int m = inst_.getM();
        std::vector<double> simRes(m);
        for (int j : targets)
            simRes[j] = (j == jClosed) ? inst_.getCapacity(jClosed)
                                       : sol.getResidualCap(j);

        // Para cada cliente de jOpen, buscar mejor instalación destino
        double deltaTransport = 0.0;

        for (int i : clients) {
            const double di   = inst_.getDemand(i);
            const double qi   = sol.getX(i, jOpen) * di;
            double demRest    = qi;

            // Ordenar targets por c[i][j] ascendente
            std::vector<std::pair<double,int>> sorted;
            sorted.reserve(targets.size());
            for (int j : targets) {
                if (sol.getIncompCount(i, j) != 0 && j != jClosed) continue;
                // Para jClosed, recalcular incompatibilidad sin i en jOpen
                if (j == jClosed) {
                    // Verificar que los incompatibles de i no están en jClosed
                    if (sol.getIncompCount(i, jClosed) != 0) continue;
                }
                sorted.emplace_back(inst_.getTransportCost(i, j), j);
            }
            std::sort(sorted.begin(), sorted.end());

            for (auto& [cost, j] : sorted) {
                if (demRest <= 1e-9) break;
                double available = simRes[j];
                if (available <= 1e-9) continue;
                double q = std::min(demRest, available);
                deltaTransport += q * (inst_.getTransportCost(i, j)
                                      - inst_.getTransportCost(i, jOpen));
                simRes[j] -= q;
                demRest   -= q;
            }

            if (demRest > 1e-9) return false;  // cliente no reasignable → infactible
        }

        delta = (inst_.getFixedCost(jClosed) - inst_.getFixedCost(jOpen))
               + deltaTransport;
        return true;
    }

    void executeSwap(MSCFLPSolution& sol,
                     int jOpen, int jClosed,
                     const std::vector<int>& otherOpen) const
    {
        // Abrir jClosed
        sol.openFacility(jClosed);

        // Reasignar todos los clientes de jOpen
        std::vector<int> clientsCopy = sol.getClientsOf(jOpen);  // copia: se modifica

        // Preparar orden de destinos por coste para cada cliente
        std::vector<int> targets;
        targets.push_back(jClosed);
        for (int j : otherOpen) targets.push_back(j);

        for (int i : clientsCopy) {
            const double di = inst_.getDemand(i);
            double demRest  = sol.getX(i, jOpen) * di;

            sol.removeAssignment(i, jOpen);

            // Ordenar destinos por c[i][j]
            std::vector<std::pair<double,int>> sorted;
            for (int j : targets)
                sorted.emplace_back(inst_.getTransportCost(i, j), j);
            std::sort(sorted.begin(), sorted.end());

            for (auto& [cost, j] : sorted) {
                if (demRest <= 1e-9) break;
                if (sol.getIncompCount(i, j) != 0) continue;
                double rj = sol.getResidualCap(j);
                if (rj <= 1e-9) continue;
                double q = std::min(demRest, rj);
                sol.assignDemand(i, j, q);
                demRest -= q;
            }
        }

        // Cerrar jOpen (debería estar vacía ya)
        if (sol.getClientsOf(jOpen).empty())
            sol.closeFacility(jOpen);
    }

    std::vector<int> collectOpenExcept(const MSCFLPSolution& sol, int jExclude) const {
        std::vector<int> result;
        for (int j = 0; j < inst_.getM(); ++j)
            if (sol.isOpen(j) && j != jExclude)
                result.push_back(j);
        return result;
    }
};
