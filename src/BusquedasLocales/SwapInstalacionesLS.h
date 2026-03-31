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
#include <numeric>

class SwapInstalacionesLS : public LocalSearch {
public:
    enum class ImprovementStrategy { FIRST_IMPROVEMENT, BEST_IMPROVEMENT };

    explicit SwapInstalacionesLS(const MSCFLPInstance& inst,
                                  ImprovementStrategy strategy
                                      = ImprovementStrategy::BEST_IMPROVEMENT)
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
        return applyBestMove(solution);
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

    struct SwapMove { int jOpen, jClosed; double delta; };

    using ClientOrder = std::vector<std::vector<int>>;

    bool applyBest(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        std::vector<int> opened, closed;
        opened.reserve(m); closed.reserve(m);
        for (int j = 0; j < m; ++j)
            (sol.isOpen(j) ? opened : closed).push_back(j);
        if (opened.empty() || closed.empty()) return false;
        ClientOrder sortedByClient = buildClientOrders(sol, opened);
        std::vector<double> simRes(m, 0.0);

        SwapMove best{-1, -1, -1e-9};
        bool found = false;

        for (int jo : opened) {
            for (int jc : closed) {
                double delta;
                if (!evaluateSwap(sol, jo, jc, opened, sortedByClient, simRes, delta))
                    continue;
                if (delta < best.delta) { best = {jo, jc, delta}; found = true; }
            }
        }

        if (!found) return false;
        executeSwap(sol, best.jOpen, best.jClosed, opened);
        return true;
    }

    bool applyFirst(MSCFLPSolution& sol) {
        const int m = inst_.getM();
        std::vector<int> opened, closed;
        opened.reserve(m); closed.reserve(m);
        for (int j = 0; j < m; ++j)
            (sol.isOpen(j) ? opened : closed).push_back(j);
        if (opened.empty() || closed.empty()) return false;

        ClientOrder sortedByClient = buildClientOrders(sol, opened);
        std::vector<double> simRes(m, 0.0);

        for (int jo : opened) {
            for (int jc : closed) {
                double delta;
                if (!evaluateSwap(sol, jo, jc, opened, sortedByClient, simRes, delta))
                    continue;
                if (delta < -1e-9) {
                    executeSwap(sol, jo, jc, opened);
                    return true;
                }
            }
        }
        return false;
    }

    ClientOrder buildClientOrders(const MSCFLPSolution& sol,
                                   const std::vector<int>& opened) const
    {
        const int n = inst_.getN();
        ClientOrder orders(n);
        for (int i = 0; i < n; ++i) {
            if (sol.getFacilitiesOf(i).empty()) continue;
            orders[i].resize(opened.size());
            std::iota(orders[i].begin(), orders[i].end(), 0);
            std::sort(orders[i].begin(), orders[i].end(), [&](int a, int b) {
                return inst_.getTransportCost(i, opened[a])
                     < inst_.getTransportCost(i, opened[b]);
            });
        }
        return orders;
    }

    bool evaluateSwap(const MSCFLPSolution& sol,
                      int jOpen, int jClosed,
                      const std::vector<int>& opened,
                      const ClientOrder&      sortedByClient,
                      std::vector<double>&    simRes,
                      double&                 delta) const
    {
        const std::vector<int>& clients = sol.getClientsOf(jOpen);

        if (clients.empty()) {
            delta = inst_.getFixedCost(jClosed) - inst_.getFixedCost(jOpen);
            return true;
        }

        // Rellenar simRes para los targets (OPTIMIZACIÓN 3)
        std::vector<int> targets;
        targets.reserve(opened.size());
        targets.push_back(jClosed);
        simRes[jClosed] = inst_.getCapacity(jClosed);
        for (int j : opened) {
            if (j == jOpen) continue;
            targets.push_back(j);
            simRes[j] = sol.getResidualCap(j);
        }

        double deltaTransport = 0.0;
        bool feasible = true;

        for (int i : clients) {
            const double qi = sol.getX(i, jOpen) * inst_.getDemand(i);
            double demRest  = qi;

            // Intentar jClosed primero
            if (sol.getIncompCount(i, jClosed) == 0 && simRes[jClosed] > 1e-9) {
                double q = std::min(demRest, simRes[jClosed]);
                deltaTransport += q * (inst_.getTransportCost(i, jClosed)
                                     - inst_.getTransportCost(i, jOpen));
                simRes[jClosed] -= q;
                demRest         -= q;
            }

            // Recorrer abiertas en orden de coste (sin sort: ya precalculado)
            if (demRest > 1e-9 && !sortedByClient[i].empty()) {
                for (int idx : sortedByClient[i]) {
                    if (demRest <= 1e-9) break;
                    int j = opened[idx];
                    if (j == jOpen || j == jClosed) continue;
                    if (sol.getIncompCount(i, j) != 0) continue;
                    if (simRes[j] <= 1e-9) continue;
                    double q = std::min(demRest, simRes[j]);
                    deltaTransport += q * (inst_.getTransportCost(i, j)
                                         - inst_.getTransportCost(i, jOpen));
                    simRes[j] -= q;
                    demRest   -= q;
                }
            }

            if (demRest > 1e-9) { feasible = false; break; }
        }

        // Restaurar simRes a 0 (OPTIMIZACIÓN 3)
        for (int j : targets) simRes[j] = 0.0;

        if (!feasible) return false;
        delta = (inst_.getFixedCost(jClosed) - inst_.getFixedCost(jOpen))
               + deltaTransport;
        return true;
    }

    void executeSwap(MSCFLPSolution& sol,
                     int jOpen, int jClosed,
                     const std::vector<int>& opened) const
    {
        sol.openFacility(jClosed);

        std::vector<int> clientsCopy = sol.getClientsOf(jOpen);

        // Destinos ordenables por cliente
        std::vector<std::pair<double,int>> targets;
        targets.reserve(opened.size());
        targets.emplace_back(0.0, jClosed);
        for (int j : opened)
            if (j != jOpen) targets.emplace_back(0.0, j);

        for (int i : clientsCopy) {
            double demRest = sol.getX(i, jOpen) * inst_.getDemand(i);
            sol.removeAssignment(i, jOpen);

            for (auto& [cost, j] : targets)
                cost = inst_.getTransportCost(i, j);
            std::sort(targets.begin(), targets.end());

            for (auto& [cost, j] : targets) {
                if (demRest <= 1e-9) break;
                if (sol.getIncompCount(i, j) != 0) continue;
                double rj = sol.getResidualCap(j);
                if (rj <= 1e-9) continue;
                double q = std::min(demRest, rj);
                sol.assignDemand(i, j, q);
                demRest -= q;
            }
        }

        if (sol.getClientsOf(jOpen).empty())
            sol.closeFacility(jOpen);
    }
};
