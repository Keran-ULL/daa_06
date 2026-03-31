/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   MSCFLPSolution.h
 * @brief  Implementación de la clase MSCFLPSolution, que representa una
 *        solución concreta para el problema MS-CFLP-CI.
 * @author Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */

#pragma once

#include "../Plantillas/Solution.h"
#include "../Instancia/MSCFLPInstance.h"   // forward-declarado abajo; incluir el .h completo

#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <memory>
#include <cassert>


class MSCFLPSolution : public Solution {
public:

    /**
     * @brief  Construye una solución vacía (sin instalaciones abiertas ni
     *         clientes asignados) asociada a la instancia dada.
     *
     * Todos los vectores se dimensionan correctamente; los costes se
     * inicializan a 0.  La solución vacía es factible (no viola nada) pero
     * tiene coste 0, que no es válida como solución del problema.
     * Los algoritmos constructivos la rellenan desde cero.
     *
     * @param  inst  Instancia del MS-CFLP-CI ya cargada.
     */
    explicit MSCFLPSolution(const MSCFLPInstance& inst)
        : Solution(inst)                              // registra referencia base
        , inst_(inst)                                 // referencia tipada concreta
        , open_ (inst.getM(), false)
        , x_    (inst.getN(), std::vector<double>(inst.getM(), 0.0))
        , w_    (inst.getN(), std::vector<bool>  (inst.getM(), false))
        , residualCap_(inst.getM())
        , clientsOf_  (inst.getM())
        , facilitiesOf_(inst.getN())
        , incompCount_(inst.getN(), std::vector<int>(inst.getM(), 0))
        , fixedCost_    (0.0)
        , transportCost_(0.0)
    {
        // residualCap inicializada a la capacidad total de cada instalación
        for (int j = 0; j < inst.getM(); ++j)
            residualCap_[j] = inst.getCapacity(j);

        totalCost_ = 0.0;
        feasible_  = false;   // vacía: no cubre toda la demanda → no factible
    }

    /**
     * @brief  Recalcula desde cero todos los costes y actualiza feasible_.
     *
     * Complejidad: O(n·m).
     * Úsalo después de modificaciones masivas; para cambios incrementales
     * (un único movimiento) es más eficiente actualizar los campos afectados
     * directamente desde el operador de entorno correspondiente.
     */
    void evaluate() override {
        fixedCost_     = 0.0;
        transportCost_ = 0.0;
        for (int j = 0; j < inst_.getM(); ++j)
            if (open_[j])
                fixedCost_ += inst_.getFixedCost(j);
        for (int i = 0; i < inst_.getN(); ++i)
            for (int j = 0; j < inst_.getM(); ++j)
                if (w_[i][j])
                    transportCost_ += inst_.getTransportCost(i, j)
                                    * inst_.getDemand(i)
                                    * x_[i][j];
        totalCost_ = fixedCost_ + transportCost_;
        checkFeasibility();   // actualiza feasible_
    }

    /**
     * @brief  Verifica las tres familias de restricciones del MS-CFLP-CI.
     *
     * (1) Satisfacción de demanda: Σj xij = 1  ∀i
     * (2) Capacidad:               residualCap[j] ≥ 0  ∀j
     * (3) Incompatibilidad:        w[i1][j] + w[i2][j] ≤ 1  ∀⟨i1,i2⟩, ∀j
     *
     * @return true si todas se cumplen.
     */
    bool checkFeasibility() override {
        constexpr double EPS = 1e-9;
        // (1) Satisfacción de demanda
        for (int i = 0; i < inst_.getN(); ++i) {
            double sum = 0.0;
            for (int j = 0; j < inst_.getM(); ++j) sum += x_[i][j];
            if (std::abs(sum - 1.0) > EPS) { feasible_ = false; return false; }
        }
        // (2) Capacidad residual no negativa
        for (int j = 0; j < inst_.getM(); ++j)
            if (residualCap_[j] < -EPS) { feasible_ = false; return false; }
        // (3) Incompatibilidades
        for (auto& [i1, i2] : inst_.getIncompatiblePairs())
            for (int j = 0; j < inst_.getM(); ++j)
                if (w_[i1][j] && w_[i2][j]) { feasible_ = false; return false; }
        feasible_ = true;
        return true;
    }

    /**
     * @brief  Copia profunda.  Todos los vectores se copian; la referencia
     *         a la instancia se mantiene compartida (es inmutable).
     */
    std::unique_ptr<Solution> clone() const override {
        return std::make_unique<MSCFLPSolution>(*this);
    }

    /**
     * @brief  Resumen legible de la solución para logs y tablas de resultados.
     */
    std::string toString() const override {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "MSCFLPSolution {\n";
        oss << "  Instancia      : " << inst_.getName()       << "\n";
        oss << "  Instalaciones  : " << inst_.getM()
            << "  |  Clientes: "    << inst_.getN()           << "\n";
        oss << "  Abiertas ("
            << countOpenFacilities() << "): [";
        for (int j = 0; j < inst_.getM(); ++j)
            if (open_[j]) oss << j << " ";
        oss << "]\n";
        oss << "  Coste fijo     : " << fixedCost_     << "\n";
        oss << "  Coste transp.  : " << transportCost_ << "\n";
        oss << "  Coste total    : " << totalCost_     << "\n";
        oss << "  Factible       : " << (feasible_ ? "SÍ" : "NO") << "\n";
        oss << "}";
        return oss.str();
    }

    /** @return true si la instalación j está abierta. */
    bool isOpen(int j) const {
      assertFacility(j); return open_[j];
    }

    /** @return Fracción de la demanda del cliente i cubierta por j (xij). */
    double getX(int i, int j) const {
      assertClient(i); assertFacility(j); return x_[i][j];
    }

    /** @return true si el cliente i es servido (aunque sea parcialmente) por j. */
    bool isServedBy(int i, int j) const {
      assertClient(i); assertFacility(j); return w_[i][j];
    }

    /** @return Capacidad libre de la instalación j. */
    double getResidualCap(int j) const {
        assertFacility(j); return residualCap_[j];
    }

    /**
     * @return Lista de clientes servidos (parcial o totalmente) por j.
     *         Referencia constante; no copiar innecesariamente.
     */
    const std::vector<int>& getClientsOf(int j) const {
      assertFacility(j); return clientsOf_[j];
    }

    /**
     * @return Lista de instalaciones que sirven al cliente i.
     */
    const std::vector<int>& getFacilitiesOf(int i) const {
      assertClient(i); return facilitiesOf_[i];
    }

    /**
     * @return Número de clientes incompatibles con i que están asignados a j.
     *         Útil para verificar rápidamente si i puede asignarse a j.
     */
    int getIncompCount(int i, int j) const {
      assertClient(i); assertFacility(j); return incompCount_[i][j];
    }

    double getFixedCost()     const { return fixedCost_;     }
    double getTransportCost() const { return transportCost_; }
    // totalCost_ está en la clase base Solution → getTotalCost()

    /**
     * @brief  Abre la instalación j (si no lo estaba) y actualiza fixedCost_.
     */
    void openFacility(int j) {
        assertFacility(j);
        if (!open_[j]) {
            open_[j]    = true;
            fixedCost_ += inst_.getFixedCost(j);
            totalCost_  = fixedCost_ + transportCost_;
        }
    }

    /**
     * @brief  Cierra la instalación j.
     *
     * @warning No reasigna a los clientes.  Llame a removeFacilityAssignments()
     *          antes si j tiene clientes asignados.
     */
    void closeFacility(int j) {
        assertFacility(j);
        if (open_[j]) {
            open_[j]    = false;
            fixedCost_ -= inst_.getFixedCost(j);
            totalCost_  = fixedCost_ + transportCost_;
        }
    }

    /**
     * @brief  Asigna una cantidad q de demanda del cliente i a la instalación j.
     *
     * Actualiza de forma incremental: x, w, residualCap, clientsOf,
     * facilitiesOf, incompCount, transportCost y totalCost.
     *
     * @param  i   Índice del cliente  (0-based).
     * @param  j   Índice de la instalación (0-based).
     * @param  q   Cantidad de demanda a asignar (unidades absolutas, no fracción).
     *
     * @pre    La instalación j debe estar abierta.
     * @pre    q > 0 y q ≤ residualCap_[j].
     * @pre    incompCount_[i][j] == 0  (no hay incompatibles de i ya en j).
     */
    void assignDemand(int i, int j, double q) {
        assertClient(i); assertFacility(j);
        assert(open_[j]           && "assignDemand: instalación cerrada");
        assert(q > 0.0            && "assignDemand: q debe ser positivo");
        assert(q <= residualCap_[j] + 1e-9 && "assignDemand: capacidad insuficiente");
        assert(incompCount_[i][j] == 0     && "assignDemand: incompatibilidad violada");
        const double di   = inst_.getDemand(i);
        const double frac = q / di;
        // --- estructuras primarias ---
        x_[i][j] += frac;
        if (!w_[i][j]) {
            w_[i][j] = true;
            // --- estructuras auxiliares: listas ---
            clientsOf_[j].push_back(i);
            facilitiesOf_[i].push_back(j);
            // --- incompCount: actualizar los incompatibles de i en j ---
            //     y los incompatibles de i en las instalaciones de todos
            //     los clientes incompatibles con i
            for (int i2 : inst_.getIncompatibleWith(i)) {
                // Si i2 ya está en j, esto no debería ocurrir (assert arriba lo caza)
                ++incompCount_[i2][j];   // j ya no es válida para i2
            }
        }

        // --- residual y coste ---
        residualCap_[j]  -= q;
        transportCost_   += inst_.getTransportCost(i, j) * q;
        totalCost_        = fixedCost_ + transportCost_;
    }

    /**
     * @brief  Elimina toda la demanda del cliente i de la instalación j.
     *
     * Actualiza de forma incremental todas las estructuras auxiliares.
     *
     * @pre  w_[i][j] == true  (el cliente i debe estar asignado a j).
     */
    void removeAssignment(int i, int j) {
        assertClient(i); assertFacility(j);
        assert(w_[i][j] && "removeAssignment: cliente no estaba en esa instalación");

        const double di = inst_.getDemand(i);
        const double q  = x_[i][j] * di;

        // --- coste y residual ---
        transportCost_   -= inst_.getTransportCost(i, j) * q;
        residualCap_[j]  += q;
        totalCost_        = fixedCost_ + transportCost_;

        // --- estructuras primarias ---
        x_[i][j] = 0.0;
        w_[i][j] = false;

        // --- clientsOf ---
        auto& cv = clientsOf_[j];
        cv.erase(std::remove(cv.begin(), cv.end(), i), cv.end());

        // --- facilitiesOf ---
        auto& fv = facilitiesOf_[i];
        fv.erase(std::remove(fv.begin(), fv.end(), j), fv.end());

        // --- incompCount ---
        for (int i2 : inst_.getIncompatibleWith(i))
            --incompCount_[i2][j];
    }

    /** @return Número de instalaciones abiertas. */
    int countOpenFacilities() const {
        int cnt = 0;
        for (bool b : open_) cnt += b ? 1 : 0;
        return cnt;
    }

    /**
     * @return true si el cliente i puede asignarse a j sin violar capacidad
     *         ni incompatibilidades.
     * @param  q  Demanda que se quiere asignar (unidades absolutas).
     */
    bool canAssign(int i, int j, double q) const {
        assertClient(i); assertFacility(j);
        return open_[j]
            && incompCount_[i][j] == 0
            && residualCap_[j] >= q - 1e-9;
    }

    /**
     * @return Número de pares incompatibles violados en la solución actual.
     *         0 en una solución factible respecto a incompatibilidades.
     */
    int countIncompatibilityViolations() const {
        int violations = 0;
        for (auto& [i1, i2] : inst_.getIncompatiblePairs())
            for (int j = 0; j < inst_.getM(); ++j)
                if (w_[i1][j] && w_[i2][j]) ++violations;
        return violations;
    }

    const MSCFLPInstance& getMSCFLPInstance() const { return inst_; }

private:

    const MSCFLPInstance& inst_;   ///< Referencia tipada (además de la de Solution)

    // --- Estructuras primarias ---
    std::vector<bool>                 open_;   ///< open[j]
    std::vector<std::vector<double>>  x_;      ///< x[i][j]
    std::vector<std::vector<bool>>    w_;      ///< w[i][j]

    // --- Estructuras auxiliares ---
    std::vector<double>               residualCap_;   ///< residualCap[j]
    std::vector<std::vector<int>>     clientsOf_;     ///< clientsOf[j]
    std::vector<std::vector<int>>     facilitiesOf_;  ///< facilitiesOf[i]
    std::vector<std::vector<int>>     incompCount_;   ///< incompCount[i][j]

    // --- Coste descompuesto ---
    double fixedCost_;       ///< Σj∈Jopen fj
    double transportCost_;   ///< Σi Σj cij·di·xij
    // totalCost_ heredado de Solution

    inline void assertClient(int i) const {
        assert(i >= 0 && i < inst_.getN() && "Índice de cliente fuera de rango");
    }
    inline void assertFacility(int j) const {
        assert(j >= 0 && j < inst_.getM() && "Índice de instalación fuera de rango");
    }
};
