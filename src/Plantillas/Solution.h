/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   Solution.h
 * @brief  Clase abstracta que representa una solución genérica para un
 *         problema de optimización combinatoria.
 * Modela el estado de una solución junto con su coste y su factibilidad.
 * Las subclases concretas añaden las estructuras de datos específicas
 * del problema (vectores de asignación, flags de apertura, etc.).
 * 
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */

 #pragma once

#include "Instance.h"

#include <memory>
#include <string>
#include <limits>
#include <stdexcept>

/**
 * @file   Solution.h
 * @brief  Clase abstracta que representa una solución genérica para un
 *         problema de optimización combinatoria.
 *
 * Modela el estado de una solución junto con su coste y su factibilidad.
 * Las subclases concretas añaden las estructuras de datos específicas
 * del problema (vectores de asignación, flags de apertura, etc.).
 *
 * Responsabilidades de esta clase:
 *  - Almacenar y exponer el coste total de la solución.
 *  - Guardar una referencia a la instancia que resuelve (nunca null).
 *  - Ofrecer operaciones de copia y comparación polimórficas.
 *  - Definir la interfaz de evaluación y verificación de factibilidad.
 *
 * @author  DAA 2025-2026
 * @version 1.0
 */
class Solution {
public:
    // -------------------------------------------------------------------------
    // Constructor / destructor
    // -------------------------------------------------------------------------

    /**
     * @brief  Constructor base.
     * @param  instance  Instancia del problema que esta solución resuelve.
     *                   La referencia debe seguir siendo válida durante toda
     *                   la vida útil de este objeto.
     */
    explicit Solution(const Instance& instance)
        : instance_(instance)
        , totalCost_(std::numeric_limits<double>::infinity())
        , feasible_(false)
    {}

    virtual ~Solution() = default;

    // Copia y movimiento se delegan a las subclases (pueden ser costosos).
    Solution(const Solution&)            = default;
    Solution& operator=(const Solution&) = default;
    Solution(Solution&&)                 = default;
    Solution& operator=(Solution&&)      = default;

    // -------------------------------------------------------------------------
    // Interfaz pública abstracta
    // -------------------------------------------------------------------------

    /**
     * @brief  Evalúa (o re-evalúa) la solución y actualiza totalCost_.
     *
     * Las subclases deben calcular los costes descompuestos
     * (p.ej. coste fijo + coste de transporte) y fijar totalCost_.
     * También deben actualizar feasible_.
     */
    virtual void evaluate() = 0;

    /**
     * @brief  Comprueba si la solución cumple todas las restricciones.
     * @return true  si la solución es factible, false en caso contrario.
     *
     * La implementación concreta debe verificar capacidades e
     * incompatibilidades, entre otras restricciones del problema.
     * Actualiza internamente feasible_.
     */
    virtual bool checkFeasibility() = 0;

    /**
     * @brief  Crea y devuelve una copia profunda de esta solución.
     *
     * Necesario para que los algoritmos guarden soluciones incumbentes
     * sin romper el encapsulamiento.
     *
     * @return Puntero a una nueva solución idéntica a ésta.
     *         El llamador es responsable de su ciclo de vida.
     */
    virtual std::unique_ptr<Solution> clone() const = 0;

    /**
     * @brief  Devuelve una representación textual de la solución (para logs).
     */
    virtual std::string toString() const = 0;

    // -------------------------------------------------------------------------
    // Getters / setters comunes
    // -------------------------------------------------------------------------

    /** @brief Coste total de la solución (∞ si no ha sido evaluada). */
    double getTotalCost() const { return totalCost_; }

    /** @brief Indica si la solución es factible. */
    bool isFeasible() const { return feasible_; }

    /**
     * @brief  Referencia a la instancia del problema que esta solución resuelve.
     */
    const Instance& getInstance() const { return instance_; }

    /**
     * @brief  Operador de comparación por coste.  Útil para ordenar soluciones.
     */
    bool operator<(const Solution& other) const {
        return totalCost_ < other.totalCost_;
    }

    bool operator>(const Solution& other) const {
        return totalCost_ > other.totalCost_;
    }

protected:
    // -------------------------------------------------------------------------
    // Estado interno protegido
    // -------------------------------------------------------------------------

    /** Referencia a la instancia (inmutable durante la vida del objeto). */
    const Instance& instance_;

    /** Valor de la función objetivo. ∞ = no evaluada todavía. */
    double totalCost_;

    /** true si la solución satisface todas las restricciones. */
    bool feasible_;
};