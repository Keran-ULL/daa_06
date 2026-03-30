/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file Instance.h
 * @brief  Interfaz abstracta para representar instancias de problemas de optimización.
 *
 * Esta clase define la interfaz común que deben implementar todas las instancias
 * de problemas (VRP, TSP, Knapsack, etc.).  Proporciona métodos para cargar
 * datos desde un fichero, validar la coherencia interna y obtener una
 * descripción legible de la instancia.
 *
 * @author Keran
 * @version 1.0
 * @date 2025-06-01
 */

#pragma once

#include <string>
#include <stdexcept>


class Instance {
public:
    Instance() = default;

    /**
     * @brief Destructor virtual para permitir herencia polimórfica segura.
     */
    virtual ~Instance() = default;

    Instance(const Instance&)            = delete;
    Instance& operator=(const Instance&) = delete;
    Instance(Instance&&)                 = default;
    Instance& operator=(Instance&&)      = default;

    /**
     * @brief  Carga los datos del problema desde un fichero de texto.
     * @param  filepath  Ruta absoluta o relativa al fichero de instancia.
     * @throws std::runtime_error  Si el fichero no existe o tiene formato
     *                             incorrecto.
     *
     * Después de una llamada exitosa a load(), el objeto queda listo para
     * ser usado.  Llamar a load() sobre un objeto ya cargado reinicia su
     * estado interno.
     */
    virtual void load(const std::string& filepath) = 0;

    /**
     * @brief  Valida la coherencia interna de la instancia ya cargada.
     * @throws std::logic_error  Si algún parámetro es inconsistente
     *                           (p.ej. demanda total > capacidad total).
     *
     * Puede llamarse explícitamente o de forma automática al final de load().
     */
    virtual void validate() const = 0;

    /**
     * @brief  Devuelve una cadena de texto con un resumen de la instancia.
     * @return Descripción legible, útil para logs.
     */
    virtual std::string toString() const = 0;

    /**
     * @brief  Devuelve el nombre/ruta de la instancia cargada.
     * @return Cadena vacía si todavía no se ha llamado a load().
     */
    const std::string& getName() const { return instanceName_; }

    /**
     * @brief  Indica si la instancia ha sido cargada correctamente.
     */
    bool isLoaded() const { return loaded_; }

protected:
    /** Nombre o ruta del fichero origen. */
    std::string instanceName_;
    /** Bandera que indica si load() se completó con éxito. */
    bool loaded_ = false;

    /**
     * @brief  Lanza std::runtime_error si la instancia no está cargada.
     *
     * Las subclases deben llamar a este método al inicio de cualquier getter
     * que requiera datos cargados.
     */
    void requireLoaded() const {
        if (!loaded_) {
            throw std::runtime_error(
                "Instance::requireLoaded() — la instancia no ha sido cargada. "
                "Llame a load() antes de acceder a los datos."
            );
        }
    }
};
