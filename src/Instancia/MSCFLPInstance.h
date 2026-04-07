/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * Asignatura: Diseño y Análisis de Algoritmos
 * 
 * @file   MSCFLPSolution.h
 * @brief  Implementación de la clase MSCFLPSolution, que representa una
 *       solución concreta para el problema MS-CFLP-CI.
 * @author  Keran Miranda González
 * @version 1.0
 * @date 2025-06-01
 */

#pragma once

#include "../Plantillas/Instance.h"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <set>

class MSCFLPInstance : public Instance {
public:

    /// Par de clientes incompatibles (ambos índices 0-based).
    using IncompatPair = std::pair<int, int>;

    MSCFLPInstance() = default;
    ~MSCFLPInstance() override = default;

    /**
     * @brief  Carga la instancia desde un fichero .dzn.
     * @param  filepath  Ruta al fichero (p.ej. "instances/wlp05.dzn").
     * @throws std::runtime_error si el fichero no existe o el formato es incorrecto.
     */
    void load(const std::string& filepath) override {
        std::ifstream file(filepath);
        if (!file.is_open())
            throw std::runtime_error("MSCFLPInstance::load: no se puede abrir '" + filepath + "'");

        // Extraer nombre de instancia del path (sin directorio ni extensión)
        instanceName_ = extractName(filepath);

        // Leer todo el contenido del fichero en un string para facilitar el parsing
        std::string content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
        file.close();

        // Parsear cada campo en orden
        m_ = parseScalar(content, "Warehouses");
        n_ = parseScalar(content, "Stores");

        capacity_   = parseVector(content, "Capacity");
        fixedCost_  = parseVectorDouble(content, "FixedCost");
        demand_     = parseVectorDouble(content, "Goods");
        transportCost_ = parseMatrix(content, "SupplyCost", n_, m_);
        int numPairs = parseScalar(content, "Incompatibilities");
        incompatiblePairs_ = parsePairs(content, "IncompatiblePairs");

        // el número de pares debe coincidir con Incompatibilities
        if (static_cast<int>(incompatiblePairs_.size()) != numPairs) {
            throw std::runtime_error(
                "MSCFLPInstance::load: número de pares incompatibles (" +
                std::to_string(incompatiblePairs_.size()) +
                ") no coincide con Incompatibilities (" +
                std::to_string(numPairs) + ")");
        }

        // Construir índice inverso: incompatibleWith_[i] = lista de clientes
        // incompatibles con i (para acceso O(1) durante la búsqueda)
        buildIncompatibilityIndex();

        loaded_ = true;
    }

    /**
     * @brief  Validación semántica básica de la instancia cargada.
     * @throws std::logic_error si algún parámetro es inconsistente.
     */
    void validate() const override {
        requireLoaded();

        auto fail = [](const std::string& msg) {
            throw std::logic_error("MSCFLPInstance::validate: " + msg);
        };

        if (m_ <= 0 || n_ <= 0)
            fail("m y n deben ser positivos");
        if (static_cast<int>(capacity_.size())      != m_) fail("tamaño de Capacity incorrecto");
        if (static_cast<int>(fixedCost_.size())     != m_) fail("tamaño de FixedCost incorrecto");
        if (static_cast<int>(demand_.size())        != n_) fail("tamaño de Goods incorrecto");
        if (static_cast<int>(transportCost_.size()) != n_) fail("número de filas de SupplyCost incorrecto");

        for (auto& row : transportCost_)
            if (static_cast<int>(row.size()) != m_)
                fail("número de columnas de SupplyCost incorrecto");

        for (double s : capacity_)  if (s <= 0)  fail("capacidad no positiva");
        for (double d : demand_)    if (d <= 0)  fail("demanda no positiva");
        for (double f : fixedCost_) if (f <  0)  fail("coste fijo negativo");

        for (auto& [i1, i2] : incompatiblePairs_) {
            if (i1 < 0 || i1 >= n_ || i2 < 0 || i2 >= n_ || i1 == i2)
                fail("par incompatible fuera de rango");
        }
    }

    /**
     * @brief  Versión que devuelve bool (para uso interno sin excepciones).
     */
    bool isValid() const noexcept {
        try { validate(); return true; }
        catch (...) { return false; }
    }

    /**
     * @brief  Resumen legible de la instancia (para logs).
     */
    std::string toString() const override {
        requireLoaded();
        std::ostringstream oss;
        oss << "MSCFLPInstance {\n"
            << "  Nombre         : " << instanceName_          << "\n"
            << "  Instalaciones  : " << m_                     << "\n"
            << "  Clientes       : " << n_                     << "\n"
            << "  Pares incompat.: " << incompatiblePairs_.size() << "\n"
            << "}";
        return oss.str();
    }

    /** @return Número de instalaciones (m). */
    int getM() const { requireLoaded(); return m_; }

    /** @return Número de clientes (n). */
    int getN() const { requireLoaded(); return n_; }

    /** @return Nombre de la instancia (extraído del path del fichero). */
    const std::string& getName() const { requireLoaded(); return instanceName_; }

    /**
     * @return Capacidad de la instalación j (sj).
     * @pre    0 ≤ j < m
     */
    double getCapacity(int j) const {
        requireLoaded();
        assertFacility(j);
        return capacity_[j];
    }

    /**
     * @return Coste fijo de abrir la instalación j (fj).
     * @pre    0 ≤ j < m
     */
    double getFixedCost(int j) const {
        requireLoaded();
        assertFacility(j);
        return fixedCost_[j];
    }

    /**
     * @return Demanda del cliente i (di).
     * @pre    0 ≤ i < n
     */
    double getDemand(int i) const {
        requireLoaded();
        assertClient(i);
        return demand_[i];
    }

    /**
     * @return Coste unitario de transporte de la instalación j al cliente i (cij).
     *         Es decir, el coste por unidad de demanda servida.
     * @pre    0 ≤ i < n,  0 ≤ j < m
     */
    double getTransportCost(int i, int j) const {
        requireLoaded();
        assertClient(i);
        assertFacility(j);
        return transportCost_[i][j];
    }

    /**
     * @return Lista inmutable de todos los pares incompatibles (0-based).
     *         Cada par ⟨i1, i2⟩ garantiza i1 < i2.
     */
    const std::vector<IncompatPair>& getIncompatiblePairs() const {
        requireLoaded();
        return incompatiblePairs_;
    }

    /**
     * @return Lista de clientes incompatibles con el cliente i (0-based).
     *         Acceso O(1); útil en los operadores de entorno.
     * @pre    0 ≤ i < n
     */
    const std::vector<int>& getIncompatibleWith(int i) const {
        requireLoaded();
        assertClient(i);
        return incompatibleWith_[i];
    }

    const std::vector<double>& getCapacities()  const { requireLoaded(); return capacity_;   }
    const std::vector<double>& getFixedCosts()  const { requireLoaded(); return fixedCost_;  }
    const std::vector<double>& getDemands()     const { requireLoaded(); return demand_;     }

    /**
     * @return Matriz de costes de transporte [n][m].
     *         transportCost_[i][j] = coste unitario de j→i.
     */
    const std::vector<std::vector<double>>& getTransportCosts() const {
        requireLoaded();
        return transportCost_;
    }

private:

    int m_ = 0;   ///< Número de instalaciones
    int n_ = 0;   ///< Número de clientes

    std::vector<double> capacity_;        ///< s[j]  — capacidad de instalación j
    std::vector<double> fixedCost_;       ///< f[j]  — coste fijo de instalación j
    std::vector<double> demand_;          ///< d[i]  — demanda del cliente i
    std::vector<std::vector<double>> transportCost_;  ///< c[i][j]

    std::vector<IncompatPair> incompatiblePairs_;   ///< Todos los pares (i1<i2, 0-based)
    std::vector<std::vector<int>> incompatibleWith_; ///< incompatibleWith_[i]: vecinos de i

    /**
     * @brief  Extrae el nombre de la instancia del path del fichero.
     *         Ejemplo: "data/instances/wlp05.dzn" → "wlp05"
     */
    static std::string extractName(const std::string& filepath) {
        size_t slashPos = filepath.find_last_of("/\\");
        std::string filename = (slashPos == std::string::npos)
                               ? filepath
                               : filepath.substr(slashPos + 1);
        size_t dotPos = filename.rfind('.');
        if (dotPos != std::string::npos)
            filename = filename.substr(0, dotPos);
        return filename;
    }

    /**
     * @brief  Extrae el texto del lado derecho de "Key = VALUE;"
     *         Ignora espacios y saltos de línea en el valor.
     */
    static std::string extractRHS(const std::string& content, const std::string& key) {
        // Buscar "Key ="  (con posibles espacios)
        std::string pattern = key + " =";
        size_t pos = content.find(pattern);
        if (pos == std::string::npos)
            throw std::runtime_error("MSCFLPInstance: clave '" + key + "' no encontrada");
        pos = content.find('=', pos) + 1;
        size_t end = content.find(';', pos);
        if (end == std::string::npos)
            throw std::runtime_error("MSCFLPInstance: falta ';' tras '" + key + "'");

        return content.substr(pos, end - pos);
    }

    /**
     * @brief  Parsea un escalar entero:  Key = N;
     */
    static int parseScalar(const std::string& content, const std::string& key) {
        std::string rhs = extractRHS(content, key);
        try {
            return std::stoi(rhs);
        } catch (...) {
            throw std::runtime_error("MSCFLPInstance: valor escalar inválido para '" + key + "'");
        }
    }

    /**
     * @brief  Parsea un vector de enteros:  Key = [v0, v1, ...];
     *         Devuelve vector<double> para uniformidad.
     */
    static std::vector<double> parseVector(const std::string& content, const std::string& key) {
        std::string rhs = extractRHS(content, key);
        size_t lb = rhs.find('['); size_t rb = rhs.rfind(']');
        if (lb == std::string::npos || rb == std::string::npos)
            throw std::runtime_error("MSCFLPInstance: falta '[' o ']' en '" + key + "'");
        rhs = rhs.substr(lb + 1, rb - lb - 1);

        return tokenizeDoubles(rhs);
    }

    /**
     * @brief  Alias de parseVector (ambos devuelven vector<double>).
     */
    static std::vector<double> parseVectorDouble(const std::string& content, const std::string& key) {
        return parseVector(content, key);
    }

    /**
     * @brief  Parsea la matriz de costes de suministro en formato MiniZinc:
     *           SupplyCost = [|row0|row1|...|row_{n-1}|];
     *         Cada fila i contiene m valores separados por comas.
     *         Las filas están separadas por '|'.
     *
     * @param  rows  Número esperado de filas (n).
     * @param  cols  Número esperado de columnas (m).
     */
    static std::vector<std::vector<double>> parseMatrix(
            const std::string& content,
            const std::string& key,
            int rows, int cols)
    {
        std::string rhs = extractRHS(content, key);
        // Eliminar el par [| ... |]
        size_t lb = rhs.find("[|");
        size_t rb = rhs.rfind("|]");
        if (lb == std::string::npos || rb == std::string::npos)
            throw std::runtime_error("MSCFLPInstance: formato de matriz inválido en '" + key + "'");
        rhs = rhs.substr(lb + 2, rb - lb - 2);

        std::vector<std::vector<double>> matrix;
        matrix.reserve(rows);

        std::istringstream ss(rhs);
        std::string rowStr;
        while (std::getline(ss, rowStr, '|')) {
            rowStr.erase(0, rowStr.find_first_not_of(" \t\n\r"));
            rowStr.erase(rowStr.find_last_not_of(" \t\n\r") + 1);
            if (rowStr.empty()) continue;

            auto vals = tokenizeDoubles(rowStr);
            if (static_cast<int>(vals.size()) != cols)
                throw std::runtime_error(
                    "MSCFLPInstance: fila de matriz con " +
                    std::to_string(vals.size()) +
                    " valores, se esperaban " + std::to_string(cols));
            matrix.push_back(std::move(vals));
        }

        if (static_cast<int>(matrix.size()) != rows)
            throw std::runtime_error(
                "MSCFLPInstance: matriz con " + std::to_string(matrix.size()) +
                " filas, se esperaban " + std::to_string(rows));

        return matrix;
    }

    /**
     * @brief  Parsea la lista de pares incompatibles en formato MiniZinc:
     *           IncompatiblePairs = [| i1, i2 | i1, i2 | ...|];
     *         Los índices en el fichero son 1-based → se convierten a 0-based.
     *         Se garantiza i1 < i2 en cada par devuelto.
     */
    std::vector<IncompatPair> parsePairs(
            const std::string& content,
            const std::string& key) const
    {
        std::string rhs = extractRHS(content, key);

        size_t lb = rhs.find("[|");
        size_t rb = rhs.rfind("|]");
        if (lb == std::string::npos || rb == std::string::npos)
            throw std::runtime_error(
                "MSCFLPInstance: formato de pares inválido en '" + key + "'");
        rhs = rhs.substr(lb + 2, rb - lb - 2);

        std::vector<IncompatPair> pairs;

        std::istringstream ss(rhs);
        std::string pairStr;
        while (std::getline(ss, pairStr, '|')) {
            pairStr.erase(0, pairStr.find_first_not_of(" \t\n\r"));
            pairStr.erase(pairStr.find_last_not_of(" \t\n\r") + 1);
            if (pairStr.empty()) continue;

            // Debe contener exactamente dos enteros separados por coma
            size_t comma = pairStr.find(',');
            if (comma == std::string::npos)
                throw std::runtime_error(
                    "MSCFLPInstance: par inválido (sin coma): '" + pairStr + "'");

            int i1, i2;
            try {
                i1 = std::stoi(pairStr.substr(0, comma))    - 1; 
                i2 = std::stoi(pairStr.substr(comma + 1))   - 1;
            } catch (...) {
                throw std::runtime_error(
                    "MSCFLPInstance: par inválido (valor no entero): '" + pairStr + "'");
            }

            if (i1 < 0 || i1 >= n_ || i2 < 0 || i2 >= n_)
                throw std::runtime_error(
                    "MSCFLPInstance: par fuera de rango (" +
                    std::to_string(i1+1) + ", " + std::to_string(i2+1) + ")");

            // Normalizar para que i1 < i2
            if (i1 > i2) std::swap(i1, i2);
            pairs.emplace_back(i1, i2);
        }

        return pairs;
    }

    /**
     * @brief  Construye el índice inverso incompatibleWith_[i] a partir de
     *         incompatiblePairs_.  Complejidad: O(K) donde K = número de pares.
     */
    void buildIncompatibilityIndex() {
        incompatibleWith_.assign(n_, {});
        for (auto& [i1, i2] : incompatiblePairs_) {
            incompatibleWith_[i1].push_back(i2);
            incompatibleWith_[i2].push_back(i1);
        }
    }

    /**
     * @brief  Extrae todos los números en coma flotante de una cadena.
     *         Ignora cualquier carácter no numérico (espacios, saltos, etc.).
     */
    static std::vector<double> tokenizeDoubles(const std::string& str) {
        std::vector<double> result;
        std::istringstream ss(str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // Eliminar espacios y saltos de línea
            token.erase(std::remove_if(token.begin(), token.end(),
                        [](char c){ return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }),
                        token.end());
            if (token.empty()) continue;
            try {
                result.push_back(std::stod(token));
            } catch (...) {
                throw std::runtime_error(
                    "MSCFLPInstance: token no numérico '" + token + "'");
            }
        }
        return result;
    }

    inline void assertFacility(int j) const {
        if (j < 0 || j >= m_)
            throw std::out_of_range(
                "MSCFLPInstance: índice de instalación " + std::to_string(j) +
                " fuera de rango [0," + std::to_string(m_) + ")");
    }

    inline void assertClient(int i) const {
        if (i < 0 || i >= n_)
            throw std::out_of_range(
                "MSCFLPInstance: índice de cliente " + std::to_string(i) +
                " fuera de rango [0," + std::to_string(n_) + ")");
    }
};