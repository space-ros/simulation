/*
 * Copyright (C) 2024 Davide Zamblera
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <iostream>
#include <vector>
#include <stdexcept>
#include <initializer_list>
#include <gz/math/Vector3.hh>

namespace simulation
{
    /**
     * @brief A class for matrix representation and operations.
     * This class provides basic matrix operations such as multiplication, addition, transpose,
     * and resizing, along with vector compatibility for 3D vectors.
     */
    class Matrix 
    {
        private:
            /** @brief Matrix data stored as a 2D vector */
            std::vector<std::vector<double>> data;

            /** @brief Number of rows in the matrix */
            size_t rows_;

            /** @brief Number of columns in the matrix */
            size_t cols_;

        public:
            // Constructors

            /**
             * @brief Creates a matrix with all elements initialized to 1.
             * @param rows Number of rows in the matrix.
             * @param cols Number of columns in the matrix.
             * @return A matrix filled with ones.
             */
            static Matrix ones(size_t rows, size_t cols);

            /**
             * @brief Creates a matrix with all elements initialized to 0.
             * @param rows Number of rows in the matrix.
             * @param cols Number of columns in the matrix.
             * @return A matrix filled with zeros.
             */
            static Matrix zeros(size_t rows, size_t cols);

            /**
             * @brief Creates an identity matrix (diagonal elements are 1, others are 0).
             * @param rows Number of rows in the matrix.
             * @param cols Number of columns in the matrix.
             * @return An identity matrix.
             */
            static Matrix eyes(size_t rows, size_t cols);

            /**
             * @brief Constructor that initializes a matrix with a given size and initial value.
             * @param rows Number of rows in the matrix.
             * @param cols Number of columns in the matrix.
             * @param initial Initial value for all elements in the matrix.
             */
            Matrix(size_t rows, size_t cols, double initial = 0.0);

            /**
             * @brief Constructor that initializes a matrix from a 2D vector of values.
             * @param values A 2D vector containing the matrix data.
             */
            Matrix(const std::vector<std::vector<double>>& values);

            /** @brief Default constructor that creates an empty matrix. */
            Matrix();

            /**
             * @brief Constructor that initializes a matrix from a 3D vector.
             * @param vector A 3D vector used to populate the matrix.
             */
            Matrix(const gz::math::Vector3<double>& vector);

            /**
             * @brief Constructor that initializes a matrix using an initializer list.
             * @param initList An initializer list of initializer lists representing matrix rows.
             */
            Matrix(std::initializer_list<std::initializer_list<double>> initList) {
                // Resize the data vector based on the number of rows
                rows_ = initList.size();
                data.resize(rows_);

                // Iterate over each initializer list to populate the matrix
                size_t i = 0;
                for (const auto& row : initList) {
                    data[i] = std::vector<double>(row);
                    ++i;
                }

                // Set the number of columns based on the first row
                cols_ = rows_ > 0 ? data[0].size() : 0;
            }
            
            // Copy Constructor

            /**
             * @brief Copy constructor that initializes a matrix as a copy of another matrix.
             * @param other The matrix to be copied.
             */
            Matrix(const Matrix& other);

            // Operator Overloads

            /**
             * @brief Access operator to retrieve or modify elements in a row of the matrix.
             * @param i Row index.
             * @return A reference to the row (a vector of doubles).
             */
            std::vector<double>& operator[](size_t i);

            /**
             * @brief Access operator to retrieve elements in a row of the matrix (const version).
             * @param i Row index.
             * @return A constant reference to the row.
             */
            const std::vector<double>& operator[](size_t i) const;

            /**
             * @brief Matrix multiplication operator.
             * @param other The matrix to be multiplied.
             * @return The product of the two matrices.
             * @throws std::invalid_argument if matrix dimensions do not allow multiplication.
             */
            Matrix operator*(const Matrix& other) const;

            /**
             * @brief Matrix addition operator.
             * @param other The matrix to be added.
             * @return The sum of the two matrices.
             * @throws std::invalid_argument if matrix dimensions do not match.
             */
            Matrix operator+(const Matrix& other) const;

            /**
             * @brief Matrix subtraction operator.
             * @param other The matrix to be subtracted.
             * @return The difference of the two matrices.
             * @throws std::invalid_argument if matrix dimensions do not match.
             */
            Matrix operator-(const Matrix& other) const;

            /**
             * @brief Matrix-scalar multiplication operator.
             * @param other Scalar value to multiply each element by.
             * @return The resulting matrix.
             */
            Matrix operator*(double other) const;

            /**
             * @brief Matrix-scalar division operator.
             * @param other Scalar value to divide each element by.
             * @return The resulting matrix.
             */
            Matrix operator/(double other) const;

            /**
             * @brief Negation operator to return the negative of the matrix.
             * @return The negated matrix.
             */
            Matrix operator-() const;

            /**
             * @brief Assignment operator to copy the contents of another matrix.
             * @param other The matrix to be assigned.
             * @return A reference to the updated matrix.
             */
            Matrix& operator=(const Matrix& other);

            // Matrix Transpose
            /**
             * @brief Transposes the matrix (swaps rows and columns).
             * @return The transposed matrix.
             */
            Matrix transpose() const;

            /**
             * @brief Append another matrix to the bottom (vertical concatenation).
             * @param other The matrix to be appended.
             * @throws std::invalid_argument if the number of columns does not match.
             */
            void appendVertical(const Matrix& other);

            /**
             * @brief Append another matrix to the right (horizontal concatenation).
             * @param other The matrix to be appended.
             * @throws std::invalid_argument if the number of rows does not match.
             */
            void appendHorizontal(const Matrix& other);

            /**
             * @brief Get the number of rows in the matrix.
             * @return Number of rows.
             */
            size_t rows() const { return rows_; };

            /**
             * @brief Get the number of columns in the matrix.
             * @return Number of columns.
             */
            size_t cols() const { return cols_; };

            // Accessors for 3D vector-style elements

            /** @brief Get the x component of a 3D vector (or the first element). */
            double x() const { return data[0][0]; }

            /** @brief Get the y component of a 3D vector (or the second element). */
            double y() const { return data[1][0]; }

            /** @brief Get the z component of a 3D vector (or the third element). */
            double z() const { return data[2][0]; }

            // Output Stream Overload

            /**
             * @brief Overloads the output stream operator to print the matrix.
             * @param os The output stream.
             * @param matrix The matrix to be printed.
             * @return The output stream with the matrix printed.
             */
            friend std::ostream& operator<<(std::ostream& os, const Matrix& matrix);
    };
}



#endif // MATRIX_HPP