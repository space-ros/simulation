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
    class Matrix 
    {
        private:
            std::vector<std::vector<double>> data;
            size_t rows_, cols_;

        public:
            // Constructors
            static Matrix ones(size_t rows, size_t cols);
            static Matrix zeros(size_t rows, size_t cols);
            static Matrix eyes(size_t rows, size_t cols);
            Matrix(size_t rows, size_t cols, double initial = 0.0);
            Matrix(const std::vector<std::vector<double>>& values);
            Matrix();
            Matrix(const gz::math::Vector3<double>& vector);
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
            //Matrix(const std::vector<double>& values);
            
            // Copy Constructor
            Matrix(const Matrix& other);

            // Operator Overloads
            std::vector<double>& operator[](size_t i);
            const std::vector<double>& operator[](size_t i) const;

            Matrix operator*(const Matrix& other) const;
            Matrix operator+(const Matrix& other) const;
            Matrix operator-(const Matrix& other) const;

            Matrix operator*(double other) const;
            Matrix operator/(double other) const;

            Matrix operator-() const;

            Matrix& operator=(const Matrix& other);

            // Matrix Transpose
            Matrix transpose() const;

            void appendVertical(const Matrix& other);

            void appendHorizontal(const Matrix& other);

            size_t rows() const { return rows_; };

            size_t cols() const { return cols_; };

            double x() const { return data[0][0]; }
            double y() const { return data[1][0]; }
            double z() const { return data[2][0]; }
            double p() const { return data[0][0]; }
            double q() const { return data[1][0]; }
            double r() const { return data[2][0]; }
            double u() const { return data[0][0]; }
            double v() const { return data[1][0]; }
            double w() const { return data[2][0]; }

            // Output Stream Overload
            friend std::ostream& operator<<(std::ostream& os, const Matrix& matrix);
    };
}



#endif // MATRIX_HPP