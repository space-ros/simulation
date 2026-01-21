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

#include "Matrix.hpp"

using namespace simulation;

// Constructors
Matrix::Matrix(size_t rows, size_t cols, double initial)
    : rows_(rows), cols_(cols), data(rows, std::vector<double>(cols, initial)) {}

Matrix Matrix::ones(size_t rows, size_t cols)
{
    // Create a 2D vector filled with 1.0
    std::vector<std::vector<double>> data(rows, std::vector<double>(cols, 1.0));

    // Construct and return the Matrix object
    return Matrix(data);
}

Matrix Matrix::zeros(size_t rows, size_t cols)
{
    // Create a 2D vector filled with 1.0
    std::vector<std::vector<double>> data(rows, std::vector<double>(cols, 0.0));

    // Construct and return the Matrix object
    return Matrix(data);
}

Matrix Matrix::eyes(size_t rows, size_t cols)
{
    // Create a 2D vector initialized to zeros
    std::vector<std::vector<double>> data(rows, std::vector<double>(cols, 0.0));

    // Set the diagonal elements to 1.0
    for (size_t i = 0; i < std::min(rows, cols); ++i) {
        data[i][i] = 1.0;
    }

    // Construct and return the Matrix object
    return Matrix(data);
}

Matrix::Matrix(const std::vector<std::vector<double>> &values)
    : rows_(values.size()), cols_(values[0].size()), data(values) {}

// Copy Constructor
Matrix::Matrix(const Matrix& other)
    : rows_(other.rows_), cols_(other.cols_), data(other.data) {}

Matrix::Matrix() : rows_(0), cols_(0), data()
{
}

Matrix::Matrix(const gz::math::Vector3<double> &vector) : rows_(3), cols_(1), data({{vector.X(), vector.Y(), vector.Z()}})
{
    
}

// Operator Overloads
std::vector<double>& Matrix::operator[](size_t i) {
    return data[i];
}

const std::vector<double>& Matrix::operator[](size_t i) const {
    return data[i];
}

Matrix Matrix::operator*(const Matrix& other) const {
    if (cols_ != other.rows_) {
        throw std::invalid_argument("Matrix dimensions do not match for multiplication.");
    }
    Matrix result(rows_, other.cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < other.cols_; ++j) {
            for (size_t k = 0; k < cols_; ++k) {
                result[i][j] += data[i][k] * other[k][j];
            }
        }
    }
    return result;
}

Matrix Matrix::operator+(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions do not match for addition.");
    }
    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result[i][j] = data[i][j] + other[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix dimensions do not match for subtraction.");
    }
    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result[i][j] = data[i][j] - other[i][j];
        }
    }
    return result;
}

Matrix& Matrix::operator=(const Matrix& other) {
    if (this == &other) return *this; // self-assignment check
    rows_ = other.rows_;
    cols_ = other.cols_;
    data = other.data;
    return *this;
}

// Matrix Transpose
Matrix Matrix::transpose() const {
    Matrix result(cols_, rows_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result[j][i] = data[i][j];
        }
    }
    return result;
}

// Output Stream Overload
std::ostream& operator<<(std::ostream& os, const Matrix& matrix) {
    for (size_t i = 0; i < matrix.rows(); ++i) {
        for (size_t j = 0; j < matrix.cols(); ++j) {
            os << matrix[i][j] << " ";
        }
        os << std::endl;
    }
    return os;
}


// Overload the multiplication operator
Matrix Matrix::operator*(double other) const {
    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result.data[i][j] = data[i][j] * other;
        }
    }
    return result;
}

// Overload the division operator using the multiplication operator
Matrix Matrix::operator/(double other) const {
    if (other == 0.0) {
        throw std::invalid_argument("Division by zero is not allowed.");
    }
    return (*this) * (1.0 / other);
}

Matrix Matrix::operator-() const {
    Matrix result(rows_, cols_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result[i][j] = -data[i][j];
        }
    }
    return result;
}

// Append a matrix vertically (add rows)
void Matrix::appendVertical(const Matrix& other) {
    if (cols_ != other.cols_) {
        throw std::invalid_argument("Matrix appendVertical: column count mismatch.");
    }

    data.insert(data.end(), other.data.begin(), other.data.end());
    rows_ += other.rows_;
}

// Append a matrix horizontally (add columns)
void Matrix::appendHorizontal(const Matrix& other) {
    if (rows_ != other.rows_) {
        throw std::invalid_argument("Matrix appendHorizontal: row count mismatch.");
    }

    for (size_t i = 0; i < rows_; ++i) {
        data[i].insert(data[i].end(), other[i].begin(), other[i].end());
    }
    cols_ += other.cols_;
}