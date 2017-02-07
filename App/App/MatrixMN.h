/////////////////////////////////////////////////////////////////////////////
// Authored by Jeong-Mo Hong for CSE4060 course at Dongguk University CSE  //
// jeongmo.hong@gmail.com                                                  //
// Do whatever you want license.                                           //
/////////////////////////////////////////////////////////////////////////////

#pragma once

#include "VectorND.h"
#include <assert.h>

template<class T>
class MatrixMN
{
public:
    int num_rows_;  // m_
    int num_cols_;  // n_
    T *values_;

    MatrixMN()
        : values_(nullptr), num_rows_(0), num_cols_(0)
    {}

    void initialize(const int& _m, const int& _n)
    {
        num_rows_ = _m;
        num_cols_ = _n;

        SAFE_DELETE_ARRAY(values_);

        values_ = new T [num_rows_ * num_cols_];

        for (int i = 0; i < num_rows_ * num_cols_; i++)
            values_[i] = (T)0;
    }

    int get1DIndex(const int& row, const int& column)
    {
        assert(row >= 0);
        assert(column >= 0);
        assert(row < num_rows_);
        assert(row < num_cols_);

        // column = i, row = j
        return column + row * num_cols_;        // data structure is for faster dot product of a row vector and VectorND input.
    }

    T& getValue(const int& row, const int& column)
    {
        return values_[get1DIndex(row, column)];
    }

    //T multiplyRowAndVectorWithBias(const int& row, const VectorND<T>& vector, const T& bias)    // (v0, v1, ..., vn-1, bias)
    //{
    //    assert(num_cols_ == vector.num_dimension_ + 1); // +1 is for bias
    //    
    //    T dot = (T)0;
    //    for (int col = 0; col < num_cols_ - 1; col++)   // num_cols_ - 1 : don't operate on bias now.
    //    {
    //        dot += getValue(row, col) * vector[col];
    //    }

    //    dot += getValue(row, num_cols_ - 1) * bias;     // last column value is the weight of bias

    //    return dot;
    //}

    //void multiplyVectorWithBias(const VectorND<T>& vector, const T& bias, VectorND<T>& result)
    //{
    //    assert(num_cols_ == (vector.num_dimension_ + 1));
    //    assert(num_rows_ == result.num_dimension_);

    //    for (int row = 0; row < num_rows_; row++)
    //    {
    //        result[row] = multiplyRowAndVectorWithBias(row, vector, bias);
    //    }
    //}

    //void multiplyTransWithBias(const VectorND<T>& vector, VectorND<T>& result)
    //{
    //    assert(num_rows_ <= vector.num_dimension_); // don't multiply last bias component
    //    assert(num_cols_ == result.num_dimension_);

    //    for (int col = 0; col < num_cols_; col++)
    //    {
    //        result[col] = (T)0;

    //        for (int row = 0; row < num_rows_; row++)
    //        {
    //            result[col] += getValue(row, col) * vector[row];
    //        }
    //    }
    //}

    void multiply(const VectorND<T>& vector, VectorND<T>& result)
    {
        assert(num_rows_ <= result.num_dimension_);
        assert(num_cols_ <= vector.num_dimension_);

        for (int row = 0; row < num_rows_; row++)
        {
            result[row] = (T)0;

            for (int col = 0; col < num_cols_; col++)
            {
                result[row] += getValue(row, col) * vector[col];
                
                //if (std::isnan(result[row]) == true)
                //{
                //    std::cout << "NAN" << std::endl;
                //    exit(1);
                //}
            }
        }
    }

    void multiplyTransposed(const VectorND<T>& vector, VectorND<T>& result)
    {
        assert(num_rows_ <= vector.num_dimension_);
        assert(num_cols_ <= result.num_dimension_);

        for (int col = 0; col < num_cols_; col++)
        {
            result[col] = (T)0;

            for (int row = 0; row < num_rows_; row++)
            {
                result[col] += getValue(row, col) * vector[row];
            }
        }
    }

    void cout()
    {
        for (int row = 0; row < num_rows_; row++)
        {
            for (int col = 0; col < num_cols_; col++)
            {
                std::cout << getValue(row, col) << " ";
            }

            std::cout << std::endl;
        }
    }
};
