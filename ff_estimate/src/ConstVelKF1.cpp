//Soorya Sridharan - Completed 12/28/23
//Uses for loops and std::array - Does Not Include Publisher
#include <iostream>
#include <cmath>
#include <array>

template <size_t RowsA, size_t ColsA, size_t RowsB, size_t ColsB>
auto matrix_multiply(const std::array<std::array<float, ColsA>, RowsA>& a, const std::array<std::array<float, ColsB>, RowsB>& b) {
  std::array<std::array<float, ColsB>, RowsA> result{};
  for (int i = 0; i < RowsA; ++i) {
    //float result_id = 0.0;
    for (int j = 0; j < ColsB; ++j) {
      //result_id = result_id + (a[i][j] * b[i]w[j]);
      //std::cout << result_id;
      //std::cout << std::endl;
        result[i][j] = 0;
      for (int z = 0; z < ColsA; ++z) {
        result[i][j] += a[i][z] * b[z][j];;
      }
    }


  }
  return result;
}



template <typename T, size_t Rows, size_t Cols>
std::array<std::array<T, Rows>, Cols> transpose(const std::array<std::array<T, Cols>, Rows>& original_matrix) {
  std::array<std::array<T, Rows>, Cols> result{};
  for (int row = 0; row < Rows; row++) {
    for (int col = 0; col < Cols; col++) {
    result[col][row] = original_matrix[row][col];
  }
  }
  return result;
}


template <size_t Rows, size_t Cols, size_t ColsD>
auto diagonal_matrix_offset(const std::array<float, ColsD>& diagonal_values, int dia_offset) {
  std::array<std::array<float, Cols>, Rows> result{};
  for (int i = 0; i < ColsD; i++) {
    if (dia_offset < 0) {
      result[i-dia_offset][i] = diagonal_values[i];
    }
    else {
      result[i][i+dia_offset] = diagonal_values[i];
    }
  }
  /*for (int i = result_row; i < Rows; i++) {
      for (int j = result_col; j < Cols; j++) {
        result[i][j] = {0};      
        std::cout << i;
      }
  }*/
  return result;
}

template<typename T, size_t Rows, size_t Cols> 
auto matrix_const_mult(std::array<std::array<T, Cols>, Rows>& a, const float& dt) {
  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      a[i][j] = a[i][j] * dt;
    }
  }
  return a;
}

template<typename T, size_t Rows, size_t Cols>
auto add_matrix(const std::array<std::array<T, Cols>, Rows>& a, const std::array<std::array<T, Cols>, Rows>& b) {
  std::array<std::array<T, Cols>, Rows> result{};
  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      result[i][j] = a[i][j] + b[i][j];
    }
  }
  return result;
}

template<typename T, size_t Rows, size_t Cols>
auto sub_matrix(const std::array<std::array<T, Cols>, Rows>& a, const std::array<std::array<T, Cols>, Rows>& b) {
  std::array<std::array<T, Cols>, Rows> result{};
  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      result[i][j] = a[i][j] - b[i][j];
    }
  }
  return result;
}

template<typename T, size_t Rows, size_t Cols>
auto inv_matrix(std::array<std::array<T, Cols>, Rows>& a) {
  for (int i = 0; i < Rows; i++) {
    for (int j = 0; j < Cols; j++) {
      a[i][j] = 1 / a[i][j];
    }
  }
  return a;
}

class ConstVelKF {
  private:
  std::array<std::array<float, 6>, 6> x;
  std::array<std::array<float, 6>, 6> P;
  int angle_idx;
  public:
  std::array<std::array<float, 6>, 6> Q;
  std::array<std::array<float, 3>, 3> R;
  std::array<float, 6> diagonalQ = {1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3};
  std::array<float, 3> diagonalR = {2.444e-3, 1.2527e-3, 4.0482e-3};
  std::array<float, 6> diagonalone = {1,1,1,1,1,1};
  float MAX_DT;
  
  ConstVelKF(const std::array<std::array<float, 6>, 6> x0, std::array<std::array<float, 6>, 6> P0, int angle_idx = 2): x(x0), P(P0), angle_idx(angle_idx)  {
    Q = diagonal_matrix_offset<6, 6>(diagonalQ, 0);
    R = diagonal_matrix_offset<3, 3>(diagonalR, 0);
    MAX_DT = 1e-3;
  }

  void process_update(float dt) {
    if (dt <= 0) {
      return;
    }

    std::array<std::array<float, 6>, 6> A;
    std::array<float, 2> dtmatrix = {dt, dt};
    A = diagonal_matrix_offset<6,6>(diagonalone, 0);
    A = diagonal_matrix_offset<6,6>(dtmatrix, 4);

    this->x = matrix_multiply(A, this->x);
    this->P = add_matrix(matrix_multiply(matrix_multiply(A, this->P), transpose(A)), matrix_const_mult(Q, dt));

    
  }
  void measurement_update(const std::array<std::array<float, 6>, 3> &z) {
    std::array<std::array<float, 6>, 3> H = diagonal_matrix_offset<3, 6>(diagonalone, 0);
  std::array<std::array<float, 3>, 3> S = add_matrix(matrix_multiply(matrix_multiply(H, this->P), transpose(H)), R);
    std::array<std::array<float, 3>, 6> K = matrix_multiply(matrix_multiply(this->P, transpose(H)), inv_matrix(S));
    std::array<std::array<float, 6>, 3> y = sub_matrix(z, H);
    y[this->angle_idx][3] = wrap_angle(y[this->angle_idx][3]);
    
    this->x = add_matrix(matrix_multiply(K, y), this->x);
    this->P = sub_matrix(matrix_multiply(matrix_multiply(K, H), this->P), this->P);
  }
  double wrap_angle(float theta) {
    return atan2(sin(theta), cos(theta));
  } 




}; 

int main() {
}
