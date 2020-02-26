#pragma once

#include <initializer_list>
#include <stdexcept>
#include <ostream>
#include <iomanip>
#include <cmath>

template<size_t m, size_t n, typename T = float>
class Matrix
{
public:
	Matrix()
	{
		if (m == 0 || n == 0) {
			throw std::logic_error("Invalid matrix size");
		}

		this->_values = new T[m * n];

		this->setZero();
	}

	Matrix(std::initializer_list<T> values)
	{
		if (m == 0 || n == 0) {
			throw std::logic_error("Invalid matrix size");
		}

		size_t length = m * n;

		if (values.size() != length) {
			throw std::logic_error("Invalid matrix values size");
		}

		this->_values = new T[length];

		std::copy(values.begin(), values.end(), this->_values);
	}

	Matrix(const Matrix<m, n, T>& rhs)
	{
		size_t length = m * n;
		this->_values = new T[length];

		std::memcpy(this->_values, rhs._values, length * sizeof(T));
	}

	~Matrix()
	{
		delete[] this->_values;
		this->_values = nullptr;
	}

	Matrix<m, n, T>& operator=(const Matrix<m, n, T>& rhs)
	{
		delete[] this->_values;
		size_t length = m * n;
		this->_values = new T[length];

		std::memcpy(this->_values, rhs._values, length * sizeof(T));

		return *this;
	}

	Matrix<m, n, T> operator+(const Matrix<m, n, T>& rhs) const
	{
		Matrix<m, n, T> mat;

		for (size_t i = 0; i < m * n; i++) {
			mat._values[i] = this->_values[i] + rhs._values[i];
		}

		return mat;
	}

	Matrix<m, n, T>& operator+=(const Matrix<m, n, T>& rhs)
	{
		for (size_t i = 0; i < m * n; i++) {
			this->_values[i] += rhs._values[i];
		}

		return *this;
	}

	Matrix<m, n, T> operator-(const Matrix<m, n, T>& rhs) const
	{
		Matrix<m, n, T> mat;

		for (size_t i = 0; i < m * n; i++) {
			mat._values[i] = this->_values[i] - rhs._values[i];
		}

		return mat;
	}

	Matrix<m, n, T> operator-() const
	{
		Matrix<m, n, T> mat;

		for (size_t row = 0; row < m; row++) {
			for (size_t col = 0; col < n; col++) {
				mat.at(row, col) = -this->at(row, col);
			}
		}

		return mat;
	}

	Matrix<m, n, T>& operator-=(const Matrix<m, n, T>& rhs)
	{
		for (size_t i = 0; i < m * n; i++) {
			this->_values[i] - rhs._values[i];
		}

		return *this;
	}

	template<size_t o>
	Matrix<m, o, T> operator*(const Matrix<n, o, T>& rhs) const
	{
		Matrix<m, o, T> mat;
		T cell;

		for (size_t row = 0; row < m; row++) {
			for (size_t col = 0; col < o; col++) {
				cell = 0;
				for (size_t i = 0; i < n; i++) {
					cell += this->at(row, i) * rhs.at(i, col);
				}
				mat.at(row, col) = cell;
			}
		}

		return mat;
	}

	template<size_t o>
	Matrix<m, o, T>& operator*=(const Matrix<n, o, T>& rhs)
	{
		T cell;

		for (size_t row = 0; row < m; row++) {
			for (size_t col = 0; col < o; col++) {
				cell = 0;
				for (size_t i = 0; i < n; i++) {
					cell += this->at(row, i) * rhs.at(i, col);
				}
				this->at(row, col) = cell;
			}
		}

		return *this;
	}

	Matrix<m, n, T> operator%(T rhs) const
	{
		Matrix<m, n, T> mat;

		for (size_t i = 0; i < m * n; i++) {
			mat._values[i] = this->_values[i] % rhs;
		}

		return mat;
	}

	bool operator==(const Matrix<m, n, T>& rhs)
	{
		for (size_t row = 0; row < m; row++) {
			for (size_t col = 0; col < n; col++) {
				if (this.at(row, col) != rhs.at(row, col)) {
					return false;
				}
			}
		}

		return true;
	}

	bool operator!=(const Matrix<m, n, T>& rhs)
	{
		return !(*this == rhs);
	}

	void setZero()
	{
		std::memset(this->_values, 0, m * n * sizeof(T));
	}

	void setIdentity()
	{
		if (m != n) {
			throw std::logic_error("Invalid operation on non-square matrix");
		}

		this->setZero();

		for (size_t i = 0; i < m; i++) {
			this->at(i, i) = 1;
		}
	}

	Matrix<n, m, T> transposed() const {
		Matrix<n, m, T> transposed;

		for (size_t i = 0; i < m; i++) {
			for (size_t j = 0; j < n; j++) {
				transposed.at(j, i) = this->at(i, j);
			}
		}

		return transposed;
	}

	T& at(size_t i, size_t j)
	{
		if (i >= m || j >= n) {
			throw std::logic_error("Invalid matrix value index");
		}

		return this->_values[i * n + j];
	}

	T at(size_t i, size_t j) const
	{
		if (i >= m || j >= n) {
			throw std::logic_error("Invalid matrix value index");
		}

		return this->_values[i * n + j];
	}

	T& at(size_t i)
	{
		return this->at(i, 0);
	}

	T at(size_t i) const
	{
		return this->at(i, 0);
	}

	T& x() {
		return this->at(0);
	}

	T x() const {
		return this->at(0);
	}

	T& y() {
		return this->at(1);
	}

	T y() const {
		return this->at(1);
	}

	T& z() {
		return this->at(2);
	}

	T z() const {
		return this->at(2);
	}

	T& w() {
		return this->at(3);
	}

	T w() const {
		return this->at(3);
	}

	T* ptr()
	{
		return this->_values;
	}

	friend Matrix<m, n, T> operator*(T value, const Matrix<m, n, T>& rhs)
	{
		Matrix<m, n, T> mat;
		T* ptr = rhs.ptr();

		for (size_t i = 0; i < m * n; i++) {
			mat = ptr[i] * value;
		}

		return mat;
	}

	friend Matrix<m, n, T>& operator*=(T value, Matrix<m, n, T>& rhs)
	{
		T* ptr = rhs.ptr();

		for (size_t i = 0; i < m * n; i++) {
			ptr[i] *= value;
		}

		return *rhs;
	}

	friend std::ostream& operator<<(std::ostream& os, const Matrix<m, n, T>& rhs)
	{
		std::ios::fmtflags flags = os.flags();

		if (n == 1) {
			for (size_t i = 0; i < m; i++) {
				os << rhs.at(i) << ' ';
			}
			os << std::endl;
		} else {
			for (size_t row = 0; row < m; row++) {
				for (size_t col = 0; col < n; col++) {
					os << std::fixed << std::setw(11) << std::setprecision(6) << rhs.at(row, col) << ' ';
				}
				os << std::endl;
			}
		}

		os.flags(flags);

		return os;
	}

private:
	T* _values;
};

typedef Matrix<2, 2, float> Matrix2f;
typedef Matrix<2, 2, float> Matrix22f;
typedef Matrix<2, 3, float> Matrix23f;
typedef Matrix<3, 2, float> Matrix32f;
typedef Matrix<3, 3, float> Matrix3f;
typedef Matrix<3, 3, float> Matrix33f;
typedef Matrix<3, 4, float> Matrix34f;
typedef Matrix<4, 2, float> Matrix42f;
typedef Matrix<4, 3, float> Matrix43f;
typedef Matrix<4, 4, float> Matrix4f;
typedef Matrix<4, 4, float> Matrix44f;

typedef Matrix<2, 2, double> Matrix2d;
typedef Matrix<2, 2, double> Matrix22d;
typedef Matrix<2, 3, double> Matrix23d;
typedef Matrix<3, 2, double> Matrix32d;
typedef Matrix<3, 3, double> Matrix3d;
typedef Matrix<3, 3, double> Matrix33d;
typedef Matrix<3, 4, double> Matrix34d;
typedef Matrix<4, 2, double> Matrix42d;
typedef Matrix<4, 3, double> Matrix43d;
typedef Matrix<4, 4, double> Matrix4d;
typedef Matrix<4, 4, double> Matrix44d;

typedef Matrix<2, 2, int> Matrix2i;
typedef Matrix<2, 2, int> Matrix22i;
typedef Matrix<2, 3, int> Matrix23i;
typedef Matrix<3, 2, int> Matrix32i;
typedef Matrix<3, 3, int> Matrix3i;
typedef Matrix<3, 3, int> Matrix33i;
typedef Matrix<3, 4, int> Matrix34i;
typedef Matrix<4, 2, int> Matrix42i;
typedef Matrix<4, 3, int> Matrix43i;
typedef Matrix<4, 4, int> Matrix4i;
typedef Matrix<4, 4, int> Matrix44i;

typedef Matrix<2, 2, unsigned int> Matrix2ui;
typedef Matrix<2, 2, unsigned int> Matrix22ui;
typedef Matrix<2, 3, unsigned int> Matrix23ui;
typedef Matrix<3, 2, unsigned int> Matrix32ui;
typedef Matrix<3, 3, unsigned int> Matrix3ui;
typedef Matrix<3, 3, unsigned int> Matrix33ui;
typedef Matrix<3, 4, unsigned int> Matrix34ui;
typedef Matrix<4, 2, unsigned int> Matrix42ui;
typedef Matrix<4, 3, unsigned int> Matrix43ui;
typedef Matrix<4, 4, unsigned int> Matrix4ui;
typedef Matrix<4, 4, unsigned int> Matrix44ui;
