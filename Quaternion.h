#pragma once

#include "Matrix.h"

template<typename T>
class Quaternion
{
public:
	Quaternion() : x(0), y(0), z(0), w(0)
	{
	}
	
	Quaternion(T x, T y, T z, T w) : x(x), y(y), z(z), w(w)
	{
	}

	Quaternion(const Quaternion<T>& rhs) : x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w)
	{
	}

	~Quaternion()
	{
	}

	Quaternion<T>& operator=(const Quaternion<T>& rhs)
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->z = rhs.z;
		this->w = rhs.w;

		return *this;
	}

	Quaternion<T> operator+(const Quaternion<T>& rhs) const
	{
		Quaternion<T> quat(
			this->x + rhs.x,
			this->y + rhs.y,
			this->z + rhs.z,
			this->w + rhs.w
		);

		return quat;
	}

	Quaternion<T>& operator+=(const Quaternion<T>& rhs)
	{
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;
		this->w += rhs.w;

		return *this;
	}

	Quaternion<T> operator-(const Quaternion<T>& rhs) const
	{
		Quaternion<T> quat(
			this->x - rhs.x,
			this->y - rhs.y,
			this->z - rhs.z, 
			this->w - rhs.w
		);

		return quat;
	}

	Quaternion<T> operator-() const
	{
		return Quaternion<T>(-this->x, -this->y, -this->z, -this->w);
	}

	Quaternion<T>& operator-=(const Quaternion<T>& rhs)
	{
		this->x -= rhs.x;
		this->y -= rhs.y;
		this->z -= rhs.z;
		this->w -= rhs.w;

		return *this;
	}

	Quaternion<T> operator*(const Quaternion<T>& rhs) const
	{
		Quaternion<T> quat(
			this->w * rhs.x + this->x * rhs.w + this->y * rhs.z - this->z * rhs.y, 
			this->w * rhs.y - this->x * rhs.z + this->y * rhs.w + this->z * rhs.x, 
			this->w * rhs.z + this->x * rhs.y - this->y * rhs.x + this->z * rhs.w, 
			this->w * rhs.w - this->x * rhs.x - this->y * rhs.y - this->z * rhs.z
		);

		return quat;
	}

	Quaternion<T>& operator*=(const Quaternion<T>& rhs)
	{
		*this *= rhs;

		return *this;
	}

	Quaternion<T> operator*(const Matrix<3, 1, T>& rhs) const
	{
		return *this * Quaternion<T>(rhs.x(), rhs.y(), rhs.z(), 0.0);
	}

	Quaternion<T>& operator*=(const Matrix<3, 1, T>& rhs)
	{
		*this *= Quaternion<T>(rhs.x(), rhs.y(), rhs.z(), 0.0);

		return *this;
	}

	Quaternion<T> operator/(const Quaternion<T>& rhs) const
	{
		return *this * rhs.inversed();
	}

	Quaternion<T>& operator/=(const Quaternion<T>& rhs)
	{
		*this *= rhs.inversed();

		return *this;
	}

	bool operator==(const Quaternion<T>& rhs) const
	{
		return this->x == rhs.x && this->y = rhs.y && this->z * rhs.z && this->w * rhs.w;
	}

	bool operator!=(const Quaternion<T>& rhs) const
	{
		return !(*this == rhs);
	}

	T norm() const {
		return x * x + y * y + z * z + w * w;
	}

	T magnitude() const {
		return std::sqrt(this->norm());
	}

	bool isNormalized() const {
		return this->magnitude() == 1.0;
	}

	Quaternion<T> normalized() const {
		Quaternion<T> quat(this->x, this->y, this->z, this->w);
		T mag = T(1.0) / quat.magnitude();

		quat.x *= mag;
		quat.y *= mag;
		quat.z *= mag;
		quat.w *= mag;

		return quat;
	}

	Quaternion<T> conjugated() const {
		return Quaternion<T>(-this->x, -this->y, -this->z, this->w);
	}

	Quaternion<T> inversed() const {
		return this->conjugated() * (T(1.0) / this->norm());
	}

	Matrix<4, 4, T> toMatrix() const {
		Quaternion<T> quat = this->normalized();

		T qxx = T(2.0) * quat.x * quat.x;
		T qyy = T(2.0) * quat.y * quat.y;
		T qzz = T(2.0) * quat.z * quat.z;

		T qxy = T(2.0) * quat.x * quat.y;
		T qxz = T(2.0) * quat.x * quat.z;
		T qxw = T(2.0) * quat.x * quat.w;

		T qyz = T(2.0) * quat.y * quat.z;
		T qyw = T(2.0) * quat.y * quat.w;

		T qzw = T(2.0) * quat.z * quat.w;

		Matrix<4, 4, T> mat({
			T(1.0) - qyy - qzz,          qxy + qzw,          qxz - qyw, 0.0,
			         qxy - qzw, T(1.0) - qxx - qzz,          qyz + qxw, 0.0,
			         qxz + qyw,          qyz - qxw, T(1.0) - qxx - qyy, 0.0,
		    0.0,                0.0,                0.0,                1.0
		});

		return mat;
	}

	friend std::ostream& operator<<(std::ostream& os, const Quaternion<T>& rhs)
	{
		std::ios::fmtflags flags = os.flags();

		os << std::fixed << std::setw(11) << std::setprecision(6);
		os << rhs.x << ' ' << rhs.y << ' ' << rhs.z << ' ' << rhs.w;

		os.flags(flags);

		return os;
	}

	friend Quaternion<T> operator*(T value, const Quaternion<T>& quat)
	{
		Quaternion<T> quat(
			value * quat.x, 
			value * quat.y,
			value * quat.z,
			value * quat.w
		);

		return quat;
	}

	static Quaternion<T> fromAxisAngle(Matrix<3, 1, T> axis, T angle)
	{
		T radians = Utils::toRadians(angle);
		T sinHalfAngle = T(std::sin(radians / 2.0));

		Quaternion<T> quat(
			axis.x() * sinHalfAngle,
			axis.y() * sinHalfAngle,
			axis.z() * sinHalfAngle,
			T(std::cos(radians / 2.0))
		);

		return quat;
	}

	T x;
	T y;
	T z;
	T w;
};

typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;
