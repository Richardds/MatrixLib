#pragma once

#define _USE_MATH_DEFINES

#include <cmath>

#include "Matrix.h"
#include "Vector.h"
#include "Quaternion.h"

enum Space {
	LOCAL,
	GLOBAL
};

class Utils {
public:
	static constexpr double PI   = M_PI;
	static constexpr float  PI_F = M_PI;

	static float toRadians(float degrees) {
		return degrees * Utils::PI_F / 180.0f;
	}

	static double toRadians(double degrees) {
		return degrees * Utils::PI / 180.0;
	}

	template<typename T>
	static Vector<3, T> cross(const Vector<3, T>& vec1, const Vector<3, T>& vec2)
	{
		Vector<3, T> vec;
		vec.x() = vec1.y() * vec2.z() - vec1.z() * vec2.y();
		vec.y() = vec1.x() * vec2.z() - vec1.z() * vec2.x();
		vec.z() = vec1.x() * vec2.y() - vec1.y() * vec2.x();

		return vec;
	}

	template<typename T, size_t m>
	static T dot(const Vector<m, T>& vec1, const Vector<m, T>& vec2)
	{
		T dotProduct = 0;
		for (size_t i = 0; i < m; i++) {
			dotProduct += vec1.at(i) * vec2.at(i);
		}

		return dotProduct;
	}

	template<typename T>
	static T dot(const Quaternion<T>& quat1, const Quaternion<T>& quat2)
	{
		T dotProduct = 0;

		dotProduct += quat1.x * quat2.x;
		dotProduct += quat1.y * quat2.y;
		dotProduct += quat1.z * quat2.z;
		dotProduct += quat1.w * quat2.w;

		return dotProduct;
	}

	template<typename T>
	static Vector<3, T> rotateVector(const Vector<3, T>& vec, const Vector<3, T>& axis, float angle)
	{
		Quaternion<T> rotation(axis, angle);
		Quaternion<T> rotated = rotation * vec * rotation.conjugated();

		return Vector<3, T>({ rotated.x, rotated.y, rotated.z });
	}

	template<typename T>
	static Vector<3, T> rotateVector(const Vector<3, T>& vec, const Quaternion<T>& rotation)
	{
		Quaternion<T> rotated = rotation * vec * rotation.conjugated();

		return Vector<3, T>({ rotated.x, rotated.y, rotated.z });
	}
	
	template<typename T>
	static Matrix<4, 4, T> translationMatrix(const Vector<3, T>& position)
	{
		Matrix<T, 4, 4> mat({
			1.0,          0.0,          0.0,          0.0,
			0.0,          1.0,          0.0,          0.0,
			0.0,          0.0,          1.0,          0.0,
			position.x(), position.y(), position.z(), 1.0,
		});

		return mat;
	}

	template<typename T>
	static Matrix<4, 4, T> rotationMatrix(T rotX, T rotY, T rotZ)
	{
		Quaternion<T> q1 = Quaternion<T>::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, rotX);
		Quaternion<T> q2 = Quaternion<T>::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, rotY);
		Quaternion<T> q3 = Quaternion<T>::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, rotZ);

		return (q1 * q2 * q3).toMatrix();
	}

	template<typename T>
	static Matrix<4, 4, T> scaleMatrix(T factor)
	{
		Matrix<T, 4, 4> mat({
			factor, 0.0,    0.0,    0.0,
			0.0,    factor, 0.0,    0.0,
			0.0,    0.0,    factor, 0.0,
			0.0,    0.0,    0.0,    1.0,
		});

		return mat;
	}

	template<typename T>
	static Matrix<4, 4, T> scaleMatrix(T factorX, T factorY, T factorZ)
	{
		Matrix<T, 4, 4> mat({
			factorX, 0.0,     0.0,     0.0,
			0.0,     factorY, 0.0,     0.0,
			0.0,     0.0,     factorZ, 0.0,
			0.0,     0.0,     0.0,     1.0,
		});

		return mat;
	}

	template<typename T>
	static Matrix<4, 4, T> transformationMatrix(const Vector<3, T>& translation, T rotX, T rotY, T rotZ, T scale, Space space)
	{
		if (space == Space::GLOBAL) {
			return Utils::rotationMatrix(rotX, rotY, rotZ) * Utils::translationMatrix(translation) * Utils::scaleMatrix(scale);
		}

		return Utils::translationMatrix(translation) * Utils::rotationMatrix(rotX, rotY, rotZ) * Utils::scaleMatrix(scale);
	}

	template<typename T>
	static Matrix<4, 4, T> projectionMatrix(T aspectRatio, T fieldOfView, T nearPlane, T farPlane)
	{
		T scaleX = (T)1.0 / std::tan(Utils::toRadians(fieldOfView) / (T)2.0);
		T scaleY = scaleX * aspectRatio;
		T frustumLength = farPlane - nearPlane;

		Matrix<T, 4, 4> mat({
			scaleX, 0.0,     0.0,                                                   0.0,
			0.0,    scaleY,  0.0,                                                   0.0,
			0.0,    0.0,    -((farPlane + nearPlane) / frustumLength),             -1.0,
			0.0,    0.0,    -(((T)2.0 * farPlane * nearPlane) / frustumLength),     0.0
		});

		return mat;
	}

	template<typename T>
	static Quaternion<T> slerp(Quaternion<T> q0, Quaternion<T> q1, T time)
	{
		if (!q0.isNormalized()) {
			q0 = q0.normalized();
		}

		if (!q1.isNormalized()) {
			q1 = q1.normalized();
		}

		T dotProduct = Utils::dot(q0, q1);

		if (dotProduct < 0.0) {
			q1 = -q1;
			dotProduct = -dotProduct;
		}

		const T DOT_THRESHOLD = 0.9995;

		if (dotProduct > DOT_THRESHOLD) {
			Quaternion<T> result = q0 + time * (q1 - q0);
			return result.normalized();
		}

		T theta = std::acos(dotProduct);
		T thetaTime = theta * time;
		T sinThetaTime = std::sin(thetaTime);
		T sinTheta = std::sin(theta);

		T s0 = cos(thetaTime) - dotProduct * sinThetaTime / sinTheta;
		T s1 = sinThetaTime / sinTheta;

		return (s0 * q0) + (s1 * q1);
	}
};
