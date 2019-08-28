#pragma once
#include <math.h>
#include <string>

struct vector3 {
	double x, y, z;
	double& operator[](int pos) { switch (pos) { case 0: return x; case 1: return y; case 2: return z; } return x; }
	vector3 operator+(vector3 v)const { return{ x + v.x, y + v.y, z + v.z }; }
	vector3 operator-(vector3 v)const { return{ x - v.x, y - v.y, z - v.z }; }
	vector3 operator*(double l)const { return{ l*x, l*y, l*z }; }
	vector3 operator/(double l)const { return{ x / l, y / l, z / l }; }

	bool     operator== (vector3& b) { return x == b.x && y == b.y && z == b.z; }
	bool     operator!= (vector3& b) { return x != b.x || y != b.y || z != b.z; }
	vector3& operator+= (vector3& b) { x += b.x; y += b.y; z += b.z; return *this; }
	vector3& operator-= (vector3& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
	vector3& operator*= (double s) { x *= s; y *= s; z *= s; return *this; }
	vector3& operator/= (double s) { double rcp = double(1) / s; x *= rcp; y *= rcp; z *= rcp; return *this; }

	double NormSq()const { return x*x + y*y + z*z; }
	double Norm()const { return sqrt(NormSq()); }
	vector3 Normalized() { return *this / Norm(); }
	void Integrate(vector3 v, double dt) { x += v.x*dt; y += v.y*dt; z += v.z*dt; }
	void Integrate(vector3 a, vector3 v, double dt) { x += v.x*dt + a.x*dt*dt / 2; y += v.y*dt + a.y*dt*dt / 2; z += v.z*dt + a.z*dt*dt / 2; }
	vector3 CrossProduct(vector3 v)const { return{ y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x }; }
	double DotProduct(vector3 v)const { return x*v.x + y*v.y + z*v.z; }

	double Angle(vector3& b) {
		if (b.Norm() == 0) { return 0.0; }
		return acos((DotProduct(b)) / (Norm()*b.Norm()));
	}
};
struct quat {
	double w, x, y, z;
	double& operator[](int pos) { switch (pos) { case 0: return w; case 1: return x; case 2: return y; case 3:return z; } return w; }
	quat operator*(quat q) {
		return{ w*q.w - x*q.x - y*q.y - z*q.z, w*q.x + x*q.w + y*q.z - z*q.y, w*q.y + y*q.w - x*q.z + z*q.x, w*q.z + z*q.w + x*q.y - y*q.x };
	}
	void Normalize() { double size = Norm(); w /= size; x /= size; y /= size; z /= size; }
	double NormSq() { return w*w + x*x + y*y + z*z; }
	double Norm() { return sqrt(NormSq()); }
	static quat DeltaQuat(vector3 omega, double dt) {
		if (omega.Norm() == 0) {
			return{ 1.0, 0.0, 0.0, 0.0 };
		}
		double s = 0, w = 0, delta = omega.NormSq()*dt*0.5*dt*0.5;
		if (delta * delta / 24.0 < 1e-50) {
			w = 1.0 - delta / 2.0;
			s = dt*0.5;
		} else {
			w = cos(sqrt(delta));
			s = sin(sqrt(delta)) / omega.Norm();
		}
		return{ w, omega.x*s, omega.y*s, omega.z*s };
	}
	static quat RotationFromTo(vector3 from, vector3 to) {
		if (from.Norm() == 0.0 || to.Norm() == 0) {
			return{ 1.0, 0.0, 0.0, 0.0 };//Доопределено от балды
		}
		vector3 v_axe = from.CrossProduct(to);
		double ang = acos(from.DotProduct(to) / from.Norm() / to.Norm()) / v_axe.Norm();
		return quat::DeltaQuat(v_axe, ang);
	}
	static quat RotationFromToOculus(vector3 from, vector3 to) {
		if (from.Norm() == 0.0 || to.Norm() == 0) {
			return{ 1.0, 0.0, 0.0, 0.0 };//Доопределено от балды
		}
		vector3 v_axe = from.CrossProduct(to);
		double ang = acos(from.DotProduct(to) / from.Norm() / to.Norm()) / v_axe.Norm();
		if (fabs(ang) > 0.1) {
			ang = ang*0.001;
		} else if (fabs(ang) > 0.001) {
			ang = 0.001;
		}
		return quat::DeltaQuat(v_axe, ang);
	}

	static quat RotateAngleAxis(double angle, vector3 axis) {
		axis.Normalized();
		return { cos(angle/2.0), axis.x * sin(angle / 2.0), axis.y * sin(angle / 2.0), axis.z * sin(angle / 2.0) };
	}

	vector3 Rotate(vector3 v) {
		quat q_v = { 0, v.x, v.y, v.z };
		quat this_inv = { w, -x, -y, -z };
		quat res_v = (*this)*q_v*this_inv;
		return{ res_v.x, res_v.y, res_v.z };
	}
	vector3 FromImmovableToMovable(vector3 v) {
		quat q_v = { 0, v.x, v.y, v.z };
		quat this_inv = { w, -x, -y, -z };
		quat res_v = this_inv*q_v*(*this);
		return{ res_v.x, res_v.y, res_v.z };
	}
	quat conj() {
		return{ w, -x, -y, -z };
	}
	void Integrate(vector3 omega, double dt) { *this = (*this)*DeltaQuat(omega, dt); }
};
