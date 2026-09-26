/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file DisturbanceObserver.hpp
 *
 * Disturbance observer of Lemma 3.1 (no leakage term), for y_dot = phi + d
 * with measured y and diagonal gains K, L:
 *
 *   y_hat_dot = phi + d_hat - K (y_hat - y)
 *   d_hat_dot = -L (y_hat - y)
 *
 * Discretized exactly (zero-order hold on phi and y over the step), per axis:
 * with e = y_hat - y, x = [e, d_hat]^T,
 *
 *   x_dot = A x + b phi,  A = [[-K, 1], [-L, 0]],  b = [1, 0]^T
 *   x(k+1) = Phi x(k) + Gamma phi,  [Phi Gamma; 0 1] = expm([A b; 0 0] dt)
 *
 * which is exact for any dt (no dt*K, dt^2*L restrictions).
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <cmath>

class DisturbanceObserver
{
public:
	void setGains(float K, float L) { _K = K; _L = L; }

	/** reset to y_hat = y, d_hat = 0 */
	void reset(const matrix::Vector3f &y)
	{
		_y_hat = y;
		_d_hat.zero();
	}

	/**
	 * Advance the observer by dt.
	 * @param phi known part of y_dot, held over the step
	 * @param y   latest measurement of y
	 */
	void update(const matrix::Vector3f &phi, const matrix::Vector3f &y, float dt)
	{
		double E[3][3];
		expmAugmented(dt, E);

		for (int i = 0; i < 3; i++) {
			const double e0 = (double)_y_hat(i) - (double)y(i);
			const double d0 = (double)_d_hat(i);
			const double e1 = E[0][0] * e0 + E[0][1] * d0 + E[0][2] * (double)phi(i);
			const double d1 = E[1][0] * e0 + E[1][1] * d0 + E[1][2] * (double)phi(i);
			_y_hat(i) = (float)((double)y(i) + e1);
			_d_hat(i) = (float)d1;
		}
	}

	const matrix::Vector3f &y_hat() const { return _y_hat; }
	const matrix::Vector3f &d_hat() const { return _d_hat; }

private:
	/** E = expm(M dt), M = [[-K, 1, 1], [-L, 0, 0], [0, 0, 0]] (scaling and squaring + Taylor) */
	void expmAugmented(float dt, double E[3][3]) const
	{
		const double h = (double)dt;
		double M[3][3] = {{-(double)_K * h, h, h},
			{-(double)_L * h, 0.0, 0.0},
			{0.0, 0.0, 0.0}
		};

		// infinity norm
		double nrm = 0.0;

		for (int i = 0; i < 3; i++) {
			double s = fabs(M[i][0]) + fabs(M[i][1]) + fabs(M[i][2]);
			nrm = s > nrm ? s : nrm;
		}

		int squarings = 0;

		while (nrm > 0.25 && squarings < 30) {
			nrm *= 0.5;
			squarings++;
		}

		const double scale = ldexp(1.0, -squarings);

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				M[i][j] *= scale;
			}
		}

		// Taylor series, order 12 (|M| <= 0.25 -> truncation error < 1e-16)
		double term[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E[i][j] = term[i][j];
			}
		}

		for (int k = 1; k <= 12; k++) {
			double next[3][3];
			mul(term, M, next);

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					term[i][j] = next[i][j] / k;
					E[i][j] += term[i][j];
				}
			}
		}

		for (int s = 0; s < squarings; s++) {
			double sq[3][3];
			mul(E, E, sq);

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					E[i][j] = sq[i][j];
				}
			}
		}
	}

	static void mul(const double A[3][3], const double B[3][3], double C[3][3])
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
			}
		}
	}

	float _K{0.f};
	float _L{0.f};
	matrix::Vector3f _y_hat{};
	matrix::Vector3f _d_hat{};
};
