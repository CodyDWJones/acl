#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "acl/core/error.h"
#include "acl/math/vector4_64.h"

#include <cstdint>

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			// TODO: not in a Constants struct...?
			constexpr uint32_t POLYNOMIAL_ORDER = 3;
			constexpr uint32_t FIRST_INTERPOLATION_KNOT_INDEX = (POLYNOMIAL_ORDER - 1) / 2;

#if SIXTY_FOUR_BIT
			inline void calculate_knots(const Vector4_64* values, const int32_t* sample_indices, double* out_knots)
			{
				out_knots[0] = 0.0;

				for (uint8_t i = 1; i < 4; ++i)
				{
					// The interpolation will fail with a division by zero if two consecutive control points happen to be equal.
					// Avoid this by adding a fifth dimension for "time".
					int32_t sample_index_difference = sample_indices[i] - sample_indices[i - 1];

					out_knots[i] = out_knots[i - 1] + pow(vector_length_squared(vector_sub(values[i], values[i - 1])) + sample_index_difference * sample_index_difference, 0.25);
				}
			}

			inline Vector4_64 interpolate_spline(const Vector4_64* values, const double* knots, const double* sample_times, double sample_time)
			{
				double knot = knots[1] + (knots[2] - knots[1]) * (sample_time - sample_times[1]) / (sample_times[2] - sample_times[1]);

				ACL_ASSERT(knots[1] <= knot && knot <= knots[2], "The knot for the interpolation point is out of bounds");

				Vector4_64 a[] =
				{
					vector_add(
						vector_mul(values[0], (knots[1] - knot) / (knots[1] - knots[0])),
						vector_mul(values[1], (knot - knots[0]) / (knots[1] - knots[0]))),

					vector_add(
						vector_mul(values[1], (knots[2] - knot) / (knots[2] - knots[1])),
						vector_mul(values[2], (knot - knots[1]) / (knots[2] - knots[1]))),

					vector_add(
						vector_mul(values[2], (knots[3] - knot) / (knots[3] - knots[2])),
						vector_mul(values[3], (knot - knots[2]) / (knots[3] - knots[2])))
				};

				Vector4_64 b[] =
				{
					vector_add(
						vector_mul(a[0], (knots[2] - knot) / (knots[2] - knots[0])),
						vector_mul(a[1], (knot - knots[0]) / (knots[2] - knots[0]))),

					vector_add(
						vector_mul(a[1], (knots[3] - knot) / (knots[3] - knots[1])),
						vector_mul(a[2], (knot - knots[1]) / (knots[3] - knots[1])))
				};

				return vector_add(
					vector_mul(b[0], (knots[2] - knot) / (knots[2] - knots[1])),
					vector_mul(b[1], (knot - knots[1]) / (knots[2] - knots[1])));
			}
#else
			inline void calculate_knots(const Vector4_32* values, const int32_t* sample_indices, float* out_knots)
			{
				out_knots[0] = 0.0f;

				for (uint8_t i = 1; i < 4; ++i)
				{
					// The interpolation will fail with a division by zero if two consecutive control points happen to be equal.
					// Avoid this by adding a fifth dimension for "time".
					int32_t sample_index_difference = sample_indices[i] - sample_indices[i - 1];

					out_knots[i] = out_knots[i - 1] + static_cast<float>(pow(vector_length_squared(vector_sub(values[i], values[i - 1])) + sample_index_difference * sample_index_difference, 0.25));
				}
			}

			inline Vector4_32 interpolate_spline(const Vector4_32* values, const float* knots, const float* sample_times, float sample_time)
			{
				float knot = knots[1] + (knots[2] - knots[1]) * (sample_time - sample_times[1]) / (sample_times[2] - sample_times[1]);

				ACL_ASSERT(knots[1] <= knot && knot <= knots[2], "The knot for the interpolation point is out of bounds");

				Vector4_32 a[] =
				{
					vector_add(
						vector_mul(values[0], (knots[1] - knot) / (knots[1] - knots[0])),
						vector_mul(values[1], (knot - knots[0]) / (knots[1] - knots[0]))),

					vector_add(
						vector_mul(values[1], (knots[2] - knot) / (knots[2] - knots[1])),
						vector_mul(values[2], (knot - knots[1]) / (knots[2] - knots[1]))),

					vector_add(
						vector_mul(values[2], (knots[3] - knot) / (knots[3] - knots[2])),
						vector_mul(values[3], (knot - knots[2]) / (knots[3] - knots[2])))
				};

				Vector4_32 b[] =
				{
					vector_add(
						vector_mul(a[0], (knots[2] - knot) / (knots[2] - knots[0])),
						vector_mul(a[1], (knot - knots[0]) / (knots[2] - knots[0]))),

					vector_add(
						vector_mul(a[1], (knots[3] - knot) / (knots[3] - knots[1])),
						vector_mul(a[2], (knot - knots[1]) / (knots[3] - knots[1])))
				};

				return vector_add(
					vector_mul(b[0], (knots[2] - knot) / (knots[2] - knots[1])),
					vector_mul(b[1], (knot - knots[1]) / (knots[2] - knots[1])));
			}
#endif
		}
	}
}
