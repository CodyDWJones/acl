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
			constexpr uint32_t POLYNOMIAL_ORDER = 3;
			constexpr uint32_t NUM_CONTROL_POINTS = POLYNOMIAL_ORDER + 1;

			/* A cubic spline interpolates values between the innermost control points:

									 [....................]

					     control     control        control      control
			             point 0     point 1        point 2      point 3

			               ^^                                      ^^
			   But the first and last points might not exist in the original data,
			   so auxiliary control points must be added to allow the use of the
			   cubic function near the start and end of the clip.
			*/
			constexpr uint32_t LEFT_INTERPOLATION_KNOT_INDEX = (POLYNOMIAL_ORDER - 1) / 2;
			constexpr uint32_t RIGHT_INTERPOLATION_KNOT_INDEX = LEFT_INTERPOLATION_KNOT_INDEX + 1;
			constexpr uint32_t NUM_LEFT_AUXILIARY_POINTS = LEFT_INTERPOLATION_KNOT_INDEX;
			constexpr uint32_t NUM_RIGHT_AUXILIARY_POINTS = POLYNOMIAL_ORDER - RIGHT_INTERPOLATION_KNOT_INDEX;

			inline float get_knot_delta(const Vector4_32& current_value, uint32_t current_sample_index, const Vector4_32& last_value, uint32_t last_sample_index)
			{
				// The interpolation will fail with a division by zero if two consecutive control points happen to be equal.
				// Avoid this by adding a fifth dimension for "time".
				uint32_t sample_index_difference = current_sample_index - last_sample_index;
				return static_cast<float>(pow(vector_length_squared(vector_sub(current_value, last_value)) + sample_index_difference * sample_index_difference, 0.25));
			}

			inline void get_knots(const Vector4_32* values, const uint32_t* sample_indices, float* out_knots)
			{
				out_knots[0] = 0.0f;

				for (uint8_t i = 1; i < 4; ++i)
					out_knots[i] = out_knots[i - 1] + get_knot_delta(values[i], sample_indices[i], values[i - 1], sample_indices[i - 1]);
			}

			inline Vector4_32 interpolate_spline(const Vector4_32* values, const float* knots, const uint32_t* sample_indices, float sample_index)
			{
				float knot = knots[1] + (knots[2] - knots[1]) * (sample_index - sample_indices[1]) / (sample_indices[2] - sample_indices[1]);

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
		}
	}
}
