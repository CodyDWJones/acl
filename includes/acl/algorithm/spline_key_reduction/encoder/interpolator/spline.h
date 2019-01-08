#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Animation Compression Library contributors
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

#include "acl/core/compiler_utils.h"
#include "acl/core/track_types.h"
#include "acl/math/math_types.h"

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			template <class SplineType>
			struct Spline
			{
				static constexpr uint32_t num_control_points = SplineType::num_control_points;

				Vector4_32 values[num_control_points];
				int32_t sample_indexes[num_control_points];
				float knots[num_control_points];

				uint16_t bone_index;
				AnimationTrackType8 track_type;

				void shift_in_control_point(const Vector4_32& value, int32_t sample_index, bool store_right)
				{
					if (store_right && sample_index > sample_indexes[num_control_points - 1])
					{
						for (uint8_t i = 0; i < num_control_points - 1; ++i)
							values[i] = values[i + 1];

						for (uint8_t i = 0; i < num_control_points - 1; ++i)
							sample_indexes[i] = sample_indexes[i + 1];

						values[num_control_points - 1] = value;
						sample_indexes[num_control_points - 1] = sample_index;
					}
					else if (!store_right && sample_index < sample_indexes[0])
					{
						for (uint8_t i = num_control_points - 1; i >= 1; --i)
							values[i] = values[i - 1];

						for (uint8_t i = num_control_points - 1; i >= 1; --i)
							sample_indexes[i] = sample_indexes[i - 1];

						values[0] = value;
						sample_indexes[0] = sample_index;
					}
				}
			};
		}
	}
}

ACL_IMPL_FILE_PRAGMA_POP
