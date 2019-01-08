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

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			struct Linear
			{
				static constexpr uint32_t num_control_points = 2;
				static constexpr uint32_t interpolates_from_index = 0;
				static constexpr uint32_t interpolates_to_index = 1;
			};

			struct CentripetalCatmullRom
			{
				static constexpr uint32_t num_control_points = 4;

				/* This spline interpolates values between the innermost control points:

				               [....................]

				   control     control        control      control
				   point 0     point 1        point 2      point 3

				     ^^                                      ^^
				   But the first and last points might not exist in the original data,
				   so auxiliary control points must be added to allow the use of the
				   cubic function near the start and end of the clip.
				*/
				static constexpr uint32_t interpolates_from_index = 1;
				static constexpr uint32_t interpolates_to_index = 2;
			};
		}
	}
}

ACL_IMPL_FILE_PRAGMA_POP
