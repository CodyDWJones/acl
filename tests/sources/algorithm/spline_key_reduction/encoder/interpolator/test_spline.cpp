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

#include <catch.hpp>

#include <acl/algorithm/spline_key_reduction/encoder/interpolator/spline.h>
#include <acl/algorithm/spline_key_reduction/shared/spline_types.h>
#include <acl/math/vector4_32.h>

#include <cstdint>

using namespace acl;
using namespace acl::spline_key_reduction::impl;

TEST_CASE("shift_in_control_point", "[algorithm][spline_key_reduction][encoder][interpolator][spline]")
{
	Spline<CentripetalCatmullRom> spline;

	for (uint32_t i = 0; i < spline.num_control_points; ++i)
	{
		const float initial_value = static_cast<float>(i);

		spline.values[i] = vector_set(initial_value);
		spline.sample_indexes[i] = i;
		spline.knots[i] = initial_value;
	}

	auto require_integers_from = [&](int32_t first)
	{
		for (int32_t i = 0; i < spline.num_control_points; ++i)
		{
			REQUIRE(vector_all_near_equal(spline.values[i], vector_set(static_cast<float>(first + i))));
			REQUIRE(spline.sample_indexes[i] == first + i);
			REQUIRE(spline.knots[i] == i);					// Knots should be unchanged since they are recomputed in a separate step
		}
	};

	SECTION("store_left")
	{
		SECTION("preceeding value")
		{
			spline.shift_in_control_point(vector_set(-1), -1, false);
			require_integers_from(-1);
		}

		SECTION("redundant value")
		{
			spline.shift_in_control_point(vector_set(0), 0, false);
			require_integers_from(0);
		}
	}

	SECTION("store_right")
	{
		SECTION("following value")
		{
			spline.shift_in_control_point(vector_set(static_cast<float>(spline.num_control_points)), spline.num_control_points, true);
			require_integers_from(1);
		}

		SECTION("redundant value")
		{
			spline.shift_in_control_point(vector_set(static_cast<float>(spline.num_control_points - 1)), spline.num_control_points - 1, true);
			require_integers_from(0);
		}
	}
}
