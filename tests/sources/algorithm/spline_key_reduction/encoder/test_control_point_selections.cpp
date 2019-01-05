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

#include <acl/algorithm/spline_key_reduction/encoder/control_point_selections.h>
#include <acl/core/ansi_allocator.h>
#include <acl/core/iterator.h>
#include <acl/core/memory_utils.h>

#include <cstdint>

using namespace acl;
using namespace acl::spline_key_reduction::impl;

TEST_CASE("control_point_selections", "[algorithm][spline_key_reduction][encoder][control_point_selections]")
{
	const uint16_t num_bones = 2;
	const uint32_t num_samples = 2;
	const uint32_t max_possible_control_points = num_bones * num_samples * get_array_size(k_track_types);

	ANSIAllocator allocator;

	ControlPointSelections p(allocator, num_bones, num_samples);

	SECTION("select_all")
	{
		p.select_all();
		REQUIRE(p.get_num_selected() == max_possible_control_points);
	}

	SECTION("deselect_all")
	{
		p.select(0, 0, AnimationTrackType8::Rotation);
		p.deselect_all();
		REQUIRE(p.get_num_selected() == 0);
	}

	struct TestCase
	{
		uint16_t bone_index;
		uint32_t sample_index;
		AnimationTrackType8 track_type;
	};

	SECTION("select_all_bones")
	{
		p.select_all_bones(0, AnimationTrackType8::Rotation);

		REQUIRE(p.get_num_selected() == num_bones);
		
		for (uint16_t bone_index = 0; bone_index < num_bones; ++bone_index)
			REQUIRE(p.is_selected(bone_index, 0, AnimationTrackType8::Rotation));
	}

	SECTION("deselect_all_bones")
	{
		p.select_all();
		p.deselect_all_bones(0, AnimationTrackType8::Rotation);

		REQUIRE(p.get_num_selected() == max_possible_control_points - num_bones);

		for (uint16_t bone_index = 0; bone_index < num_bones; ++bone_index)
			REQUIRE(!p.is_selected(bone_index, 0, AnimationTrackType8::Rotation));
	}

	SECTION("select")
	{
		p.select(0, 0, AnimationTrackType8::Rotation);
		p.select(0, 0, AnimationTrackType8::Translation, true);
		p.select(0, 0, AnimationTrackType8::Scale, false);

		REQUIRE(p.get_num_selected() == 2);
		REQUIRE(p.is_selected(0, 0, AnimationTrackType8::Rotation));
		REQUIRE(p.is_selected(0, 0, AnimationTrackType8::Translation));
	}

	SECTION("deselect individual")
	{
		p.select_all();
		p.deselect(0, 0, AnimationTrackType8::Rotation);

		REQUIRE(p.get_num_selected() == max_possible_control_points - 1);
		REQUIRE(!p.is_selected(0, 0, AnimationTrackType8::Rotation));
	}

	SECTION("deselect batch")
	{
		p.select(0, 0, AnimationTrackType8::Rotation);
		p.select(0, 0, AnimationTrackType8::Translation);

		ControlPointSelections mask(allocator, num_bones, num_samples);
		mask.select(0, 0, AnimationTrackType8::Rotation);
		mask.select(0, 0, AnimationTrackType8::Scale);

		p.deselect(mask);

		REQUIRE(p.get_num_selected() == 1);
		REQUIRE(p.is_selected(0, 0, AnimationTrackType8::Translation));
	}

	SECTION("any bones selected")
	{
		p.select(0, 0, AnimationTrackType8::Rotation);
		REQUIRE(p.any_bones_selected(0));

		REQUIRE(!p.any_bones_selected(1));
		p.select(1, 1, AnimationTrackType8::Scale);
		REQUIRE(p.any_bones_selected(1));
	}
}
