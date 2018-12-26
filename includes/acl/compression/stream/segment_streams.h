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

#include "acl/core/iallocator.h"
#include "acl/core/compiler_utils.h"
#include "acl/core/error.h"
#include "acl/compression/compression_settings.h"
#include "acl/compression/stream/clip_context.h"

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	inline void segment_streams(IAllocator& allocator, ClipContext& clip_context, const SegmentingSettings& settings)
	{
		if (!settings.enabled)
			return;

		ACL_ASSERT(clip_context.num_segments == 1, "ClipContext must have a single segment.");
		ACL_ASSERT(settings.ideal_num_samples <= settings.max_num_samples, "Invalid num samples for segmenting settings. %u > %u", settings.ideal_num_samples, settings.max_num_samples);

		if (clip_context.num_samples <= settings.max_num_samples)
			return;

		uint32_t num_segments = (clip_context.num_samples + settings.ideal_num_samples - 1) / settings.ideal_num_samples;
		uint32_t max_num_samples = num_segments * settings.ideal_num_samples;

		uint32_t original_num_segments = num_segments;
		uint32_t* num_samples_per_segment = allocate_type_array<uint32_t>(allocator, num_segments);
		std::fill(num_samples_per_segment, num_samples_per_segment + num_segments, settings.ideal_num_samples);

		uint32_t num_leftover_samples = settings.ideal_num_samples - (max_num_samples - clip_context.num_samples);
		if (num_leftover_samples != 0)
			num_samples_per_segment[num_segments - 1] = num_leftover_samples;

		uint32_t slack = settings.max_num_samples - settings.ideal_num_samples;
		if ((num_segments - 1) * slack >= num_leftover_samples)
		{
			// Enough segments to distribute the leftover samples of the last segment
			while (num_samples_per_segment[num_segments - 1] != 0)
			{
				for (uint32_t segment_index = 0; segment_index < num_segments - 1 && num_samples_per_segment[num_segments - 1] != 0; ++segment_index)
				{
					num_samples_per_segment[segment_index]++;
					num_samples_per_segment[num_segments - 1]--;
				}
			}

			num_segments--;
		}

		ACL_ASSERT(num_segments != 1, "Expected a number of segments greater than 1.");

		SegmentContext* clip_segment = clip_context.segments;
		clip_context.segments = allocate_type_array<SegmentContext>(allocator, num_segments);
		clip_context.num_segments = safe_static_cast<uint16_t>(num_segments);

		uint32_t clip_sample_index = 0;
		for (uint32_t segment_index = 0; segment_index < num_segments; ++segment_index)
		{
			uint32_t num_samples_in_segment = num_samples_per_segment[segment_index];

			SegmentContext& segment = clip_context.segments[segment_index];
			segment.clip = &clip_context;
			segment.bone_streams = allocate_type_array<BoneStreams>(allocator, clip_context.num_bones);
			segment.ranges = nullptr;
			segment.num_bones = clip_context.num_bones;
			segment.num_samples = safe_static_cast<uint16_t>(num_samples_in_segment);
			segment.clip_sample_offset = clip_sample_index;
			segment.segment_index = segment_index;
			segment.distribution = SampleDistribution8::Uniform;
			segment.are_rotations_normalized = false;
			segment.are_translations_normalized = false;
			segment.are_scales_normalized = false;
			segment.animated_pose_bit_size = 0;
			segment.animated_data_size = 0;
			segment.range_data_size = 0;
			segment.total_header_size = 0;

			for (uint16_t bone_index = 0; bone_index < clip_context.num_bones; ++bone_index)
			{
				const BoneStreams& clip_bone_stream = clip_segment->bone_streams[bone_index];
				BoneStreams& segment_bone_stream = segment.bone_streams[bone_index];

				segment_bone_stream.segment = &segment;
				segment_bone_stream.bone_index = bone_index;
				segment_bone_stream.parent_bone_index = clip_bone_stream.parent_bone_index;
				segment_bone_stream.output_index = clip_bone_stream.output_index;

				if (!clip_bone_stream.rotations.are_animated())
				{
					segment_bone_stream.rotations = clip_bone_stream.rotations.duplicate();
				}
				else
				{
					const RotationTrackStream& clip_stream = clip_bone_stream.rotations;
					const uint32_t sample_size = clip_stream.get_sample_size();

					RotationTrackStream segment_stream(allocator, num_samples_in_segment, sample_size, clip_stream.get_sample_rate(), clip_stream.get_rotation_format(), clip_stream.are_constant(), clip_stream.are_default(), bone_index, clip_stream.get_bit_rate());
					memcpy(segment_stream.get_raw_sample_ptr(0), clip_stream.get_raw_sample_ptr(clip_sample_index), size_t(num_samples_in_segment) * sample_size);

					segment_bone_stream.rotations = std::move(segment_stream);
				}

				if (!clip_bone_stream.translations.are_animated())
				{
					segment_bone_stream.translations = clip_bone_stream.translations.duplicate();
				}
				else
				{
					const TranslationTrackStream& clip_stream = clip_bone_stream.translations;
					const uint32_t sample_size = clip_stream.get_sample_size();

					TranslationTrackStream segment_stream(allocator, num_samples_in_segment, sample_size, clip_stream.get_sample_rate(), clip_stream.get_vector_format(), clip_stream.are_constant(), clip_stream.are_default(), bone_index, clip_stream.get_bit_rate());
					memcpy(segment_stream.get_raw_sample_ptr(0), clip_stream.get_raw_sample_ptr(clip_sample_index), size_t(num_samples_in_segment) * sample_size);

					segment_bone_stream.translations = std::move(segment_stream);
				}

				if (!clip_bone_stream.scales.are_animated())
				{
					segment_bone_stream.scales = clip_bone_stream.scales.duplicate();
				}
				else
				{
					const ScaleTrackStream& clip_stream = clip_bone_stream.scales;
					const uint32_t sample_size = clip_stream.get_sample_size();

					ScaleTrackStream segment_stream(allocator, num_samples_in_segment, sample_size, clip_stream.get_sample_rate(), clip_stream.get_vector_format(), clip_stream.are_constant(), clip_stream.are_default(), bone_index, clip_stream.get_bit_rate());
					memcpy(segment_stream.get_raw_sample_ptr(0), clip_stream.get_raw_sample_ptr(clip_sample_index), size_t(num_samples_in_segment) * sample_size);

					segment_bone_stream.scales = std::move(segment_stream);
				}
			}

			clip_sample_index += num_samples_in_segment;
		}

		deallocate_type_array(allocator, num_samples_per_segment, original_num_segments);
		destroy_segment_context(allocator, *clip_segment);
		deallocate_type_array(allocator, clip_segment, 1);
	}
}

ACL_IMPL_FILE_PRAGMA_POP
