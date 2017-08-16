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

#include "acl/core/memory.h"
#include "acl/core/error.h"
#include "acl/math/vector4_32.h"
#include "acl/compression/stream/clip_context.h"

#include <stdint.h>

namespace acl
{
	namespace impl
	{
		inline void extend_stream(const TrackStream& existing_stream, TrackStream& extended_stream,
			const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
		{
			ACL_ENSURE(existing_stream.get_sample_size() == sizeof(Vector4_32), "Unexpected sample size. %u != %u", stream.get_sample_size(), sizeof(Vector4_32));
			ACL_ENSURE(existing_stream.get_sample_size() == extended_stream.get_sample_size(), "Mismatched sample size.");
			ACL_ENSURE(existing_stream.get_num_samples() + num_prefixes + num_suffixes == extended_stream.get_num_samples(), "Target stream is the wrong length.");

			for (uint32_t prefix_index = 0; prefix_index < num_prefixes; ++prefix_index)
			{
				const Vector4_32& sample = prefixes[prefix_index];
				extended_stream.set_raw_sample(prefix_index, sample);
			}

			std::memcpy(extended_stream.get_raw_sample_ptr(num_prefixes), existing_stream.get_raw_sample_ptr(0), existing_stream.get_num_samples() * existing_stream.get_sample_size());

			for (uint32_t suffix_index = 0; suffix_index < num_suffixes; ++suffix_index)
			{
				const Vector4_32& sample = suffixes[suffix_index];
				extended_stream.set_raw_sample(num_prefixes + existing_stream.get_num_samples() + suffix_index, sample);
			}
		}

		inline void extend_range(TrackStreamRange& range, const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
		{
			Vector4_32 min = range.get_min();
			Vector4_32 max = range.get_max();

			for (uint32_t prefix_index = 0; prefix_index < num_prefixes; ++prefix_index)
			{
				const Vector4_32& sample = prefixes[prefix_index];
				min = vector_min(min, sample);
				max = vector_max(max, sample);
			}

			for (uint32_t suffix_index = 0; suffix_index < num_suffixes; ++suffix_index)
			{
				const Vector4_32& sample = suffixes[suffix_index];
				min = vector_min(min, sample);
				max = vector_max(max, sample);
			}

			range = TrackStreamRange(min, max);
		}
	}
	
	inline void extend_rotation_stream(Allocator& allocator, BoneStreams& bone_streams, BoneRanges* bone_ranges, const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
	{
		const RotationTrackStream& r = bone_streams.rotations;
		RotationTrackStream extended_rotations(allocator, num_prefixes + r.get_num_samples() + num_suffixes, r.get_sample_size(), r.get_sample_rate(), r.get_rotation_format(), r.get_bit_rate());
		impl::extend_stream(r, extended_rotations, prefixes, num_prefixes, suffixes, num_suffixes);
		bone_streams.rotations = std::move(extended_rotations);

		if (bone_ranges != nullptr)
			impl::extend_range(bone_ranges->rotation, prefixes, num_prefixes, suffixes, num_suffixes);
	}

	inline void extend_translation_stream(Allocator& allocator, BoneStreams& bone_streams, BoneRanges* bone_ranges, const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
	{
		const TranslationTrackStream& t = bone_streams.translations;
		TranslationTrackStream extended_translations(allocator, num_prefixes + t.get_num_samples() + num_suffixes, t.get_sample_size(), t.get_sample_rate(), t.get_vector_format(), t.get_bit_rate());
		impl::extend_stream(t, extended_translations, prefixes, num_prefixes, suffixes, num_suffixes);
		bone_streams.translations = std::move(extended_translations);

		if (bone_ranges != nullptr)
			impl::extend_range(bone_ranges->translation, prefixes, num_prefixes, suffixes, num_suffixes);
	}
}
