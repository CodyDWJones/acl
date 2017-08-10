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
		inline TrackStreamRange extend_stream(const TrackStream& existing_stream, const TrackStreamRange& existing_range, TrackStream& extended_stream,
			const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
		{
			ACL_ENSURE(existing_stream.get_sample_size() == sizeof(Vector4_32), "Unexpected sample size. %u != %u", stream.get_sample_size(), sizeof(Vector4_32));
			ACL_ENSURE(existing_stream.get_sample_size() == extended_stream.get_sample_size(), "Mismatched sample size.");
			ACL_ENSURE(existing_stream.get_num_samples() + num_prefixes + num_suffixes == extended_stream.get_num_samples(), "Target stream is the wrong length.");

			Vector4_32 min = existing_range.get_min();
			Vector4_32 max = existing_range.get_max();

			for (uint32_t prefix_index = 0; prefix_index < num_prefixes; ++prefix_index)
			{
				const Vector4_32& sample = prefixes[prefix_index];
				extended_stream.set_raw_sample(prefix_index, sample);
				min = vector_min(min, sample);
				max = vector_max(max, sample);
			}

			std::memcpy(extended_stream.get_raw_sample_ptr(num_prefixes), existing_stream.get_raw_sample_ptr(0), existing_stream.get_num_samples() * existing_stream.get_sample_size());

			for (uint32_t suffix_index = 0; suffix_index < num_suffixes; ++suffix_index)
			{
				const Vector4_32& sample = suffixes[suffix_index];
				extended_stream.set_raw_sample(num_prefixes + existing_stream.get_num_samples() + suffix_index, sample);
				min = vector_min(min, sample);
				max = vector_max(max, sample);
			}

			return TrackStreamRange(min, max);
		}
	}
	
	inline void extend_rotation_stream(Allocator& allocator, BoneStreams& bone_stream, const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
	{
		const RotationTrackStream& r = bone_stream.rotations;
		RotationTrackStream extended_rotations(allocator, num_prefixes + r.get_num_samples() + num_suffixes, r.get_sample_size(), r.get_sample_rate(), r.get_rotation_format(), r.get_bit_rate());
		bone_stream.rotation_range = impl::extend_stream(r, bone_stream.rotation_range, extended_rotations, prefixes, num_prefixes, suffixes, num_suffixes);
		bone_stream.rotations = std::move(extended_rotations);
	}

	inline void extend_translation_stream(Allocator& allocator, BoneStreams& bone_stream, const Vector4_32* prefixes, uint32_t num_prefixes, const Vector4_32* suffixes, uint32_t num_suffixes)
	{
		const TranslationTrackStream& t = bone_stream.translations;
		TranslationTrackStream extended_translations(allocator, num_prefixes + t.get_num_samples() + num_suffixes, t.get_sample_size(), t.get_sample_rate(), t.get_vector_format(), t.get_bit_rate());
		bone_stream.translation_range = impl::extend_stream(t, bone_stream.translation_range, extended_translations, prefixes, num_prefixes, suffixes, num_suffixes);
		bone_stream.translations = std::move(extended_translations);
	}
}
