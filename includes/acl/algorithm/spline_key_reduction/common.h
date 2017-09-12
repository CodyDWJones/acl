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

#include "acl/algorithm/spline_key_reduction/spline.h"
#include "acl/core/compressed_clip.h"
#include "acl/core/memory.h"
#include "acl/core/track_types.h"
#include "acl/math/vector4_32.h"
#include <stdint.h>

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			struct Constants
			{
				static constexpr uint32_t NUM_TRACKS_PER_BONE = 2;
			};

			struct FrameHeader
			{
				// TODO: reallocate a bit to indicate that all bones have data, in which case bones_having_data[] will have zero elements.
				uint32_t				frame_type_and_offsets;
				uint32_t				sample_index;
				uint32_t				bones_having_data[];						// TODO: change to be [1] instead and adjust the sizeof computation

				static constexpr uint32_t FRAME_TYPE_MASK = 0xC0000000;
				static constexpr uint32_t FRAME_TYPE_SHIFT = 30;

				static constexpr uint32_t NEXT_FRAME_OFFSET_MASK = 0x3FFF8000;
				static constexpr uint32_t NEXT_FRAME_OFFSET_SHIFT = 15;

				static constexpr uint32_t PREVIOUS_FRAME_OFFSET_MASK = 0x00007FFF;
				static constexpr uint32_t PREVIOUS_FRAME_OFFSET_SHIFT = 0;

				AnimationTrackType8 get_frame_type() const { return static_cast<AnimationTrackType8>(frame_type_and_offsets >> FRAME_TYPE_SHIFT); }

				void set_frame_type(AnimationTrackType8 track_type)
				{
					frame_type_and_offsets &= ~FRAME_TYPE_MASK;
					frame_type_and_offsets |= static_cast<uint32_t>(track_type) << FRAME_TYPE_SHIFT;
				}

				uint32_t get_next_frame_offset() const { return ((frame_type_and_offsets & NEXT_FRAME_OFFSET_MASK) >> NEXT_FRAME_OFFSET_SHIFT) << 2; }

				void set_next_frame_offset(uint32_t length)
				{
					ACL_ENSURE(length & 3 == 0, "Offset %d to the next frame is not a multiple of 4", length);
					uint32_t num_32_bit_words = length >> 2;

					ACL_ENSURE(num_32_bit_words & (NEXT_FRAME_OFFSET_MASK >> NEXT_FRAME_OFFSET_SHIFT) == num_32_bit_words, "Frame length %d is too large to store", length);

					frame_type_and_offsets &= ~NEXT_FRAME_OFFSET_MASK;
					frame_type_and_offsets |= num_32_bit_words << NEXT_FRAME_OFFSET_SHIFT;
				}

				uint32_t get_previous_frame_offset() const { return ((frame_type_and_offsets & PREVIOUS_FRAME_OFFSET_MASK) >> PREVIOUS_FRAME_OFFSET_SHIFT) << 2; }

				void set_previous_frame_offset(uint32_t length)
				{
					ACL_ENSURE(length & 3 == 0, "Offset %d to the previous frame is not a multiple of 4", length);
					uint32_t num_32_bit_words = length >> 2;

					ACL_ENSURE(num_32_bit_words & (PREVIOUS_FRAME_OFFSET_MASK >> PREVIOUS_FRAME_OFFSET_SHIFT) == num_32_bit_words, "Previous frame length %d is too large to store", length);

					frame_type_and_offsets &= ~PREVIOUS_FRAME_OFFSET_MASK;
					frame_type_and_offsets |= num_32_bit_words << PREVIOUS_FRAME_OFFSET_SHIFT;
				}

				const float* get_knots(uint16_t num_bones) const
				{
					uint32_t bitset_size = get_bitset_size(num_bones);
					uint32_t num_bones_having_data = bitset_count_set_bits(bones_having_data, bitset_size);
					const float* knots = safe_ptr_cast<float, const uint8_t*>(reinterpret_cast<const uint8_t*>(this) + get_next_frame_offset() - num_bones_having_data * sizeof(float));
				}
			};

			// TODO: merge with uniformly sampled's and put in a common place.
			// I think all formats will have a common header, potentially followed by custom data.
			struct SegmentHeader
			{
				uint32_t				num_samples;

				// TODO: Only need one offset, calculate the others from the information we have?
				PtrOffset32<uint8_t>	format_per_track_data_offset;				// TODO: Make this offset optional? Only present if variable
				PtrOffset32<uint8_t>	range_data_offset;							// TODO: Make this offset optional? Only present if normalized
				PtrOffset32<uint8_t>	track_data_offset;
			};

			// TODO: merge with uniformly sampled's and put in a common place.
			// I think all formats will have a common header, potentially followed by custom data.
			struct ClipHeader
			{
				uint16_t				num_bones;
				uint16_t				num_segments;
				RotationFormat8			rotation_format;
				VectorFormat8			translation_format;
				RangeReductionFlags8	clip_range_reduction;
				RangeReductionFlags8	segment_range_reduction;
				uint32_t				num_samples;
				uint32_t				sample_rate;								// TODO: Store duration as float instead

				PtrOffset16<SegmentHeader>	segment_headers_offset;
				PtrOffset16<uint32_t>		default_tracks_bitset_offset;
				PtrOffset16<uint32_t>		constant_tracks_bitset_offset;
				PtrOffset16<uint8_t>		constant_track_data_offset;
				PtrOffset16<uint8_t>		clip_range_data_offset;					// TODO: Make this offset optional? Only present if normalized

				//////////////////////////////////////////////////////////////////////////

				SegmentHeader*			get_segment_headers() { return segment_headers_offset.add_to(this); }
				const SegmentHeader*	get_segment_headers() const { return segment_headers_offset.add_to(this); }
				
				uint32_t*		get_default_tracks_bitset() { return default_tracks_bitset_offset.add_to(this); }
				const uint32_t*	get_default_tracks_bitset() const { return default_tracks_bitset_offset.add_to(this); }

				uint32_t*		get_constant_tracks_bitset() { return constant_tracks_bitset_offset.add_to(this); }
				const uint32_t*	get_constant_tracks_bitset() const { return constant_tracks_bitset_offset.add_to(this); }

				uint8_t*		get_constant_track_data() { return constant_track_data_offset.safe_add_to(this); }
				const uint8_t*	get_constant_track_data() const { return constant_track_data_offset.safe_add_to(this); }

				uint8_t*		get_format_per_track_data(const SegmentHeader& header) { return header.format_per_track_data_offset.safe_add_to(this); }
				const uint8_t*	get_format_per_track_data(const SegmentHeader& header) const { return header.format_per_track_data_offset.safe_add_to(this); }

				uint8_t*		get_clip_range_data() { return clip_range_data_offset.safe_add_to(this); }
				const uint8_t*	get_clip_range_data() const { return clip_range_data_offset.safe_add_to(this); }

				uint8_t*		get_track_data(const SegmentHeader& header) { return header.track_data_offset.safe_add_to(this); }
				const uint8_t*	get_track_data(const SegmentHeader& header) const { return header.track_data_offset.safe_add_to(this); }

				uint8_t*		get_segment_range_data(const SegmentHeader& header) { return header.range_data_offset.safe_add_to(this); }
				const uint8_t*	get_segment_range_data(const SegmentHeader& header) const { return header.range_data_offset.safe_add_to(this); }
			};

			constexpr ClipHeader& get_clip_header(CompressedClip& clip)
			{
				return *add_offset_to_ptr<ClipHeader>(&clip, sizeof(CompressedClip));
			}

			constexpr const ClipHeader& get_clip_header(const CompressedClip& clip)
			{
				return *add_offset_to_ptr<const ClipHeader>(&clip, sizeof(CompressedClip));
			}
		}
	}
}
