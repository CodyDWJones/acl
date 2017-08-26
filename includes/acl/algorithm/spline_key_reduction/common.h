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

				static constexpr uint32_t LAST_FRAME_LENGTH_LOW_BIT		=  0;
				static constexpr uint32_t LAST_FRAME_LENGTH_HIGH_BIT	= 14;
				static constexpr uint32_t FRAME_LENGTH_LOW_BIT			= 15;
				static constexpr uint32_t FRAME_LENGTH_HIGH_BIT			= 29;
				static constexpr uint32_t FRAME_TYPE_LOW_BIT			= 30;
			};

			struct FrameHeader
			{
				// TODO: reallocate a bit to indicate that all bones have data, in which case bones_having_data[] will have zero elements.
				uint32_t				frame_type_and_offsets;
				uint32_t				sample_index;
				uint32_t				bones_having_data[];						// TODO: change to be [1] instead and adjust the sizeof computation
			};

			struct SegmentHeader
			{
				uint32_t				num_samples;
				uint32_t				animated_pose_bit_size;						// TODO: Calculate from bitsets and formats?

				// TODO: Only need one offset, calculate the others from the information we have?
				PtrOffset32<uint8_t>	format_per_track_data_offset;				// TODO: Make this offset optional? Only present if variable
				PtrOffset32<uint8_t>	range_data_offset;							// TODO: Make this offset optional? Only present if normalized
				PtrOffset32<uint8_t>	track_data_offset;
			};

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
