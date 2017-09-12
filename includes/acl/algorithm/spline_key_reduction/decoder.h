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

#include "acl/core/compressed_clip.h"
#include "acl/core/utils.h"
#include "acl/core/range_reduction_types.h"
#include "acl/math/quat_32.h"
#include "acl/math/vector4_32.h"
#include "acl/math/quat_packing.h"
#include "acl/algorithm/spline_key_reduction/common.h"
#include "acl/decompression/output_writer.h"

#include <stdint.h>

//////////////////////////////////////////////////////////////////////////
// See encoder for details
//////////////////////////////////////////////////////////////////////////

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			struct ControlPoints
			{
				Vector4_32 values[NUM_CONTROL_POINTS];
				float knots[NUM_CONTROL_POINTS];
				uint32_t sample_indices[NUM_CONTROL_POINTS];
			};

			struct DecompressionContext
			{
				// Read-only data
				const SegmentHeader* segment_headers;

				const uint32_t* constant_tracks_bitset;
				const uint8_t* constant_track_data;
				const uint32_t* default_tracks_bitset;

				const uint8_t* clip_range_data;

				const uint8_t* format_per_track_data;
				const uint8_t* segment_range_data;
				const uint8_t* animated_track_data;

				uint32_t bitset_size;
				uint32_t frame_bitset_size;
				uint8_t num_rotation_components;

				float clip_duration;
				uint16_t num_bones;

				// Read-write data
				uint32_t constant_track_offset;
				uint32_t constant_track_data_offset;
				uint32_t default_track_offset;
				uint32_t clip_range_data_offset;

				FrameHeader* frame;

				ControlPoints* rotation_control_points;
				ControlPoints* translation_control_points;
			};

			template<class SettingsType>
			inline void initialize_context(const SettingsType& settings, const ClipHeader& header, DecompressionContext& context)
			{
				const RotationFormat8 rotation_format = settings.get_rotation_format(header.rotation_format);
				const VectorFormat8 translation_format = settings.get_translation_format(header.translation_format);
				const RangeReductionFlags8 clip_range_reduction = settings.get_clip_range_reduction(header.clip_range_reduction);
				const RangeReductionFlags8 segment_range_reduction = settings.get_segment_range_reduction(header.segment_range_reduction);

#if defined(ACL_USE_ERROR_CHECKS)
				ACL_ENSURE(rotation_format == header.rotation_format, "Statically compiled rotation format (%s) differs from the compressed rotation format (%s)!", get_rotation_format_name(rotation_format), get_rotation_format_name(header.rotation_format));
				ACL_ENSURE(settings.is_rotation_format_supported(rotation_format), "Rotation format (%s) isn't statically supported!", get_rotation_format_name(rotation_format));
				ACL_ENSURE(translation_format == header.translation_format, "Statically compiled translation format (%s) differs from the compressed translation format (%s)!", get_vector_format_name(translation_format), get_vector_format_name(header.translation_format));
				ACL_ENSURE(settings.is_translation_format_supported(translation_format), "Translation format (%s) isn't statically supported!", get_vector_format_name(translation_format));
				ACL_ENSURE(clip_range_reduction == header.clip_range_reduction, "Statically compiled clip range reduction settings (%u) differs from the compressed settings (%u)!", clip_range_reduction, header.clip_range_reduction);
				ACL_ENSURE(settings.are_clip_range_reduction_flags_supported(clip_range_reduction), "Clip range reduction settings (%u) aren't statically supported!", clip_range_reduction);
				ACL_ENSURE(segment_range_reduction == header.segment_range_reduction, "Statically compiled segment range reduction settings (%u) differs from the compressed settings (%u)!", segment_range_reduction, header.segment_range_reduction);
				ACL_ENSURE(settings.are_segment_range_reduction_flags_supported(segment_range_reduction), "Segment range reduction settings (%u) aren't statically supported!", segment_range_reduction);
				if (is_rotation_format_variable(rotation_format))
				{
					RotationFormat8 highest_bit_rate_format = get_highest_variant_precision(get_rotation_variant(rotation_format));
					ACL_ENSURE(settings.is_rotation_format_supported(highest_bit_rate_format), "Variable rotation format requires the highest bit rate to be supported: %s", get_rotation_format_name(highest_bit_rate_format));
				}
				if (is_vector_format_variable(translation_format))
				{
					ACL_ENSURE(settings.is_translation_format_supported(VectorFormat8::Vector3_96), "Variable translation format requires the highest bit rate to be supported: %s", get_vector_format_name(VectorFormat8::Vector3_96));
				}
#endif

				context.num_bones = header.num_bones;
				context.clip_duration = float(header.num_samples - 1) / float(header.sample_rate);

				context.segment_headers = header.get_segment_headers();
				context.default_tracks_bitset = header.get_default_tracks_bitset();

				context.constant_tracks_bitset = header.get_constant_tracks_bitset();
				context.constant_track_data = header.get_constant_track_data();
				context.clip_range_data = header.get_clip_range_data();

				context.format_per_track_data0 = nullptr;
				context.format_per_track_data1 = nullptr;

				context.segment_range_data0 = nullptr;
				context.segment_range_data1 = nullptr;

				context.animated_track_data0 = nullptr;
				context.animated_track_data1 = nullptr;

				context.bitset_size = get_bitset_size(header.num_bones * Constants::NUM_TRACKS_PER_BONE);
				context.frame_bitset_size = get_bitset_size(header.num_bones);
				context.num_rotation_components = rotation_format == RotationFormat8::Quat_128 ? 4 : 3;

				// TODO: initialize to the first frame
				context.frame = nullptr;

				for (uint16_t bone_index = 0; bone_index < header.num_bones; ++bone_index)
				{
					ControlPoints& control_points = context.bone_control_points + bone_index;

					control_points.values[NUM_CONTROL_POINTS - 1] = vector_zero_32();	// TODO - read from frame
					control_points.knots[NUM_CONTROL_POINTS - 1] = 0;
					control_points.sample_indices[NUM_CONTROL_POINTS - 1] = 0;
					
					// TODO: apply same values to all indices? Decide after proto-implementing seek().

					// The other array elements will be initialized by seek() when it is first called.
				}
			}

			// TODO: check with Nicholas whether to use _0 and _1 instead of 0 and 1 suffixes. I can switch uniform code if necessary.

			inline void read_frame(float sample_key, DecompressionContext& context, uint16_t only_bone_index = INVALID_BONE_INDEX)
			{
				AnimationTrackType8 track_type = context.frame->get_frame_type();
				const float* knots = context.frame->get_knots(context.num_bones);

				for (uint16_t bone_index = 0; bone_index < context.num_bones; ++bone_index)
				{
					if (only_bone_index != INVALID_BONE_INDEX && only_bone_index != bone_index ||
					    !bitset_test(context.frame->bones_having_data, context.frame_bitset_size, bone_index))
					{
						continue;
					}

					ControlPoints* control_points = nullptr;

					switch (track_type)
					{
					case AnimationTrackType8::Rotation:
						control_points = context.rotation_control_points + bone_index;
						break;

					case AnimationTrackType8::Translation:
						control_points = context.translation_control_points + bone_index;
						break;
					}

					if (control_points->sample_indices[RIGHT_INTERPOLATION_KNOT_INDEX] < sample_key)
					{
						// Shift new control point in from the right
						std::copy(control_points->values, control_points->values + NUM_CONTROL_POINTS - 1, control_points->values);
						std::copy(control_points->knots, control_points->knots + NUM_CONTROL_POINTS - 1, control_points->knots);
						std::copy(control_points->sample_indices, control_points->sample_indices + NUM_CONTROL_POINTS - 1, control_points->sample_indices);
						
						// Read next animated value into NUM_CONTROL_POINTS - 1

						control_points->sample_indices[NUM_CONTROL_POINTS - 1] = context.frame->sample_index;
						control_points->knots[NUM_CONTROL_POINTS - 1] = *knots;
						++knots;
					}
					else if (control_points->sample_indices[LEFT_INTERPOLATION_KNOT_INDEX] > sample_key)
					{
						// Shift new control point in from the left
						std::copy_backward(control_points->values, control_points->values + NUM_CONTROL_POINTS - 2, control_points->values + NUM_CONTROL_POINTS - 1);
						std::copy_backward(control_points->knots, control_points->knots + NUM_CONTROL_POINTS - 2, control_points->knots + NUM_CONTROL_POINTS - 1);
						std::copy_backward(control_points->sample_indices, control_points->sample_indices + NUM_CONTROL_POINTS - 2, control_points->sample_indices + NUM_CONTROL_POINTS - 1);

						// Read next animated sample into [0]

						control_points->sample_indices[0] = context.frame->sample_index;
						control_points->knots[0] = *knots;
						++knots;
					}
				}
			}

			inline bool must_read_left(float sample_key, const DecompressionContext& context, uint16_t only_bone_index = INVALID_BONE_INDEX)
			{
				for (uint16_t bone_index = 0; bone_index < context.num_bones; ++bone_index)
				{
					if (only_bone_index != INVALID_BONE_INDEX && only_bone_index != bone_index)
						continue;

					if (context.rotation_control_points[bone_index].sample_indices[LEFT_INTERPOLATION_KNOT_INDEX] > sample_key ||
						context.translation_control_points[bone_index].sample_indices[LEFT_INTERPOLATION_KNOT_INDEX] > sample_key)
					{
						return true;
					}
				}

				return false;
			}

			inline bool must_read_right(float sample_key, const DecompressionContext& context, uint16_t only_bone_index = INVALID_BONE_INDEX)
			{
				for (uint16_t bone_index = 0; bone_index < context.num_bones; ++bone_index)
				{
					if (only_bone_index != INVALID_BONE_INDEX && only_bone_index != bone_index)
						continue;

					if (context.rotation_control_points[bone_index].sample_indices[RIGHT_INTERPOLATION_KNOT_INDEX] < sample_key ||
						context.translation_control_points[bone_index].sample_indices[RIGHT_INTERPOLATION_KNOT_INDEX] < sample_key)
					{
						return true;
					}
				}

				return false;
			}

			template<class SettingsType>
			inline void seek(const SettingsType& settings, const ClipHeader& header, float sample_time, DecompressionContext& context)
			{
				context.constant_track_offset = 0;
				context.constant_track_data_offset = 0;
				context.default_track_offset = 0;
				context.clip_range_data_offset = 0;
				context.format_per_track_data_offset = 0;
				context.segment_range_data_offset = 0;

				float sample_key = calculate_sample_key(header.num_samples, context.clip_duration, sample_time);

				while (true)
				{
					bool read_left = must_read_left(sample_key, context);
					bool read_right = must_read_right(sample_key, context);

					if (!read_left && !read_right)
						break;

					FrameHeader* started_at = context.frame;

					if (read_left)
					{
						// TODO: use more of Nicholas's memory functions in my code, like I am here
						uint32_t offset = context.frame->get_previous_frame_offset();
						ACL_ENSURE(offset != 0, "Cannot read before the first frame");
						context.frame = add_offset_to_ptr<const FrameHeader*>(context.frame, -offset);
					}
					
					if (read_right)
					{
						if (read_left)
							context.frame = started_at;

						uint32_t offset = context.frame->get_next_frame_offset();
						ACL_ENSURE(offset != 0, "Cannot read beyond the last frame");
						context.frame = add_offset_to_ptr<const FrameHeader*>(context.frame, offset);
					}
				}

				// TODO: generalize seek or create custom version that ignores bones that decompress_bone doesn't need?
				// Can't load fake sample indices into the context for all but the interested bone, because that'll break
				// a following call to seek().
				//
				// Need a way to actually read frames but keep data for just a specific bone!
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// Deriving from this struct and overriding these constexpr functions
		// allow you to control which code is stripped for maximum performance.
		// With these, you can:
		//    - Support only a subset of the formats and statically strip the rest
		//    - Force a single format and statically strip the rest
		//    - Decide all of this at runtime by not making the overrides constexpr
		//
		// By default, all formats are supported.
		//////////////////////////////////////////////////////////////////////////
		struct DecompressionSettings
		{
			constexpr bool is_rotation_format_supported(RotationFormat8 format) const { return true; }
			constexpr bool is_translation_format_supported(VectorFormat8 format) const { return true; }
			constexpr RotationFormat8 get_rotation_format(RotationFormat8 format) const { return format; }
			constexpr VectorFormat8 get_translation_format(VectorFormat8 format) const { return format; }

			constexpr bool are_clip_range_reduction_flags_supported(RangeReductionFlags8 flags) const { return true; }
			constexpr bool are_segment_range_reduction_flags_supported(RangeReductionFlags8 flags) const { return true; }
			constexpr RangeReductionFlags8 get_clip_range_reduction(RangeReductionFlags8 flags) const { return flags; }
			constexpr RangeReductionFlags8 get_segment_range_reduction(RangeReductionFlags8 flags) const { return flags; }

			// Whether tracks must all be variable or all fixed width, or if they can be mixed and require padding
			constexpr bool supports_mixed_packing() const { return true; }
		};

		template<class SettingsType>
		inline void* allocate_decompression_context(Allocator& allocator, const SettingsType& settings, const CompressedClip& clip)
		{
			using namespace impl;

			const ClipHeader& clip_header = get_clip_header(clip);

			DecompressionContext* context = allocate_type<DecompressionContext>(allocator);
			
			context.rotation_control_points = allocate_type_array<ControlPoints>(allocator, clip_header.num_bones);
			context.rotation_segment_offsets = allocate_type_array<SegmentOffsets>(allocator, clip_header.num_bones);
			
			context.translation_control_points = allocate_type_array<ControlPoints>(allocator, clip_header.num_bones);
			context.translation_segment_offsets = allocate_type_array<SegmentOffsets>(allocator, clip_header.num_bones);

			initialize_context(settings, clip_header, *context);

			return context;
		}

		inline void deallocate_decompression_context(Allocator& allocator, void* opaque_context)
		{
			using namespace impl;

			DecompressionContext* context = safe_ptr_cast<DecompressionContext>(opaque_context);
			
			deallocate_type_array(allocator, context->rotation_control_points, context->num_bones);
			deallocate_type_array(allocator, context->rotation_segment_offsets, context->num_bones);
			
			deallocate_type_array(allocator, context->translation_control_points, context->num_bones);
			deallocate_type_array(allocator, context->translation_segment_offsets, context->num_bones);
			
			deallocate_type<DecompressionContext>(allocator, context);
		}

		template<class SettingsType, class OutputWriterType>
		inline void decompress_pose(const SettingsType& settings, const CompressedClip& clip, void* opaque_context, float sample_time, OutputWriterType& writer)
		{
			static_assert(std::is_base_of<DecompressionSettings, SettingsType>::value, "SettingsType must derive from DecompressionSettings!");
			static_assert(std::is_base_of<OutputWriter, OutputWriterType>::value, "OutputWriterType must derive from OutputWriter!");

			using namespace impl;

			ACL_ENSURE(clip.get_algorithm_type() == AlgorithmType8::UniformlySampled, "Invalid algorithm type [%s], expected [%s]", get_algorithm_name(clip.get_algorithm_type()), get_algorithm_name(AlgorithmType8::UniformlySampled));
			ACL_ENSURE(clip.is_valid(false), "Clip is invalid");

			const ClipHeader& header = get_clip_header(clip);

			DecompressionContext& context = *safe_ptr_cast<DecompressionContext>(opaque_context);

			seek(settings, header, sample_time, context);

			for (uint32_t bone_index = 0; bone_index < header.num_bones; ++bone_index)
			{
				Quat_32 rotation = decompress_rotation(settings, header, context);
				writer.write_bone_rotation(bone_index, rotation);

				Vector4_32 translation = decompress_translation(settings, header, context);
				writer.write_bone_translation(bone_index, translation);
			}
		}

		template<class SettingsType>
		inline void decompress_bone(const SettingsType& settings, const CompressedClip& clip, void* opaque_context, float sample_time, uint16_t sample_bone_index, Quat_32* out_rotation, Vector4_32* out_translation)
		{
			static_assert(std::is_base_of<DecompressionSettings, SettingsType>::value, "SettingsType must derive from DecompressionSettings!");

			using namespace impl;

			ACL_ENSURE(clip.get_algorithm_type() == AlgorithmType8::UniformlySampled, "Invalid algorithm type [%s], expected [%s]", get_algorithm_name(clip.get_algorithm_type()), get_algorithm_name(AlgorithmType8::UniformlySampled));
			ACL_ENSURE(clip.is_valid(false), "Clip is invalid");

			const ClipHeader& header = get_clip_header(clip);

			DecompressionContext& context = *safe_ptr_cast<DecompressionContext>(opaque_context);

			seek(settings, header, sample_time, context);

			// TODO: Optimize this by counting the number of bits set, we can use the pop-count instruction on
			// architectures that support it (e.g. xb1/ps4). This would entirely avoid looping here.
			for (uint32_t bone_index = 0; bone_index < header.num_bones; ++bone_index)
			{
				if (bone_index == sample_bone_index)
					break;

				skip_rotation(settings, header, context);
				skip_translation(settings, header, context);
			}

			// TODO: Skip if not interested in return value
			Quat_32 rotation = decompress_rotation(settings, header, context);
			if (out_rotation != nullptr)
				*out_rotation = rotation;

			Vector4_32 translation = decompress_translation(settings, header, context);
			if (out_translation != nullptr)
				*out_translation = translation;
		}
	}
}
