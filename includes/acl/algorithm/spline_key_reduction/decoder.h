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
				uint8_t num_rotation_components;

				float clip_duration;
				uint16_t num_bones;

				// Read-write data
				uint32_t constant_track_offset;
				uint32_t constant_track_data_offset;
				uint32_t default_track_offset;
				uint32_t clip_range_data_offset;

				ControlPoint* rotation_control_points;
				FrameHeader* rotation_frame;

				ControlPoint* translation_control_points;
				FrameHeader* translation_frame;
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
				context.num_rotation_components = rotation_format == RotationFormat8::Quat_128 ? 4 : 3;

				// TODO: initialize to the first rotation frame
				context.rotation_frame = nullptr;

				// TODO: initialize to the first translation frame
				context.translation_frame = nullptr;

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

			enum class Direction
			{
				Left,
				Right
			};

			inline void load_frames_stopping_after(AnimationTrackType8 track_type, Direction direction)
			{
				// Read the previous frame and for each set bone, shift the control points right one position and load
				// new values into the leftmost.

				// Do same but stop after reading a translation frame. Allow reading of rotation frames.

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

			move_loop:
				bool restart = false;

				for (uint16_t bone_index = 0; bone_index < num_bones; ++bone_index)
				{
					while (context.rotation_control_points[bone_index].sample_indices[LEFT_INTERPOLATION_KNOT_INDEX] > sample_key)
					{
						load_frames_stopping_after(AnimationTrackType8::Rotation, Direction::Left);
						restart = true;
					}
					
					if (restart) goto move_loop;

					while (context.translation_control_points[bone_index].sample_indices[LEFT_INTERPOLATION_KNOT_INDEX] > sample_key)
					{
						load_frames_stopping_after(AnimationTrackType8::Translation, Direction::Left);
						restart = true;
					}

					if (restart) goto move_loop;

					while (context.rotation_control_points[bone_index].sample_indices[RIGHT_INTERPOLATION_KNOT_INDEX] < sample_key)
					{
						load_frames_stopping_after(AnimationTrackType8::Rotation, Direction::Right);
						restart = true;
					}

					if (restart) goto move_loop;

					while (context.translation_control_points[bone_index].sample_indices[RIGHT_INTERPOLATION_KNOT_INDEX] < sample_key)
					{
						load_frames_stopping_after(AnimationTrackType8::Translation, Direction::Right);
						restart = true;
					}

					if (restart) goto move_loop;
				}

				// Read through all the bones compiling the smallest range of samples that are permitted.

				// If sample_time is within that range, we're done.  Interpolator will need to compute t, blending functions, and weight control points
				//    for desired bone(s).

				// If OUT of that range
				//		if > max then we're reading forward. While there is some bone whose interpolation range doesn't include t,
				//		process the data from the next frame, ie.
				//		read the bitset indicating which bones need new rotation control points
				//		(TODO: make the encoder sweep forward through time looking for the highest error, then try adding control points across the bones
				//        **for that sample index** until the error is met. This will maximize number of bits set.)
				//		shift those values and knots into working memory from the RIGHT side
				//		shift in sample index too
				//		repeat for translations
				//
				//		if < min we're skipping backward. While there is some bone whose interpolation range doesn't include t,
				//		read the size of the last frame
				//		go back that many bytes
				//		Skip this frame, because it's already applied.
				//		read the size of the last frame
				//		go back that many bytes
				//		shift those values and knots into working memory from the LEFT side
				//		shift in sample index too

				// TODO: generalize seek or create custom version that ignores bones that decompress_bone doesn't need?

				uint32_t segment_key_frame0;
				uint32_t segment_key_frame1;

				// Find segments
				// TODO: Use binary search?
				uint32_t segment_key_frame = 0;
				const SegmentHeader* segment_header0;
				const SegmentHeader* segment_header1;
				for (uint16_t segment_index = 0; segment_index < header.num_segments; ++segment_index)
				{
					const SegmentHeader& segment_header = context.segment_headers[segment_index];

					if (key_frame0 >= segment_key_frame && key_frame0 < segment_key_frame + segment_header.num_samples)
					{
						segment_header0 = &segment_header;
						segment_key_frame0 = key_frame0 - segment_key_frame;

						if (key_frame1 >= segment_key_frame && key_frame1 < segment_key_frame + segment_header.num_samples)
						{
							segment_header1 = &segment_header;
							segment_key_frame1 = key_frame1 - segment_key_frame;
						}
						else
						{
							ACL_ENSURE(segment_index + 1 < header.num_segments, "Invalid segment index: %u", segment_index + 1);
							const SegmentHeader& next_segment_header = context.segment_headers[segment_index + 1];
							segment_header1 = &next_segment_header;
							segment_key_frame1 = key_frame1 - (segment_key_frame + segment_header.num_samples);
						}

						break;
					}

					segment_key_frame += segment_header.num_samples;
				}

				context.format_per_track_data0 = header.get_format_per_track_data(*segment_header0);
				context.format_per_track_data1 = header.get_format_per_track_data(*segment_header1);
				context.segment_range_data0 = header.get_segment_range_data(*segment_header0);
				context.segment_range_data1 = header.get_segment_range_data(*segment_header1);
				context.animated_track_data0 = header.get_track_data(*segment_header0);
				context.animated_track_data1 = header.get_track_data(*segment_header1);

				context.key_frame_byte_offset0 = (segment_key_frame0 * segment_header0->animated_pose_bit_size) / 8;
				context.key_frame_byte_offset1 = (segment_key_frame1 * segment_header1->animated_pose_bit_size) / 8;
				context.key_frame_bit_offset0 = segment_key_frame0 * segment_header0->animated_pose_bit_size;
				context.key_frame_bit_offset1 = segment_key_frame1 * segment_header1->animated_pose_bit_size;
			}

			template<class SettingsType>
			inline void skip_rotation(const SettingsType& settings, const ClipHeader& header, DecompressionContext& context)
			{
				bool is_rotation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.default_track_offset);
				if (!is_rotation_default)
				{
					const RotationFormat8 rotation_format = settings.get_rotation_format(header.rotation_format);

					bool is_rotation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.constant_track_offset);
					if (is_rotation_constant)
					{
						const RotationFormat8 packed_format = is_rotation_format_variable(rotation_format) ? get_highest_variant_precision(get_rotation_variant(rotation_format)) : rotation_format;
						context.constant_track_data_offset += get_packed_rotation_size(packed_format);
					}
					else
					{
						if (is_rotation_format_variable(rotation_format))
						{
							uint8_t bit_rate0 = context.format_per_track_data0[context.format_per_track_data_offset];
							uint8_t bit_rate1 = context.format_per_track_data1[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate0 = get_num_bits_at_bit_rate(bit_rate0) * 3;	// 3 components
							uint8_t num_bits_at_bit_rate1 = get_num_bits_at_bit_rate(bit_rate1) * 3;	// 3 components

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								num_bits_at_bit_rate0 = align_to(num_bits_at_bit_rate0, MIXED_PACKING_ALIGNMENT_NUM_BITS);
								num_bits_at_bit_rate1 = align_to(num_bits_at_bit_rate1, MIXED_PACKING_ALIGNMENT_NUM_BITS);
							}

							context.key_frame_bit_offset0 += num_bits_at_bit_rate0;
							context.key_frame_bit_offset1 += num_bits_at_bit_rate1;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}
						else
						{
							uint32_t rotation_size = get_packed_rotation_size(rotation_format);
							context.key_frame_byte_offset0 += rotation_size;
							context.key_frame_byte_offset1 += rotation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}

						const RangeReductionFlags8 clip_range_reduction = settings.get_clip_range_reduction(header.clip_range_reduction);
						const RangeReductionFlags8 segment_range_reduction = settings.get_segment_range_reduction(header.segment_range_reduction);
						if (is_enum_flag_set(clip_range_reduction, RangeReductionFlags8::Rotations))
							context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;

						if (is_enum_flag_set(segment_range_reduction, RangeReductionFlags8::Rotations))
							context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
			}

			template<class SettingsType>
			inline void skip_translation(const SettingsType& settings, const ClipHeader& header, DecompressionContext& context)
			{
				bool is_translation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.default_track_offset);
				if (!is_translation_default)
				{
					bool is_translation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.constant_track_offset);
					if (is_translation_constant)
					{
						// Constant translation tracks store the remaining sample with full precision
						context.constant_track_data_offset += get_packed_vector_size(VectorFormat8::Vector3_96);
					}
					else
					{
						const VectorFormat8 translation_format = settings.get_translation_format(header.translation_format);

						if (is_vector_format_variable(translation_format))
						{
							uint8_t bit_rate0 = context.format_per_track_data0[context.format_per_track_data_offset];
							uint8_t bit_rate1 = context.format_per_track_data1[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate0 = get_num_bits_at_bit_rate(bit_rate0) * 3;	// 3 components
							uint8_t num_bits_at_bit_rate1 = get_num_bits_at_bit_rate(bit_rate1) * 3;	// 3 components

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								num_bits_at_bit_rate0 = align_to(num_bits_at_bit_rate0, MIXED_PACKING_ALIGNMENT_NUM_BITS);
								num_bits_at_bit_rate1 = align_to(num_bits_at_bit_rate1, MIXED_PACKING_ALIGNMENT_NUM_BITS);
							}

							context.key_frame_bit_offset0 += num_bits_at_bit_rate0;
							context.key_frame_bit_offset1 += num_bits_at_bit_rate1;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}
						else
						{
							uint32_t translation_size = get_packed_vector_size(translation_format);
							context.key_frame_byte_offset0 += translation_size;
							context.key_frame_byte_offset1 += translation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}

						const RangeReductionFlags8 clip_range_reduction = settings.get_clip_range_reduction(header.clip_range_reduction);
						const RangeReductionFlags8 segment_range_reduction = settings.get_segment_range_reduction(header.segment_range_reduction);
						if (is_enum_flag_set(clip_range_reduction, RangeReductionFlags8::Translations))
							context.clip_range_data_offset += 3 * sizeof(float) * 2;

						if (is_enum_flag_set(segment_range_reduction, RangeReductionFlags8::Translations))
							context.segment_range_data_offset += 3 * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
			}

			// TODO: holy crap. This should be common code.
			template<class SettingsType>
			inline Quat_32 decompress_rotation(const SettingsType& settings, const ClipHeader& header, DecompressionContext& context)
			{
				Quat_32 rotation;

				bool is_rotation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.default_track_offset);
				if (is_rotation_default)
				{
					rotation = quat_identity_32();
				}
				else
				{
					const RotationFormat8 rotation_format = settings.get_rotation_format(header.rotation_format);

					bool is_rotation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.constant_track_offset);
					if (is_rotation_constant)
					{
						const RotationFormat8 packed_format = is_rotation_format_variable(rotation_format) ? get_highest_variant_precision(get_rotation_variant(rotation_format)) : rotation_format;

						if (packed_format == RotationFormat8::Quat_128 && settings.is_rotation_format_supported(RotationFormat8::Quat_128))
							rotation = unpack_quat_128(context.constant_track_data + context.constant_track_data_offset);
						else if (packed_format == RotationFormat8::QuatDropW_96 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_96))
							rotation = unpack_quat_96(context.constant_track_data + context.constant_track_data_offset);
						else if (packed_format == RotationFormat8::QuatDropW_48 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_48))
							rotation = unpack_quat_48(context.constant_track_data + context.constant_track_data_offset);
						else if (packed_format == RotationFormat8::QuatDropW_32 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_32))
							rotation = unpack_quat_32(context.constant_track_data + context.constant_track_data_offset);

						ACL_ENSURE(quat_is_finite(rotation), "Rotation is not valid!");
						ACL_ENSURE(quat_is_normalized(rotation), "Rotation is not normalized!");

						context.constant_track_data_offset += get_packed_rotation_size(packed_format);
					}
					else
					{
						const RangeReductionFlags8 clip_range_reduction = settings.get_clip_range_reduction(header.clip_range_reduction);
						const RangeReductionFlags8 segment_range_reduction = settings.get_segment_range_reduction(header.segment_range_reduction);
						const bool are_clip_rotations_normalized = is_enum_flag_set(clip_range_reduction, RangeReductionFlags8::Rotations);
						const bool are_segment_rotations_normalized = is_enum_flag_set(segment_range_reduction, RangeReductionFlags8::Rotations);

						Quat_32 rotation0;
						Quat_32 rotation1;

						if (rotation_format == RotationFormat8::Quat_128 && settings.is_rotation_format_supported(RotationFormat8::Quat_128))
						{
							Vector4_32 rotation0_xyzw = unpack_vector4_128(context.animated_track_data0 + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyzw = unpack_vector4_128(context.animated_track_data1 + context.key_frame_byte_offset1);

							if (are_segment_rotations_normalized)
							{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
								Vector4_32 segment_range_min0 = unpack_vector4_32(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector4_32(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector4_32(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector4_32(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
#else
								Vector4_32 segment_range_min0 = unpack_vector4_64(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector4_64(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector4_64(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector4_64(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
#endif

								rotation0_xyzw = vector_mul_add(rotation0_xyzw, segment_range_extent0, segment_range_min0);
								rotation1_xyzw = vector_mul_add(rotation1_xyzw, segment_range_extent1, segment_range_min1);

								context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
							}

							if (are_clip_rotations_normalized)
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (context.num_rotation_components * sizeof(float)));

								rotation0_xyzw = vector_mul_add(rotation0_xyzw, clip_range_extent, clip_range_min);
								rotation1_xyzw = vector_mul_add(rotation1_xyzw, clip_range_extent, clip_range_min);

								context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;
							}

							rotation0 = vector_to_quat(rotation0_xyzw);
							rotation1 = vector_to_quat(rotation1_xyzw);

							const uint32_t rotation_size = get_packed_rotation_size(rotation_format);
							context.key_frame_byte_offset0 += rotation_size;
							context.key_frame_byte_offset1 += rotation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (rotation_format == RotationFormat8::QuatDropW_96 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_96))
						{
							Vector4_32 rotation0_xyz = unpack_vector3_96(context.animated_track_data0 + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_96(context.animated_track_data1 + context.key_frame_byte_offset1);

							if (are_segment_rotations_normalized)
							{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
								Vector4_32 segment_range_min0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
#else
								Vector4_32 segment_range_min0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
#endif

								rotation0_xyz = vector_mul_add(rotation0_xyz, segment_range_extent0, segment_range_min0);
								rotation1_xyz = vector_mul_add(rotation1_xyz, segment_range_extent1, segment_range_min1);

								context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
							}

							if (are_clip_rotations_normalized)
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (context.num_rotation_components * sizeof(float)));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;
							}

							rotation0 = quat_from_positive_w(rotation0_xyz);
							rotation1 = quat_from_positive_w(rotation1_xyz);

							const uint32_t rotation_size = get_packed_rotation_size(rotation_format);
							context.key_frame_byte_offset0 += rotation_size;
							context.key_frame_byte_offset1 += rotation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (rotation_format == RotationFormat8::QuatDropW_48 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_48))
						{
							Vector4_32 rotation0_xyz = unpack_vector3_48(context.animated_track_data0 + context.key_frame_byte_offset0, are_clip_rotations_normalized);
							Vector4_32 rotation1_xyz = unpack_vector3_48(context.animated_track_data1 + context.key_frame_byte_offset1, are_clip_rotations_normalized);

							if (are_segment_rotations_normalized)
							{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
								Vector4_32 segment_range_min0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
#else
								Vector4_32 segment_range_min0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
#endif

								rotation0_xyz = vector_mul_add(rotation0_xyz, segment_range_extent0, segment_range_min0);
								rotation1_xyz = vector_mul_add(rotation1_xyz, segment_range_extent1, segment_range_min1);

								context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
							}

							if (are_clip_rotations_normalized)
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (context.num_rotation_components * sizeof(float)));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;
							}

							rotation0 = quat_from_positive_w(rotation0_xyz);
							rotation1 = quat_from_positive_w(rotation1_xyz);

							const uint32_t rotation_size = get_packed_rotation_size(rotation_format);
							context.key_frame_byte_offset0 += rotation_size;
							context.key_frame_byte_offset1 += rotation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (rotation_format == RotationFormat8::QuatDropW_32 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_32))
						{
							Vector4_32 rotation0_xyz = unpack_vector3_32(11, 11, 10, are_clip_rotations_normalized, context.animated_track_data0 + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_32(11, 11, 10, are_clip_rotations_normalized, context.animated_track_data1 + context.key_frame_byte_offset1);

							if (are_segment_rotations_normalized)
							{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
								Vector4_32 segment_range_min0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);
#else
								Vector4_32 segment_range_min0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
								Vector4_32 segment_range_min1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);
#endif

								rotation0_xyz = vector_mul_add(rotation0_xyz, segment_range_extent0, segment_range_min0);
								rotation1_xyz = vector_mul_add(rotation1_xyz, segment_range_extent1, segment_range_min1);

								context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
							}

							if (are_clip_rotations_normalized)
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (context.num_rotation_components * sizeof(float)));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;
							}

							rotation0 = quat_from_positive_w(rotation0_xyz);
							rotation1 = quat_from_positive_w(rotation1_xyz);

							const uint32_t rotation_size = get_packed_rotation_size(rotation_format);
							context.key_frame_byte_offset0 += rotation_size;
							context.key_frame_byte_offset1 += rotation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (rotation_format == RotationFormat8::QuatDropW_Variable && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_Variable))
						{
							uint8_t bit_rate0 = context.format_per_track_data0[context.format_per_track_data_offset];
							uint8_t bit_rate1 = context.format_per_track_data1[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate0 = get_num_bits_at_bit_rate(bit_rate0);
							uint8_t num_bits_at_bit_rate1 = get_num_bits_at_bit_rate(bit_rate1);

							Vector4_32 rotation0_xyz;
							Vector4_32 rotation1_xyz;

							if (is_pack_0_bit_rate(bit_rate0))
								(void)bit_rate0;
							else if (is_pack_72_bit_rate(bit_rate0))
								rotation0_xyz = unpack_vector3_72(are_clip_rotations_normalized, context.animated_track_data0, context.key_frame_bit_offset0);
							else if (is_pack_96_bit_rate(bit_rate0))
								rotation0_xyz = unpack_vector3_96(context.animated_track_data0, context.key_frame_bit_offset0);
							else
								rotation0_xyz = unpack_vector3_n(num_bits_at_bit_rate0, num_bits_at_bit_rate0, num_bits_at_bit_rate0, are_clip_rotations_normalized, context.animated_track_data0, context.key_frame_bit_offset0);

							if (is_pack_0_bit_rate(bit_rate1))
								(void)bit_rate1;
							else if (is_pack_72_bit_rate(bit_rate1))
								rotation1_xyz = unpack_vector3_72(are_clip_rotations_normalized, context.animated_track_data1, context.key_frame_bit_offset1);
							else if (is_pack_96_bit_rate(bit_rate1))
								rotation1_xyz = unpack_vector3_96(context.animated_track_data1, context.key_frame_bit_offset1);
							else
								rotation1_xyz = unpack_vector3_n(num_bits_at_bit_rate1, num_bits_at_bit_rate1, num_bits_at_bit_rate1, are_clip_rotations_normalized, context.animated_track_data1, context.key_frame_bit_offset1);

							if (are_segment_rotations_normalized)
							{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
								if (is_pack_0_bit_rate(bit_rate0))
									rotation0_xyz = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
								else
								{
									Vector4_32 segment_range_min0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset, true);
									Vector4_32 segment_range_extent0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);

									rotation0_xyz = vector_mul_add(rotation0_xyz, segment_range_extent0, segment_range_min0);
								}

								if (is_pack_0_bit_rate(bit_rate1))
									rotation1_xyz = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
								else
								{
									Vector4_32 segment_range_min1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset, true);
									Vector4_32 segment_range_extent1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint8_t)), true);

									rotation1_xyz = vector_mul_add(rotation1_xyz, segment_range_extent1, segment_range_min1);
								}
#else
								if (is_pack_0_bit_rate(bit_rate0))
									rotation0_xyz = unpack_vector3_96(context.segment_range_data0 + context.segment_range_data_offset);
								else
								{
									Vector4_32 segment_range_min0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
									Vector4_32 segment_range_extent0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);

									rotation0_xyz = vector_mul_add(rotation0_xyz, segment_range_extent0, segment_range_min0);
								}

								if (is_pack_0_bit_rate(bit_rate1))
									rotation1_xyz = unpack_vector3_96(context.segment_range_data1 + context.segment_range_data_offset);
								else
								{
									Vector4_32 segment_range_min1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
									Vector4_32 segment_range_extent1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset + (context.num_rotation_components * sizeof(uint16_t)), true);

									rotation1_xyz = vector_mul_add(rotation1_xyz, segment_range_extent1, segment_range_min1);
								}
#endif

								context.segment_range_data_offset += context.num_rotation_components * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
							}

							if (are_clip_rotations_normalized)
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (context.num_rotation_components * sizeof(float)));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.clip_range_data_offset += context.num_rotation_components * sizeof(float) * 2;
							}

							rotation0 = quat_from_positive_w(rotation0_xyz);
							rotation1 = quat_from_positive_w(rotation1_xyz);

							uint8_t num_bits_read0 = num_bits_at_bit_rate0 * 3;
							uint8_t num_bits_read1 = num_bits_at_bit_rate1 * 3;
							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								num_bits_read0 = align_to(num_bits_read0, MIXED_PACKING_ALIGNMENT_NUM_BITS);
								num_bits_read1 = align_to(num_bits_read1, MIXED_PACKING_ALIGNMENT_NUM_BITS);
							}

							context.key_frame_bit_offset0 += num_bits_read0;
							context.key_frame_bit_offset1 += num_bits_read1;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}

						rotation = quat_lerp(rotation0, rotation1, context.interpolation_alpha);

						ACL_ENSURE(quat_is_finite(rotation), "Rotation is not valid!");
						ACL_ENSURE(quat_is_normalized(rotation), "Rotation is not normalized!");
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
				return rotation;
			}

			template<class SettingsType>
			inline Vector4_32 decompress_translation(const SettingsType& settings, const ClipHeader& header, DecompressionContext& context)
			{
				Vector4_32 translation;

				bool is_translation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.default_track_offset);
				if (is_translation_default)
				{
					translation = vector_zero_32();
				}
				else
				{
					bool is_translation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.constant_track_offset);
					if (is_translation_constant)
					{
						// Constant translation tracks store the remaining sample with full precision
						translation = unpack_vector3_96(context.constant_track_data + context.constant_track_data_offset);

						ACL_ENSURE(vector_is_finite3(translation), "Translation is not valid!");

						context.constant_track_data_offset += get_packed_vector_size(VectorFormat8::Vector3_96);
					}
					else
					{
						const VectorFormat8 translation_format = settings.get_translation_format(header.translation_format);
						const RangeReductionFlags8 clip_range_reduction = settings.get_clip_range_reduction(header.clip_range_reduction);
						const RangeReductionFlags8 segment_range_reduction = settings.get_segment_range_reduction(header.segment_range_reduction);

						Vector4_32 translation0;
						Vector4_32 translation1;

						uint8_t bit_rate0;
						uint8_t bit_rate1;

						if (translation_format == VectorFormat8::Vector3_96 && settings.is_translation_format_supported(VectorFormat8::Vector3_96))
						{
							translation0 = unpack_vector3_96(context.animated_track_data0 + context.key_frame_byte_offset0);
							translation1 = unpack_vector3_96(context.animated_track_data1 + context.key_frame_byte_offset1);

							const uint32_t translation_size = get_packed_vector_size(translation_format);
							context.key_frame_byte_offset0 += translation_size;
							context.key_frame_byte_offset1 += translation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (translation_format == VectorFormat8::Vector3_48 && settings.is_translation_format_supported(VectorFormat8::Vector3_48))
						{
							translation0 = unpack_vector3_48(context.animated_track_data0 + context.key_frame_byte_offset0, true);
							translation1 = unpack_vector3_48(context.animated_track_data1 + context.key_frame_byte_offset1, true);

							const uint32_t translation_size = get_packed_vector_size(translation_format);
							context.key_frame_byte_offset0 += translation_size;
							context.key_frame_byte_offset1 += translation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (translation_format == VectorFormat8::Vector3_32 && settings.is_translation_format_supported(VectorFormat8::Vector3_32))
						{
							translation0 = unpack_vector3_32(11, 11, 10, true, context.animated_track_data0 + context.key_frame_byte_offset0);
							translation1 = unpack_vector3_32(11, 11, 10, true, context.animated_track_data1 + context.key_frame_byte_offset1);

							const uint32_t translation_size = get_packed_vector_size(translation_format);
							context.key_frame_byte_offset0 += translation_size;
							context.key_frame_byte_offset1 += translation_size;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_bit_offset0 = context.key_frame_byte_offset0 * 8;
								context.key_frame_bit_offset1 = context.key_frame_byte_offset1 * 8;
							}
						}
						else if (translation_format == VectorFormat8::Vector3_Variable && settings.is_translation_format_supported(VectorFormat8::Vector3_Variable))
						{
							bit_rate0 = context.format_per_track_data0[context.format_per_track_data_offset];
							bit_rate1 = context.format_per_track_data1[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate0 = get_num_bits_at_bit_rate(bit_rate0);
							uint8_t num_bits_at_bit_rate1 = get_num_bits_at_bit_rate(bit_rate1);

							if (is_pack_0_bit_rate(bit_rate0))
								(void)bit_rate0;
							else if (is_pack_72_bit_rate(bit_rate0))
								translation0 = unpack_vector3_72(true, context.animated_track_data0, context.key_frame_bit_offset0);
							else if (is_pack_96_bit_rate(bit_rate0))
								translation0 = unpack_vector3_96(context.animated_track_data0, context.key_frame_bit_offset0);
							else
								translation0 = unpack_vector3_n(num_bits_at_bit_rate0, num_bits_at_bit_rate0, num_bits_at_bit_rate0, true, context.animated_track_data0, context.key_frame_bit_offset0);

							if (is_pack_0_bit_rate(bit_rate1))
								(void)bit_rate1;
							else if (is_pack_72_bit_rate(bit_rate1))
								translation1 = unpack_vector3_72(true, context.animated_track_data1, context.key_frame_bit_offset1);
							else if (is_pack_96_bit_rate(bit_rate1))
								translation1 = unpack_vector3_96(context.animated_track_data1, context.key_frame_bit_offset1);
							else
								translation1 = unpack_vector3_n(num_bits_at_bit_rate1, num_bits_at_bit_rate1, num_bits_at_bit_rate1, true, context.animated_track_data1, context.key_frame_bit_offset1);

							uint8_t num_bits_read0 = num_bits_at_bit_rate0 * 3;
							uint8_t num_bits_read1 = num_bits_at_bit_rate1 * 3;
							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								num_bits_read0 = align_to(num_bits_read0, MIXED_PACKING_ALIGNMENT_NUM_BITS);
								num_bits_read1 = align_to(num_bits_read1, MIXED_PACKING_ALIGNMENT_NUM_BITS);
							}

							context.key_frame_bit_offset0 += num_bits_read0;
							context.key_frame_bit_offset1 += num_bits_read1;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}

						if (is_enum_flag_set(segment_range_reduction, RangeReductionFlags8::Translations))
						{
#if ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BIT_SIZE == 8
							if (translation_format == VectorFormat8::Vector3_Variable && settings.is_translation_format_supported(VectorFormat8::Vector3_Variable) && is_pack_0_bit_rate(bit_rate0))
								translation0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
							else
							{
								Vector4_32 segment_range_min0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_24(context.segment_range_data0 + context.segment_range_data_offset + (3 * sizeof(uint8_t)), true);

								translation0 = vector_mul_add(translation0, segment_range_extent0, segment_range_min0);
							}

							if (translation_format == VectorFormat8::Vector3_Variable && settings.is_translation_format_supported(VectorFormat8::Vector3_Variable) && is_pack_0_bit_rate(bit_rate1))
								translation1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
							else
							{
								Vector4_32 segment_range_min1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_24(context.segment_range_data1 + context.segment_range_data_offset + (3 * sizeof(uint8_t)), true);

								translation1 = vector_mul_add(translation1, segment_range_extent1, segment_range_min1);
							}
#else
							if (translation_format == VectorFormat8::Vector3_Variable && settings.is_translation_format_supported(VectorFormat8::Vector3_Variable) && is_pack_0_bit_rate(bit_rate0))
								translation0 = unpack_vector3_96(context.segment_range_data0 + context.segment_range_data_offset, );
							else
							{
								Vector4_32 segment_range_min0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent0 = unpack_vector3_48(context.segment_range_data0 + context.segment_range_data_offset + (3 * sizeof(uint16_t)), true);

								translation0 = vector_mul_add(translation0, segment_range_extent0, segment_range_min0);
							}

							if (translation_format == VectorFormat8::Vector3_Variable && settings.is_translation_format_supported(VectorFormat8::Vector3_Variable) && is_pack_0_bit_rate(bit_rate1))
								translation1 = unpack_vector3_96(context.segment_range_data1 + context.segment_range_data_offset);
							else
							{
								Vector4_32 segment_range_min1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset, true);
								Vector4_32 segment_range_extent1 = unpack_vector3_48(context.segment_range_data1 + context.segment_range_data_offset + (3 * sizeof(uint16_t)), true);

								translation1 = vector_mul_add(translation1, segment_range_extent1, segment_range_min1);
							}
#endif

							context.segment_range_data_offset += 3 * ACL_PER_SEGMENT_RANGE_REDUCTION_COMPONENT_BYTE_SIZE * 2;
						}

						if (is_enum_flag_set(clip_range_reduction, RangeReductionFlags8::Translations))
						{
							Vector4_32 clip_range_min = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset);
							Vector4_32 clip_range_extent = vector_unaligned_load(context.clip_range_data + context.clip_range_data_offset + (3 * sizeof(float)));

							translation0 = vector_mul_add(translation0, clip_range_extent, clip_range_min);
							translation1 = vector_mul_add(translation1, clip_range_extent, clip_range_min);

							context.clip_range_data_offset += 3 * sizeof(float) * 2;
						}

						translation = vector_lerp(translation0, translation1, context.interpolation_alpha);

						ACL_ENSURE(vector_is_finite3(translation), "Translation is not valid!");
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
				return translation;
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
