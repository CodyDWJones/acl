#if false

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
			// TODO: this decoder initializes the context every time a pose is sampled.
			// It assumes every sample is stored with the same amount of data.
			// If the new wanted sample time is greater than current, then read forward until the SECOND most recent control point is >= the desired time.
			// If wanted sample time is < current, then rewind to beginning and repeat above line.

			struct SplineParameters
			{
				Vector4_32 values[4];
				float knots[4];
				float sample_times[4];
			};

			// spline data in memory:
			// 
			struct alignas(64) DecompressionContext
			{
				// Read-only data
				uint16_t num_animated_bones;

				const uint32_t* constant_tracks_bitset;
				const uint8_t* constant_track_data;
				const uint32_t* default_tracks_bitset;

				const uint8_t* format_per_track_data;
				const uint8_t* range_data;

				uint32_t bitset_size;
				uint32_t range_rotation_size;
				uint32_t range_translation_size;

				bool has_mixed_packing;
				uint8_t* animated_data;

				// Read-write data.
				uint32_t constant_track_offset;
				uint32_t constant_track_data_offset;
				uint32_t default_track_offset;
				uint32_t format_per_track_data_offset;
				uint32_t range_data_offset;

				uint32_t current_frame_size;
				float current_sample_time;
				uint32_t frame_offset;

				uint32_t spline_parameters_offset;
				SplineParameters spline_parameters[];
			};

			template<class SettingsType>
			inline void initialize_context(const SettingsType& settings, const Header& header, DecompressionContext& context)
			{
#if defined(ACL_USE_ERROR_CHECKS)
				ACL_ENSURE(clip.get_algorithm_type() == AlgorithmType8::SplineKeyReduction, "Invalid algorithm type [%s], expected [%s]", get_algorithm_name(clip.get_algorithm_type()), get_algorithm_name(AlgorithmType8::SplineKeyReduction));
				ACL_ENSURE(clip.is_valid(false), "Clip is invalid");
#endif

			}

			template<class SettingsType>
			inline void seek(const SettingsType& settings, const FullPrecisionHeader& header, float sample_time, DecompressionContext& context)
			{
				// Convert sample time into a pair of sample keyframes + t.

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
			}

#if false
			template<class SettingsType>
			inline void initialize_context(const SettingsType& settings, const FullPrecisionHeader& header, float sample_time, DecompressionContext& context)
			{
				const RotationFormat8 rotation_format = settings.get_rotation_format(header.rotation_format);
				const VectorFormat8 translation_format = settings.get_translation_format(header.translation_format);
				const RangeReductionFlags8 range_reduction = settings.get_range_reduction(header.range_reduction);

#if defined(ACL_USE_ERROR_CHECKS)
				ACL_ENSURE(rotation_format == header.rotation_format, "Statically compiled rotation format (%s) differs from the compressed rotation format (%s)!", get_rotation_format_name(rotation_format), get_rotation_format_name(header.rotation_format));
				ACL_ENSURE(settings.is_rotation_format_supported(rotation_format), "Rotation format (%s) isn't statically supported!", get_rotation_format_name(rotation_format));
				ACL_ENSURE(translation_format == header.translation_format, "Statically compiled translation format (%s) differs from the compressed translation format (%s)!", get_vector_format_name(translation_format), get_vector_format_name(header.translation_format));
				ACL_ENSURE(settings.is_translation_format_supported(translation_format), "Translation format (%s) isn't statically supported!", get_vector_format_name(translation_format));
				ACL_ENSURE(range_reduction == header.range_reduction, "Statically compiled range reduction settings (%u) differ from the compressed settings (%u)!", range_reduction, header.range_reduction);
				ACL_ENSURE(settings.are_range_reduction_flags_supported(range_reduction), "Range reduction settings (%u) aren't statically supported!", range_reduction);
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

				const uint32_t range_rotation_size = get_range_reduction_rotation_size(rotation_format);
				const uint32_t range_translation_size = get_range_reduction_vector_size(translation_format);
				const bool has_clip_range_reduction = is_enum_flag_set(range_reduction, RangeReductionFlags8::PerClip);

				float clip_duration = float(header.num_samples - 1) / float(header.sample_rate);

				uint32_t key_frame0;
				uint32_t key_frame1;
				float interpolation_alpha;
				calculate_interpolation_keys(header.num_samples, clip_duration, sample_time, key_frame0, key_frame1, interpolation_alpha);

				context.default_tracks_bitset = header.get_default_tracks_bitset();

				context.constant_tracks_bitset = header.get_constant_tracks_bitset();
				context.constant_track_data = header.get_constant_track_data();

				context.format_per_track_data = header.get_format_per_track_data();
				context.range_data = header.get_clip_range_data();

				context.animated_track_data = header.get_track_data();

				context.bitset_size = get_bitset_size(header.num_bones * FullPrecisionConstants::NUM_TRACKS_PER_BONE);
				context.range_rotation_size = has_clip_range_reduction && is_enum_flag_set(range_reduction, RangeReductionFlags8::Rotations) ? range_rotation_size : 0;
				context.range_translation_size = has_clip_range_reduction && is_enum_flag_set(range_reduction, RangeReductionFlags8::Translations) ? range_translation_size : 0;

				context.interpolation_alpha = interpolation_alpha;

				// If all tracks are variable, no need for any extra padding except at the very end of the data
				// If our tracks are mixed variable/not variable, we need to add some padding to ensure alignment
				context.has_mixed_packing = is_rotation_format_variable(rotation_format) != is_vector_format_variable(translation_format);

				context.constant_track_offset = 0;
				context.default_track_offset = 0;
				context.constant_track_data_offset = 0;
				context.format_per_track_data_offset = 0;
				context.range_data_offset = 0;
				context.key_frame_byte_offset0 = (key_frame0 * header.animated_pose_bit_size) / 8;
				context.key_frame_byte_offset1 = (key_frame1 * header.animated_pose_bit_size) / 8;
				context.key_frame_bit_offset0 = key_frame0 * header.animated_pose_bit_size;
				context.key_frame_bit_offset1 = key_frame1 * header.animated_pose_bit_size;
			}

			template<class SettingsType>
			inline void skip_rotation(const SettingsType& settings, const FullPrecisionHeader& header, DecompressionContext& context)
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
							uint8_t bit_rate = context.format_per_track_data[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate = get_num_bits_at_bit_rate(bit_rate) * 3;	// 3 components

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
								num_bits_at_bit_rate = align_to(num_bits_at_bit_rate, MIXED_PACKING_ALIGNMENT_NUM_BITS);

							context.key_frame_bit_offset0 += num_bits_at_bit_rate;
							context.key_frame_bit_offset1 += num_bits_at_bit_rate;

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

						context.range_data_offset += context.range_rotation_size;
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
			}

			template<class SettingsType>
			inline void skip_translation(const SettingsType& settings, const FullPrecisionHeader& header, DecompressionContext& context)
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
							uint8_t bit_rate = context.format_per_track_data[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate = get_num_bits_at_bit_rate(bit_rate) * 3;	// 3 components

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
								num_bits_at_bit_rate = align_to(num_bits_at_bit_rate, MIXED_PACKING_ALIGNMENT_NUM_BITS);

							context.key_frame_bit_offset0 += num_bits_at_bit_rate;
							context.key_frame_bit_offset1 += num_bits_at_bit_rate;

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

						context.range_data_offset += context.range_translation_size;
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
			}

			template<class SettingsType>
			inline Quat_32 decompress_rotation(const SettingsType& settings, const FullPrecisionHeader& header, DecompressionContext& context)
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

						if (packed_format == RotationFormat8::QuatDropW_96 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_96))
							rotation = unpack_quat_96(context.constant_track_data + context.constant_track_data_offset);
						else if (packed_format == RotationFormat8::QuatDropW_48 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_48))
							rotation = unpack_quat_48(context.constant_track_data + context.constant_track_data_offset);
						else if (packed_format == RotationFormat8::QuatDropW_32 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_32))
							rotation = unpack_quat_32(context.constant_track_data + context.constant_track_data_offset);

						ACL_ENSURE(quat_is_valid(rotation), "Rotation is not valid!");
						ACL_ENSURE(quat_is_normalized(rotation), "Rotation is not normalized!");

						context.constant_track_data_offset += get_packed_rotation_size(packed_format);
					}
					else
					{
						const RangeReductionFlags8 range_reduction = settings.get_range_reduction(header.range_reduction);

						Quat_32 rotation0;
						Quat_32 rotation1;

						if (rotation_format == RotationFormat8::QuatDropW_96 && settings.is_rotation_format_supported(RotationFormat8::QuatDropW_96))
						{
							Vector4_32 rotation0_xyz = unpack_vector3_96(context.animated_track_data + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_96(context.animated_track_data + context.key_frame_byte_offset1);

							if (are_enum_flags_set(range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Rotations))
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.range_data + context.range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.range_data + context.range_data_offset + (context.range_rotation_size / 2));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.range_data_offset += context.range_rotation_size;
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
							Vector4_32 rotation0_xyz = unpack_vector3_48(context.animated_track_data + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_48(context.animated_track_data + context.key_frame_byte_offset1);

							if (are_enum_flags_set(range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Rotations))
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.range_data + context.range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.range_data + context.range_data_offset + (context.range_rotation_size / 2));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.range_data_offset += context.range_rotation_size;
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
							Vector4_32 rotation0_xyz = unpack_vector3_32<11, 11, 10>(context.animated_track_data + context.key_frame_byte_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_32<11, 11, 10>(context.animated_track_data + context.key_frame_byte_offset1);

							if (are_enum_flags_set(range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Rotations))
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.range_data + context.range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.range_data + context.range_data_offset + (context.range_rotation_size / 2));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.range_data_offset += context.range_rotation_size;
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
							uint8_t bit_rate = context.format_per_track_data[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate = get_num_bits_at_bit_rate(bit_rate);

							Vector4_32 rotation0_xyz = unpack_vector3_n(num_bits_at_bit_rate, num_bits_at_bit_rate, num_bits_at_bit_rate, context.animated_track_data, context.key_frame_bit_offset0);
							Vector4_32 rotation1_xyz = unpack_vector3_n(num_bits_at_bit_rate, num_bits_at_bit_rate, num_bits_at_bit_rate, context.animated_track_data, context.key_frame_bit_offset1);

							if (are_enum_flags_set(range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Rotations))
							{
								Vector4_32 clip_range_min = vector_unaligned_load(context.range_data + context.range_data_offset);
								Vector4_32 clip_range_extent = vector_unaligned_load(context.range_data + context.range_data_offset + (context.range_rotation_size / 2));

								rotation0_xyz = vector_mul_add(rotation0_xyz, clip_range_extent, clip_range_min);
								rotation1_xyz = vector_mul_add(rotation1_xyz, clip_range_extent, clip_range_min);

								context.range_data_offset += context.range_rotation_size;
							}

							rotation0 = quat_from_positive_w(rotation0_xyz);
							rotation1 = quat_from_positive_w(rotation1_xyz);

							uint8_t num_bits_read = num_bits_at_bit_rate * 3;
							if (settings.supports_mixed_packing() && context.has_mixed_packing)
								num_bits_read = align_to(num_bits_read, MIXED_PACKING_ALIGNMENT_NUM_BITS);

							context.key_frame_bit_offset0 += num_bits_read;
							context.key_frame_bit_offset1 += num_bits_read;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}

						rotation = quat_lerp(rotation0, rotation1, context.interpolation_alpha);

						ACL_ENSURE(quat_is_valid(rotation), "Rotation is not valid!");
						ACL_ENSURE(quat_is_normalized(rotation), "Rotation is not normalized!");
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
				return rotation;
			}

			template<class SettingsType>
			inline Vector4_32 decompress_translation(const SettingsType& settings, const FullPrecisionHeader& header, DecompressionContext& context)
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

						ACL_ENSURE(vector_is_valid3(translation), "Translation is not valid!");

						context.constant_track_data_offset += get_packed_vector_size(VectorFormat8::Vector3_96);
					}
					else
					{
						const VectorFormat8 translation_format = settings.get_translation_format(header.translation_format);
						const RangeReductionFlags8 range_reduction = settings.get_range_reduction(header.range_reduction);

						Vector4_32 translation0;
						Vector4_32 translation1;

						if (translation_format == VectorFormat8::Vector3_96 && settings.is_translation_format_supported(VectorFormat8::Vector3_96))
						{
							translation0 = unpack_vector3_96(context.animated_track_data + context.key_frame_byte_offset0);
							translation1 = unpack_vector3_96(context.animated_track_data + context.key_frame_byte_offset1);

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
							translation0 = unpack_vector3_48(context.animated_track_data + context.key_frame_byte_offset0);
							translation1 = unpack_vector3_48(context.animated_track_data + context.key_frame_byte_offset1);

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
							translation0 = unpack_vector3_32<11, 11, 10>(context.animated_track_data + context.key_frame_byte_offset0);
							translation1 = unpack_vector3_32<11, 11, 10>(context.animated_track_data + context.key_frame_byte_offset1);

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
							uint8_t bit_rate = context.format_per_track_data[context.format_per_track_data_offset++];
							uint8_t num_bits_at_bit_rate = get_num_bits_at_bit_rate(bit_rate);

							translation0 = unpack_vector3_n(num_bits_at_bit_rate, num_bits_at_bit_rate, num_bits_at_bit_rate, context.animated_track_data, context.key_frame_bit_offset0);
							translation1 = unpack_vector3_n(num_bits_at_bit_rate, num_bits_at_bit_rate, num_bits_at_bit_rate, context.animated_track_data, context.key_frame_bit_offset1);

							uint8_t num_bits_read = num_bits_at_bit_rate * 3;
							if (settings.supports_mixed_packing() && context.has_mixed_packing)
								num_bits_read = align_to(num_bits_read, MIXED_PACKING_ALIGNMENT_NUM_BITS);

							context.key_frame_bit_offset0 += num_bits_read;
							context.key_frame_bit_offset1 += num_bits_read;

							if (settings.supports_mixed_packing() && context.has_mixed_packing)
							{
								context.key_frame_byte_offset0 = context.key_frame_bit_offset0 / 8;
								context.key_frame_byte_offset1 = context.key_frame_bit_offset1 / 8;
							}
						}

						if (are_enum_flags_set(range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Translations))
						{
							Vector4_32 clip_range_min = vector_unaligned_load(context.range_data + context.range_data_offset);
							Vector4_32 clip_range_extent = vector_unaligned_load(context.range_data + context.range_data_offset + (context.range_translation_size / 2));

							translation0 = vector_mul_add(translation0, clip_range_extent, clip_range_min);
							translation1 = vector_mul_add(translation1, clip_range_extent, clip_range_min);

							context.range_data_offset += context.range_translation_size;
						}

						translation = vector_lerp(translation0, translation1, context.interpolation_alpha);

						ACL_ENSURE(vector_is_valid3(translation), "Translation is not valid!");
					}
				}

				context.default_track_offset++;
				context.constant_track_offset++;
				return translation;
			}
		}
#endif

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

			constexpr bool are_range_reduction_flags_supported(RangeReductionFlags8 flags) const { return true; }
			constexpr RangeReductionFlags8 get_range_reduction(RangeReductionFlags8 flags) const { return flags; }

			// Whether tracks must all be variable or all fixed width, or if they can be mixed and require padding
			constexpr bool supports_mixed_packing() const { return true; }
		};

		template<class SettingsType>
		inline Quat_32 sample_rotation(const SettingsType& settings, uint16_t bone_index, float sample_time, DecompressionContext& context)
		{
			using namespace impl;

			Quat_32 rotation;
			
			bool is_rotation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.constant_track_offset);
			if (is_rotation_default)
			{
				rotation = quat_identity_32();
			}
			else
			{
				bool is_rotation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.default_track_offset);
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

					context.constant_track_data_offset += get_packed_rotation_size(packed_format);

					ACL_ENSURE(quat_is_valid(rotation), "Rotation is not valid!");
					ACL_ENSURE(quat_is_normalized(rotation), "Rotation is not normalized!");
				}
				else
				{
					const SplineParameters& spline = context.spline_parameters[context.spline_parameters_offset];
					++context.spline_parameters_offset;

					rotation = quat_normalize(vector_to_quat(interpolate(spline.values, spline.knots, spline.sample_times, sample_time)));
				}
			}
			
			++context.constant_track_offset;
			++context.default_track_offset;
			return rotation;
		}

		inline Vector4_32 sample_translation(uint16_t bone_index, float sample_time, DecompressionContext& context)
		{
			using namespace impl;

			Vector4_32 translation;
			
			bool is_translation_default = bitset_test(context.default_tracks_bitset, context.bitset_size, context.constant_track_offset);
			if (is_translation_default)
			{
				translation = vector_zero_32();
			}
			else
			{
				bool is_translation_constant = bitset_test(context.constant_tracks_bitset, context.bitset_size, context.default_track_offset);
				if (is_translation_constant)
				{
					translation = unpack_vector3_96(context.constant_track_data + context.constant_track_data_offset);
					context.constant_track_data_offset += get_packed_vector_size(VectorFormat8::Vector3_96);

					ACL_ENSURE(vector_is_valid3(translation), "Translation is not valid!");
				}
				else
				{
					const SplineParameters& spline = context.spline_parameters[context.spline_parameters_offset];
					++context.spline_parameters_offset;

					translation = interpolate(spline.values, spline.knots, spline.sample_times, sample_time);
				}
			}
			
			++context.constant_track_offset;
			++context.default_track_offset;
			return translation;
		}
		
		template<class SettingsType, class OutputWriterType>
		inline void decompress_pose(const SettingsType& settings, const CompressedClip& clip, void* opaque_context, float sample_time, OutputWriterType& writer)
		{
			static_assert(std::is_base_of<DecompressionSettings, SettingsType>::value, "SettingsType must derive from DecompressionSettings!");
			static_assert(std::is_base_of<OutputWriter, OutputWriterType>::value, "OutputWriterType must derive from OutputWriter!");

			using namespace impl;

			const Header& header = get_header(clip);
			DecompressionContext& context = *safe_ptr_cast<DecompressionContext>(opaque_context);

			seek(settings, header, sample_time, context);
			
			for (uint32_t bone_index = 0; bone_index < header.num_bones; ++bone_index)
			{
				writer.write_bone_rotation(bone_index, sample_rotation(settings, bone_index, sample_time, context));
				writer.write_bone_translation(bone_index, sample_translation(bone_index, sample_time, context));
			}
		}

		template<class SettingsType>
		inline void decompress_bone(const SettingsType& settings, const CompressedClip& clip, void* opaque_context, float sample_time, uint16_t sample_bone_index, Quat_32* out_rotation, Vector4_32* out_translation)
		{
			static_assert(std::is_base_of<DecompressionSettings, SettingsType>::value, "SettingsType must derive from DecompressionSettings!");

			using namespace impl;

			const Header& header = get_header(clip);
			DecompressionContext& context = *safe_ptr_cast<DecompressionContext>(opaque_context);

			seek(settings, header, sample_time, context);

			// TODO: Optimize this by counting the number of bits set, we can use the pop-count instruction on
			// architectures that support it (e.g. xb1/ps4). This would entirely avoid looping here.
			for (uint32_t bone_index = 0; bone_index < header.num_bones; ++bone_index)
			{
				if (bone_index == sample_bone_index)
					break;

				skip_rotation(settings, context);
				skip_translation(settings, context);
			}

			template <typename InterpolatedType>
			inline InterpolatedType interpolate(const Interpolator* interpolator, float sample_time)

			Quat_32 rotation = decompress_rotation(settings, header, context);
			if (out_rotation != nullptr)
				*out_rotation = rotation;

			Vector4_32 translation = decompress_translation(settings, header, context);
			if (out_translation != nullptr)
				*out_translation = translation;
		}

	}
}

#endif