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

// TODO: minimize the includes.
#include "acl/core/algorithm_types.h"
#include "acl/core/bitset.h"
#include "acl/core/enum_utils.h"
#include "acl/core/error.h"
#include "acl/core/hash.h"
#include "acl/core/memory.h"
#include "acl/core/scope_profiler.h"
#include "acl/core/track_types.h"
#include "acl/algorithm/spline_key_reduction/common.h"
#include "acl/algorithm/spline_key_reduction/spline.h"
#include "acl/compression/compressed_clip_impl.h"
#include "acl/compression/skeleton.h"
#include "acl/compression/animation_clip.h"
#include "acl/compression/output_stats.h"
#include "acl/compression/stream/segment_streams.h"
#include "acl/compression/stream/track_stream.h"
#include "acl/compression/stream/convert_rotation_streams.h"
#include "acl/compression/stream/compact_constant_streams.h"
#include "acl/compression/stream/extend_streams.h"
#include "acl/compression/stream/normalize_streams.h"
#include "acl/compression/stream/quantize_streams.h"
#include "acl/compression/stream/write_range_data.h"
#include "acl/compression/stream/write_stream_bitsets.h"
#include "acl/compression/stream/write_stream_data.h"
#include "acl/compression/stream/write_stream_stats.h"

#include <stdint.h>
#include <cstdio>
#include <cstdint>
#include <functional>

//////////////////////////////////////////////////////////////////////////
// Spline Key Reduction Encoder
//
// TODO
//
// Data format:
// TODO: description of header common to all existing formats
//
// For each segment:
//		TODO: common descriptions
//
//		FRAMES:
//			Each frame contains either rotation, translation, or scale data for a subset of bones at a given sample point.
//			The header of a frame includes offsets which can be used to linearly seek to a specific point in time.
//			Although each segment contains control points for effective sample points [-1, 0, N, N+1], in general seeking
//			requires discovery of previous or future frames because not all the control points needed to interpolate will
//			be in one frame.
//
//   List of frames:
//		32 bit frame header
//			+ top two bits - indicates rotation (00) translation (01) or scale (10)
//			+ 15 bits - multiply by four to get the byte offset to start of previous frame.  0 means this is the first frame.
//			+ 15 bits - multiply by four to get the offset to start of next frame.  0 means this is the last frame.
//		uint32_t - sample index this frame's data is for
//		bitset - one bit per bone, each bit indicates if a spline control point follows for that bone
//		for each enabled bone:
//			the packed quaternion or vector
//		for each enabled bone:
//			float storing the knot distance from the previous control point for the bone.
//		padding to meet 4 byte alignment for next frame.
// 
//////////////////////////////////////////////////////////////////////////

// TODO: test this encoder with clips that only have 0, 1, 2, or 3 samples per bone.

namespace acl
{
	namespace spline_key_reduction
	{
		struct CompressionSettings
		{
			RotationFormat8 rotation_format;
			VectorFormat8 translation_format;
			VectorFormat8 scale_format;

			RangeReductionFlags8 range_reduction;

			SegmentingSettings segmenting;

			CompressionSettings()
				: rotation_format(RotationFormat8::Quat_128)
				, translation_format(VectorFormat8::Vector3_96)
				, scale_format(VectorFormat8::Vector3_96)
				, range_reduction(RangeReductionFlags8::None)
				, segmenting()
			{}

			uint32_t hash() const
			{
				return hash_combine(hash_combine(hash_combine(hash_combine(hash32(rotation_format), hash32(translation_format)), hash32(scale_format)), hash32(range_reduction)), segmenting.hash());
			}
		};

		namespace impl
		{
			class Selections
			{
			public:
				Selections(Allocator& allocator, uint32_t num_samples)
					: m_allocator(allocator)
				{
					m_selected_size = get_bitset_size(num_samples);
					m_selected = allocate_type_array<uint32_t>(m_allocator, m_selected_size);
					m_selected_backup = allocate_type_array<uint32_t>(m_allocator, m_selected_size);
				}

				~Selections()
				{
					deallocate_type_array(m_allocator, m_selected, m_selected_size);
					deallocate_type_array(m_allocator, m_selected_backup, m_selected_size);
				}

				bool is_selected(uint32_t sample_index) const { return bitset_test(m_selected, m_selected_size, sample_index); }
				void deselect_all() { bitset_reset(m_selected, m_selected_size, false); }
				void deselect(uint32_t sample_index) { bitset_set(m_selected, m_selected_size, sample_index, false); }
				void select(uint32_t sample_index) { bitset_set(m_selected, m_selected_size, sample_index, true); }

				void save_state() { std::memcpy(m_selected_backup, m_selected, m_selected_size * sizeof(uint32_t)); }
				void restore_state() { std::memcpy(m_selected, m_selected_backup, m_selected_size * sizeof(uint32_t)); }

			private:
				Allocator& m_allocator;
				uint32_t* m_selected;
				uint32_t* m_selected_backup;
				uint32_t m_selected_size;
			};

			inline void reset_control_point_choices(Selections& selections, uint32_t num_samples)
			{
				selections.deselect_all();

				for (uint32_t sample_index = 0; sample_index <= NUM_LEFT_AUXILIARY_POINTS; ++sample_index)
					selections.select(sample_index);

				for (uint32_t sample_index = num_samples - 1; sample_index >= num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS; --sample_index)
					selections.select(sample_index);
			}

			inline void reset_control_point_choices(const SegmentContext& segment, Selections** rotation_selections, Selections** translation_selections, Selections** scale_selections)
			{
				Selections* selections;

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					selections = rotation_selections[bone_index];
					if (selections != nullptr)
						reset_control_point_choices(*selections, segment.num_samples);

					selections = translation_selections[bone_index];
					if (selections != nullptr)
						reset_control_point_choices(*selections, segment.num_samples);

					selections = scale_selections[bone_index];
					if (selections != nullptr)
						reset_control_point_choices(*selections, segment.num_samples);
				}
			}

			inline uint32_t get_sample_num_packed_bits(const BoneStreams& bone_streams, AnimationTrackType8 track_type)
			{
				switch (track_type)
				{
				case AnimationTrackType8::Rotation:
				{
					const RotationTrackStream& rotations = bone_streams.rotations;

					if (rotations.is_bit_rate_variable())
						return get_num_bits_at_bit_rate(rotations.get_bit_rate()) * 3;		// 3 components
					else
						return get_packed_rotation_size(rotations.get_rotation_format()) * 8;
				}
				case AnimationTrackType8::Translation:
				{
					const TranslationTrackStream& translations = bone_streams.translations;

					if (translations.is_bit_rate_variable())
						return get_num_bits_at_bit_rate(translations.get_bit_rate()) * 3;	// 3 components
					else
						return get_packed_vector_size(translations.get_vector_format()) * 8;
				}
				case AnimationTrackType8::Scale:
				{
					const ScaleTrackStream& scales = bone_streams.scales;

					if (scales.is_bit_rate_variable())
						return get_num_bits_at_bit_rate(scales.get_bit_rate()) * 3;			// 3 components
					else
						return get_packed_vector_size(scales.get_vector_format()) * 8;
				}
				default:
					// TODO:
					//ACL_ENSURE(1 == 0, "Unknown track type");
					break;
				}
			}

			inline Vector4_32 sample(const BoneStreams& bone_streams, AnimationTrackType8 track_type, int32_t sample_index)
			{
				switch (track_type)
				{
				case AnimationTrackType8::Rotation:
				{
					int32_t num_samples = static_cast<int32_t>(bone_streams.rotations.get_num_samples());

					if (sample_index < 0)
					{
						int32_t right_index = std::min(-sample_index, num_samples - 1);

						// Reflect across the first sample to create an auxiliary control point beyond the clip that will ensure a reasonable interpolation near time 0.
						return quat_to_vector(quat_normalize(vector_to_quat(
							vector_lerp(get_rotation_sample(bone_streams, 0), get_rotation_sample(bone_streams, right_index), -1.0))));
					}
					else if (sample_index >= num_samples)
					{
						int32_t left_index = std::max(0, 2 * num_samples - 2 - sample_index);

						return quat_to_vector(quat_normalize(vector_to_quat(
							vector_lerp(get_rotation_sample(bone_streams, left_index), get_rotation_sample(bone_streams, num_samples - 1), 2.0))));
					}
					else
					{
						return get_rotation_sample(bone_streams, sample_index);
					}

					break;
				}
				case AnimationTrackType8::Translation:
				{
					int32_t num_samples = static_cast<int32_t>(bone_streams.translations.get_num_samples());

					if (sample_index < 0)
					{
						int32_t right_index = std::min(-sample_index, num_samples - 1);

						return vector_lerp(get_translation_sample(bone_streams, 0), get_translation_sample(bone_streams, right_index), -1.0);
					}
					else if (sample_index >= num_samples)
					{
						int32_t left_index = std::max(0, 2 * num_samples - 2 - sample_index);

						return vector_lerp(get_translation_sample(bone_streams, left_index), get_translation_sample(bone_streams, num_samples - 1), 2.0);
					}
					else
					{
						return get_translation_sample(bone_streams, sample_index);
					}

					break;
				}
				case AnimationTrackType8::Scale:
				{
					int32_t num_samples = static_cast<int32_t>(bone_streams.scales.get_num_samples());

					if (sample_index < 0)
					{
						int32_t right_index = std::min(-sample_index, num_samples - 1);

						return vector_lerp(get_scale_sample(bone_streams, 0), get_scale_sample(bone_streams, right_index), -1.0);
					}
					else if (sample_index >= num_samples)
					{
						int32_t left_index = std::max(0, 2 * num_samples - 2 - sample_index);

						return vector_lerp(get_scale_sample(bone_streams, left_index), get_scale_sample(bone_streams, num_samples - 1), 2.0);
					}
					else
					{
						return get_scale_sample(bone_streams, sample_index);
					}

					break;
				}
				default:
					// TODO: ACL_ENSURE

					break;
				}
			}

			Vector4_32 interpolate(const BoneStreams& bone_streams, AnimationTrackType8 track_type, const Selections& selections, uint32_t at_sample_index)
			{
				if (selections.is_selected(at_sample_index))
				{
					return sample(bone_streams, track_type, at_sample_index);
				}

				Vector4_32 values[NUM_CONTROL_POINTS];
				uint32_t sample_indices[NUM_CONTROL_POINTS];

				Vector4_32 value;
				uint32_t sample_index = at_sample_index;

				for (int8_t control_point_index = LEFT_INTERPOLATION_KNOT_INDEX; control_point_index >= 0; --control_point_index)
				{
					while (true)
					{
						--sample_index;
						if (selections.is_selected(sample_index))
						{
							value = sample(bone_streams, track_type, sample_index);
							break;
						}
					}

					values[control_point_index] = value;
					sample_indices[control_point_index] = sample_index;
				}

				sample_index = at_sample_index;

				for (uint8_t control_point_index = RIGHT_INTERPOLATION_KNOT_INDEX; control_point_index < NUM_CONTROL_POINTS; ++control_point_index)
				{
					while (true)
					{
						++sample_index;
						if (selections.is_selected(sample_index))
						{
							value = sample(bone_streams, track_type, sample_index);
							break;
						}
					}

					values[control_point_index] = value;
					sample_indices[control_point_index] = sample_index;
				}

				float knots[NUM_CONTROL_POINTS];
				get_knots(values, sample_indices, knots);

				return interpolate_spline(values, knots, sample_indices, static_cast<float>(at_sample_index));
			}

			inline void interpolate_pose(const SegmentContext& segment, Selections*const* rotation_selections, Selections*const* translation_selections, Selections*const* scale_selections,
				uint32_t sample_index, Transform_32* out_local_pose)
			{
				ACL_ASSERT(NUM_LEFT_AUXILIARY_POINTS <= sample_index && sample_index <= segment.num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS, "sample_index is out of range");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& bone_streams = segment.bone_streams[bone_index];
					Selections* selections;

					Quat_32 rotation;
					selections = rotation_selections[bone_index];
					if (selections != nullptr)
						rotation = quat_normalize(interpolate(bone_streams, AnimationTrackType8::Rotation, *selections, sample_index));
					else if (bone_streams.is_rotation_constant)
						rotation = get_rotation_sample(bone_streams, 0);
					else
						rotation = quat_identity_32();

					Vector4_32 translation;
					selections = translation_selections[bone_index];
					if (selections != nullptr)
						translation = interpolate(bone_streams, AnimationTrackType8::Translation, *selections, sample_index);
					else if (bone_streams.is_translation_constant)
						translation = get_translation_sample(bone_streams, 0);
					else
						translation = vector_zero_32();

					Vector4_32 scale;
					selections = scale_selections[bone_index];
					if (selections != nullptr)
						scale = interpolate(bone_streams, AnimationTrackType8::Scale, *selections, sample_index);
					else if (bone_streams.is_scale_constant)
						scale = get_scale_sample(bone_streams, 0);
					else
						scale = vector_set(1.0f);

					out_local_pose[bone_index] = transform_set(rotation, translation, scale);
				}
			}

			inline void get_auxiliary_control_points(uint32_t segment_clip_sample_offset, uint32_t num_samples, const BoneStreams& raw_bone_streams, AnimationTrackType8 track_type,
				Vector4_32* out_prefixes, uint32_t& out_num_prefixes, Vector4_32* out_suffixes, uint32_t& out_num_suffixes)
			{
				for (out_num_prefixes = 0; out_num_prefixes < NUM_LEFT_AUXILIARY_POINTS; ++out_num_prefixes)
				{
					out_prefixes[out_num_prefixes] = sample(raw_bone_streams, track_type, static_cast<int32_t>(segment_clip_sample_offset) - NUM_LEFT_AUXILIARY_POINTS + out_num_prefixes);
				}

				out_num_suffixes = 0;

				while (true)
				{
					if (out_num_suffixes >= NUM_RIGHT_AUXILIARY_POINTS && out_num_prefixes + num_samples + out_num_suffixes >= NUM_CONTROL_POINTS)
						break;

					out_suffixes[out_num_suffixes] = sample(raw_bone_streams, track_type, segment_clip_sample_offset + num_samples + out_num_suffixes);
					++out_num_suffixes;
				}
			}

			// Each segment must include control points outside the duration of the sample to calculate the interpolation polynomial near
			// its start and end.  The auxiliary points are taken from the original (ie. unsegmented) clip to better approximate the overall
			// clip and to ensure the tangents are correct when there is a discontinuity at a segment boundary.
			inline void extend_segment_with_auxiliary_control_points(Allocator& allocator, SegmentContext& segment, BoneRanges* bone_ranges, const ClipContext& raw_clip_context)
			{
				ACL_ASSERT(raw_clip_context.num_segments == 1, "Raw clip cannot be segmented");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& raw_bone_streams = raw_clip_context.segments[0].bone_streams[bone_index];
					BoneStreams& bone_streams = segment.bone_streams[bone_index];
					BoneRanges* bone_range = bone_ranges == nullptr ? nullptr : bone_ranges + bone_index;

					Vector4_32 prefixes[NUM_LEFT_AUXILIARY_POINTS];
					uint32_t num_prefixes;

					Vector4_32 suffixes[NUM_CONTROL_POINTS - 1];
					uint32_t num_suffixes;

					if (bone_streams.is_rotation_animated())
					{
						RotationTrackStream& stream = bone_streams.rotations;

						get_auxiliary_control_points(segment.clip_sample_offset, stream.get_num_samples(), raw_bone_streams, AnimationTrackType8::Rotation, prefixes, num_prefixes, suffixes, num_suffixes);
						extend_rotation_stream(allocator, stream, bone_range, prefixes, num_prefixes, suffixes, num_suffixes);

						// TODO: won't this cause a problem if the clip is longer than 2^16 samples and segmenting is disabled??
						ACL_ENSURE(stream.get_num_samples() <= UINT16_MAX, "The stream was extended with auxiliary control points but now exceeds %d samples", UINT16_MAX);
						segment.num_samples = static_cast<uint16_t>(stream.get_num_samples());
					}

					if (bone_streams.is_translation_animated())
					{
						TranslationTrackStream& stream = bone_streams.translations;

						get_auxiliary_control_points(segment.clip_sample_offset, stream.get_num_samples(), raw_bone_streams, AnimationTrackType8::Translation, prefixes, num_prefixes, suffixes, num_suffixes);
						extend_translation_stream(allocator, stream, bone_range, prefixes, num_prefixes, suffixes, num_suffixes);

						ACL_ENSURE(stream.get_num_samples() <= UINT16_MAX, "The stream was extended with auxiliary control points but now exceeds %d samples", UINT16_MAX);
						segment.num_samples = static_cast<uint16_t>(stream.get_num_samples());
					}

					if (bone_streams.is_scale_animated())
					{
						ScaleTrackStream& stream = bone_streams.scales;

						get_auxiliary_control_points(segment.clip_sample_offset, stream.get_num_samples(), raw_bone_streams, AnimationTrackType8::Scale, prefixes, num_prefixes, suffixes, num_suffixes);
						extend_scale_stream(allocator, stream, bone_range, prefixes, num_prefixes, suffixes, num_suffixes);

						ACL_ENSURE(stream.get_num_samples() <= UINT16_MAX, "The stream was extended with auxiliary control points but now exceeds %d samples", UINT16_MAX);
						segment.num_samples = static_cast<uint16_t>(stream.get_num_samples());
					}
				}
			}

			inline void try_control_points_at(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				uint32_t sample_index, Selections** rotation_selections, Selections** translation_selections, Selections** scale_selections,
				float* error_per_bone, BoneTrackError* error_per_stream, Transform_32* raw_local_pose, Transform_32* lossy_local_pose,
				Selections*& out_modified_rotation_selections, Selections*& out_modified_translation_selections, Selections*& out_modified_scale_selections)
			{
				sample_streams(segment.bone_streams, segment.num_bones, sample_index, raw_local_pose);
				interpolate_pose(segment, rotation_selections, translation_selections, scale_selections, sample_index, lossy_local_pose);

				double error = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose, error_per_bone);

				if (error <= segment.clip->error_threshold)
				{
					out_modified_rotation_selections = nullptr;
					out_modified_translation_selections = nullptr;
					out_modified_scale_selections = nullptr;
					return;
				}

				double worst_error = 0.0;
				uint16_t bad_bone_index = INVALID_BONE_INDEX;

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (error_per_bone[bone_index] > worst_error)
					{
						worst_error = error_per_bone[bone_index];
						bad_bone_index = bone_index;
					}
				}

				ACL_ASSERT(bad_bone_index != INVALID_BONE_INDEX, "Failed to find the bone with the worst error");

				// Find which bone in the chain contributes the most error that is being interpolated.
				calculate_skeleton_error_contribution(skeleton, raw_local_pose, lossy_local_pose, bad_bone_index, error_per_stream);

				bad_bone_index = INVALID_BONE_INDEX;
				double worst_error_contribution = 0.0;

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					bool rotation_is_interpolated = rotation_selections[bone_index] != nullptr && !rotation_selections[bone_index]->is_selected(sample_index);
					bool translation_is_interpolated = translation_selections[bone_index] != nullptr && !translation_selections[bone_index]->is_selected(sample_index);
					bool scale_is_interpolated = scale_selections[bone_index] != nullptr && !scale_selections[bone_index]->is_selected(sample_index);

					if (!rotation_is_interpolated && !translation_is_interpolated && !scale_is_interpolated)
						continue;

					const BoneTrackError& stream_error = error_per_stream[bone_index];

					if (stream_error.rotation + stream_error.translation > worst_error_contribution)
					{
						bad_bone_index = bone_index;
						worst_error_contribution = stream_error.rotation + stream_error.translation;
					}
				}

				ACL_ASSERT(bad_bone_index != INVALID_BONE_INDEX, "Failed to find the bone with the worst error");

				Selections* bad_bone_rotations = rotation_selections[bad_bone_index];
				bool rotation_was_selected = bad_bone_rotations != nullptr && bad_bone_rotations->is_selected(sample_index);

				Selections* bad_bone_translations = translation_selections[bad_bone_index];
				bool translation_was_selected = bad_bone_translations != nullptr && bad_bone_translations->is_selected(sample_index);

				Selections* bad_bone_scales = scale_selections[bad_bone_index];
				bool scale_was_selected = bad_bone_scales != nullptr && bad_bone_scales->is_selected(sample_index);

				// **************************************************************************************************************************************
				// TODO: need more combinations here :(
				// **************************************************************************************************************************************

				if (bad_bone_rotations != nullptr && !bad_bone_rotations->is_selected(sample_index))
				{
					if (bad_bone_translations == nullptr || bad_bone_translations->is_selected(sample_index))
					{
						bad_bone_rotations->select(sample_index);
					}
					else
					{
						bad_bone_rotations->select(sample_index);

						interpolate_pose(segment, rotation_selections, translation_selections, scale_selections, sample_index, lossy_local_pose);
						double error_without_interpolated_rotation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

						if (error_without_interpolated_rotation > segment.clip->error_threshold)
						{
							bad_bone_rotations->deselect(sample_index);
							bad_bone_translations->select(sample_index);

							interpolate_pose(segment, rotation_selections, translation_selections, scale_selections, sample_index, lossy_local_pose);
							double error_without_interpolated_translation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

							if (error_without_interpolated_translation > error_without_interpolated_rotation)
							{
								bad_bone_rotations->select(sample_index);
								bad_bone_translations->deselect(sample_index);
							}
						}
					}
				}
				else
				{
					bad_bone_translations->select(sample_index);
				}

				bool changed_rotation = bad_bone_rotations != nullptr && rotation_was_selected != bad_bone_rotations->is_selected(sample_index);
				out_modified_rotation_selections = changed_rotation ? bad_bone_rotations : nullptr;

				bool changed_translation = bad_bone_translations != nullptr && translation_was_selected != bad_bone_translations->is_selected(sample_index);
				out_modified_translation_selections = changed_translation ? bad_bone_translations : nullptr;

				bool changed_scale = bad_bone_scales != nullptr && scale_was_selected != bad_bone_scales->is_selected(sample_index);
				out_modified_scale_selections = changed_scale ? bad_bone_scales : nullptr;

				ACL_ASSERT(changed_rotation || changed_translation, "No changes were made to the bone with the worst error contribution");
			}

			inline void rewind_to_first_affected_sample(const Selections* modified_rotation_selections, const Selections* modified_translation_selections, const Selections* modified_scale_selections, uint32_t& out_sample_index)
			{
				if (modified_rotation_selections == nullptr && modified_translation_selections == nullptr && modified_scale_selections == nullptr)
					return;

				uint32_t num_rotation_points = 0,
					min_rotation_points = modified_rotation_selections == nullptr ? 0 : NUM_CONTROL_POINTS - 1,
					num_translation_points = 0,
					min_translation_points = modified_translation_selections == nullptr ? 0 : NUM_CONTROL_POINTS - 1,
					num_scale_points = 0,
					min_scale_points = modified_scale_selections == nullptr ? 0 : NUM_CONTROL_POINTS - 1;

				while (out_sample_index > NUM_LEFT_AUXILIARY_POINTS + 1 &&
					(num_rotation_points < min_rotation_points || min_rotation_points == 0) &&
					(num_translation_points <= min_translation_points || min_translation_points == 0) &&
					(num_scale_points <= min_scale_points || min_scale_points == 0))
				{
					if (modified_rotation_selections != nullptr && modified_rotation_selections->is_selected(out_sample_index))
						++num_rotation_points;

					if (modified_translation_selections != nullptr && modified_translation_selections->is_selected(out_sample_index))
						++num_translation_points;

					if (modified_scale_selections != nullptr && modified_scale_selections->is_selected(out_sample_index))
						++num_scale_points;

					--out_sample_index;
				}
			}

			inline uint32_t get_frame_header_size(uint32_t num_bones)
			{
				return sizeof(FrameHeader) + get_bitset_size(num_bones) * sizeof(uint32_t);
			}

			inline uint32_t get_frame_data_size(const SegmentContext& segment, Selections*const* selections, AnimationTrackType8 track_type, uint32_t sample_index, bool count_knots)
			{
				uint32_t num_bits = 0;

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (selections[bone_index] != nullptr && selections[bone_index]->is_selected(sample_index))
					{
						const BoneStreams& bone_streams = segment.bone_streams[bone_index];
						num_bits += get_sample_num_packed_bits(bone_streams, track_type);

						if (count_knots)
							num_bits += sizeof(float) * 8;
					}
				}

				return (num_bits + (num_bits % 32)) / 8;
			}

			inline uint32_t get_frame_size(const SegmentContext& segment, Selections*const* selections, AnimationTrackType8 track_type, uint32_t sample_index)
			{
				return get_frame_header_size(segment.num_bones) + get_frame_data_size(segment, selections, track_type, sample_index, true);
			}

			inline uint32_t get_animated_data_size(const SegmentContext& segment, Selections*const* rotation_selections, Selections*const* translation_selections, Selections*const* scale_selections)
			{
				uint32_t animated_data_size = 0;

				for (uint32_t sample_index = 0; sample_index < segment.num_samples; ++sample_index)
				{
					uint32_t data_size = get_frame_data_size(segment, rotation_selections, AnimationTrackType8::Rotation, sample_index, true);
					if (data_size > 0)
						animated_data_size += get_frame_header_size(segment.num_bones) + data_size;

					data_size = get_frame_data_size(segment, translation_selections, AnimationTrackType8::Translation, sample_index, true);
					if (data_size > 0)
						animated_data_size += get_frame_header_size(segment.num_bones) + data_size;

					data_size = get_frame_data_size(segment, scale_selections, AnimationTrackType8::Scale, sample_index, true);
					if (data_size > 0)
						animated_data_size += get_frame_header_size(segment.num_bones) + data_size;
				}

				return animated_data_size;
			}

			inline void calculate_animated_data_size(ClipContext& clip_context, Selections*const* rotation_selections, Selections*const* translation_selections, Selections*const* scale_selections)
			{
				for (SegmentContext& segment : clip_context.segment_iterator())
				{
					segment.animated_data_size = get_animated_data_size(segment, rotation_selections, translation_selections, scale_selections);

					// This field is irrelevant since each frame contains control points for a variable number of bones.
					segment.animated_pose_bit_size = 0;
				}
			}

			inline FrameHeader* write_frame_header(const SegmentContext& segment, Selections*const* selections, AnimationTrackType8 track_type, uint16_t num_bones, uint32_t sample_index, uint8_t* animated_track_data_begin,
				FrameHeader*& out_previous_frame_header, uint8_t*& out_animated_track_data, uint64_t& out_bit_offset)
			{
				uint32_t bitset_size = get_bitset_size(num_bones);

				FrameHeader* frame_header = safe_ptr_cast<FrameHeader, uint8_t*>(out_animated_track_data);
				out_animated_track_data += sizeof(FrameHeader) + bitset_size * sizeof(uint32_t);
				out_bit_offset = (out_animated_track_data - animated_track_data_begin) * 8;

				frame_header->set_frame_type(track_type);
				frame_header->set_next_frame_offset(get_frame_size(segment, selections, track_type, sample_index));
				frame_header->set_previous_frame_offset(out_previous_frame_header == nullptr ? 0 : out_previous_frame_header->get_next_frame_offset());
				frame_header->sample_index = sample_index;
				bitset_reset(frame_header->bones_having_data, bitset_size, false);

				out_previous_frame_header = frame_header;

				return frame_header;
			}

			struct ControlPoint
			{
				uint32_t sample_index;
				Vector4_32 value;
				float knot;
			};

			inline void write_frame_bone_data(const BoneStreams& bone_streams, const TrackStream& track_stream, AnimationTrackType8 track_type, uint16_t num_bones, uint32_t num_samples,
				uint32_t sample_index, const Selections& selections, uint8_t* animated_track_data_begin,
				FrameHeader& out_frame_header, ControlPoint& out_control_point, uint8_t*& out_values, uint64_t& out_values_bit_offset, float*& out_knots)
			{
				ControlPoint current = out_control_point;
				const ControlPoint last = current;

				if (sample_index == 0)
				{
					current.sample_index = 0;
					current.value = sample(bone_streams, track_type, 0);
					current.knot = 0.0;
				}
				else if (selections.is_selected(sample_index))
				{
					current.sample_index = sample_index;
					current.value = sample(bone_streams, track_type, sample_index);
					current.knot = last.knot + get_knot_delta(current.value, current.sample_index, last.value, last.sample_index);
				}

				if (current.sample_index == sample_index)
				{
					bitset_set(out_frame_header.bones_having_data, get_bitset_size(num_bones), bone_streams.bone_index, true);
					write_animated_track_data(track_stream, sample_index, false, animated_track_data_begin, out_values, out_values_bit_offset);
					*out_knots++ = current.knot;
				}

				out_control_point = current;
			}

			inline void write_frame(const SegmentContext& segment, AnimationTrackType8 track_type, Selections*const* sample_selections, uint32_t sample_index, uint8_t* animated_track_data_begin, const uint8_t* animated_track_data_end,
				FrameHeader*& out_previous_frame_header, uint8_t*& out_animated_track_data, uint64_t& out_bit_offset, ControlPoint* out_control_points)
			{
				FrameHeader* frame_header = nullptr;
				float* knots = nullptr;

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const Selections* selections = sample_selections[bone_index];
					if (selections != nullptr && selections->is_selected(sample_index))
					{
						const BoneStreams& bone_streams = segment.bone_streams[bone_index];
						const TrackStream& track_stream = bone_streams.get_track_stream(track_type);

						if (frame_header == nullptr)
						{
							frame_header = write_frame_header(segment, sample_selections, track_type, segment.num_bones, sample_index, animated_track_data_begin, out_previous_frame_header, out_animated_track_data, out_bit_offset);

							uint32_t frame_samples_size = get_frame_data_size(segment, sample_selections, track_type, sample_index, false);
							knots = safe_ptr_cast<float, uint8_t*>(out_animated_track_data + frame_samples_size);
						}

						write_frame_bone_data(bone_streams, track_stream, track_type, segment.num_bones, track_stream.get_num_samples(), sample_index,
							*selections, animated_track_data_begin, *frame_header, out_control_points[bone_index], out_animated_track_data, out_bit_offset, knots);

						ACL_ENSURE(out_animated_track_data <= animated_track_data_end, "Invalid animated track data offset. Wrote too much data.");
					}
				}

				if (knots != nullptr)
				{
					out_animated_track_data = safe_ptr_cast<uint8_t, float*>(knots);
					out_bit_offset = (out_animated_track_data - animated_track_data_begin) * 8;
				}
			}

			inline void write_animated_track_data(Allocator& allocator, const SegmentContext& segment, Selections*const* rotation_selections, Selections*const* translation_selections, Selections*const* scale_selections, uint8_t* animated_track_data, uint32_t animated_data_size)
			{
				uint8_t* animated_track_data_begin = animated_track_data;
				const uint8_t* animated_track_data_end = add_offset_to_ptr<uint8_t>(animated_track_data, animated_data_size);
				uint64_t bit_offset = 0;
				FrameHeader* previous_frame_header = nullptr;

				ControlPoint* rotation_control_points = allocate_type_array<ControlPoint>(allocator, segment.num_bones);
				ControlPoint* translation_control_points = allocate_type_array<ControlPoint>(allocator, segment.num_bones);
				ControlPoint* scale_control_points = allocate_type_array<ControlPoint>(allocator, segment.num_bones);

				for (uint32_t sample_index = 0; sample_index < segment.num_samples; ++sample_index)
				{
					write_frame(segment, AnimationTrackType8::Rotation, rotation_selections, sample_index, animated_track_data_begin, animated_track_data_end, previous_frame_header, animated_track_data, bit_offset, rotation_control_points);
					write_frame(segment, AnimationTrackType8::Translation, translation_selections, sample_index, animated_track_data_begin, animated_track_data_end, previous_frame_header, animated_track_data, bit_offset, translation_control_points);
					write_frame(segment, AnimationTrackType8::Scale, scale_selections, sample_index, animated_track_data_begin, animated_track_data_end, previous_frame_header, animated_track_data, bit_offset, scale_control_points);
				}

				// A zero offset indicates this is the last frame for the segment.
				if (previous_frame_header != nullptr)
					previous_frame_header->set_next_frame_offset(0);

				if (bit_offset != 0)
					animated_track_data = animated_track_data_begin + (align_to(bit_offset, 8) / 8);

				ACL_ENSURE(animated_track_data == animated_track_data_end, "Invalid animated track data offset. Wrote too little data.");

				deallocate_type_array(allocator, rotation_control_points, segment.num_bones);
				deallocate_type_array(allocator, translation_control_points, segment.num_bones);
				deallocate_type_array(allocator, scale_control_points, segment.num_bones);
			}

			// TODO: merge all instances to write_stream_data.h
			inline void write_segment_headers(const ClipContext& clip_context, const CompressionSettings& settings, SegmentHeader* segment_headers, uint16_t segment_headers_start_offset)
			{
				const uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format, settings.scale_format);

				uint32_t data_offset = segment_headers_start_offset;
				for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
				{
					const SegmentContext& segment = clip_context.segments[segment_index];
					SegmentHeader& header = segment_headers[segment_index];

					header.num_samples = segment.num_samples;
					header.format_per_track_data_offset = data_offset;
					header.range_data_offset = align_to(header.format_per_track_data_offset + format_per_track_data_size, 2);		// Aligned to 2 bytes
					header.track_data_offset = align_to(header.range_data_offset + segment.range_data_size, 4);						// Aligned to 4 bytes

					data_offset = header.track_data_offset + segment.animated_data_size;
				}
			}

			// TODO: merge all instances to write_stream_data.h.  Add a lambda parameter so caller can override writing of animated data.
			inline void write_segment_data(Allocator& allocator, const ClipContext& clip_context, const CompressionSettings& settings, ClipHeader& header, Selections*const* rotation_selections, Selections*const* translation_selections, Selections*const* scale_selections)
			{
				SegmentHeader* segment_headers = header.get_segment_headers();
				const uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format, settings.scale_format);

				for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
				{
					const SegmentContext& segment = clip_context.segments[segment_index];
					SegmentHeader& segment_header = segment_headers[segment_index];

					if (format_per_track_data_size > 0)
						write_format_per_track_data(clip_context, segment, header.get_format_per_track_data(segment_header), format_per_track_data_size);
					else
						segment_header.format_per_track_data_offset = InvalidPtrOffset();

					if (segment.range_data_size > 0)
						write_segment_range_data(clip_context, segment, settings.range_reduction, header.get_segment_range_data(segment_header), segment.range_data_size);
					else
						segment_header.range_data_offset = InvalidPtrOffset();

					if (segment.animated_data_size > 0)
						write_animated_track_data(allocator, segment, rotation_selections, translation_selections, scale_selections, header.get_track_data(segment_header), segment.animated_data_size);
					else
						segment_header.track_data_offset = InvalidPtrOffset();
				}
			}

			inline void choose_control_points(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				Selections** rotation_selections, Selections** translation_selections, Selections** scale_selections)
			{
				float* error_per_bone = allocate_type_array<float>(allocator, segment.num_bones);
				BoneTrackError* error_per_stream = allocate_type_array<BoneTrackError>(allocator, segment.num_bones);
				Transform_32* raw_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);
				Transform_32* lossy_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);

				Selections* modified_rotation_selections;
				Selections* modified_translation_selections;
				Selections* modified_scale_selections;

				// Try assigning control points sequentially.  Most of the time this will provide a better compression ratio.
				reset_control_point_choices(segment, rotation_selections, translation_selections, scale_selections);

				for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); ++sample_index)
				{
					try_control_points_at(allocator, skeleton, segment, sample_index, rotation_selections, translation_selections, scale_selections,
						error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_selections, modified_translation_selections, modified_scale_selections);

					rewind_to_first_affected_sample(modified_rotation_selections, modified_translation_selections, modified_scale_selections, sample_index);
				}

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (rotation_selections[bone_index] != nullptr)
						rotation_selections[bone_index]->save_state();

					if (translation_selections[bone_index] != nullptr)
						translation_selections[bone_index]->save_state();

					if (scale_selections[bone_index] != nullptr)
						scale_selections[bone_index]->save_state();
				}

				uint32_t size_with_sequential_selection = get_animated_data_size(segment, rotation_selections, translation_selections, scale_selections);

				// Now try assigning control points more distant from already tried points; occasionally this beats the prior method.
				reset_control_point_choices(segment, rotation_selections, translation_selections, scale_selections);

				uint32_t step = segment.num_samples / 2;
				while (step >= 1)
				{
					for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); sample_index += step)
					{
						try_control_points_at(allocator, skeleton, segment, sample_index, rotation_selections, translation_selections, scale_selections,
							error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_selections, modified_translation_selections, modified_scale_selections);

						rewind_to_first_affected_sample(modified_rotation_selections, modified_translation_selections, modified_scale_selections, sample_index);
					}

					step >>= 1;
				}

				uint32_t size_with_maximally_distant_selection = get_animated_data_size(segment, rotation_selections, translation_selections, scale_selections);

				if (size_with_maximally_distant_selection > size_with_sequential_selection)
				{
					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						if (rotation_selections[bone_index] != nullptr)
							rotation_selections[bone_index]->restore_state();

						if (translation_selections[bone_index] != nullptr)
							translation_selections[bone_index]->restore_state();

						if (scale_selections[bone_index] != nullptr)
							scale_selections[bone_index]->restore_state();
					}
				}

				deallocate_type_array(allocator, error_per_bone, segment.num_bones);
				deallocate_type_array(allocator, error_per_stream, segment.num_bones);
				deallocate_type_array(allocator, raw_local_pose, segment.num_bones);
				deallocate_type_array(allocator, lossy_local_pose, segment.num_bones);
			}
		}

		// Encoder entry point
		inline CompressedClip* compress_clip(Allocator& allocator, const AnimationClip& clip, const RigidSkeleton& skeleton, const CompressionSettings& settings, OutputStats& stats)
		{
			using namespace impl;

			ScopeProfiler compression_time;

			const uint16_t num_bones = clip.get_num_bones();
			const uint32_t num_samples = clip.get_num_samples();

			if (ACL_TRY_ASSERT(num_bones > 0, "Clip has no bones!"))
				return nullptr;
			if (ACL_TRY_ASSERT(num_samples > 0, "Clip has no samples!"))
				return nullptr;

			if (settings.translation_format != VectorFormat8::Vector3_96)
			{
				bool has_clip_range_reduction = is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::Translations);
				bool has_segment_range_reduction = settings.segmenting.enabled && is_enum_flag_set(settings.segmenting.range_reduction, RangeReductionFlags8::Translations);
				if (ACL_TRY_ASSERT(has_clip_range_reduction | has_segment_range_reduction, "%s quantization requires range reduction to be enabled at the clip or segment level!", get_vector_format_name(settings.translation_format)))
					return nullptr;
			}

			if (settings.segmenting.enabled && settings.segmenting.range_reduction != RangeReductionFlags8::None)
			{
				if (ACL_TRY_ASSERT(settings.range_reduction != RangeReductionFlags8::None, "Per segment range reduction requires per clip range reduction to be enabled!"))
					return nullptr;
			}

			ClipContext raw_clip_context;
			initialize_clip_context(allocator, clip, skeleton, raw_clip_context);

			ClipContext clip_context;
			initialize_clip_context(allocator, clip, skeleton, clip_context);

			convert_rotation_streams(allocator, clip_context, settings.rotation_format);

			// Extract our clip ranges now, we need it for compacting the constant streams
			extract_clip_bone_ranges(allocator, clip_context);

			// TODO: Expose this, especially the translation threshold depends on the unit scale.
			// Centimeters VS meters, a different threshold should be used. Perhaps we should pass an
			// argument to the compression algorithm that states the units used or we should force centimeters
			compact_constant_streams(allocator, clip_context, 0.00001f, 0.001f, 0.00001f);

			if (settings.segmenting.enabled)
			{
				segment_streams(allocator, clip_context, settings.segmenting);

				for (SegmentContext& segment : clip_context.segment_iterator())
				{
					// The segment ranges haven't been initialized yet, but the clip ranges have, and they must be updated to include the new control points.
					impl::extend_segment_with_auxiliary_control_points(allocator, segment, clip_context.ranges, raw_clip_context);
				}
			}
			else
			{
				impl::extend_segment_with_auxiliary_control_points(allocator, clip_context.segments[0], clip_context.ranges, raw_clip_context);
			}

			uint32_t clip_range_data_size = 0;
			if (settings.range_reduction != RangeReductionFlags8::None)
			{
				normalize_clip_streams(clip_context, settings.range_reduction);
				clip_range_data_size = get_stream_range_data_size(clip_context, settings.range_reduction, settings.rotation_format, settings.translation_format);
			}

			if (settings.segmenting.enabled && settings.segmenting.range_reduction != RangeReductionFlags8::None)
			{
				extract_segment_bone_ranges(allocator, clip_context);
				normalize_segment_streams(clip_context, settings.range_reduction);
			}

			quantize_streams(allocator, clip_context, settings.rotation_format, settings.translation_format, settings.scale_format, clip, skeleton, raw_clip_context);

			uint32_t num_selections = clip_context.num_segments * clip_context.num_bones;
			Selections** rotation_selections = allocate_type_array<Selections*>(allocator, num_selections);
			Selections** translation_selections = allocate_type_array<Selections*>(allocator, num_selections);
			Selections** scale_selections = allocate_type_array<Selections*>(allocator, num_selections);

			// TODO: std::fill
			for (uint32_t selections_index = 0; selections_index < num_selections; ++selections_index)
			{
				rotation_selections[selections_index] = nullptr;
				translation_selections[selections_index] = nullptr;
				scale_selections[selections_index] = nullptr;
			}

			for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
			{
				SegmentContext& segment = clip_context.segments[segment_index];

				Selections** segment_rotation_selections = rotation_selections + segment_index * clip_context.num_bones;
				Selections** segment_translation_selections = translation_selections + segment_index * clip_context.num_bones;
				Selections** segment_scale_selections = scale_selections + segment_index * clip_context.num_bones;

				for (uint16_t bone_index = 0; bone_index < clip_context.num_bones; ++bone_index)
				{
					const BoneStreams& bone_streams = segment.bone_streams[bone_index];

					if (bone_streams.is_rotation_animated() && !is_pack_0_bit_rate(bone_streams.rotations.get_bit_rate()))
						segment_rotation_selections[bone_index] = allocate_type<Selections>(allocator, allocator, segment.num_samples);

					if (bone_streams.is_translation_animated() && !is_pack_0_bit_rate(bone_streams.translations.get_bit_rate()))
						segment_translation_selections[bone_index] = allocate_type<Selections>(allocator, allocator, segment.num_samples);

					if (bone_streams.is_scale_animated() && !is_pack_0_bit_rate(bone_streams.scales.get_bit_rate()))
						segment_scale_selections[bone_index] = allocate_type<Selections>(allocator, allocator, segment.num_samples);
				}

				choose_control_points(allocator, skeleton, segment, segment_rotation_selections, segment_translation_selections, segment_scale_selections);
			}

			const uint32_t constant_data_size = get_constant_data_size(clip_context);

			calculate_animated_data_size(clip_context, rotation_selections, translation_selections, scale_selections);

			const uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format, settings.scale_format);

			const uint32_t num_tracks_per_bone = clip_context.has_scale ? 3 : 2;
			const uint32_t num_tracks = uint32_t(num_bones) * num_tracks_per_bone;
			const uint32_t bitset_size = get_bitset_size(num_tracks);

			uint32_t buffer_size = 0;

			// Per clip data
			buffer_size += sizeof(CompressedClip);
			buffer_size += sizeof(ClipHeader);
			buffer_size += sizeof(SegmentHeader) * clip_context.num_segments;	// Segment headers
			buffer_size += sizeof(uint32_t) * bitset_size;						// Default tracks bitset
			buffer_size += sizeof(uint32_t) * bitset_size;						// Constant tracks bitset
			buffer_size = align_to(buffer_size, 4);								// Align constant track data
			buffer_size += constant_data_size;									// Constant track data
			buffer_size = align_to(buffer_size, 4);								// Align range data
			buffer_size += clip_range_data_size;								// Range data

			// Per segment data
			for (const SegmentContext& segment : clip_context.segment_iterator())
			{
				buffer_size += format_per_track_data_size;						// Format per track data
				buffer_size = align_to(buffer_size, 2);							// Align range data
				buffer_size += segment.range_data_size;							// Range data
				buffer_size = align_to(buffer_size, 4);							// Align animated data
				buffer_size += segment.animated_data_size;						// Animated track data
			}

			printf("total size %d\n", buffer_size);

			uint8_t* buffer = allocate_type_array_aligned<uint8_t>(allocator, buffer_size, 16);

			CompressedClip* compressed_clip = make_compressed_clip(buffer, buffer_size, AlgorithmType8::SplineKeyReduction);

			ClipHeader& header = get_clip_header(*compressed_clip);
			header.num_bones = num_bones;
			header.num_segments = clip_context.num_segments;
			header.rotation_format = settings.rotation_format;
			header.translation_format = settings.translation_format;
			header.scale_format = settings.scale_format;
			header.clip_range_reduction = settings.range_reduction;
			header.segment_range_reduction = settings.segmenting.range_reduction;
			header.has_scale = clip_context.has_scale ? 1 : 0;
			header.num_samples = num_samples;
			header.sample_rate = clip.get_sample_rate();
			header.segment_headers_offset = sizeof(ClipHeader);
			header.default_tracks_bitset_offset = header.segment_headers_offset + (sizeof(SegmentHeader) * clip_context.num_segments);
			header.constant_tracks_bitset_offset = header.default_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size);
			header.constant_track_data_offset = align_to(header.constant_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size), 4);	// Aligned to 4 bytes
			header.clip_range_data_offset = align_to(header.constant_track_data_offset + constant_data_size, 4);						// Aligned to 4 bytes

			const uint16_t segment_headers_start_offset = safe_static_cast<uint16_t>(header.clip_range_data_offset + clip_range_data_size);
			impl::write_segment_headers(clip_context, settings, header.get_segment_headers(), segment_headers_start_offset);

			write_default_track_bitset(clip_context, header.get_default_tracks_bitset(), bitset_size);
			write_constant_track_bitset(clip_context, header.get_constant_tracks_bitset(), bitset_size);

			if (constant_data_size > 0)
				write_constant_track_data(clip_context, header.get_constant_track_data(), constant_data_size);
			else
				header.constant_track_data_offset = InvalidPtrOffset();

			if (settings.range_reduction != RangeReductionFlags8::None)
				write_clip_range_data(clip_context, settings.range_reduction, header.get_clip_range_data(), clip_range_data_size);
			else
				header.clip_range_data_offset = InvalidPtrOffset();

			write_segment_data(allocator, clip_context, settings, header, rotation_selections, translation_selections, scale_selections);

			finalize_compressed_clip(*compressed_clip);

			for (uint32_t selections_index = 0; selections_index < num_selections; ++selections_index)
			{
				deallocate_type(allocator, rotation_selections[selections_index]);
				deallocate_type(allocator, translation_selections[selections_index]);
				deallocate_type(allocator, scale_selections[selections_index]);
			}

			deallocate_type_array(allocator, rotation_selections, num_selections);
			deallocate_type_array(allocator, translation_selections, num_selections);
			deallocate_type_array(allocator, scale_selections, num_selections);

			compression_time.stop();

#if false
			// TODO: I don't think the encoder should be responsible for writing "diagnostic" data like this.
			// It can't be split out into general code as is, though, because the decoder is needed.
			// Make an attempt though!  If print_stats() was moved...

			if (stats.get_logging() != StatLogging::None)
			{
				uint32_t raw_size = clip.get_raw_size();
				uint32_t compressed_size = compressed_clip->get_size();
				double compression_ratio = double(raw_size) / double(compressed_size);

				uint32_t num_default_tracks = bitset_count_set_bits(header.get_default_tracks_bitset(), bitset_size);
				uint32_t num_constant_tracks = bitset_count_set_bits(header.get_constant_tracks_bitset(), bitset_size);
				uint32_t num_animated_tracks = num_tracks - num_default_tracks - num_constant_tracks;

				auto alloc_ctx_fun = [&](Allocator& allocator)
				{
					DecompressionSettings settings;
					return allocate_decompression_context(allocator, settings, *compressed_clip);
				};

				auto free_ctx_fun = [&](Allocator& allocator, void* context)
				{
					deallocate_decompression_context(allocator, context);
				};

				auto sample_fun = [&](void* context, float sample_time, Transform_32* out_transforms, uint16_t num_transforms)
				{
					DecompressionSettings settings;
					DefaultOutputWriter writer(out_transforms, num_transforms);
					decompress_pose(settings, *compressed_clip, context, sample_time, writer);
				};

				// Use the compressed clip to make sure the decoder works properly
				BoneError error = calculate_compressed_clip_error(allocator, clip, skeleton, clip_context.has_scale, alloc_ctx_fun, free_ctx_fun, sample_fun);

				SJSONObjectWriter& writer = stats.get_writer();
				writer["algorithm_name"] = get_algorithm_name(AlgorithmType8::SplineKeyReduction);
				writer["algorithm_uid"] = settings.hash();
				writer["clip_name"] = clip.get_name().c_str();
				writer["raw_size"] = raw_size;
				writer["compressed_size"] = compressed_size;
				writer["compression_ratio"] = compression_ratio;
				writer["max_error"] = error.error;
				writer["worst_bone"] = error.index;
				writer["worst_time"] = error.sample_time;
				writer["compression_time"] = cycles_to_seconds(compression_time.get_elapsed_cycles());
				writer["duration"] = clip.get_duration();
				writer["num_samples"] = clip.get_num_samples();
				writer["rotation_format"] = get_rotation_format_name(settings.rotation_format);
				writer["translation_format"] = get_vector_format_name(settings.translation_format);
				writer["scale_format"] = get_vector_format_name(settings.scale_format); 
				writer["range_reduction"] = get_range_reduction_name(settings.range_reduction);
				writer["has_scale"] = clip_context.has_scale;

				if (stats.get_logging() == StatLogging::Detailed)
				{
					writer["num_bones"] = clip.get_num_bones();
					writer["num_default_tracks"] = num_default_tracks;
					writer["num_constant_tracks"] = num_constant_tracks;
					writer["num_animated_tracks"] = num_animated_tracks;
				}

				if (settings.segmenting.enabled)
				{
					writer["segmenting"] = [&](SJSONObjectWriter& writer)
					{
						writer["num_segments"] = header.num_segments;
						writer["range_reduction"] = get_range_reduction_name(settings.segmenting.range_reduction);
						writer["ideal_num_samples"] = settings.segmenting.ideal_num_samples;
						writer["max_num_samples"] = settings.segmenting.max_num_samples;
					};
				}

				if (stats.get_logging() == StatLogging::Detailed)
					write_stream_stats(allocator, clip_context, raw_clip_context, skeleton, writer);
			}
#endif

			destroy_clip_context(allocator, clip_context);
			destroy_clip_context(allocator, raw_clip_context);

			return compressed_clip;
		}
	}
}
