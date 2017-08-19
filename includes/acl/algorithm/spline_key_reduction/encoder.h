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
#include "acl/core/memory.h"
#include "acl/core/error.h"
#include "acl/core/bitset.h"
#include "acl/core/enum_utils.h"
#include "acl/core/algorithm_types.h"
#include "acl/core/track_types.h"
#include "acl/algorithm/spline_key_reduction/common.h"
#include "acl/algorithm/spline_key_reduction/spline.h"
#include "acl/compression/compressed_clip_impl.h"
#include "acl/compression/skeleton.h"
#include "acl/compression/animation_clip.h"
#include "acl/compression/stream/segment_streams.h"
#include "acl/compression/stream/track_stream.h"
#include "acl/compression/stream/convert_rotation_streams.h"
#include "acl/compression/stream/compact_constant_streams.h"
#include "acl/compression/stream/extend_streams.h"
#include "acl/compression/stream/normalize_streams.h"
#include "acl/compression/stream/quantize_streams.h"
#include "acl/compression/stream/write_stream_bitsets.h"
#include "acl/compression/stream/write_stream_data.h"
#include "acl/compression/stream/write_range_data.h"

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
//    TODO: Detail the format
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

			RangeReductionFlags8 range_reduction;

			SegmentingSettings segmenting;

			CompressionSettings()
				: rotation_format(RotationFormat8::Quat_128)
				, translation_format(VectorFormat8::Vector3_96)
				, range_reduction(RangeReductionFlags8::None)
				, segmenting()
			{}
		};

		namespace impl
		{
			class Selections
			{
			public:
				Selections(Allocator& allocator, uint32_t num_samples)
					: m_allocator(allocator)
				{
					m_remove_size = get_bitset_size(num_samples);
					m_remove = allocate_type_array<uint32_t>(m_allocator, m_remove_size);
					m_remove_backup = allocate_type_array<uint32_t>(m_allocator, m_remove_size);
				}

				~Selections()
				{
					deallocate_type_array(m_allocator, m_remove, m_remove_size);
					deallocate_type_array(m_allocator, m_remove_backup, m_remove_size);
				}

				bool is_selected(uint32_t sample_index) const { return bitset_test(m_remove, m_remove_size, sample_index); }
				void deselect_all() { bitset_reset(m_remove, m_remove_size, true); }
				void deselect(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, true); }
				void select(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, false); }

				void save_state() { std::memcpy(m_remove_backup, m_remove, m_remove_size * sizeof(uint32_t)); }
				void restore_state() { std::memcpy(m_remove, m_remove_backup, m_remove_size * sizeof(uint32_t)); }

				uint32_t get_num_selected() const
				{
					return bitset_count_set_bits(m_remove, m_remove_size);
				}

			private:
				Allocator& m_allocator;
				uint32_t* m_remove;
				uint32_t* m_remove_backup;
				uint32_t m_remove_size;
			};

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
				default:
					// TODO: assert
					break;
				}
			}

			Vector4_32 interpolate(const BoneStreams& bone_streams, AnimationTrackType8 track_type, const Selections& selections, uint32_t at_sample_index)
			{
				if (!selections.is_selected(at_sample_index))
				{
					return sample(bone_streams, track_type, at_sample_index);
				}

				Vector4_32 values[POLYNOMIAL_ORDER + 1];
				uint32_t sample_indices[POLYNOMIAL_ORDER + 1];

				Vector4_32 value;
				uint32_t sample_index = at_sample_index;

				for (int8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX; control_point_index >= 0; --control_point_index)
				{
					while (true)
					{
						--sample_index;
						if (!selections.is_selected(sample_index))
						{
							value = sample(bone_streams, track_type, sample_index);
							break;
						}
					}

					values[control_point_index] = value;
					sample_indices[control_point_index] = sample_index;
				}

				sample_index = at_sample_index;

				for (uint8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX + 1; control_point_index <= POLYNOMIAL_ORDER; ++control_point_index)
				{
					while (true)
					{
						++sample_index;
						if (!selections.is_selected(sample_index))
						{
							value = sample(bone_streams, track_type, sample_index);
							break;
						}
					}

					values[control_point_index] = value;
					sample_indices[control_point_index] = sample_index;
				}

				float knots[POLYNOMIAL_ORDER + 1];
				calculate_knots(values, sample_indices, knots);

				return interpolate_spline(values, knots, sample_indices, static_cast<float>(at_sample_index));
			}

			inline void get_auxiliary_control_points(uint32_t segment_clip_sample_offset, uint32_t num_samples, const BoneStreams& raw_bone_streams, AnimationTrackType8 track_type,
				Vector4_32* out_prefixes, uint32_t& out_num_prefixes, Vector4_32* out_suffixes, uint32_t& out_num_suffixes)
			{
				// Reflect across the first sample to create an auxiliary control point beyond the clip that will ensure a reasonable interpolation near time 0.
				for (out_num_prefixes = 0; out_num_prefixes < NUM_LEFT_AUXILIARY_POINTS; ++out_num_prefixes)
				{
					out_prefixes[out_num_prefixes] = sample(raw_bone_streams, track_type, static_cast<int32_t>(segment_clip_sample_offset) - NUM_LEFT_AUXILIARY_POINTS + out_num_prefixes);
				}

				out_num_suffixes = 0;

				while (true)
				{
					if (out_num_suffixes >= NUM_RIGHT_AUXILIARY_POINTS && out_num_prefixes + num_samples + out_num_suffixes >= POLYNOMIAL_ORDER + 1)
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

					Vector4_32 suffixes[POLYNOMIAL_ORDER];
					uint32_t num_suffixes;

					if (bone_streams.is_rotation_animated())
					{
						RotationTrackStream& stream = bone_streams.rotations;
						get_auxiliary_control_points(segment.clip_sample_offset, stream.get_num_samples(), raw_bone_streams, AnimationTrackType8::Rotation, prefixes, num_prefixes, suffixes, num_suffixes);
						extend_rotation_stream(allocator, stream, bone_range, prefixes, num_prefixes, suffixes, num_suffixes);
						segment.num_samples = stream.get_num_samples();
					}

					if (bone_streams.is_translation_animated())
					{
						TranslationTrackStream& stream = bone_streams.translations;
						get_auxiliary_control_points(segment.clip_sample_offset, stream.get_num_samples(), raw_bone_streams, AnimationTrackType8::Translation, prefixes, num_prefixes, suffixes, num_suffixes);
						extend_translation_stream(allocator, stream, bone_range, prefixes, num_prefixes, suffixes, num_suffixes);
						segment.num_samples = stream.get_num_samples();
					}
				}
			}

			inline uint32_t get_animated_data_size(const SegmentContext& segment, Selections*const* rotation_selections, Selections*const* translation_selections)
			{
				uint32_t animated_data_size = 0;

				for (uint32_t sample_index = 0; sample_index < segment.num_samples; ++sample_index)
				{
					uint32_t num_rotation_bits = 0;
					uint32_t num_translation_bits = 0;

					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						const BoneStreams& bone_streams = segment.bone_streams[bone_index];
						const Selections* selections;

						selections = rotation_selections[bone_index];
						if (selections != nullptr && !selections->is_selected(sample_index))
						{
							if (num_rotation_bits == 0 && !use_implicit_frame_header(sample_index, segment.num_samples))
							{
								num_rotation_bits += sizeof(uint32_t) * 8;						// Flags and offsets
								num_rotation_bits += sizeof(uint32_t) * 8;						// Sample index
								num_rotation_bits += get_bitset_size(segment.num_bones) * 8;
							}

							if (bone_streams.rotations.is_bit_rate_variable())
							{
								uint8_t bit_rate = bone_streams.rotations.get_bit_rate();
								num_rotation_bits += get_num_bits_at_bit_rate(bit_rate) * 3;	// 3 components
							}
							else
							{
								RotationFormat8 format = bone_streams.rotations.get_rotation_format();
								num_rotation_bits += get_packed_rotation_size(format) * 8;
							}

							num_rotation_bits += sizeof(float) * 8;								// Knot
						}

						selections = translation_selections[bone_index];
						if (selections != nullptr && !selections->is_selected(sample_index))
						{
							if (num_translation_bits == 0 && !use_implicit_frame_header(sample_index, segment.num_samples))
							{
								num_translation_bits += sizeof(uint32_t) * 8;						// Flags and offsets
								num_translation_bits += sizeof(uint32_t) * 8;						// Sample index
								num_translation_bits += get_bitset_size(segment.num_bones) * 8;
							}

							if (bone_streams.translations.is_bit_rate_variable())
							{
								uint8_t bit_rate = bone_streams.translations.get_bit_rate();
								num_translation_bits += get_num_bits_at_bit_rate(bit_rate) * 3;		// 3 components
							}
							else
							{
								VectorFormat8 format = bone_streams.translations.get_vector_format();
								num_translation_bits += get_packed_vector_size(format) * 8;
							}

							num_translation_bits += sizeof(float) * 8;								// Knot
						}
					}

					num_rotation_bits += num_rotation_bits % 32;
					num_translation_bits += num_translation_bits % 32;
					animated_data_size += num_rotation_bits / 8 + num_translation_bits / 8;
				}

				return animated_data_size;
			}

			inline void interpolate_pose(const SegmentContext& segment, Selections*const* rotation_selections, Selections*const* translation_selections,
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

					out_local_pose[bone_index] = transform_set(rotation, translation);
				}
			}

			inline void reset_control_point_choices(Selections& selections, uint32_t num_samples)
			{
				selections.deselect_all();

				for (uint32_t sample_index = 0; sample_index <= NUM_LEFT_AUXILIARY_POINTS; ++sample_index)
					selections.select(sample_index);

				for (uint32_t sample_index = num_samples - 1; sample_index >= num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS; --sample_index)
					selections.select(sample_index);
			}

			inline void reset_control_point_choices(const SegmentContext& segment, Selections** rotation_selections, Selections** translation_selections)
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
				}
			}

			inline void try_control_points_at(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				uint32_t sample_index, Selections** rotation_selections, Selections** translation_selections,
				float* error_per_bone, BoneTrackError* error_per_stream, Transform_32* raw_local_pose, Transform_32* lossy_local_pose,
				Selections*& out_modified_rotation_selections, Selections*& out_modified_translation_selections)
			{
				sample_streams(segment.bone_streams, segment.num_bones, sample_index, raw_local_pose);
				interpolate_pose(segment, rotation_selections, translation_selections, sample_index, lossy_local_pose);

				double error = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose, error_per_bone);

				if (error <= segment.clip->error_threshold)
				{
					out_modified_rotation_selections = nullptr;
					out_modified_translation_selections = nullptr;
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
					bool rotation_is_interpolated = rotation_selections[bone_index] != nullptr && rotation_selections[bone_index]->is_selected(sample_index);
					bool translation_is_interpolated = translation_selections[bone_index] != nullptr && translation_selections[bone_index]->is_selected(sample_index);

					if (!rotation_is_interpolated && !translation_is_interpolated)
						continue;

					const BoneTrackError& error = error_per_stream[bone_index];

					if (error.rotation + error.translation > worst_error_contribution)
					{
						bad_bone_index = bone_index;
						worst_error_contribution = error.rotation + error.translation;
					}
				}

				ACL_ASSERT(bad_bone_index != INVALID_BONE_INDEX, "Failed to find the bone with the worst error");

				Selections* bad_bone_rotations = rotation_selections[bad_bone_index];
				bool old_removed_rotation = bad_bone_rotations != nullptr && bad_bone_rotations->is_selected(sample_index);

				Selections* bad_bone_translations = translation_selections[bad_bone_index];
				bool old_removed_translation = bad_bone_translations != nullptr && bad_bone_translations->is_selected(sample_index);

				if (bad_bone_rotations != nullptr && bad_bone_rotations->is_selected(sample_index))
				{
					if (bad_bone_translations == nullptr || !bad_bone_translations->is_selected(sample_index))
					{
						bad_bone_rotations->select(sample_index);
					}
					else
					{
						bad_bone_rotations->select(sample_index);

						interpolate_pose(segment, rotation_selections, translation_selections, sample_index, lossy_local_pose);
						double error_without_interpolated_rotation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

						if (error_without_interpolated_rotation > segment.clip->error_threshold)
						{
							bad_bone_rotations->deselect(sample_index);
							bad_bone_translations->select(sample_index);

							interpolate_pose(segment, rotation_selections, translation_selections, sample_index, lossy_local_pose);
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

				bool changed_rotation = bad_bone_rotations != nullptr && old_removed_rotation != bad_bone_rotations->is_selected(sample_index);
				out_modified_rotation_selections = changed_rotation ? bad_bone_rotations : nullptr;

				bool changed_translation = bad_bone_translations != nullptr && old_removed_translation != bad_bone_translations->is_selected(sample_index);
				out_modified_translation_selections = changed_translation ? bad_bone_translations : nullptr;

				ACL_ASSERT(changed_rotation || changed_translation, "No changes were made to the bone with the worst error contribution");
			}

			inline void rewind_to_first_affected_sample(const Selections* modified_rotation_selections, const Selections* modified_translation_selections, uint32_t& out_sample_index)
			{
				if (modified_rotation_selections == nullptr && modified_translation_selections == nullptr)
					return;

				uint32_t num_rotation_points = 0,
					min_rotation_points = modified_rotation_selections == nullptr ? 0 : POLYNOMIAL_ORDER,
					num_translation_points = 0,
					min_translation_points = modified_translation_selections == nullptr ? 0 : POLYNOMIAL_ORDER;

				while (out_sample_index > NUM_LEFT_AUXILIARY_POINTS + 1 &&
					(num_rotation_points < min_rotation_points || min_rotation_points == 0) &&
					(num_translation_points <= min_translation_points || min_translation_points == 0))
				{
					if (modified_rotation_selections != nullptr && !modified_rotation_selections->is_selected(out_sample_index))
						++num_rotation_points;

					if (modified_translation_selections != nullptr && !modified_translation_selections->is_selected(out_sample_index))
						++num_translation_points;

					--out_sample_index;
				}
			}

			inline void choose_control_points(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				Selections** rotation_selections, Selections** translation_selections)
			{
				float* error_per_bone = allocate_type_array<float>(allocator, segment.num_bones);
				BoneTrackError* error_per_stream = allocate_type_array<BoneTrackError>(allocator, segment.num_bones);
				Transform_32* raw_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);
				Transform_32* lossy_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);

				Selections* modified_rotation_selections;
				Selections* modified_translation_selections;

				// Try assigning control points sequentially.  Most of the time this will provide a better compression ratio.
				reset_control_point_choices(segment, rotation_selections, translation_selections);

				for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); ++sample_index)
				{
					try_control_points_at(allocator, skeleton, segment, sample_index, rotation_selections, translation_selections,
						error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_selections, modified_translation_selections);

					rewind_to_first_affected_sample(modified_rotation_selections, modified_translation_selections, sample_index);
				}

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (rotation_selections[bone_index] != nullptr)
						rotation_selections[bone_index]->save_state();

					if (translation_selections[bone_index] != nullptr)
						translation_selections[bone_index]->save_state();
				}

				uint32_t size_with_sequential_selection = get_animated_data_size(segment, rotation_selections, translation_selections);

				// Now try assigning control points more distant from already tried points; occasionally this beats the prior method.
				reset_control_point_choices(segment, rotation_selections, translation_selections);

				uint32_t step = segment.num_samples / 2;
				while (step >= 1)
				{
					for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); sample_index += step)
					{
						try_control_points_at(allocator, skeleton, segment, sample_index, rotation_selections, translation_selections,
							error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_selections, modified_translation_selections);

						rewind_to_first_affected_sample(modified_rotation_selections, modified_translation_selections, sample_index);
					}

					step >>= 1;
				}

				uint32_t size_with_maximally_distant_selection = get_animated_data_size(segment, rotation_selections, translation_selections);

				if (size_with_maximally_distant_selection > size_with_sequential_selection)
				{
					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						if (rotation_selections[bone_index] != nullptr)
							rotation_selections[bone_index]->restore_state();

						if (translation_selections[bone_index] != nullptr)
							translation_selections[bone_index]->restore_state();
					}
				}

				deallocate_type_array(allocator, error_per_bone, segment.num_bones);
				deallocate_type_array(allocator, error_per_stream, segment.num_bones);
				deallocate_type_array(allocator, raw_local_pose, segment.num_bones);
				deallocate_type_array(allocator, lossy_local_pose, segment.num_bones);
			}

			inline void write_segment_headers(const ClipContext& clip_context, const CompressionSettings& settings, SegmentHeader* segment_headers, uint16_t segment_headers_start_offset)
			{
				uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format);

				uint32_t data_offset = segment_headers_start_offset;
				for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
				{
					const SegmentContext& segment = clip_context.segments[segment_index];
					SegmentHeader& header = segment_headers[segment_index];

					header.num_samples = segment.num_samples;
					header.animated_pose_bit_size = segment.animated_pose_bit_size;
					header.format_per_track_data_offset = data_offset;
					header.range_data_offset = align_to(header.format_per_track_data_offset + format_per_track_data_size, 2);		// Aligned to 2 bytes
					header.track_data_offset = align_to(header.range_data_offset + segment.range_data_size, 4);						// Aligned to 4 bytes

					data_offset = header.track_data_offset + segment.animated_data_size;
				}
			}

			inline void write_segment_data(const ClipContext& clip_context, const CompressionSettings& settings, ClipHeader& header)
			{
				SegmentHeader* segment_headers = header.get_segment_headers();
				uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format);

				for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
				{
					const SegmentContext& segment = clip_context.segments[segment_index];
					SegmentHeader& segment_header = segment_headers[segment_index];

					if (format_per_track_data_size > 0)
						write_format_per_track_data(segment.bone_streams, segment.num_bones, header.get_format_per_track_data(segment_header), format_per_track_data_size);
					else
						segment_header.format_per_track_data_offset = InvalidPtrOffset();

					if (segment.range_data_size > 0)
						write_segment_range_data(segment, settings.range_reduction, header.get_segment_range_data(segment_header), segment.range_data_size);
					else
						segment_header.range_data_offset = InvalidPtrOffset();

					if (segment.animated_data_size > 0)
						write_animated_track_data(segment, settings.rotation_format, settings.translation_format, header.get_track_data(segment_header), segment.animated_data_size);
					else
						segment_header.track_data_offset = InvalidPtrOffset();
				}
			}

			struct ControlPoint
			{
				uint32_t sample_index;
				Vector4_32 value;
				float knot;
			};

#if false
			inline void write_animated_track_data(const TrackSteamEncoder& encoder, int32_t first_sample_index, uint32_t num_samples, uint32_t control_point_flags_size,
				uint8_t* animated_track_data, ControlPoint& out_control_point, uint32_t*& out_control_point_flags)
			{
				// Frame format:
				// 32 bit packed structure:
				//  top two bits - indicates rotation (00) translation (01) or scale (10)
				//  15 bits - offset to start of previous frame
				//  15 bits - offset to start of next frame
				// uint32_t - sample index
				// bitset - one bit per bone, each bit indicates if data follows for that bone or not
				// list of N quats/vectors in appropriate format, possibly packed.
				// padding to meet 4 byte alignment for next frame.
				// 

				ControlPoint current = out_control_point;
				const ControlPoint last = current;
				
				if (sample_index == first_sample_index)
				{
					current.sample_index = sample_index;
					current.value = encoder.get_left_auxiliary_control_point(sample_index);
					current.knot = 0.0;
				}
				else if (sample_index < 0)
				{
					current.sample_index = sample_index;
					current.value = encoder.get_left_auxiliary_control_point(sample_index));
					current.knot = calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
				}
				else if (sample_index >= num_samples)
				{
					current.sample_index = sample_index;
					current.value = encoder.get_right_auxiliary_control_point(sample_index);
					current.knot = calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
				}
				else if (!encoder.removed_sample(sample_index))
				{
					current.sample_index = sample_index;
					current.value = encoder.get_sample(sample_index);
					current.knot = calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
				}

				if (current.sample_index == sample_index)
				{
					if (0 <= sample_index && sample_index < num_samples && out_control_point_flags == nullptr)
					{
						AnimationTrackType8 frame_type = AnimationTrackType8::Rotation;
						memcpy(animated_track_data, &frame_type, sizeof(frame_type));
						animated_track_data += sizeof(frame_type);

						memcpy(animated_track_data, &sample_index, sizeof(sample_index));
						animated_track_data += sizeof(sample_index);

						// TODO: align to 4 bytes here, and in size calculation function

						out_control_point_flags = safe_ptr_cast<uint32_t, uint8_t*>(animated_track_data);
						animated_track_data += control_point_flags_size;

						bitset_reset(out_control_point_flags, control_point_flags_size, false);
					}

					if (out_control_point_flags != nullptr)
						bitset_set(out_control_point_flags, control_point_flags_size, bone_index, true);

					float delta = current.knot - last.knot;
					memcpy(animated_track_data, &delta, sizeof(delta));
					animated_track_data += sizeof(delta);

					// TODO: write the quantized auxiliary control point or the true bone stream one
					// currently this tries to read from out of bounds

					// TODO: use either maximum or configured format for the auxiliary control points -
					// if variable selected, use max resolution
					// Does variable mean different rate for each sample, or for the entire bone?
					
					// TODO: use std::functions here?
					const uint8_t* rotation_ptr = bone.rotations.get_raw_sample_ptr(sample_index);
					uint32_t sample_size = bone.rotations.get_sample_size();
					memcpy(animated_track_data, rotation_ptr, sample_size);
					animated_track_data += sample_size;
				}

				out_control_point = current;
			}

			inline void write_animated_track_data(Allocator& allocator, const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders,
				uint8_t* animated_track_data, uint32_t animated_data_size)
			{
				const uint8_t* animated_track_data_end = add_offset_to_ptr<uint8_t>(animated_track_data, animated_data_size);

				ControlPoint* rotation_control_points = allocate_type_array<ControlPoint>(allocator, segment.num_bones);
				ControlPoint* translation_control_points = allocate_type_array<ControlPoint>(allocator, segment.num_bones);

				uint32_t control_point_flags_size = get_bitset_size(segment.num_bones);

				int32_t first_sample_index, last_sample_index;
				get_output_sample_index_range(segment, first_sample_index, last_sample_index);

				for (int32_t sample_index = first_sample_index; sample_index <= last_sample_index; ++sample_index)
				{
					uint32_t* rotation_control_point_flags = nullptr;
					uint32_t* translation_control_point_flags = nullptr;

					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						const TrackStreamEncoder* rotation_encoder = rotation_encoders[bone_index];
						if (rotation_encoder != nullptr)
						{
							write_animated_track_data(rotation_encoder, first_sample_index, segment.num_samples, control_point_flags_size,
								animated_track_data, rotation_control_points[bone_index], rotation_control_point_flags);
							ACL_ENSURE(animated_track_data <= animated_track_data_end, "Invalid animated track data offset. Wrote too much data.");
						}

						const TrackStreamEncoder* translation_encoder = translation_encoders[bone_index];
						if (translation_encoder != nullptr)
						{
							write_animated_track_data(translation_encoder, first_sample_index, segment.num_samples, control_point_flags_size,
								animated_track_data, translation_control_points[bone_index], translation_control_point_flags);
							ACL_ENSURE(animated_track_data <= animated_track_data_end, "Invalid animated track data offset. Wrote too much data.");
						}
					}
				}

				ACL_ENSURE(animated_track_data == animated_track_data_end, "Invalid animated track data offset. Wrote too little data.");

				deallocate_type_array(allocator, rotation_control_points, segment.num_bones);
				deallocate_type_array(allocator, translation_control_points, segment.num_bones);
			}
#endif
		}

		// Encoder entry point
		inline CompressedClip* compress_clip(Allocator& allocator, const AnimationClip& clip, const RigidSkeleton& skeleton, const CompressionSettings& settings)
		{
			using namespace impl;

			uint16_t num_bones = clip.get_num_bones();
			uint32_t num_samples = clip.get_num_samples();

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
			initialize_clip_context(allocator, clip, raw_clip_context);

			ClipContext clip_context;
			initialize_clip_context(allocator, clip, clip_context);

			convert_rotation_streams(allocator, clip_context, settings.rotation_format);

			// Extract our clip ranges now, we need it for compacting the constant streams
			extract_clip_bone_ranges(allocator, clip_context);

			// TODO: Expose this, especially the translation threshold depends on the unit scale.
			// Centimeters VS meters, a different threshold should be used. Perhaps we should pass an
			// argument to the compression algorithm that states the units used or we should force centimeters
			compact_constant_streams(allocator, clip_context, 0.00001f, 0.001f);

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

			quantize_streams(allocator, clip_context, settings.rotation_format, settings.translation_format, clip, skeleton, raw_clip_context);

			uint32_t num_selections = clip_context.num_segments * clip_context.num_bones;
			Selections** rotation_selections = allocate_type_array<Selections*>(allocator, num_selections);
			Selections** translation_selections = allocate_type_array<Selections*>(allocator, num_selections);

			for (uint32_t selections_index = 0; selections_index < num_selections; ++selections_index)
			{
				rotation_selections[selections_index] = nullptr;
				translation_selections[selections_index] = nullptr;
			}

			for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
			{
				SegmentContext& segment = clip_context.segments[segment_index];

				Selections** segment_rotation_selections = rotation_selections + segment_index * clip_context.num_bones;
				Selections** segment_translation_selections = translation_selections + segment_index * clip_context.num_bones;

				for (uint16_t bone_index = 0; bone_index < clip_context.num_bones; ++bone_index)
				{
					const BoneStreams& bone_streams = segment.bone_streams[bone_index];

					if (bone_streams.is_rotation_animated())
						segment_rotation_selections[bone_index] = allocate_type<Selections>(allocator, allocator, segment.num_samples);
					
					if (bone_streams.is_translation_animated())
						segment_translation_selections[bone_index] = allocate_type<Selections>(allocator, allocator, segment.num_samples);
				}

				choose_control_points(allocator, skeleton, segment, segment_rotation_selections, segment_translation_selections);
			}

			const SegmentContext& clip_segment = clip_context.segments[0];

			uint32_t constant_data_size = get_constant_data_size(clip_context);

			for (SegmentContext& segment : clip_context.segment_iterator())
				segment.animated_data_size = get_animated_data_size(segment, rotation_selections, translation_selections);

			uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format);

			uint32_t bitset_size = get_bitset_size(num_bones * Constants::NUM_TRACKS_PER_BONE);

			uint32_t buffer_size = 0;

			// Per clip data
			buffer_size += sizeof(CompressedClip);
			buffer_size += sizeof(ClipHeader);
			buffer_size += sizeof(SegmentHeader) * clip_context.num_segments;	// Segment headers
			buffer_size += sizeof(uint32_t) * bitset_size;		// Default tracks bitset
			buffer_size += sizeof(uint32_t) * bitset_size;		// Constant tracks bitset
			buffer_size = align_to(buffer_size, 4);				// Align constant track data
			buffer_size += constant_data_size;					// Constant track data
			buffer_size = align_to(buffer_size, 4);				// Align range data
			buffer_size += clip_range_data_size;				// Range data

			// Per segment data
			for (const SegmentContext& segment : clip_context.segment_iterator())
			{
				buffer_size += format_per_track_data_size;			// Format per track data
				buffer_size = align_to(buffer_size, 2);				// Align range data
				buffer_size += segment.range_data_size;				// Range data
				buffer_size = align_to(buffer_size, 4);				// Align animated data
				buffer_size += segment.animated_data_size;			// Animated track data
			}

			printf("total size %d\n", buffer_size);

#if false
			uint8_t* buffer = allocate_type_array_aligned<uint8_t>(allocator, buffer_size, 16);

			CompressedClip* compressed_clip = make_compressed_clip(buffer, buffer_size, AlgorithmType8::UniformlySampled);

			Header& header = get_header(*compressed_clip);
			header.num_bones = num_bones;
			header.num_segments = clip_context.num_segments;
			header.rotation_format = settings.rotation_format;
			header.translation_format = settings.translation_format;
			header.clip_range_reduction = settings.range_reduction;
			header.segment_range_reduction = settings.segmenting.range_reduction;
			header.num_samples = num_samples;
			header.sample_rate = clip.get_sample_rate();
			header.segment_headers_offset = sizeof(FullPrecisionHeader);
			header.default_tracks_bitset_offset = header.segment_headers_offset + (sizeof(SegmentHeader) * clip_context.num_segments);
			header.constant_tracks_bitset_offset = header.default_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size);
			header.constant_track_data_offset = align_to(header.constant_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size), 4);	// Aligned to 4 bytes
			header.clip_range_data_offset = align_to(header.constant_track_data_offset + constant_data_size, 4);						// Aligned to 4 bytes

			uint16_t segment_headers_start_offset = header.clip_range_data_offset + clip_range_data_size;
			impl::write_segment_headers(clip_context, settings, header.get_segment_headers(), segment_headers_start_offset);

			write_default_track_bitset(clip_context, header.get_default_tracks_bitset(), bitset_size);
			write_constant_track_bitset(clip_context, header.get_constant_tracks_bitset(), bitset_size);

			if (constant_data_size > 0)
				write_constant_track_data(clip_context, header.get_constant_track_data(), constant_data_size);
			else
				header.constant_track_data_offset = InvalidPtrOffset();

			if (settings.range_reduction != RangeReductionFlags8::None)
				write_clip_range_data(clip_segment, settings.range_reduction, header.get_clip_range_data(), clip_range_data_size);
			else
				header.clip_range_data_offset = InvalidPtrOffset();

			write_segment_data(clip_context, settings, header);

			finalize_compressed_clip(*compressed_clip);
#endif

			for (uint32_t selections_index = 0; selections_index < num_selections; ++selections_index)
			{
				deallocate_type(allocator, rotation_selections[selections_index]);
				deallocate_type(allocator, translation_selections[selections_index]);
			}

			deallocate_type_array(allocator, rotation_selections, num_selections);
			deallocate_type_array(allocator, translation_selections, num_selections);

			destroy_clip_context(allocator, clip_context);
			destroy_clip_context(allocator, raw_clip_context);

#if false
			return compressed_clip;
#else
			return nullptr;
#endif
		}

		void print_stats(const CompressedClip& clip, std::FILE* file, const CompressionSettings& settings)
		{
			using namespace impl;

			const ClipHeader& header = get_clip_header(clip);

			uint32_t num_tracks = header.num_bones * Constants::NUM_TRACKS_PER_BONE;
			uint32_t bitset_size = get_bitset_size(num_tracks);

			uint32_t num_default_tracks = bitset_count_set_bits(header.get_default_tracks_bitset(), bitset_size);
			uint32_t num_constant_tracks = bitset_count_set_bits(header.get_constant_tracks_bitset(), bitset_size);
			uint32_t num_animated_tracks = num_tracks - num_default_tracks - num_constant_tracks;

			fprintf(file, "Clip rotation format: %s\n", get_rotation_format_name(header.rotation_format));
			fprintf(file, "Clip translation format: %s\n", get_vector_format_name(header.translation_format));
			fprintf(file, "Clip clip range reduction: %s\n", get_range_reduction_name(header.clip_range_reduction));
			fprintf(file, "Clip segment range reduction: %s\n", get_range_reduction_name(header.segment_range_reduction));
			fprintf(file, "Clip num default tracks: %u\n", num_default_tracks);
			fprintf(file, "Clip num constant tracks: %u\n", num_constant_tracks);
			fprintf(file, "Clip num animated tracks: %u\n", num_animated_tracks);
			fprintf(file, "Clip num segments: %u\n", header.num_segments);

			if (settings.segmenting.enabled)
			{
				fprintf(file, "Clip segmenting ideal num samples: %u\n", settings.segmenting.ideal_num_samples);
				fprintf(file, "Clip segmenting max num samples: %u\n", settings.segmenting.max_num_samples);
			}
		}
	}
}
