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
			Vector4_32 sample(const BoneStreams& bone_streams, AnimationTrackType8 track_type, int32_t sample_index)
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

			class TrackStreamEncoder
			{
			public:
				TrackStreamEncoder(Allocator& allocator, const BoneStreams& bone_streams, AnimationTrackType8 track_type, uint32_t num_samples, float duration)
					: m_allocator(allocator)
					, m_bone_streams(bone_streams)
					, m_track_type(track_type)
					, m_num_samples(num_samples)
					, m_duration(duration)
				{
					m_remove_size = get_bitset_size(m_num_samples);
					m_remove = allocate_type_array<uint32_t>(m_allocator, m_remove_size);
					m_remove_backup = allocate_type_array<uint32_t>(m_allocator, m_remove_size);
				}

				~TrackStreamEncoder()
				{
					deallocate_type_array(m_allocator, m_remove, m_remove_size);
					deallocate_type_array(m_allocator, m_remove_backup, m_remove_size);
				}

				bool removed_sample(uint32_t sample_index) const { return bitset_test(m_remove, m_remove_size, sample_index); }
				void remove_all_samples() { bitset_reset(m_remove, m_remove_size, true); }
				void remove_sample(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, true); }
				void keep_sample(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, false); }

				void save_state() { std::memcpy(m_remove_backup, m_remove, m_remove_size * sizeof(uint32_t)); }
				void restore_state() { std::memcpy(m_remove, m_remove_backup, m_remove_size * sizeof(uint32_t)); }

				Vector4_32 interpolate(uint32_t at_sample_index) const
				{
					if (!removed_sample(at_sample_index))
					{
						return sample(m_bone_streams, m_track_type, at_sample_index);
					}

					Vector4_32 values[POLYNOMIAL_ORDER + 1];
					uint32_t sample_indices[POLYNOMIAL_ORDER + 1];
					float sample_times[POLYNOMIAL_ORDER + 1];

					Vector4_32 value;
					uint32_t sample_index = at_sample_index;

					for (int8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX; control_point_index >= 0; --control_point_index)
					{
						find_left_control_point(value, sample_index);

						values[control_point_index] = value;
						sample_indices[control_point_index] = sample_index;
						sample_times[control_point_index] = m_duration * float(sample_index) / float(m_num_samples - 1);
					}

					sample_index = at_sample_index;

					for (uint8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX + 1; control_point_index <= POLYNOMIAL_ORDER; ++control_point_index)
					{
						find_right_control_point(value, sample_index);

						values[control_point_index] = value;
						sample_indices[control_point_index] = sample_index;
						sample_times[control_point_index] = m_duration * float(sample_index) / float(m_num_samples - 1);
					}

					float knots[POLYNOMIAL_ORDER + 1];
					calculate_knots(values, sample_indices, knots);

					return interpolate_spline(values, knots, sample_times, m_duration * float(at_sample_index) / float(m_num_samples - 1));
				}

				uint32_t get_num_control_points() const
				{
					uint32_t result = 0;

					for (uint32_t sample_index = 0; sample_index < m_num_samples; ++sample_index)
						if (!removed_sample(sample_index))
							++result;

					return result;
				}

			private:
				Allocator& m_allocator;

				const BoneStreams& m_bone_streams;
				AnimationTrackType8 m_track_type;

				uint32_t m_num_samples;
				float m_duration;

				uint32_t* m_remove;
				uint32_t* m_remove_backup;
				uint32_t m_remove_size;

				void find_left_control_point(Vector4_32& out_value, uint32_t& out_sample_index) const
				{
					while (true)
					{
						--out_sample_index;

						if (!removed_sample(out_sample_index))
						{
							out_value = sample(m_bone_streams, m_track_type, out_sample_index);
							break;
						}
					}
				}

				void find_right_control_point(Vector4_32& out_value, uint32_t& out_sample_index) const
				{
					while (true)
					{
						++out_sample_index;

						if (!removed_sample(out_sample_index))
						{
							out_value = sample(m_bone_streams, m_track_type, out_sample_index);
							break;
						}
					}
				}
			};

			// Each segment must include control points outside the duration of the sample to calculate the interpolation polynomial near
			// its start and end.  The auxiliary points are taken from the original (ie. unsegmented) clip to better approximate the overall
			// clip and to ensure the tangents are correct when there is a discontinuity at a segment boundary.
			void extend_segment_with_auxiliary_control_points(Allocator& allocator, SegmentContext& segment, const ClipContext& raw_clip_context)
			{
				ACL_ASSERT(raw_clip_context.num_segments == 1, "Raw clip cannot be segmented");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& raw_bone_streams = raw_clip_context.segments[0].bone_streams[bone_index];
					BoneStreams& bone_streams = segment.bone_streams[bone_index];
					BoneRanges* bone_ranges = segment.ranges == nullptr ? nullptr : segment.ranges + bone_index;

					Vector4_32 prefixes[NUM_LEFT_AUXILIARY_POINTS];
					Vector4_32 suffixes[NUM_RIGHT_AUXILIARY_POINTS];

					if (bone_streams.is_rotation_animated())
					{
						// Reflect across the first sample to create an auxiliary control point beyond the clip that will ensure a reasonable interpolation near time 0.
						for (int32_t prefix_index = 0; prefix_index < NUM_LEFT_AUXILIARY_POINTS; ++prefix_index)
							prefixes[prefix_index] = sample(raw_bone_streams, AnimationTrackType8::Rotation, static_cast<int32_t>(segment.clip_sample_offset) - NUM_LEFT_AUXILIARY_POINTS + prefix_index);
						// todo: proper negative indices passed above?

						for (uint32_t suffix_index = 0; suffix_index < NUM_RIGHT_AUXILIARY_POINTS; ++suffix_index)
							suffixes[suffix_index] = sample(raw_bone_streams, AnimationTrackType8::Rotation, segment.clip_sample_offset + segment.num_samples + suffix_index);

						extend_rotation_stream(allocator, bone_streams, bone_ranges, prefixes, NUM_LEFT_AUXILIARY_POINTS, suffixes, NUM_RIGHT_AUXILIARY_POINTS);
					}

					if (bone_streams.is_translation_animated())
					{
						for (int32_t prefix_index = 0; prefix_index < NUM_LEFT_AUXILIARY_POINTS; ++prefix_index)
							prefixes[prefix_index] = sample(bone_streams, AnimationTrackType8::Translation, static_cast<int32_t>(segment.clip_sample_offset) - NUM_LEFT_AUXILIARY_POINTS + prefix_index);

						for (uint32_t suffix_index = 0; suffix_index < NUM_RIGHT_AUXILIARY_POINTS; ++suffix_index)
							suffixes[suffix_index] = sample(bone_streams, AnimationTrackType8::Translation, segment.clip_sample_offset + segment.num_samples + suffix_index);

						extend_translation_stream(allocator, bone_streams, bone_ranges, prefixes, NUM_LEFT_AUXILIARY_POINTS, suffixes, NUM_RIGHT_AUXILIARY_POINTS);
					}
				}
			}

			uint32_t get_animated_data_size(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders)
			{
				uint32_t animated_data_size = 0;

				for (uint32_t sample_index = 0; sample_index < segment.num_samples; ++sample_index)
				{
					uint32_t num_rotation_bits = 0;
					uint32_t num_translation_bits = 0;

					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						const BoneStreams& bone_streams = segment.bone_streams[bone_index];

						const TrackStreamEncoder* rotation_encoder = rotation_encoders[bone_index];
						if (rotation_encoder != nullptr)
						{
							if (!rotation_encoder->removed_sample(sample_index))
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
						}

						const TrackStreamEncoder* translation_encoder = translation_encoders[bone_index];
						if (translation_encoder != nullptr)
						{
							if (!translation_encoder->removed_sample(sample_index))
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
					}

					num_rotation_bits += num_rotation_bits % 32;
					num_translation_bits += num_translation_bits % 32;
					animated_data_size += num_rotation_bits / 8 + num_translation_bits / 8;
				}

				return animated_data_size;
			}

			void interpolate_pose(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders,
				uint32_t sample_index, Transform_32* out_local_pose)
			{
				ACL_ASSERT(NUM_LEFT_AUXILIARY_POINTS <= sample_index && sample_index <= segment.num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS, "sample_index is out of range");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& bone_streams = segment.bone_streams[bone_index];

					Quat_32 rotation;
					if (rotation_encoders[bone_index] != nullptr)
						rotation = quat_normalize(rotation_encoders[bone_index]->interpolate(sample_index));
					else if (bone_streams.is_rotation_constant)
						rotation = get_rotation_sample(bone_streams, 0);
					else
						rotation = quat_identity_32();

					Vector4_32 translation;
					if (translation_encoders[bone_index] != nullptr)
						translation = translation_encoders[bone_index]->interpolate(sample_index);
					else if (bone_streams.is_translation_constant)
						translation = get_translation_sample(bone_streams, 0);
					else
						translation = vector_zero_32();

					out_local_pose[bone_index] = transform_set(rotation, translation);
				}
			}

			void reset_control_point_choices(const SegmentContext& segment, TrackStreamEncoder** rotation_encoders, TrackStreamEncoder** translation_encoders)
			{
				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					TrackStreamEncoder* rotation_encoder = rotation_encoders[bone_index];
					if (rotation_encoder != nullptr)
					{
						rotation_encoder->remove_all_samples();

						for (uint32_t sample_index = 0; sample_index <= NUM_LEFT_AUXILIARY_POINTS; ++sample_index)
							rotation_encoder->keep_sample(sample_index);

						for (uint32_t sample_index = segment.num_samples - 1; sample_index >= segment.num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS; --sample_index)
							rotation_encoder->keep_sample(sample_index);
					}

					TrackStreamEncoder* translation_encoder = translation_encoders[bone_index];
					if (translation_encoder != nullptr)
					{
						translation_encoder->remove_all_samples();

						for (uint32_t sample_index = 0; sample_index <= NUM_LEFT_AUXILIARY_POINTS; ++sample_index)
							translation_encoder->keep_sample(sample_index);

						for (uint32_t sample_index = segment.num_samples - 1; sample_index >= segment.num_samples - 1 - NUM_RIGHT_AUXILIARY_POINTS; --sample_index)
							translation_encoder->keep_sample(sample_index);
					}
				}
			}

			void try_control_points_at(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				uint32_t sample_index, TrackStreamEncoder** rotation_encoders, TrackStreamEncoder** translation_encoders,
				float* error_per_bone, BoneTrackError* error_per_stream, Transform_32* raw_local_pose, Transform_32* lossy_local_pose,
				TrackStreamEncoder*& out_modified_rotation_encoder, TrackStreamEncoder*& out_modified_translation_encoder)
			{
				sample_streams(segment.bone_streams, segment.num_bones, sample_index, raw_local_pose);
				interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);

				double error = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose, error_per_bone);

				if (error <= segment.clip->error_threshold)
				{
					out_modified_rotation_encoder = nullptr;
					out_modified_translation_encoder = nullptr;
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
					bool rotation_is_interpolated = rotation_encoders[bone_index] != nullptr && rotation_encoders[bone_index]->removed_sample(sample_index);
					bool translation_is_interpolated = translation_encoders[bone_index] != nullptr && translation_encoders[bone_index]->removed_sample(sample_index);

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

				TrackStreamEncoder* bad_bone_rotations = rotation_encoders[bad_bone_index];
				bool old_removed_rotation = bad_bone_rotations != nullptr && bad_bone_rotations->removed_sample(sample_index);

				TrackStreamEncoder* bad_bone_translations = translation_encoders[bad_bone_index];
				bool old_removed_translation = bad_bone_translations != nullptr && bad_bone_translations->removed_sample(sample_index);

				if (bad_bone_rotations != nullptr && bad_bone_rotations->removed_sample(sample_index))
				{
					if (bad_bone_translations == nullptr || !bad_bone_translations->removed_sample(sample_index))
					{
						bad_bone_rotations->keep_sample(sample_index);
					}
					else
					{
						bad_bone_rotations->keep_sample(sample_index);

						interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);
						double error_without_interpolated_rotation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

						if (error_without_interpolated_rotation > segment.clip->error_threshold)
						{
							bad_bone_rotations->remove_sample(sample_index);
							bad_bone_translations->keep_sample(sample_index);

							interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);
							double error_without_interpolated_translation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

							if (error_without_interpolated_translation > error_without_interpolated_rotation)
							{
								bad_bone_rotations->keep_sample(sample_index);
								bad_bone_translations->remove_sample(sample_index);
							}
						}
					}
				}
				else
				{
					bad_bone_translations->keep_sample(sample_index);
				}

				bool changed_rotation = bad_bone_rotations != nullptr && old_removed_rotation != bad_bone_rotations->removed_sample(sample_index);
				out_modified_rotation_encoder = changed_rotation ? bad_bone_rotations : nullptr;

				bool changed_translation = bad_bone_translations != nullptr && old_removed_translation != bad_bone_translations->removed_sample(sample_index);
				out_modified_translation_encoder = changed_translation ? bad_bone_translations : nullptr;

				ACL_ASSERT(changed_rotation || changed_translation, "No changes were made to the bone with the worst error contribution");
			}

			void rewind_to_first_affected_sample(const TrackStreamEncoder* modified_rotation_encoder, const TrackStreamEncoder* modified_translation_encoder, uint32_t& out_sample_index)
			{
				if (modified_rotation_encoder == nullptr && modified_translation_encoder == nullptr)
					return;

				uint32_t num_rotation_points = 0,
					min_rotation_points = modified_rotation_encoder == nullptr ? 0 : POLYNOMIAL_ORDER,
					num_translation_points = 0,
					min_translation_points = modified_translation_encoder == nullptr ? 0 : POLYNOMIAL_ORDER;

				while (out_sample_index > NUM_LEFT_AUXILIARY_POINTS + 1 &&
					(num_rotation_points < min_rotation_points || min_rotation_points == 0) &&
					(num_translation_points <= min_translation_points || min_translation_points == 0))
				{
					if (modified_rotation_encoder != nullptr && !modified_rotation_encoder->removed_sample(out_sample_index))
						++num_rotation_points;

					if (modified_translation_encoder != nullptr && !modified_translation_encoder->removed_sample(out_sample_index))
						++num_translation_points;

					--out_sample_index;
				}
			}

			void choose_control_points(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				TrackStreamEncoder** rotation_encoders, TrackStreamEncoder** translation_encoders)
			{
				float* error_per_bone = allocate_type_array<float>(allocator, segment.num_bones);
				BoneTrackError* error_per_stream = allocate_type_array<BoneTrackError>(allocator, segment.num_bones);
				Transform_32* raw_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);
				Transform_32* lossy_local_pose = allocate_type_array<Transform_32>(allocator, segment.num_bones);

				TrackStreamEncoder* modified_rotation_encoder;
				TrackStreamEncoder* modified_translation_encoder;

				// Try assigning control points sequentially.  Most of the time this will provide a better compression ratio.
				reset_control_point_choices(segment, rotation_encoders, translation_encoders);

				for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); ++sample_index)
				{
					try_control_points_at(allocator, skeleton, segment, sample_index, rotation_encoders, translation_encoders,
						error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_encoder, modified_translation_encoder);

					rewind_to_first_affected_sample(modified_rotation_encoder, modified_translation_encoder, sample_index);
				}

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (rotation_encoders[bone_index] != nullptr)
						rotation_encoders[bone_index]->save_state();

					if (translation_encoders[bone_index] != nullptr)
						translation_encoders[bone_index]->save_state();
				}

				uint32_t size_with_sequential_selection = get_animated_data_size(segment, rotation_encoders, translation_encoders);

				// Now try assigning control points more distant from already tried points; occasionally this beats the prior method.
				reset_control_point_choices(segment, rotation_encoders, translation_encoders);

				uint32_t step = segment.num_samples / 2;
				while (step >= 1)
				{
					for (uint32_t sample_index = NUM_LEFT_AUXILIARY_POINTS + 1; sample_index <= segment.num_samples - 1 - (NUM_RIGHT_AUXILIARY_POINTS + 1); sample_index += step)
					{
						try_control_points_at(allocator, skeleton, segment, sample_index, rotation_encoders, translation_encoders,
							error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose, modified_rotation_encoder, modified_translation_encoder);

						rewind_to_first_affected_sample(modified_rotation_encoder, modified_translation_encoder, sample_index);
					}

					step >>= 1;
				}

				uint32_t size_with_maximally_distant_selection = get_animated_data_size(segment, rotation_encoders, translation_encoders);

				if (size_with_maximally_distant_selection > size_with_sequential_selection)
				{
					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						if (rotation_encoders[bone_index] != nullptr)
							rotation_encoders[bone_index]->restore_state();

						if (translation_encoders[bone_index] != nullptr)
							translation_encoders[bone_index]->restore_state();
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
			void write_animated_track_data(const TrackSteamEncoder& encoder, int32_t first_sample_index, uint32_t num_samples, uint32_t control_point_flags_size,
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

			void write_animated_track_data(Allocator& allocator, const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders,
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
				if (ACL_TRY_ASSERT(are_enum_flags_set(settings.range_reduction, RangeReductionFlags8::PerClip | RangeReductionFlags8::Translations), "Translation quantization requires range reduction to be enabled!"))
					return nullptr;
			}

			if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerSegment))
			{
				if (ACL_TRY_ASSERT(is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerClip), "Per segment range reduction requires per clip range reduction to be enabled!"))
					return nullptr;

				if (ACL_TRY_ASSERT(settings.segmenting.enabled, "Per segment range reduction requires segmenting to be enabled!"))
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

			uint32_t clip_range_data_size = 0;
			if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerClip))
			{
				normalize_clip_streams(clip_context, settings.range_reduction);
				clip_range_data_size = get_stream_range_data_size(clip_context, settings.range_reduction, settings.rotation_format, settings.translation_format);
			}

			if (settings.segmenting.enabled)
			{
				segment_streams(allocator, clip_context, settings.segmenting);

				if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerSegment))
				{
					extract_segment_bone_ranges(allocator, clip_context);
					normalize_segment_streams(clip_context, settings.range_reduction);
				}
			}

			// TODO: dump a test bone and validate the results vs the source data
			// use translations so easier to check

			// segmenting results are same with it enabled and not enabled??
			for (SegmentContext& segment : clip_context.segment_iterator())
				impl::extend_segment_with_auxiliary_control_points(allocator, segment, raw_clip_context);

			quantize_streams(allocator, clip_context, settings.rotation_format, settings.translation_format, clip, skeleton, raw_clip_context);

			uint32_t num_track_stream_encoders = clip_context.num_segments * clip_context.num_bones;
			TrackStreamEncoder** rotation_encoders = allocate_type_array<TrackStreamEncoder*>(allocator, num_track_stream_encoders);
			TrackStreamEncoder** translation_encoders = allocate_type_array<TrackStreamEncoder*>(allocator, num_track_stream_encoders);

			for (uint32_t encoder_index = 0; encoder_index < num_track_stream_encoders; ++encoder_index)
			{
				rotation_encoders[encoder_index] = nullptr;
				translation_encoders[encoder_index] = nullptr;
			}

			for (uint16_t segment_index = 0; segment_index < clip_context.num_segments; ++segment_index)
			{
				SegmentContext& segment = clip_context.segments[segment_index];

				TrackStreamEncoder** segment_rotation_encoders = rotation_encoders + segment_index * clip_context.num_bones;
				TrackStreamEncoder** segment_translation_encoders = translation_encoders + segment_index * clip_context.num_bones;

				for (uint16_t bone_index = 0; bone_index < clip_context.num_bones; ++bone_index)
				{
					const BoneStreams& bone_streams = segment.bone_streams[bone_index];
					float segment_duration = float(segment.num_samples) / float(clip_context.sample_rate);

					if (bone_streams.is_rotation_animated())
					{
						segment_rotation_encoders[bone_index] = allocate_type<TrackStreamEncoder>(allocator,
							allocator, bone_streams, AnimationTrackType8::Rotation, segment.num_samples, segment_duration);
					}
					
					if (bone_streams.is_translation_animated())
					{
						segment_translation_encoders[bone_index] = allocate_type<TrackStreamEncoder>(allocator,
							allocator, bone_streams, AnimationTrackType8::Translation, segment.num_samples, segment_duration);
					}
				}

				choose_control_points(allocator, skeleton, segment, segment_rotation_encoders, segment_translation_encoders);
			}

			const SegmentContext& clip_segment = clip_context.segments[0];

			uint32_t constant_data_size = get_constant_data_size(clip_context);

			for (SegmentContext& segment : clip_context.segment_iterator())
				segment.animated_data_size = get_animated_data_size(segment, rotation_encoders, translation_encoders);

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
			header.range_reduction = settings.range_reduction;
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

			if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerClip))
				write_clip_range_data(clip_segment, settings.range_reduction, header.get_clip_range_data(), clip_range_data_size);
			else
				header.clip_range_data_offset = InvalidPtrOffset();

			write_segment_data(clip_context, settings, header);

			finalize_compressed_clip(*compressed_clip);
#endif

			for (uint32_t encoder_index = 0; encoder_index < num_track_stream_encoders; ++encoder_index)
			{
				deallocate_type(allocator, rotation_encoders[encoder_index]);
				deallocate_type(allocator, translation_encoders[encoder_index]);
			}

			deallocate_type_array(allocator, rotation_encoders, num_track_stream_encoders);
			deallocate_type_array(allocator, translation_encoders, num_track_stream_encoders);

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
			fprintf(file, "Clip range reduction: %s\n", get_range_reduction_name(header.range_reduction));
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
