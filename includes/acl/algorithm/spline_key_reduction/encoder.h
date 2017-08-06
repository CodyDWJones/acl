#pragma once

#define SIXTY_FOUR_BIT 0
#define LINEAR_SEQUENCE 0
#define SAMPLE_INDEX_PER_CONTROL_POINT 0
#define SEPARATE_TRANSLATION_PASS 0

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
#include "acl/compression/stream/track_stream.h"
#include "acl/compression/stream/convert_clip_to_streams.h"
#include "acl/compression/stream/convert_rotation_streams.h"
#include "acl/compression/stream/compact_constant_streams.h"
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

namespace acl
{
	namespace spline_key_reduction
	{
		struct CompressionSettings
		{
			RotationFormat8 rotation_format;
			VectorFormat8 translation_format;

			RangeReductionFlags8 range_reduction;

			CompressionSettings(RotationFormat8 rotation_format, VectorFormat8 translation_format, RangeReductionFlags8 range_reduction)
				: rotation_format(rotation_format)
				, translation_format(translation_format)
				, range_reduction(range_reduction)
			{
			}

			CompressionSettings()
				: rotation_format(RotationFormat8::Quat_128)
				, translation_format(VectorFormat8::Vector3_96)
				, range_reduction(RangeReductionFlags8::None)
			{}
		};

		namespace impl
		{
			typedef std::function<Vector4_32(uint32_t)> Sampler;

			class TrackStreamEncoder
			{
			public:
				TrackStreamEncoder(Allocator& allocator, uint32_t num_samples, float duration, Sampler get_sample)
					: m_allocator(allocator)
					, m_num_samples(num_samples)
					, m_duration(duration)
					, m_get_sample(get_sample)
				{
					m_remove_size = get_bitset_size(m_num_samples);
					m_remove = allocate_type_array<uint32_t>(m_allocator, m_remove_size);
					bitset_reset(m_remove, m_remove_size, true);
				}

				~TrackStreamEncoder()
				{
					deallocate_type_array(m_allocator, m_remove, m_remove_size);
				}

				bool removed_sample(uint32_t sample_index) const { return bitset_test(m_remove, m_remove_size, sample_index); } 
				void remove_sample(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, true); }
				void keep_sample(uint32_t sample_index) { bitset_set(m_remove, m_remove_size, sample_index, false); }

#if SIXTY_FOUR_BIT
				Vector4_64 interpolate(int32_t at_sample_index) const
				{
					if (!removed_sample(at_sample_index))
					{
						return vector_cast(m_get_sample(at_sample_index));
					}

					Vector4_64 values[POLYNOMIAL_ORDER + 1];
					int32_t sample_indices[POLYNOMIAL_ORDER + 1];
					double sample_times[POLYNOMIAL_ORDER + 1];

					Vector4_64 value;
					int32_t sample_index = at_sample_index;

					for (int8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX; control_point_index >= 0; --control_point_index)
					{
						find_left_control_point(value, sample_index);

						values[control_point_index] = value;
						sample_indices[control_point_index] = sample_index;
						sample_times[control_point_index] = m_duration * double(sample_index) / double(m_num_samples - 1);
					}

					sample_index = at_sample_index;

					for (uint8_t control_point_index = FIRST_INTERPOLATION_KNOT_INDEX + 1; control_point_index <= POLYNOMIAL_ORDER; ++control_point_index)
					{
						find_right_control_point(value, sample_index);

						values[control_point_index] = value;
						sample_indices[control_point_index] = sample_index;
						sample_times[control_point_index] = m_duration * double(sample_index) / double(m_num_samples - 1);
					}

					double knots[POLYNOMIAL_ORDER + 1];
					calculate_knots(values, sample_indices, knots);

					Vector4_64 result = interpolate_spline(values, knots, sample_times, m_duration * double(at_sample_index) / double(m_num_samples - 1));

					return result;
				}
#else
				Vector4_32 interpolate(int32_t at_sample_index) const
				{
					if (!removed_sample(at_sample_index))
					{
						return m_get_sample(at_sample_index);
					}

					Vector4_32 values[POLYNOMIAL_ORDER + 1];
					int32_t sample_indices[POLYNOMIAL_ORDER + 1];
					float sample_times[POLYNOMIAL_ORDER + 1];

					Vector4_32 value;
					int32_t sample_index = at_sample_index;

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
#endif

				uint32_t get_num_control_points() const
				{
					uint32_t result = 0;

					for (uint32_t sample_index = 0; sample_index < m_num_samples; ++sample_index)
						if (!removed_sample(sample_index))
							++result;

					// Include the auxiliary control points that will be required to interpolate near the beginning or end.
					return result + POLYNOMIAL_ORDER - 1;
				}

			private:
				Allocator& m_allocator;

				uint32_t m_num_samples;
				float m_duration;

				Sampler m_get_sample;

				uint32_t* m_remove;
				uint32_t m_remove_size;

#if SIXTY_FOUR_BIT
				Vector4_64 get_left_auxiliary_control_point(int32_t sample_index) const
				{
					// Reflect across the first sample to create an auxiliary control point beyond the clip that will ensure a reasonable interpolation near time 0.
					return vector_lerp(vector_cast(m_get_sample(0)), vector_cast(m_get_sample(-sample_index)), -1.0);
				}

				Vector4_64 get_right_auxiliary_control_point(int32_t sample_index) const
				{
					return vector_lerp(vector_cast(m_get_sample(2 * (m_num_samples - 1) - sample_index)), vector_cast(m_get_sample(m_num_samples - 1)), 2.0);
				}

				void find_left_control_point(Vector4_64& out_value, int32_t& out_sample_index) const
				{
					while (true)
					{
						--out_sample_index;

						if (out_sample_index < 0)
						{
							out_value = get_left_auxiliary_control_point(out_sample_index);
							break;
						}

						if (!removed_sample(out_sample_index))
						{
							out_value = vector_cast(m_get_sample(out_sample_index));
							break;
						}
					}
				}

				void find_right_control_point(Vector4_64& out_value, int32_t& out_sample_index) const
				{
					ACL_ASSERT(out_sample_index >= 0, "Sample index is out of range");

					while (true)
					{
						++out_sample_index;

						if (static_cast<uint32_t>(out_sample_index) >= m_num_samples)
						{
							out_value = get_right_auxiliary_control_point(out_sample_index);
							break;
						}

						if (!removed_sample(out_sample_index))
						{
							out_value = vector_cast(m_get_sample(out_sample_index));
							break;
						}
					}
				}
			};

			void interpolate_pose(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders,
				uint32_t sample_index, Transform_32* out_local_pose)
			{
				ACL_ASSERT(0 <= sample_index && sample_index <= INT32_MAX, "sample_index is out of range");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& bone = segment.bone_streams[bone_index];
					
					Quat_32 rotation = rotation_encoders[bone_index] != nullptr ?
						quat_cast(quat_normalize(vector_to_quat(rotation_encoders[bone_index]->interpolate(sample_index)))) : bone.get_rotation_sample(sample_index);

					Vector4_32 translation = translation_encoders[bone_index] != nullptr ?
						vector_cast(translation_encoders[bone_index]->interpolate(sample_index)) : bone.get_translation_sample(sample_index);

					out_local_pose[bone_index] = transform_set(rotation, translation);
				}
			}
#else
				Vector4_32 get_left_auxiliary_control_point(int32_t sample_index) const
				{
					// Reflect across the first sample to create an auxiliary control point beyond the clip that will ensure a reasonable interpolation near time 0.
					return vector_lerp(m_get_sample(0), m_get_sample(-sample_index), -1.0);
				}

				Vector4_32 get_right_auxiliary_control_point(int32_t sample_index) const
				{
					return vector_lerp(m_get_sample(2 * (m_num_samples - 1) - sample_index), m_get_sample(m_num_samples - 1), 2.0);
				}

				void find_left_control_point(Vector4_32& out_value, int32_t& out_sample_index) const
				{
					while (true)
					{
						--out_sample_index;

						if (out_sample_index < 0)
						{
							out_value = get_left_auxiliary_control_point(out_sample_index);
							break;
						}

						if (!removed_sample(out_sample_index))
						{
							out_value = m_get_sample(out_sample_index);
							break;
						}
					}
				}

				void find_right_control_point(Vector4_32& out_value, int32_t& out_sample_index) const
				{
					ACL_ASSERT(out_sample_index >= 0, "Sample index is out of range");

					while (true)
					{
						++out_sample_index;

						if (static_cast<uint32_t>(out_sample_index) >= m_num_samples)
						{
							out_value = get_right_auxiliary_control_point(out_sample_index);
							break;
						}

						if (!removed_sample(out_sample_index))
						{
							out_value = m_get_sample(out_sample_index);
							break;
						}
					}
				}
			};

			void interpolate_pose(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders,
				uint32_t sample_index, Transform_32* out_local_pose)
			{
				ACL_ASSERT(0 <= sample_index && sample_index <= INT32_MAX, "sample_index is out of range");

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					const BoneStreams& bone = segment.bone_streams[bone_index];

					Quat_32 rotation = rotation_encoders[bone_index] != nullptr ?
						quat_normalize(rotation_encoders[bone_index]->interpolate(sample_index)) : bone.get_rotation_sample(sample_index);

					Vector4_32 translation = translation_encoders[bone_index] != nullptr ?
						translation_encoders[bone_index]->interpolate(sample_index) : bone.get_translation_sample(sample_index);

					out_local_pose[bone_index] = transform_set(rotation, translation);
				}
			}
#endif

#if SEPARATE_TRANSLATION_PASS
			void choose_samples_to_remove(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				TrackStreamEncoder** rotation_encoders, TrackStreamEncoder** translation_encoders, AnimationTrackType8 track_type_to_remove,
				float* error_per_bone, BoneTrackError* error_per_stream, Transform_32* raw_local_pose, Transform_32* lossy_local_pose)
			{
#if LINEAR_SEQUENCE
				for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; ++sample_index)
				{
#else
				uint32_t step = segment.num_clip_samples / 2;

				while (step >= 1)
				{
					for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; sample_index += step)
					{
#endif
						while (true)
						{
							acl::sample_pose(segment.bone_streams, segment.num_bones, sample_index, raw_local_pose, segment.num_bones);
							interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);

							double error = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose, error_per_bone);

							if (error <= segment.clip->error_threshold)
								break;

							double worst_error = 0.0;
							uint16_t bad_bone_index = INVALID_BONE_INDEX;

							for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
							{
								//bool is_interpolated =
								//	track_type_to_remove == AnimationTrackType8::Rotation && rotation_encoders[bone_index] != nullptr && rotation_encoders[bone_index]->removed_sample(sample_index) ||
								//	track_type_to_remove == AnimationTrackType8::Translation && translation_encoders[bone_index] != nullptr && translation_encoders[bone_index]->removed_sample(sample_index);

								//if (!is_interpolated)
								//	continue;

								if (error_per_bone[bone_index] > worst_error)
								{
									worst_error = error_per_bone[bone_index];
									bad_bone_index = bone_index;
								}
							}

							//ACL_ASSERT(bad_bone_index != INVALID_BONE_INDEX, "Failed to find the bone with the worst error");
							if (bad_bone_index == INVALID_BONE_INDEX)
							{
								printf("ERROR: can't find an interpolated point at sample %d but the error is %f!\n", sample_index, error);
								break;
							}

							// Find which bone in the chain contributes the most error that is being interpolated.
							calculate_skeleton_error_contribution(skeleton, raw_local_pose, lossy_local_pose, bad_bone_index, error_per_stream);

							bad_bone_index = INVALID_BONE_INDEX;
							double worst_error_contribution = 0.0;

							for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
							{
								bool is_interpolated;
								float error_contribution;

								const BoneTrackError& error = error_per_stream[bone_index];

								switch (track_type_to_remove)
								{
								case AnimationTrackType8::Rotation:
									is_interpolated = rotation_encoders[bone_index] != nullptr && rotation_encoders[bone_index]->removed_sample(sample_index);
									error_contribution = error.rotation;
									break;

								case AnimationTrackType8::Translation:
									is_interpolated = translation_encoders[bone_index] != nullptr && translation_encoders[bone_index]->removed_sample(sample_index);
									error_contribution = error.translation;
									break;
								}

								if (!is_interpolated)
									continue;

								if (error_contribution > worst_error_contribution)
								{
									bad_bone_index = bone_index;
									worst_error_contribution = error_contribution;
								}
							}

							//ACL_ASSERT(bad_bone_index != INVALID_BONE_INDEX, "Failed to find the bone with the worst error");
							if (bad_bone_index == INVALID_BONE_INDEX)
							{
								printf("ERROR: can't find an interpolated point at sample %d but the error is %f!\n", sample_index, error);
								break;
							}

							//printf("Bone %d contributes %f at sample_index %d\n", bad_bone_index, worst_error_contribution, sample_index);

							TrackStreamEncoder* bad_bone_encoder;

							switch (track_type_to_remove)
							{
							case AnimationTrackType8::Rotation:
								bad_bone_encoder = rotation_encoders[bad_bone_index];
								break;

							case AnimationTrackType8::Translation:
								bad_bone_encoder = translation_encoders[bad_bone_index];
								break;
							}

							bad_bone_encoder->keep_sample(sample_index);
						}
#if LINEAR_SEQUENCE
					}
#else
					}

					step >>= 1;
				}
#endif
			}
#else
			void choose_samples_to_remove(Allocator& allocator, const RigidSkeleton& skeleton, const SegmentContext& segment,
				TrackStreamEncoder** rotation_encoders, TrackStreamEncoder** translation_encoders,
				float* error_per_bone, BoneTrackError* error_per_stream, Transform_32* raw_local_pose, Transform_32* lossy_local_pose)
			{
				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (rotation_encoders[bone_index] != nullptr)
					{
						rotation_encoders[bone_index]->keep_sample(0);
						rotation_encoders[bone_index]->keep_sample(segment.num_clip_samples - 1);
					}

					if (translation_encoders[bone_index] != nullptr)
					{
						translation_encoders[bone_index]->keep_sample(0);
						translation_encoders[bone_index]->keep_sample(segment.num_clip_samples - 1);
					}
				}

#if LINEAR_SEQUENCE
				for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; ++sample_index)
				{
#else
				uint32_t step = segment.num_clip_samples / 2;

				while (step >= 1)
				{
					// TODO: use a bitset to skip revisited indices
					for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; sample_index += step)
					{
#endif
						acl::sample_pose(segment.bone_streams, segment.num_bones, sample_index, raw_local_pose, segment.num_bones);
						interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);

						double error = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose, error_per_bone);

						//printf("Error at %d is %f\n", sample_index, error);

						if (error <= segment.clip->error_threshold)
							continue;

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
						TrackStreamEncoder* bad_bone_translations = translation_encoders[bad_bone_index];

						bool old_removed_rotation = bad_bone_rotations != nullptr && bad_bone_rotations->removed_sample(sample_index);
						bool old_removed_translation = bad_bone_translations != nullptr && bad_bone_translations->removed_sample(sample_index);

						// TODO: try removing all rotation samples first, then all translation samples. It woudl really simplify this function.
						// So much checking for nullptr...

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

									if (bad_bone_translations != nullptr)
										bad_bone_translations->keep_sample(sample_index);

									interpolate_pose(segment, rotation_encoders, translation_encoders, sample_index, lossy_local_pose);
									double error_without_interpolated_translation = calculate_skeleton_error(allocator, skeleton, raw_local_pose, lossy_local_pose);

									if (error_without_interpolated_translation > error_without_interpolated_rotation)
									{
										bad_bone_rotations->keep_sample(sample_index);

										if (bad_bone_translations != nullptr)
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
						bool changed_translation = bad_bone_translations != nullptr && old_removed_translation != bad_bone_translations->removed_sample(sample_index);

						ACL_ASSERT(changed_rotation || changed_translation, "No changes were made to the bone with the worst error contribution; an infinite loop would have occurred.");

						// Only rewind as far back as is affected by this change.
						uint8_t num_rotation_points = 0,
							min_rotation_points = changed_rotation ? POLYNOMIAL_ORDER : 0,
							num_translation_points = 0,
							min_translation_points = changed_translation ? POLYNOMIAL_ORDER : 0;

						while (sample_index > 0 &&
							(num_rotation_points < min_rotation_points || min_rotation_points == 0) &&
							(num_translation_points <= min_translation_points || min_translation_points == 0))
						{
							if (bad_bone_rotations == nullptr || !bad_bone_rotations->removed_sample(sample_index))
								++num_rotation_points;

							if (bad_bone_translations == nullptr || !bad_bone_translations->removed_sample(sample_index))
								++num_translation_points;

							--sample_index;
						}

#if LINEAR_SEQUENCE
					}
#else
					}

					step >>= 1;
				}
#endif
			}
#endif	// SEPARATE_TRANSLATION_PASS

			void get_output_sample_index_range(const SegmentContext& segment, int32_t& out_first, int32_t& out_last)
			{
				out_first = -FIRST_INTERPOLATION_KNOT_INDEX;
				out_last = (segment.num_clip_samples - 1) + POLYNOMIAL_ORDER - (FIRST_INTERPOLATION_KNOT_INDEX + 1);
			}

#if SAMPLE_INDEX_PER_CONTROL_POINT
			uint32_t get_animated_data_size(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders)
			{
				uint32_t animated_data_size = 0;

				int32_t first_sample_index, last_sample_index;
				get_output_sample_index_range(segment, first_sample_index, last_sample_index);

				for (int32_t sample_index = first_sample_index; sample_index <= last_sample_index; ++sample_index)
				{
					uint32_t num_rotations = 0, num_translations = 0;

					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						const BoneStreams& bone = segment.bone_streams[bone_index];

						const TrackStreamEncoder* rotation_encoder = rotation_encoders[bone_index];
						if (rotation_encoder != nullptr)
						{
							if (sample_index < 0 || sample_index >= segment.num_clip_samples || !rotation_encoder->removed_sample(sample_index))
							{
								if (0 <= sample_index && sample_index < segment.num_clip_samples)
								{
									animated_data_size += sizeof(sample_index);
								}

								// Knot
								animated_data_size += sizeof(float);

								RotationFormat8 format = bone.rotations.get_rotation_format();
								animated_data_size += get_packed_rotation_size(format);

								++num_rotations;
							}
						}

						const TrackStreamEncoder* translation_encoder = translation_encoders[bone_index];
						if (translation_encoder != nullptr)
						{
							if (sample_index < 0 || sample_index >= segment.num_clip_samples || !translation_encoder->removed_sample(sample_index))
							{
								if (0 <= sample_index && sample_index < segment.num_clip_samples)
								{
									animated_data_size += sizeof(sample_index);
								}

								// Knot
								animated_data_size += sizeof(float);

								VectorFormat8 format = bone.translations.get_vector_format();
								animated_data_size += get_packed_vector_size(format);

								++num_translations;
							}
						}
					}

					//printf("Sample_index %d has %d rotations and %d translations\n", sample_index, num_rotations, num_translations);
				}

				return animated_data_size;
			}
		}
#else
			uint32_t get_animated_data_size(const SegmentContext& segment, TrackStreamEncoder*const* rotation_encoders, TrackStreamEncoder*const* translation_encoders)
			{
				uint32_t animated_data_size = 0;

				int32_t first_sample_index, last_sample_index;
				get_output_sample_index_range(segment, first_sample_index, last_sample_index);

				for (int32_t sample_index = first_sample_index; sample_index <= last_sample_index; ++sample_index)
				{
					bool add_rotation_frame_header = false;
					bool add_translation_frame_header = false;

					for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
					{
						const BoneStreams& bone = segment.bone_streams[bone_index];

						const TrackStreamEncoder* rotation_encoder = rotation_encoders[bone_index];
						if (rotation_encoder != nullptr)
						{
							if (sample_index < 0 || sample_index >= segment.num_clip_samples || !rotation_encoder->removed_sample(sample_index))
							{
								if (0 <= sample_index && sample_index < segment.num_clip_samples)
									add_rotation_frame_header = true;

								// Knot
								animated_data_size += sizeof(float);

								RotationFormat8 format = bone.rotations.get_rotation_format();
								animated_data_size += get_packed_rotation_size(format);
							}
						}

						const TrackStreamEncoder* translation_encoder = translation_encoders[bone_index];
						if (translation_encoder != nullptr)
						{
							if (sample_index < 0 || sample_index >= segment.num_clip_samples || !translation_encoder->removed_sample(sample_index))
							{
								if (0 <= sample_index && sample_index < segment.num_clip_samples)
									add_translation_frame_header = true;

								// Knot
								animated_data_size += sizeof(float);

								VectorFormat8 format = bone.translations.get_vector_format();
								animated_data_size += get_packed_vector_size(format);
							}
						}
					}

					if (add_rotation_frame_header)
					{
						animated_data_size += sizeof(AnimationTrackType8);
						animated_data_size += sizeof(sample_index);
						animated_data_size += get_bitset_size(segment.num_bones);
					}

					if (add_translation_frame_header)
					{
						animated_data_size += sizeof(AnimationTrackType8);
						animated_data_size += sizeof(sample_index);
						animated_data_size += get_bitset_size(segment.num_bones);
					}
				}

				return animated_data_size;
			}
		}
#endif

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

			ClipContext raw_clip_context;
			initialize_clip_context(allocator, clip, raw_clip_context);

			ClipContext clip_context;
			initialize_clip_context(allocator, clip, clip_context);

			convert_rotation_streams(allocator, clip_context, settings.rotation_format);

			// TODO: Expose this, especially the translation threshold depends on the unit scale.
			// Centimeters VS meters, a different threshold should be used. Perhaps we should pass an
			// argument to the compression algorithm that states the units used or we should force centimeters
			compact_constant_streams(allocator, clip_context, 0.00001f, 0.001f);

			uint32_t clip_range_data_size = 0;
			if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerClip))
			{
				normalize_streams(clip_context, settings.range_reduction, settings.rotation_format);
				clip_range_data_size = get_stream_range_data_size(clip_context, settings.range_reduction, settings.rotation_format, settings.translation_format);
			}

			// TODO: If we are segmented, split our streams

			quantize_streams(allocator, clip_context, settings.rotation_format, settings.translation_format, clip, skeleton, raw_clip_context);

			float* error_per_bone = allocate_type_array<float>(allocator, clip_context.num_bones);
			BoneTrackError* error_per_stream = allocate_type_array<BoneTrackError>(allocator, clip_context.num_bones);
			Transform_32* raw_local_pose = allocate_type_array<Transform_32>(allocator, clip_context.num_bones);
			Transform_32* lossy_local_pose = allocate_type_array<Transform_32>(allocator, clip_context.num_bones);

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
				const SegmentContext& segment = clip_context.segments[segment_index];

				TrackStreamEncoder** segment_rotation_encoders = rotation_encoders + segment_index * clip_context.num_bones;
				TrackStreamEncoder** segment_translation_encoders = translation_encoders + segment_index * clip_context.num_bones;

				for (uint16_t bone_index = 0; bone_index < clip_context.num_bones; ++bone_index)
				{
					const BoneStreams& bone = segment.bone_streams[bone_index];

					if (bone.is_rotation_animated())
					{
						Sampler sampler = [segment, bone_index](uint32_t sample_index)
						{
							return segment.bone_streams[bone_index].get_rotation_sample(sample_index);
						};

						segment_rotation_encoders[bone_index] = allocate_type<TrackStreamEncoder>(allocator, allocator,
							segment.num_clip_samples, float(segment.num_clip_samples) / float(clip_context.sample_rate), sampler);
					}

					if (bone.is_translation_animated())
					{
						Sampler sampler = [segment, bone_index](uint32_t sample_index)
						{
							return segment.bone_streams[bone_index].get_translation_sample(sample_index);
						};

						segment_translation_encoders[bone_index] = allocate_type<TrackStreamEncoder>(allocator, allocator,
							segment.num_clip_samples, float(segment.num_clip_samples) / float(clip_context.sample_rate), sampler);
					}
				}

#if SEPARATE_TRANSLATION_PASS
				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (segment_rotation_encoders[bone_index] != nullptr)
					{
						segment_rotation_encoders[bone_index]->keep_sample(0);
						segment_rotation_encoders[bone_index]->keep_sample(segment.num_clip_samples - 1);
					}

					if (segment_translation_encoders[bone_index] != nullptr)
					{
						// TODO: could be done more efficiently with a reset
						for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; ++sample_index)
							segment_translation_encoders[bone_index]->keep_sample(sample_index);
					}
			}

				choose_samples_to_remove(allocator, skeleton, segment, segment_rotation_encoders, segment_translation_encoders, AnimationTrackType8::Rotation,
					error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose);

				for (uint16_t bone_index = 0; bone_index < segment.num_bones; ++bone_index)
				{
					if (segment_translation_encoders[bone_index] != nullptr)
					{
						// TODO: could be done more efficiently with a reset + two calls to keep_sample
						for (uint32_t sample_index = 0; sample_index < segment.num_clip_samples; ++sample_index)
							segment_translation_encoders[bone_index]->keep_sample(sample_index == 0 || sample_index == segment.num_clip_samples - 1);
					}
				}

				choose_samples_to_remove(allocator, skeleton, segment, segment_rotation_encoders, segment_translation_encoders, AnimationTrackType8::Translation,
					error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose);
#else
				choose_samples_to_remove(allocator, skeleton, segment, segment_rotation_encoders, segment_translation_encoders,
					error_per_bone, error_per_stream, raw_local_pose, lossy_local_pose);
#endif
			}

			deallocate_type_array(allocator, error_per_bone, clip_context.num_bones);
			deallocate_type_array(allocator, error_per_stream, clip_context.num_bones);
			deallocate_type_array(allocator, raw_local_pose, clip_context.num_bones);
			deallocate_type_array(allocator, lossy_local_pose, clip_context.num_bones);

			const SegmentContext& clip_segment = clip_context.segments[0];

			uint32_t constant_data_size = get_constant_data_size(clip_context);
			uint32_t animated_pose_bit_size;
			uint32_t animated_data_size = get_animated_data_size(clip_segment, rotation_encoders, translation_encoders);		// TODO: run for each segment

			printf("Animated data size is %d\n", animated_data_size);

#if false
			// *** testing
			animated_pose_bit_size = 0;
			// ***********

			uint32_t format_per_track_data_size = get_format_per_track_data_size(clip_context, settings.rotation_format, settings.translation_format);

			uint32_t bitset_size = get_bitset_size(num_bones * Constants::NUM_TRACKS_PER_BONE);

			uint32_t buffer_size = 0;
			buffer_size += sizeof(CompressedClip);
			buffer_size += sizeof(Header);
			buffer_size += sizeof(uint32_t) * bitset_size;		// Default tracks bitset
			buffer_size += sizeof(uint32_t) * bitset_size;		// Constant tracks bitset
			buffer_size = align_to(buffer_size, 4);				// Align constant track data
			buffer_size += constant_data_size;					// Constant track data
			buffer_size += format_per_track_data_size;			// Format per track data
			buffer_size = align_to(buffer_size, 4);				// Align range data
			buffer_size += clip_range_data_size;				// Range data
			buffer_size = align_to(buffer_size, 4);				// Align animated data
			buffer_size += animated_data_size;					// Animated track data

			uint8_t* buffer = allocate_type_array_aligned<uint8_t>(allocator, buffer_size, 16);

			CompressedClip* compressed_clip = make_compressed_clip(buffer, buffer_size, AlgorithmType8::UniformlySampled);

			Header& header = get_header(*compressed_clip);
			header.num_bones = num_bones;
			header.rotation_format = settings.rotation_format;
			header.translation_format = settings.translation_format;
			header.range_reduction = settings.range_reduction;
			header.num_samples = num_samples;
			header.sample_rate = clip.get_sample_rate();
			header.animated_pose_bit_size = animated_pose_bit_size;
			header.default_tracks_bitset_offset = sizeof(Header);
			header.constant_tracks_bitset_offset = header.default_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size);
			header.constant_track_data_offset = align_to(header.constant_tracks_bitset_offset + (sizeof(uint32_t) * bitset_size), 4);	// Aligned to 4 bytes
			header.format_per_track_data_offset = header.constant_track_data_offset + constant_data_size;
			header.clip_range_data_offset = align_to(header.format_per_track_data_offset + format_per_track_data_size, 4);				// Aligned to 4 bytes
			header.track_data_offset = align_to(header.clip_range_data_offset + clip_range_data_size, 4);								// Aligned to 4 bytes

			write_default_track_bitset(clip_context, header.get_default_tracks_bitset(), bitset_size);
			write_constant_track_bitset(clip_context, header.get_constant_tracks_bitset(), bitset_size);

			if (constant_data_size > 0)
				write_constant_track_data(clip_context, header.get_constant_track_data(), constant_data_size);
			else
				header.constant_track_data_offset = InvalidPtrOffset();

			if (format_per_track_data_size > 0)
				write_format_per_track_data(clip_context, header.get_format_per_track_data(), format_per_track_data_size);
			else
				header.format_per_track_data_offset = InvalidPtrOffset();

			if (is_enum_flag_set(settings.range_reduction, RangeReductionFlags8::PerClip))
				write_range_track_data(clip_segment, settings.range_reduction, settings.rotation_format, settings.translation_format, header.get_clip_range_data(), clip_range_data_size);
			else
				header.clip_range_data_offset = InvalidPtrOffset();

			if (animated_data_size > 0)
				write_animated_track_data(clip_segment, settings.rotation_format, settings.translation_format, header.get_track_data(), animated_data_size);
			else
				header.track_data_offset = InvalidPtrOffset();

			finalize_compressed_clip(*compressed_clip);

			for (uint32_t encoder_index = 0; encoder_index < num_track_stream_encoders; ++encoder_index)
			{
				deallocate_type(allocator, rotation_encoders[encoder_index]);
				deallocate_type(allocator, translation_encoders[encoder_index]);
			}
#endif

			deallocate_type_array(allocator, rotation_encoders, num_track_stream_encoders);
			deallocate_type_array(allocator, translation_encoders, num_track_stream_encoders);

			destroy_clip_context(allocator, clip_context);
			destroy_clip_context(allocator, raw_clip_context);

//			return compressed_clip;
			return nullptr;
		}

		/* TODO: need to assert that number of samples is >= get_polynomial_order(). If it isn't, write the samples out as is and revert to linear
		interpolation.  Or, add additional samples to ensure there are at least four, which simplifies the decoder. */

#if false
		// TODO: use templated SplineControlPoint, same as decoder
			struct ControlPoint
			{
				int32_t sample_index;
				Vector4_32 value;
				float knot;
			};


			uint32_t get_rotation_control_point_flag_offset(uint16_t bone_index) const { return bone_index * Constants::NUM_TRACKS_PER_BONE + 0; }
			uint32_t get_translation_control_point_flag_offset(uint16_t bone_index) const { return bone_index * Constants::NUM_TRACKS_PER_BONE + 1; }

			void write_animated_track_data(uint8_t* animated_track_data, uint32_t animated_data_size)
			{
				const uint8_t* animated_track_data_end = add_offset_to_ptr<uint8_t>(animated_track_data, animated_data_size);

				ControlPoint* rotation_control_points = allocate_type_array<ControlPoint>(m_allocator, m_num_bones);
				ControlPoint* translation_control_points = allocate_type_array<ControlPoint>(m_allocator, m_num_bones);

				for (uint16_t bone_index = 0; bone_index < m_num_bones; ++bone_index)
				{
					rotation_control_points[bone_index].knot = 0.0;
					translation_control_points[bone_index].knot = 0.0;
				}

				uint32_t control_point_flags_size = get_bitset_size(m_num_bones * Constants::NUM_TRACKS_PER_BONE);

				int32_t first_sample_index = get_first_output_sample_index();
				int32_t last_sample_index = get_last_output_sample_index();

				for (int32_t sample_index = first_sample_index; sample_index <= last_sample_index; ++sample_index)
				{
					uint32_t* control_point_flags = nullptr;

					for (uint16_t bone_index = 0; bone_index < m_num_bones; ++bone_index)
					{
						const BoneStreams& bone = m_bone_streams[bone_index];

						// TODO: this logic could be halved if you were able to treat every stream as a Vector4_32,
						// test if it is animated, etc.  Rest of the class would be simplified somewhat too.

						if (bone.rotations.is_animated())
						{
							ControlPoint& current = rotation_control_points[bone_index];
							const ControlPoint last = current;

							if (sample_index == first_sample_index)
							{
								current.sample_index = sample_index;
								current.value = quat_to_vector(get_rotation_left_auxiliary_control_point(bone, sample_index));
								current.knot = 0.0;
							}
							else if (sample_index < 0)
							{
								current.sample_index = sample_index;
								current.value = quat_to_vector(get_rotation_left_auxiliary_control_point(bone, current.sample_index));
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}
							else if (sample_index >= m_num_samples)
							{
								current.sample_index = sample_index;
								current.value = quat_to_vector(get_rotation_right_auxiliary_control_point(bone, current.sample_index));
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}
							else if (!removed_rotation(bone_index, sample_index))
							{
								current.sample_index = sample_index;
								current.value = quat_to_vector(bone.rotations.get_sample(current.sample_index));
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}

							if (current.sample_index == sample_index)
							{
								if (0 <= sample_index && sample_index < m_num_samples && control_point_flags == nullptr)
								{
									const size_t sample_index_size = sizeof(sample_index);
									memcpy(animated_track_data, &sample_index, sample_index_size);
									animated_track_data += sample_index_size;

									control_point_flags = safe_ptr_cast<uint32_t, uint8_t*>(animated_track_data);
									animated_track_data += control_point_flags_size;

									bitset_reset(control_point_flags, control_point_flags_size, false);
								}

								if (control_point_flags != nullptr)
									bitset_set(control_point_flags, control_point_flags_size, get_rotation_control_point_flag_offset(bone_index), true);

								if (!m_spline.uniform_knots())
								{
									float delta = current.knot - last.knot;
									const size_t delta_size = sizeof(delta);
									memcpy(animated_track_data, &delta, delta_size);
									animated_track_data += delta_size;
								}

								// TODO: write the quantized auxiliary control point or the true bone stream one
								// currently this tries to read from out of bounds

								const uint8_t* rotation_ptr = bone.rotations.get_raw_sample_ptr(sample_index);
								uint32_t sample_size = bone.rotations.get_sample_size();
								memcpy(animated_track_data, rotation_ptr, sample_size);
								animated_track_data += sample_size;
							}
						}

						if (bone.translations.is_animated())
						{
							ControlPoint& current = translation_control_points[bone_index];
							const ControlPoint last = current;

							if (sample_index == first_sample_index)
							{
								current.sample_index = sample_index;
								current.value = get_translation_left_auxiliary_control_point(bone, sample_index);
								current.knot = 0.0;
							}
							else if (sample_index < 0)
							{
								current.sample_index = sample_index;
								current.value = get_translation_left_auxiliary_control_point(bone, current.sample_index);
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}
							else if (sample_index >= m_num_samples)
							{
								current.sample_index = sample_index;
								current.value = get_translation_right_auxiliary_control_point(bone, current.sample_index);
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}
							else if (!removed_translation(bone_index, sample_index))
							{
								current.sample_index = sample_index;
								current.value = bone.translations.get_sample(current.sample_index);
								current.knot = m_spline.calculate_next_knot(current.value, current.sample_index, last.knot, last.value, last.sample_index);
							}

							if (current.sample_index == sample_index)
							{
								if (0 <= sample_index && sample_index < m_num_samples && control_point_flags == nullptr)
								{
									const size_t sample_index_size = sizeof(sample_index);
									memcpy(animated_track_data, &sample_index, sample_index_size);
									animated_track_data += sample_index_size;

									control_point_flags = safe_ptr_cast<uint32_t, uint8_t*>(animated_track_data);
									animated_track_data += control_point_flags_size;

									bitset_reset(control_point_flags, control_point_flags_size, false);
								}

								if (control_point_flags != nullptr)
									bitset_set(control_point_flags, control_point_flags_size, get_rotation_control_point_flag_offset(bone_index), true);

								if (!m_spline.uniform_knots())
								{
									float delta = current.knot - last.knot;
									const size_t delta_size = sizeof(delta);
									memcpy(animated_track_data, &delta, delta_size);
									animated_track_data += delta_size;
								}

								// TODO: write the quantized auxiliary control point or the true bone stream one
								// currently this tries to read from out of bounds

								const uint8_t* translation_ptr = bone.translations.get_raw_sample_ptr(sample_index);
								uint32_t sample_size = bone.translations.get_sample_size();
								memcpy(animated_track_data, translation_ptr, sample_size);
								animated_track_data += sample_size;
							}
						}

						ACL_ENSURE(animated_track_data <= animated_track_data_end, "Invalid animated track data offset. Wrote too much data.");
					}
				}

				ACL_ENSURE(animated_track_data == animated_track_data_end, "Invalid animated track data offset. Wrote too little data.");

				deallocate_type_array(m_allocator, rotation_control_points, m_num_bones);
				deallocate_type_array(m_allocator, translation_control_points, m_num_bones);
			}
#endif

		void print_stats(const CompressedClip& clip, std::FILE* file)
		{
			using namespace impl;

			const Header& header = get_header(clip);

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
		}
	}
}
