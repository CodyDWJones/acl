#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Animation Compression Library contributors
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

#include "acl/core/bitset.h"
#include "acl/core/compiler_utils.h"
#include "acl/core/error.h"
#include "acl/core/iallocator.h"
#include "acl/core/track_types.h"

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	namespace spline_key_reduction
	{
		namespace impl
		{
			class ControlPointSelections
			{
			public:
				ControlPointSelections(IAllocator& allocator, uint16_t num_bones, uint32_t num_samples)
					: m_allocator(&allocator)
					, m_num_samples(num_samples)
					, m_num_bones(num_bones)
					, m_num_bits(num_bones * num_samples * get_array_size(k_track_types))
				{
					m_selected_desc = BitSetDescription::make_from_num_bits(m_num_bits);
					m_selected = allocate_type_array<uint32_t>(*m_allocator, m_selected_desc.get_size());

					deselect_all();
				}

				ControlPointSelections(const ControlPointSelections&) = delete;
				ControlPointSelections(ControlPointSelections&& other)
					: m_allocator(other.m_allocator)
					, m_num_samples(other.m_num_samples)
					, m_num_bones(other.m_num_bones)
					, m_num_bits(other.m_num_bits)
					, m_selected_desc(other.m_selected_desc)
					, m_selected(other.m_selected)
				{
					other.m_allocator = nullptr;
					other.m_selected = nullptr;
				}

				~ControlPointSelections()
				{
					if (m_allocator != nullptr)
						deallocate_type_array(*m_allocator, m_selected, m_selected_desc.get_size());
				}

				ControlPointSelections& operator=(const ControlPointSelections&) = delete;
				ControlPointSelections& operator=(ControlPointSelections&& rhs)
				{
					std::swap(m_allocator, rhs.m_allocator);
					std::swap(m_num_samples, rhs.m_num_samples);
					std::swap(m_num_bones, rhs.m_num_bones);
					std::swap(m_num_bits, rhs.m_num_bits);
					std::swap(m_selected_desc, rhs.m_selected_desc);
					std::swap(m_selected, rhs.m_selected);
					return *this;
				}

				void select_all()
				{
					bitset_reset(m_selected, m_selected_desc, true);
					bitset_set_range(m_selected, m_selected_desc, m_num_bits, m_selected_desc.get_num_bits() - m_num_bits, false);
				}

				void deselect_all() { bitset_reset(m_selected, m_selected_desc, false); }

				void select_all_bones(uint32_t sample_index, AnimationTrackType8 track_type)
				{
					for (uint16_t bone_index = 0; bone_index < m_num_bones; ++bone_index)
						select(bone_index, sample_index, track_type);
				}

				void deselect_all_bones(uint32_t sample_index, AnimationTrackType8 track_type)
				{
					for (uint16_t bone_index = 0; bone_index < m_num_bones; ++bone_index)
						deselect(bone_index, sample_index, track_type);
				}

				void select(uint16_t bone_index, uint32_t sample_index, AnimationTrackType8 track_type) { select(bone_index, sample_index, track_type, true); }
				void deselect(uint16_t bone_index, uint32_t sample_index, AnimationTrackType8 track_type) { select(bone_index, sample_index, track_type, false); }
				void deselect(const ControlPointSelections& other) { bitset_inverse_mask(m_selected, m_selected_desc, other.m_selected, other.m_selected_desc); }

				void select(uint16_t bone_index, uint32_t sample_index, AnimationTrackType8 track_type, bool selected)
				{
					bitset_set(m_selected, m_selected_desc, get_bit_index(bone_index, sample_index, track_type), selected);
				}

				bool is_selected(uint16_t bone_index, uint32_t sample_index, AnimationTrackType8 track_type) const
				{
					return bitset_test(m_selected, m_selected_desc, get_bit_index(bone_index, sample_index, track_type));
				}

				bool any_bones_selected(uint32_t sample_index, AnimationTrackType8 track_type) const
				{
					for (uint16_t bone_index = 0; bone_index < m_num_bones; ++bone_index)
						if (is_selected(bone_index, sample_index, track_type))
							return true;

					return false;
				}

				bool any_bones_selected(uint32_t sample_index) const
				{
					for (AnimationTrackType8 track_type : k_track_types)
						if (any_bones_selected(sample_index, track_type))
							return true;

					return false;
				}

				uint32_t get_num_selected() const { return bitset_count_set_bits(m_selected, m_selected_desc); }
				uint32_t get_num_samples() const { return m_num_samples; }
				uint16_t get_num_bones() const { return m_num_bones; }

			private:
				IAllocator* m_allocator;
				uint32_t* m_selected;
				BitSetDescription m_selected_desc;

				uint16_t m_num_bones;
				uint32_t m_num_samples;
				uint32_t m_num_bits;

				inline uint32_t get_bit_index(uint16_t bone_index, uint32_t sample_index, AnimationTrackType8 track_type) const
				{
					ACL_ASSERT(sample_index < m_num_samples, "Sample index %d is invalid; there are only %d samples", sample_index, m_num_samples);
					ACL_ASSERT(bone_index < m_num_bones, "Bone index %d is invalid; there are only %d bones", bone_index, m_num_bones);

					return static_cast<uint32_t>(track_type) * m_num_samples * m_num_bones + sample_index * m_num_bones + bone_index;
				}
			};
		}
	}
}

ACL_IMPL_FILE_PRAGMA_POP
