// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/shape_annotation_set.h"
#include "../include/rlibv/data_utils.h"
#include "../include/rlibv/geometry.h"
#include "../include/rlibv/simple_profiler.h"
#include "../include/rlibv/linear_range.h"

namespace rlibv
{

	void rlibv::shape_annotation_set::initialize(
		const shape_annotation& prototype, const std::vector<shape_annotation>& data, const std::map<std::string, int>& n_dense_internals)
	{
		prototype_ = prototype;
		data_ = data;
		rough_data_ = data;
		is_prototype_set_ = true;
		n_internals_for_dense_curves_ = n_dense_internals;
	}


	void rlibv::shape_annotation_set::set_all_data(const std::vector<shape_annotation>& data)
	{
		data_ = data;
		rough_data_ = data;
	}


	void shape_annotation_set::remove_example_at_index(int index)
	{
		if (index >= data_.size())
		{
			return;
		}
		data_.erase(data_.begin() + index);
		rough_data_.erase(rough_data_.begin() + index);
		data_.shrink_to_fit();
		rough_data_.shrink_to_fit();
	}

	void shape_annotation_set::remove_all_examples()
	{
		while (data_.size() > 0)
		{
			remove_example_at_index(0);
		}
	}

	void shape_annotation_set::add_example(const shape_annotation& data)
	{
		data_.emplace_back(data);
		rough_data_.emplace_back(data);
	}

	void shape_annotation_set::merge_data(const shape_annotation_set& other)
	{
		data_.insert(data_.end(), other.data_.begin(), other.data_.end());
		rough_data_.insert(rough_data_.end(), other.rough_data_.begin(), other.rough_data_.end());
		sort_data();
	}

	void shape_annotation_set::set_prototype(const shape_annotation& exemplar)
	{
		prototype_ = exemplar;
		is_prototype_set_ = true;
	}

	shape_annotation* const shape_annotation_set::item(size_t index)
	{
		if (index >= data_.size())
		{
			return nullptr;
		}
		return &(data_[index]);
	}

	shape_annotation* const shape_annotation_set::rough_item(size_t index)
	{
		if (index >= rough_data_.size())
		{
			return nullptr;
		}
		return &(rough_data_[index]);
	}

	shape_annotation* const shape_annotation_set::prototype()
	{
		return &prototype_;
	}

	bool shape_annotation_set::is_prototype_set() const
	{
		return is_prototype_set_;
	}


	bool shape_annotation_set::is_consistent(std::string& first_error) const
	{
		first_error.clear();
		if (!is_prototype_set())
		{
			first_error = "Prototype isn't set.";
			return false;
		}
		for (const auto& item : data_)
		{
			std::string match_error;
			if (!prototype_.matches_scheme(item, match_error))
			{
				first_error = "shape annotation set is inconsistent with prototype at " + item.image_filename() + " ( " + match_error + " )";
				return false;
			}
		}
		return true;
	}

	int shape_annotation_set::n_examples() const
	{
		return static_cast<int>(data_.size());
	}

	void shape_annotation_set::sort_data()
	{
		std::sort(data_.begin(), data_.end(), shape_annotation_less_than());
		std::sort(rough_data_.begin(), rough_data_.end(), shape_annotation_less_than());

		//Any items where rough data doesn't match smooth data should automatically be sent to the back
		std::vector<int> move_to_back;
		int n_items = static_cast<int>(data_.size());
		for (int i = 0; i < n_items; ++i)
		{
			if (data_[i].keypoints() != rough_data_[i].keypoints() || data_[i].keypoint_curves() != rough_data_[i].keypoint_curves())
			{
				move_to_back.emplace_back(i);
			}
		}

		std::sort(move_to_back.begin(), move_to_back.end(), std::greater<>());
		for (int i : move_to_back)
		{
			auto it = data_.begin() + i;
			std::rotate(it, it + 1, data_.end());

			auto rough_it = rough_data_.begin() + i;
			std::rotate(rough_it, rough_it + 1, rough_data_.end());
		}
		

	}

	void shape_annotation_set::remove_duplicates()
	{
		auto original_data(data_);
		auto original_rough_data(rough_data_);
		data_.clear();
		rough_data_.clear();
		int n_items = static_cast<int>(original_data.size());
		for (int i = 0; i < n_items; ++i)
		{
			bool is_good = true;
			for (const auto& existing_item : data_)
			{
				if (refer_to_same_image(original_data[i], existing_item))
				{
					is_good = false;
					break;
				}
			}
			if (is_good)
			{
				data_.emplace_back(original_data[i]);
				rough_data_.emplace_back(original_rough_data[i]);
			}
		}
	}

	void shape_annotation_set::remove_bad_data()
	{
		auto original_data(data_);
		auto original_rough_data(rough_data_);
		data_.clear();
		rough_data_.clear();
		int n_items = static_cast<int>(original_data.size());
		for (int i = 0; i < n_items; ++i)
		{
			try
			{
				std::vector<std::vector<int>> inbound_links;
				std::vector<std::vector<int>> outbound_links;
				std::map<std::string,std::vector<int>> curve_lookup;
				for (const auto& kp : original_data[i].keypoints())
				{
					auto p = kp.second.pos;
					if (p.x() < 0 || p.x() > 99 || p.y() < 0 || p.y() > 99)
					{
						throw std::runtime_error("bad point");
					}
				}
				for (const auto& kp : original_data[i].keypoint_curves())
				{
					for (auto p : kp.second.internal_points)
					{
						if (p.x() < 0 || p.x() > 99 || p.y() < 0 || p.y() > 99)
						{
							throw std::runtime_error("bad point");
						}
					}
				}
			}
			catch (...)
			{
				continue;
			}
			data_.emplace_back(original_data[i]);
			rough_data_.emplace_back(original_rough_data[i]);
		}
	}

	void shape_annotation_set::remove_curves(const std::vector<std::string>& curve_names)
	{
		std::map<std::string, keypoint_curve> proto_curves = prototype_.keypoint_curves();
		std::map<std::string, keypoint> proto_keypoints = prototype_.keypoints();

		// Erase the named curves
		for (const auto& curve_name : curve_names)
		{	
			proto_curves.erase(curve_name);
		}

		// Look for hanging points
		std::set<std::string> keypoints_still_required;
		for (const auto& item : proto_curves)
		{
			keypoints_still_required.insert(item.second.end_keypoint_name);
			keypoints_still_required.insert(item.second.start_keypoint_name);
		}
		std::vector<std::string> keypoint_names_to_remove;
		for (const auto& item : proto_keypoints)
		{
			if (keypoints_still_required.find(item.first) == keypoints_still_required.end())
			{
				keypoint_names_to_remove.emplace_back(item.first);
			}
		}

		for (const auto& kp_name : keypoint_names_to_remove)
		{
			proto_keypoints.erase(kp_name);
		}

		prototype_.initialize(prototype_.image_filename(), proto_keypoints, proto_curves);

		for (auto& example : data_)
		{
			std::map<std::string, keypoint_curve> curves = example.keypoint_curves();
			std::map<std::string, keypoint> keypoints = example.keypoints();
			for (const auto& kp_name : keypoint_names_to_remove)
			{
				keypoints.erase(kp_name);
			}
			for (const auto& curve_name : curve_names)
			{
				curves.erase(curve_name);
			}

			example.initialize(example.image_filename(), keypoints, curves);
		}
		for (auto& example : rough_data_)
		{
			std::map<std::string, keypoint_curve> curves = example.keypoint_curves();
			std::map<std::string, keypoint> keypoints = example.keypoints();
			for (const auto& kp_name : keypoint_names_to_remove)
			{
				keypoints.erase(kp_name);
			}
			for (const auto& curve_name : curve_names)
			{
				curves.erase(curve_name);
			}

			example.initialize(example.image_filename(), keypoints, curves);
		}
	}


	void shape_annotation_set::remove_free_points()
	{

		std::map<std::string, keypoint> proto_keypoints = prototype_.keypoints();

		// Look for hanging points
		std::set<std::string> keypoints_still_required;
		for (const auto& item : prototype_.keypoint_curves())
		{
			keypoints_still_required.insert(item.second.end_keypoint_name);
			keypoints_still_required.insert(item.second.start_keypoint_name);
		}
		std::vector<std::string> keypoint_names_to_remove;
		for (const auto& item : proto_keypoints)
		{
			if (keypoints_still_required.find(item.first) == keypoints_still_required.end())
			{
				keypoint_names_to_remove.emplace_back(item.first);
			}
		}

		for (const auto& kp_name : keypoint_names_to_remove)
		{
			proto_keypoints.erase(kp_name);
		}

		prototype_.initialize(prototype_.image_filename(), proto_keypoints, prototype_.keypoint_curves());

		for (auto& example : data_)
		{
			std::map<std::string, keypoint> keypoints = example.keypoints();
			for (const auto& kp_name : keypoint_names_to_remove)
			{
				keypoints.erase(kp_name);
			}
			example.initialize(example.image_filename(), keypoints, example.keypoint_curves());
		}
		for (auto& example : rough_data_)
		{
			std::map<std::string, keypoint> keypoints = example.keypoints();
			for (const auto& kp_name : keypoint_names_to_remove)
			{
				keypoints.erase(kp_name);
			}
			example.initialize(example.image_filename(), keypoints, example.keypoint_curves());
		}
	}

	const std::vector<shape_annotation>& shape_annotation_set::data() const
	{
		return data_;
	}

	const std::map<std::string, int>& shape_annotation_set::n_internals_for_dense_curves() const
	{
		return n_internals_for_dense_curves_;
	}

	void shape_annotation_set::set_n_internals_for_dense_curves(const std::map<std::string, int>& n_internals)
	{
		n_internals_for_dense_curves_ = n_internals;
	}

	void serialize(const shape_annotation_set& item, std::ostream& out)
	{
		serialize(item.prototype_, out);
		serialize(item.data_, out);
		serialize(item.is_prototype_set_, out);
		serialize(item.n_internals_for_dense_curves_, out);
		serialize(item.rough_data_, out);
	}

	void deserialize(shape_annotation_set& item, std::istream& in)
	{
		auto pos = in.tellg();
		try
		{
			deserialize(item.prototype_, in);
			deserialize(item.data_, in);
			deserialize(item.is_prototype_set_, in);
			deserialize(item.n_internals_for_dense_curves_, in);
			if (item.n_internals_for_dense_curves_.size() != item.prototype_.keypoint_curves().size())
			{
				throw(dlib::serialization_error("n_internals_for_dense_curves_ is wrong size"));
			}
		}
		catch (dlib::serialization_error& e)
		{
			e;
			in.clear();
			in.seekg(pos);
			deserialize(item.prototype_, in);
			deserialize(item.data_, in);
			deserialize(item.is_prototype_set_, in);

			//Temporary hack
			if (rlibv::map_contains_key(item.data_[0].keypoints(), std::string("pt_mouth_corner_l")) ||
				rlibv::map_contains_key(item.data_[0].keypoints(), std::string("corner_l")))
			{
				item.n_internals_for_dense_curves_.clear();
				item.n_internals_for_dense_curves_["crv_lip_upper_outer_r"] = 23;
				item.n_internals_for_dense_curves_["crv_lip_philtrum_r"] = 6;
				item.n_internals_for_dense_curves_["crv_lip_philtrum_l"] = 6;
				item.n_internals_for_dense_curves_["crv_lip_upper_outer_l"] = 23;
				item.n_internals_for_dense_curves_["crv_lip_lower_outer_r"] = 27;
				item.n_internals_for_dense_curves_["crv_lip_lower_outer_l"] = 27;
				item.n_internals_for_dense_curves_["crv_lip_lower_inner_r"] = 24;
				item.n_internals_for_dense_curves_["crv_lip_lower_inner_l"] = 24;
				item.n_internals_for_dense_curves_["crv_lip_upper_inner_r"] = 24;
				item.n_internals_for_dense_curves_["crv_lip_upper_inner_l"] = 24;
				item.n_internals_for_dense_curves_["crv_nasolabial_l"] = 23;
				item.n_internals_for_dense_curves_["crv_nasolabial_r"] = 23;
			}
			//Temporary hack
			if (rlibv::map_contains_key(item.data_[0].keypoints(), std::string("pt_eye_corner_outer_r")))
			{
				//item.n_internals_for_dense_curves_.clear();
				//item.n_internals_for_dense_curves_["crv_brow_lower_r"] = 23;
				//item.n_internals_for_dense_curves_["crv_brow_upper_r"] = 23;
				//item.n_internals_for_dense_curves_["crv_brow_lower_l"] = 23;
				//item.n_internals_for_dense_curves_["crv_brow_upper_l"] = 23;
				//item.n_internals_for_dense_curves_["crv_eyelid_lower_r"] = 18;
				//item.n_internals_for_dense_curves_["crv_eyelid_lower_l"] = 18;
				//item.n_internals_for_dense_curves_["crv_eyelid_upper_r"] = 18;
				//item.n_internals_for_dense_curves_["crv_eyelid_upper_l"] = 18;
				//item.n_internals_for_dense_curves_["crv_iris_r"] = 24;
				//item.n_internals_for_dense_curves_["crv_iris_l"] = 24;
			}
		}

		pos = in.tellg();
		try
		{
			deserialize(item.rough_data_, in);
		}
		catch (dlib::serialization_error& e)
		{
			e;
			in.clear();
			in.seekg(pos);
			item.rough_data_ = item.data_;
		}
	}

	
}


