// Copyright Epic Games, Inc. All Rights Reserved.

#include "../include/rlibv/mv_tracker_gallery.h"

#include <variant>

namespace rlibv
{
	void mv_tracker_gallery::load(std::istream& ifs)
	{
		auto pos = ifs.tellg();
		std::string pixel_t;
		dlib::deserialize(pixel_t, ifs);
		int Rt, St;
		bool At, Ct;
		dlib::deserialize(Rt, ifs);
		dlib::deserialize(At, ifs);  // Not used any more, but needed for backward compatibility
		dlib::deserialize(Ct, ifs);  // Not used any more, but needed for backward compatibility
		dlib::deserialize(St, ifs);
		if ("dlib::rgb_pixel" == pixel_t && // replication of if condition on line 116
			14 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<standard_colour_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading standard_colour tracker");
			}
		}
		else if ("dlib::rgb_pixel" == pixel_t &&
			14 == Rt &&
			4 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<four_level_colour_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading four_level_colour tracker");
			}
		}
		else if ("dlib::rgb_pixel" == pixel_t &&
			8 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<fast_colour_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading fast_colour tracker");
			}
		}
		else if ("unsigned char" == pixel_t && // replication of if condition on line 132
			14 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<standard_grey_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading standard_grey tracker");
			}
		}
		else if ("unsigned char" == pixel_t &&
			14 == Rt &&
			4 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<four_level_grey_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading four_level_grey tracker");
			}
		}
		else if ("unsigned char" == pixel_t &&
			8 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<fast_grey_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading fast_grey tracker");
			}
		}
#if(0)
		else if ("dlib::rgb_pixel" == pixel_t && // replication of if condition on line 20
			14 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<combined_colour_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading combined_colour tracker");
			}
		}
		else if ("unsigned char" == pixel_t && // replication of if condition on line 68
			14 == Rt &&
			3 == St)
		{
			try
			{
				ifs.seekg(pos, std::ios::beg);
				tracker_ptr_ = std::make_unique<combined_grey_tracker>();
				auto visitor = [&ifs](auto& t) { t->load(ifs); };
				std::visit(visitor, tracker_ptr_);
			}
			catch (...)
			{
				throw std::runtime_error("Error loading combined_grey tracker");
			}
		}
#endif
		else
		{
			ifs.seekg(pos, std::ios::beg);
			throw std::runtime_error("Error loading tracker - not recognized as a standard type");
		}
	}

	void mv_tracker_gallery::save(std::ostream& ofs) const
	{
		auto visitor = [&ofs](auto& t) { t->save(ofs); };
		std::visit(visitor, tracker_ptr_);
	}

	void mv_tracker_gallery::initialize_training(mv_tracker_gallery_item gallery_selection,
		const std::vector<multiview_shape2d<double>>& mv_shapes,
		const view_vector<std::vector<simple_curve>>& curves,
		std::array<double, 4> augmentation_factors,
		std::vector<double> stage_scaling,
		alignment_type align_method)
	{
		std::string msg;
		DLIB_ASSERT(is_mv_tracker_initialization_valid(mv_shapes, curves, augmentation_factors, stage_scaling, msg));

		double disp_size = 0.25;
		switch(gallery_selection)
		{
		case mv_tracker_gallery_item::standard_colour:
			tracker_ptr_ = std::make_unique<standard_colour_tracker>();
			break;
		case mv_tracker_gallery_item::tree_colour:
			tracker_ptr_ = std::make_unique<tree_colour_tracker>();
			disp_size = 0.5;
			break;
		case mv_tracker_gallery_item::four_level_colour:
			tracker_ptr_ = std::make_unique<four_level_colour_tracker>();
			break;
		case mv_tracker_gallery_item::fast_colour:
			tracker_ptr_ = std::make_unique<fast_colour_tracker>();
			break;
		case mv_tracker_gallery_item::combined_colour:
			tracker_ptr_ = std::make_unique<combined_colour_tracker>();
			break;
		case mv_tracker_gallery_item::standard_grey:
			tracker_ptr_ = std::make_unique<standard_grey_tracker>();
			break;	
		case mv_tracker_gallery_item::tree_grey:
			tracker_ptr_ = std::make_unique<tree_grey_tracker>();
			disp_size = 0.5;
			break;
		case mv_tracker_gallery_item::four_level_grey:
			tracker_ptr_ = std::make_unique<four_level_grey_tracker>();
			break;	
		case mv_tracker_gallery_item::fast_grey:
			tracker_ptr_ = std::make_unique<fast_grey_tracker>();

			break;
		case mv_tracker_gallery_item::combined_grey:
			tracker_ptr_ = std::make_unique<combined_grey_tracker>();
			break;
		default:
			tracker_ptr_ = std::make_unique<standard_colour_tracker>();
			break;
		}

		auto visitor = [&](auto& t) { t->initialize_training(mv_shapes, curves, augmentation_factors, stage_scaling, align_method, 0.25); };
		std::visit(visitor, tracker_ptr_);
	}

	bool mv_tracker_gallery::add_training_example(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& view_imgs, const multiview_shape2d<double>& view_shapes, int expected_number_of_examples, int min_target_displacements, int min_samples_per_node)
	{
		auto visitor = [&](auto& t) {t->add_training_example(view_imgs, view_shapes, expected_number_of_examples, min_target_displacements, min_samples_per_node); };
		std::visit(visitor, tracker_ptr_);		
		return true;
	}

	const std::vector<double> mv_tracker_gallery::stage_scaling() const
	{
		auto visitor = [&](auto& t) { return t->stage_scaling(); };
		return std::visit(visitor, tracker_ptr_);
	}

	std::vector<std::vector<double>> mv_tracker_gallery::autosearch(
		const std::vector<int>& iterations_per_stage,
			int max_modes_per_stage,
			bool bilinear,
			multiview_shape2d<double>& mv_shape
	) const
	{
		auto visitor = [&](auto& t) { return t->autosearch(iterations_per_stage, max_modes_per_stage, bilinear, mv_shape); };
		return std::visit(visitor, tracker_ptr_);
	}

	bool mv_tracker_gallery::set_images(const dlib::array<dlib::array2d<dlib::rgb_pixel>>& views)
	{
		auto visitor = [&](auto& t) {t->set_images(views); };
		std::visit(visitor, tracker_ptr_);
		return true;
	}

	int mv_tracker_gallery::n_views()
	{
		return static_cast<int>(local_approx_saxy_min().size());
	}

	void mv_tracker_gallery::reset()
	{
		auto visitor = [&](auto& t) {return t.reset(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const linear_pdm<double>& mv_tracker_gallery::global_pdm() const
	{
		auto visitor = [&](auto& t) -> const linear_pdm<double>& {return t->global_pdm(); };
		return std::visit(visitor, tracker_ptr_);
	}

	dlib::array2d<dlib::rgb_pixel> mv_tracker_gallery::sampling_diagnostic_image(const std::vector<int>& pt_indices, int stage, int view) 
	{
		auto visitor = [&](auto& t) {return t->sampling_diagnostic_image(pt_indices, stage, view); };
		return std::visit(visitor, tracker_ptr_);
	}

	const view_vector<std::array<double, 4>>& mv_tracker_gallery::local_approx_saxy_min() const
	{
		auto visitor = [&](auto& t) -> const view_vector<std::array<double, 4>>&{ return t->local_approx_saxy_min(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const view_vector<std::array<double, 4>>& mv_tracker_gallery::local_approx_saxy_mid() const
	{
		auto visitor = [&](auto& t) -> const view_vector<std::array<double, 4>>& {return t->local_approx_saxy_mid(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const view_vector<std::array<double, 4>>& mv_tracker_gallery::local_approx_saxy_max() const
	{
		auto visitor = [&](auto& t) -> const view_vector<std::array<double, 4>>& {return t->local_approx_saxy_max(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const std::vector<multiview_shape2d<double>>& mv_tracker_gallery::training_shapes() const
	{
		auto visitor = [&](auto& t) -> const std::vector<multiview_shape2d<double>>& {return t->training_shapes(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const  multiview_shape2d<double>& mv_tracker_gallery::first_training_shapes() const
	{
		auto visitor = [&](auto& t) -> const multiview_shape2d<double>&{return t->first_training_shapes(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const int mv_tracker_gallery::total_number_of_features() const
	{
		auto visitor = [&](auto& t) {return t->total_number_of_features(); };
		return std::visit(visitor, tracker_ptr_);
	}

	const int mv_tracker_gallery::n_texture_modes() const
	{
		auto visitor = [&](auto& t) {return t->n_texture_modes(); };
		return std::visit(visitor, tracker_ptr_);
	}

	void mv_tracker_gallery::finalize_training(int min_samples_per_node)
	{
		auto visitor = [&](auto& t) {t->finalize_training(min_samples_per_node); };
		std::visit(visitor, tracker_ptr_);
	}

}