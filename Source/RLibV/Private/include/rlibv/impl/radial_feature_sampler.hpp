// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "enum_ext.h"
#include "basic_types.h"
#include "linear_range.h"
#include "data_utils.h"

#include "disable_dlib_warnings.h"
RLIBV_DISABLE_DLIB_WARNINGS
#include "../ThirdParty/dlib/par_image_pyramid.h"
#include <dlib/matrix.h>
#include <dlib/image_transforms/interpolation.h>
#include <dlib/gui_widgets.h>
RLIBV_RENABLE_WARNINGS

#include <vector>
#include <numeric>
#include <iostream>
#include <utility>



namespace rlibv
{
	using dlib::serialize;
	using dlib::deserialize;

	constexpr double cexp_pow(double v, double e)
	{
		return (e == 0) ? 1 : v * cexp_pow(v, e - 1);
	}
	
	template<int M, int P>
	struct raw_moment_multiplier {
		constexpr raw_moment_multiplier() : data() {
			for (auto p = 0; p < P; ++p)
			{
				data[0][p] = 1.0;
			}
			for (auto m = 1; m < M; ++m)
			{
				for (auto p = 0; p < P; ++p)
				{
					data[m][p] = cexp_pow(double(p) + 1.0, m);
				}
			}
		}
		double data[M][P];
	};

	/**
	 * Generates multipliers for moment calculations at compile time.
	 */
	template<int M, int P>
	struct raw_moment_multiplier {
		constexpr raw_moment_multiplier() : data() {
			for (auto p = 0; p < P; ++p)
			{
				data[0][p] = 1.0;
			}
			for (auto m = 1; m < M; ++m)
			{
				for (auto p = 0; p < P; ++p)
				{
					data[m][p] = cexp_pow(double(p) + 1.0, m);
				}
			}
		}
		double data[M][P];
	};

	/**
	 * Generates multipliers for moment calculations at compile time.
	 */
	constexpr auto raw_multipliers = raw_moment_multiplier<12, 64>();

	namespace meta
	{
		/**
		 * Special template meta-programming expression to compute the
		 * number of feature outputs depending on the variable number of
		 * features and their types.
		 *
		 * @param ...args variable length argument list of feature types
		 * @return the total number of features extracted by a sampler.
		 */
		template<int R, typename... ArgTypes>
		constexpr int count_feature_outputs(ArgTypes... args) { return 0; }
		template<int R, typename U, typename... ArgTypes>
		constexpr int count_feature_outputs(U t, ArgTypes... args)
		{
			switch (t)
			{
			case feature_type::poly1:
				return 2 + count_feature_outputs<R>(args...);
			case feature_type::poly2:
				return 3 + count_feature_outputs<R>(args...);
			case feature_type::gradmag_poly2:
				return 3 + count_feature_outputs<R>(args...);
			case feature_type::poly3:
				return 4 + count_feature_outputs<R>(args...);
			case feature_type::moments:
				return 6 + count_feature_outputs<R>(args...);
			case feature_type::raw:
				return R + count_feature_outputs<R>(args...);
			default:
				return 0;
			}
		}
		template<int R> constexpr int count_feature_outputs()
		{
			return 0;
		}
	}


	template<typename T, int R, int S, feature_type... args>
	constexpr int radial_feature_sampler<T,R,S,args...>::n_channels()
	{
		if (dlib::pixel_traits<T>::rgb)
		{
			return 3;
		}
		if (dlib::pixel_traits<T>::rgb_alpha)
		{
			return  4;
		}
		return  1;

	}

	template<typename T, int R, int S, feature_type... args>
	constexpr int n_radials()
	{
		return R / 2;
	}

	template<typename T, int R, int S, feature_type... args>
	constexpr int number_of_raw_samples()
	{
		return n_channels() * n_radials() * S * R;
	}

	template<typename T, int R, int S, feature_type... args>
	constexpr int total_number_of_features()
	{
		return n_channels() * n_radials() * S * meta::count_feature_outputs<R>(args...);
	}

	template<typename T, int R, int S, feature_type... args>
	dlib::dpoint get_sample_point(
		dlib::dpoint pt,
		long pyr_level) const
	{
		auto sample_pt = pt;
		for (int i = 0; i < pyr_level; ++i)
		{
			sample_pt = sample_pt / 2.0 - dlib::dpoint(1.25, 0.75);
		}
		sample_pt += pyr_rects_[pyr_level].tl_corner();
		return sample_pt;
	}

	template<typename T, int R, int S, feature_type... args>
	std::array<std::array<std::array<dlib::dpoint, R>, n_radials()>, S> radial_feature_sampler<T,R,S,args...>::radial_sample_points(dlib::dpoint pt, double angle, double scale) const
	{
		std::array<std::array<std::array<dlib::dpoint, R>, n_radials()>, S> points;

		for (auto s : linear_range<int, S>(0, S - 1))
		{
			double this_scale = 1.0 / (std::pow(2, s) * scale);
			double this_half_length_ = half_length_ / this_scale;
			auto pyramid_level = static_cast<long>(std::log(this_scale) / log_pyramid_rate_ + 0.5);
			pyramid_level = dlib::put_in_range(0, (long)pyr_rects_.size() - 1, pyramid_level);
			double this_angle = angle;

			for (auto a : linear_range<int, n_radials()>(0, n_radials() - 1))
			{
				double x1 = this_half_length_ * sin(this_angle) + pt.x();
				double y1 = this_half_length_ * cos(this_angle) + pt.y();
				double reflected_angle = this_angle + dlib::pi;
				double x2 = this_half_length_ * sin(reflected_angle) + pt.x();
				double y2 = this_half_length_ * cos(reflected_angle) + pt.y();
				auto first = get_sample_point(dlib::dpoint(x1, y1), pyramid_level);
				auto last = get_sample_point(dlib::dpoint(x2, y2), pyramid_level);
				auto pts = linear_range<dlib::dpoint, R>(first, last);
				for (auto i : linear_range<int, R>(0, R - 1))
				{
					points[s][a][i] = pts[i];
				}
				this_angle += angle_step_;
			}
		}
		return points;
	}

	template<typename T, int R, int S, feature_type... args>
	void radial_feature_sampler<T,R,S,args...>::set_image(const dlib::array2d<dlib::rgb_pixel>& img)
	{
		par_pyramid_down<2> pyr;
		if (last_nc_ != img.nc() || last_nr_ != img.nr())
		{
			rlibv::impl::compute_tiled_image_pyramid_details(pyr, img.nr(), img.nc(), 1, 0, pyr_rects_, tp_nr_, tp_nc_);
			set_image_size(tiled_pyr_img_, tp_nr_, tp_nc_);
			assign_all_pixels(tiled_pyr_img_, 0);
			last_nc_ = img.nc();
			last_nr_ = img.nr();
		}


		// now build the image pyramid into out_img
		auto si = sub_image(tiled_pyr_img_, pyr_rects_[0]);
		assign_image(si, img);
		for (size_t i = 1; i < pyr_rects_.size(); ++i)
		{
			auto s1 = sub_image(tiled_pyr_img_, pyr_rects_[i - 1]);
			auto s2 = sub_image(tiled_pyr_img_, pyr_rects_[i]);
			pyr(s1, s2);
		}
	}

	template<typename T, int R, int S, feature_type... args>
	const dlib::array2d<T>& radial_feature_sampler<T,R,S,args...>::tiled_pyr_image() const
	{
		return tiled_pyr_img_;
	}

	template<typename T, int R, int S, feature_type... args>
	typename std::enable_if<std::is_same<T, dlib::rgb_pixel>::value, fixed_size_array2d<unsigned char, R, 3>>::type 
		radial_feature_sampler<T,R,S,args...>::get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
	{
		fixed_size_array2d<unsigned char, R, 3> raw_line_sample;
		for (int p = 0; p < R; ++p)
		{
			auto y = locations[p].y();
			auto x = locations[p].x();
			y = std::min(static_cast<double>(tiled_pyr_img_.nr()) - 1, std::max(0., y));
			x = std::min(static_cast<double>(tiled_pyr_img_.nc()) - 1, std::max(0., x));
			dlib::rgb_pixel interp;
			if (bilinear)
			{
				bilin_(dlib::const_image_view(tiled_pyr_img_), dlib::dpoint(x, y), interp);
			}
			else
			{
				interp = tiled_pyr_img_[std::round(y)][std::round(x)];
			}
			raw_line_sample[p][0] = interp.red;
			raw_line_sample[p][1] = interp.green;
			raw_line_sample[p][2] = interp.blue;
		}
		return raw_line_sample;
	}


	template<typename T, int R, int S, feature_type... args>
	typename std::enable_if<std::is_same<T, dlib::rgb_alpha_pixel>::value, fixed_size_array2d<unsigned char, R, 4>>::type
		radial_feature_sampler<T,R,S,args...>::get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
	{
		fixed_size_array2d<unsigned char, R, 4> raw_line_sample;
		for (int p = 0; p < R; ++p)
		{
			auto y = locations[p].y();
			auto x = locations[p].x();
			y = std::min(static_cast<double>(tiled_pyr_img_.nr()) - 1, std::max(0., y));
			x = std::min(static_cast<double>(tiled_pyr_img_.nc()) - 1, std::max(0., x));
			dlib::rgb_pixel interp;
			if (bilinear)
			{
				bilin_(dlib::const_image_view(tiled_pyr_img_), dlib::dpoint(x, y), interp);
			}
			else
			{
				interp = tiled_pyr_img_[std::round(y)][std::round(x)];
			}
			raw_line_sample[p][0] = interp.red;
			raw_line_sample[p][1] = interp.green;
			raw_line_sample[p][2] = interp.blue;
			raw_line_sample[p][3] = interp.alpha;
		}
		return raw_line_sample;
	}

	template<typename T, int R, int S, feature_type... args>
	typename std::enable_if<std::is_same<T, unsigned char>::value, fixed_size_array2d<unsigned char, R, 1>>::type
		radial_feature_sampler<T,R,S,args...>::get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
	{
		fixed_size_array2d<unsigned char, R, 1> raw_line_sample;
		for (int p = 0; p < R; ++p)
		{
			auto y = locations[p].y();
			auto x = locations[p].x();
			y = std::min(static_cast<double>(tiled_pyr_img_.nr()) - 1, std::max(0., y));
			x = std::min(static_cast<double>(tiled_pyr_img_.nc()) - 1, std::max(0., x));
			unsigned char interp;
			if (bilinear)
			{
				bilin_(dlib::const_image_view(tiled_pyr_img_), dlib::dpoint(x, y), interp);
			}
			else
			{
				interp = tiled_pyr_img_[std::round(y)][std::round(x)];
			}
			raw_line_sample[p][0] = interp;

		}
		return raw_line_sample;
	}

	
	template<typename T, int R, int S, feature_type... args>
	std::vector<unsigned char> radial_feature_sampler<T,R,S,args...>::get_radial_samples(dlib::dpoint pt, T angle, T scale, bool bilinear) const
	{
		std::vector<unsigned char> samples(number_of_raw_samples());
		auto sample_locations = radial_sample_points(pt, angle, scale);
		int pos = 0;
		for (int s = 0; s < S; ++s)
		{
			for (int a = 0; a < n_radials(); ++a)
			{
				auto uchar_line_sample = get_uchar_line_sample(sample_locations[s][a], bilinear);
				for (int c = 0; c < n_channels(); ++c)
				{
					for (int i = 0; i < R; ++i)
					{
						samples[pos++] = uchar_line_sample[i][c];

					}
				}
			}
		}
		return samples;
	}

	template<typename T, int R, int S, feature_type... args>
	dlib::matrix<double, total_number_of_features(), 1> radial_feature_sampler<T,R,S,args...>::features_from_samples(const std::vector<unsigned char>& sample) const
	{
		dlib::matrix<double, total_number_of_features(), 1> features;
		dlib::matrix<double, R, 1> line_sample;
		int pos = 0;
		for (int s = 0; s < sample.size(); s += R)
		{
			int ls = 0;
			for (int i = s; i < s + R; ++i)
			{
				line_sample(ls++) = sample[i];
			}
			line_sample = line_sample - dlib::mean(line_sample);

			for (const auto& ft : features_)
			{
				if (ft == feature_type::poly1)
				{
					std::array<double, R> y;
					for (int i = 0; i < R; ++i)
					{
						y[i] = line_sample(i);
					}
					std::array<double, 2> coeffs;
					linfit(y, coeffs);
					features(pos++) = coeffs[0];
					features(pos++) = coeffs[1];

				}
				else if (ft == feature_type::poly2)
				{

					std::array<double, R> y;
					for (int i = 0; i < R; ++i)
					{
						y[i] = line_sample(i);
					}
					std::array<double, 3> coeffs;
					quadfit(y, coeffs);
					features(pos++) = coeffs[0];
					features(pos++) = coeffs[1];
					features(pos++) = coeffs[2];

				}
				else if (ft == feature_type::gradmag_poly2)
				{
					std::array<double, R> y;
					y[0] = 0;
					for (int i = 1; i < R; ++i)
					{
						y[i] = std::abs(line_sample(i) - line_sample(i - 1));
					}
					std::array<double, 3> coeffs;
					quadfit(y, coeffs);
					features(pos++) = coeffs[0];
					features(pos++) = coeffs[1];
					features(pos++) = coeffs[2];

				}
				else if (ft == feature_type::poly3)
				{

					std::array<double, R> y;
					y[0] = 0;
					for (int i = 0; i < R; ++i)
					{
						y[i] = std::abs(line_sample(i) - line_sample(i - 1));
					}
					std::array<double, 4> coeffs;
					cubicfit(y, coeffs);
					features(pos++) = coeffs[0];
					features(pos++) = coeffs[1];
					features(pos++) = coeffs[2];
					features(pos++) = coeffs[3];

				}
				else if (ft == feature_type::moments)
				{
					const auto rows = line_sample.nr();
					for (int m = 1; m <= 6; ++m)
					{

						auto multi_ptr = &raw_multipliers.data[m][0];
						features(pos) = 0.0;
						for (int i = 0; i < rows; ++i)
						{
							features(pos) += *multi_ptr++ * line_sample(i);
						}
						pos++;
					}

				}
				else if (ft == feature_type::raw)
				{

					for (int i = 0; i < R; ++i)
					{
						features(pos++) = line_sample(i);
					}

				}
			}

		}
		return features;
	}


	template<typename T, int R, int S, feature_type... args>
	bool radial_feature_sampler<T,R,S,args...>::is_set() const
	{
		return (pyr_rects_.size() > 0);
	}

	
}

