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

	DECLARE_ENUM(feature_type, raw, poly2, poly3, gradmag_poly2, moments, poly1)

	/**
	* \defgroup ImageProcessing Image Processing
	* @{
	*/

	/**
	 * @brief Compile-time function to compute v^e
	 * @param v base
	 * @param e exponent
	 * @return v^e
	 */
	constexpr double cexp_pow(double v, double e)
	{
		return (e == 0) ? 1 : v * cexp_pow(v, e - 1);
	}

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

	/**
	 * @brief A class representing a Radial Feature Sampler (RFS)
	 * @tparam T the image type
	 * @tparam R the resolution of each sampling line, which also automatically
	 *           sets the number of sampling lines to be R/2. R must be even.
	 * @tparam S the number of sampling scales
	 * @tparam args any number of feature types
	 * @pre R must be even
	 */
	template<typename T, int R, int S, feature_type... args>
	class radial_feature_sampler
	{
		static_assert(
			std::is_same<T, dlib::rgb_pixel>::value ||
			std::is_same<T, dlib::rgb_alpha_pixel>::value ||
			std::is_same<T, unsigned char>::value,
			"T must be a pixel type");

		static_assert(R > 0 && (R % 2) == 0,
			"R must be even and greater than 2");

		static_assert(S > 0 && S < 9,
			"S must be between 1 and 9");

	public:

		/**
		 * Get the number of image channels used by this sampler.
		 *
		 * @return the number of channels.
		 */
		static constexpr int n_channels()
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

		/**
		 * Get the number of radials in this sampler.
		 *
		 * @return the max number of radials.
		 */
		static constexpr int n_radials()
		{
			return R / 2;
		}

		/**
		 * Return the number of features extracted by this sampler.
		 *
		 * @return the number of features
		 */
		static constexpr int number_of_raw_samples()
		{
			return n_channels() * n_radials() * S * R;
		}

		/**
		 * Return the number of features extracted by this sampler.
		 *
		 * @return the number of features
		 */
		static constexpr int total_number_of_features()
		{
			return n_channels() * n_radials() * S * meta::count_feature_outputs<R>(args...);
		}

		/**
		 * Given a point and pyramid level, compute its location in the tiled image pyramid.
		 *
		 * @param pt sample pt in the original image world
		 * @param pyr_level the pyramid level
		 * @return location of the suitably scaled sample point in the tiled pyramid
		 */
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

		/**
		 * Get the entire set of sampling locations on the radials around a centre point at a given global scale and angle.
		 *
		 * @param pt The centre point
		 * @param angle The global angle
		 * @param scale The global scale
		 * @return A nested array of points at S scales, A angles, at resolution R
		 */
		std::array<std::array<std::array<dlib::dpoint, R>, n_radials()>, S> radial_sample_points(dlib::dpoint pt, double angle, double scale) const
		{
			std::array<std::array<std::array<dlib::dpoint, R>, n_radials()>, S> points;

			for (auto s : linear_range<int, S>(0, S - 1))
			{
				double this_scale = 1.0 / (std::pow(2, s) * scale);
				double this_half_length_ = half_length_ / this_scale;
				auto pyramid_level = static_cast<long>(std::log(this_scale) / log_pyramid_rate_ + 0.5);
				pyramid_level = static_cast<long>(dlib::put_in_range(0, (long)pyr_rects_.size() - 1, pyramid_level));
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

		/**
		 * Initialize the sampler with an image, setting it up for feature extraction.
		 *
		 * @param img The image.
		 */
		void set_image(const dlib::array2d<dlib::rgb_pixel>& img)
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

		/**
		 * Get a constant reference to the tiled image pyramid created by set_image(...)
		 *
		 * @return const ref to tiled image pyramid
		 */
		const dlib::array2d<T>& tiled_pyr_image() const
		{
			return tiled_pyr_img_;
		}

		/**
		 * Extract the sample vector from a 3 channel image for a given line.
		 *
		 * @param locations sample point locations (in original image space)
		 * @param bilinear use bilinear interpolation (true by default)
		 * @return Rx3 unsigned char fixed_size_array2d of sample values (one column for each channel)
		 */
		template<typename U = T>
		typename std::enable_if<std::is_same<U, dlib::rgb_pixel>::value, fixed_size_array2d<unsigned char, R, 3>>::type 
			get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
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
					interp = tiled_pyr_img_[static_cast<long>(std::round(y))][static_cast<long>(std::round(x))];
				}
				raw_line_sample[p][0] = interp.red;
				raw_line_sample[p][1] = interp.green;
				raw_line_sample[p][2] = interp.blue;
			}
			return raw_line_sample;
		}

		/**
		* Extract the sample vector from a 4 channel image for a given line.
		*
		* @param locations sample point locations (in original image space)
		* @param bilinear use bilinear interpolation (true by default)
		* @return Rx3 unsigned char fixed_size_array2d of sample values (one column for each channel)
		*/
#if 0 // interp.alpha does not exist, interp variable is a dlib::rgb_pixel
		template<typename U = T>
		typename std::enable_if<std::is_same<U, dlib::rgb_alpha_pixel>::value, fixed_size_array2d<unsigned char, R, 4>>::type
			get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
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
#endif

		/**
		* Extract the sample vector from a single channel image for a given line.
		*
		* @param locations sample point locations (in original image space)
		* @param bilinear use bilinear interpolation (true by default)
		* @return Rx3 unsigned char fixed_size_array2d of sample values (one column for each channel)
		*/
		template<typename U = T>
		typename std::enable_if<std::is_same<U, unsigned char>::value, fixed_size_array2d<unsigned char, R, 1>>::type
			get_uchar_line_sample(const std::array<dlib::dpoint, R>& locations, bool bilinear) const
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
					interp = tiled_pyr_img_[static_cast<int>(std::round(y))][static_cast<int>(std::round(x))];
				}
				raw_line_sample[p][0] = interp;

			}
			return raw_line_sample;
		}

		/**
		* Get the raw unsigned char radial samples
		*
		* @param pt The centre point
		* @param angle The global angle for the point
		* @param scale The global scale for the point
		* @param bilinear Set to true if you want to use bilinear interpolation
		* @return vector of unsigned char containing all the extracted features
		*/
		std::vector<unsigned char> get_radial_samples(dlib::dpoint pt, double angle, double scale, bool bilinear) const
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

		/**
		 * Extract the complete vector of features from an extracted raw sample.
		 *
		 * @param sample The raw sample
		 * @return dlib column vector containing all the extracted features
		 */
		dlib::matrix<double, total_number_of_features(), 1> features_from_samples(const std::vector<unsigned char>& sample) const
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


		/**
		 * Check if set_image has been called successfully.
		 *
		 * @return true if and only set_image has been called successfully.
		 */
		bool is_set() const
		{
			return (pyr_rects_.size() > 0);
		}

	private:
		dlib::array2d<T> tiled_pyr_img_;
		std::vector<dlib::rectangle> pyr_rects_;
		const double pyramid_rate_ = 0.5;
		const double log_pyramid_rate_ = std::log(pyramid_rate_);
		const double angle_step_ = dlib::pi / n_radials();
		const double half_length_ = R / 2.0;
		inline static auto features_ = { args... };
		dlib::interpolate_bilinear bilin_;
		int last_nr_ = -1;
		int last_nc_ = -1;
		long tp_nr_ = -1;
		long tp_nc_ = -1;

		template<size_t TR = R>
		void linfit(const std::array<double, TR>& y, std::array<double, 2>& coeffs) const
		{
			dlib::matrix<double, 0, 2> X; X.set_size(static_cast<long>(TR), 2);
			dlib::matrix<double, 0, 1> Y; Y.set_size(static_cast<long>(y.size()), 1);
			for (auto i = 0; i < TR; ++i)
			{
				X(i, 0) = 1;
				X(i, 1) = i;
				Y(i) = y[i];
			}
			dlib::matrix<double> coeffs_mat = pinv(X) * Y;
			for (auto d = 0; d < 2; ++d)
			{
				coeffs[d] = coeffs_mat(d);
			}
		}

		template<size_t TR = R>
		void cubicfit(const std::array<double, TR>& y, std::array<double, 4>& coeffs) const
		{
			dlib::matrix<double, 0, 4> X; X.set_size(static_cast<long>(TR), 4);
			dlib::matrix<double, 0, 1> Y; Y.set_size(static_cast<long>(y.size()), 1);
			for (auto i = 0; i < TR; ++i)
			{
				X(i, 0) = 1;
				Y(i) = y[i];
				double xp = i;
				for (auto d = 1; d < 4; ++d)
				{
					X(i, d) = xp;
					xp *= xp;
				}
			}
			dlib::matrix<double> coeffs_mat = pinv(X) * Y;
			for (auto d = 0; d < 4; ++d)
			{
				coeffs[d] = coeffs_mat(d);
			}
		}

		template<size_t TR = R>
		void quadfit(const std::array<double, TR>& y, std::array<double, 3>& coeffs) const
		{
			dlib::matrix<double, 3, 3> X;
			dlib::matrix<double, 3, 1> Y;

			double sx4 = 0;
			double sx3 = 0;
			double sx2 = 0;
			double sx = 0;
			double n = 0;
			double sx2y = 0;
			double sxy = 0;
			double sy = 0;

			for (int i = 0; i < TR; ++i)
			{
				double tx = i;
				double ty = y[i];
				sxy += tx * ty;
				sy += ty;
				sx += tx;
				sx2 += tx * tx;
				sx2y += tx * tx * ty;
				sx3 += tx * tx * tx;
				sx4 += tx * tx * tx * tx;
				n++;
			}
			X(0, 0) = sx4; X(0, 1) = sx3; X(0, 2) = sx2;
			X(1, 0) = sx3; X(1, 1) = sx2; X(1, 2) = sx;
			X(2, 0) = sx2; X(2, 1) = sx;  X(2, 2) = n;
			Y(0) = sx2y;
			Y(1) = sxy;
			Y(2) = sy;

			dlib::matrix<double, 3, 1> CM = inv(X) * Y; // Using just C clashes with template parameter
			coeffs[0] = CM(2);
			coeffs[1] = CM(1);
			coeffs[2] = CM(0);

		}
	};
	/**@}*/
}

