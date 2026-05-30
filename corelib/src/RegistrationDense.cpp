/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/RegistrationDense.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace rtabmap {

RegistrationDense::RegistrationDense(const ParametersMap & parameters, Registration * child) :
	Registration(parameters, child),
	_iterations(Parameters::defaultDenseIterations()),
	_pyramidLevels(Parameters::defaultDensePyramidLevels()),
	_decimation(Parameters::defaultDenseDecimation()),
	_minDepth(Parameters::defaultDenseMinDepth()),
	_maxDepth(Parameters::defaultDenseMaxDepth()),
	_minGradient(Parameters::defaultDenseMinGradient()),
	_geometricWeight(Parameters::defaultDenseGeometricWeight()),
	_photometricWeight(Parameters::defaultDensePhotometricWeight()),
	_photoIlluminationInvariant(Parameters::defaultDenseIlluminationInvariant()),
	_huberThreshold(Parameters::defaultDenseHuberThreshold()),
	_convergenceEps(Parameters::defaultDenseConvergenceEps()),
	_maxCorrespondenceDepthDiff(Parameters::defaultDenseMaxCorrespondenceDepthDiff()),
	_maxTranslation(Parameters::defaultDenseMaxTranslation()),
	_maxRotation(Parameters::defaultDenseMaxRotation())
{
	this->parseParameters(parameters);
}

RegistrationDense::~RegistrationDense()
{
}

void RegistrationDense::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kDenseIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kDensePyramidLevels(), _pyramidLevels);
	Parameters::parse(parameters, Parameters::kDenseDecimation(), _decimation);
	Parameters::parse(parameters, Parameters::kDenseMinDepth(), _minDepth);
	Parameters::parse(parameters, Parameters::kDenseMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kDenseMinGradient(), _minGradient);
	Parameters::parse(parameters, Parameters::kDenseGeometricWeight(), _geometricWeight);
	Parameters::parse(parameters, Parameters::kDensePhotometricWeight(), _photometricWeight);
	Parameters::parse(parameters, Parameters::kDenseIlluminationInvariant(), _photoIlluminationInvariant);
	Parameters::parse(parameters, Parameters::kDenseHuberThreshold(), _huberThreshold);
	Parameters::parse(parameters, Parameters::kDenseConvergenceEps(), _convergenceEps);
	Parameters::parse(parameters, Parameters::kDenseMaxCorrespondenceDepthDiff(), _maxCorrespondenceDepthDiff);
	Parameters::parse(parameters, Parameters::kDenseMaxTranslation(), _maxTranslation);
	Parameters::parse(parameters, Parameters::kDenseMaxRotation(), _maxRotation);

	if(_pyramidLevels < 1)
	{
		_pyramidLevels = 1;
	}
	if(_decimation < 1)
	{
		_decimation = 1;
	}
}

//////////////////////////////////////////////////////////////////////////////
// Local helpers
//////////////////////////////////////////////////////////////////////////////
namespace {

// Per-pyramid-level pre-computed data.
struct DenseLevel
{
	// reference ("from") frame
	cv::Mat intensityFrom; // CV_32F (0-255)
	cv::Mat depthFrom;     // CV_32F (meters)
	// target ("to") frame
	cv::Mat intensityTo;   // CV_32F (0-255)
	cv::Mat gradXTo;       // CV_32F (dI/du, intensity/pixel)
	cv::Mat gradYTo;       // CV_32F (dI/dv, intensity/pixel)
	cv::Mat depthTo;       // CV_32F (meters)
	cv::Mat normalsTo;     // CV_32FC3 (unit normal in target optical frame, (0,0,0)=invalid)
	// scaled intrinsics
	double fxFrom, fyFrom, cxFrom, cyFrom;
	double fxTo, fyTo, cxTo, cyTo;
	int width, height;
	// source pixel sampling step
	int step;
};

// Convert an image to single-channel float intensity (0-255 range).
cv::Mat toIntensity(const cv::Mat & image)
{
	cv::Mat gray;
	if(image.channels() == 3)
	{
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	else if(image.channels() == 4)
	{
		cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
	}
	else
	{
		gray = image;
	}
	cv::Mat out;
	gray.convertTo(out, CV_32F);
	return out;
}

// Convert a depth image (CV_16U mm or CV_32F m) to CV_32F meters.
cv::Mat toDepthMeters(const cv::Mat & depth)
{
	cv::Mat out;
	if(depth.type() == CV_16UC1)
	{
		depth.convertTo(out, CV_32F, 1.0/1000.0);
	}
	else if(depth.type() == CV_32FC1)
	{
		out = depth.clone();
	}
	else
	{
		depth.convertTo(out, CV_32F);
	}
	return out;
}

// Bilinear sampling of a CV_32F single-channel image. Returns false if out of bounds.
inline bool sampleBilinear(const cv::Mat & img, double u, double v, double & value)
{
	if(u < 0.0 || v < 0.0 || u > img.cols-1.0 || v > img.rows-1.0)
	{
		return false;
	}
	int u0 = (int)std::floor(u);
	int v0 = (int)std::floor(v);
	int u1 = std::min(u0+1, img.cols-1);
	int v1 = std::min(v0+1, img.rows-1);
	double au = u - u0;
	double av = v - v0;
	const float * r0 = img.ptr<float>(v0);
	const float * r1 = img.ptr<float>(v1);
	double top = r0[u0]*(1.0-au) + r0[u1]*au;
	double bot = r1[u0]*(1.0-au) + r1[u1]*au;
	value = top*(1.0-av) + bot*av;
	return true;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d & w)
{
	Eigen::Matrix3d m;
	m <<     0.0, -w.z(),  w.y(),
		   w.z(),    0.0, -w.x(),
		  -w.y(),  w.x(),    0.0;
	return m;
}

// SE(3) exponential map for a twist [v(0:3); w(3:6)].
Eigen::Matrix4d expSE3(const Eigen::Matrix<double,6,1> & xi)
{
	Eigen::Vector3d v = xi.head<3>();
	Eigen::Vector3d w = xi.tail<3>();
	double th = w.norm();
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d wx = skew(w);
	Eigen::Matrix3d R;
	Eigen::Matrix3d V;
	if(th < 1e-10)
	{
		R = I + wx;
		V = I + 0.5*wx;
	}
	else
	{
		double th2 = th*th;
		double th3 = th2*th;
		double a = std::sin(th)/th;
		double b = (1.0-std::cos(th))/th2;
		double c = (th-std::sin(th))/th3;
		R = I + a*wx + b*(wx*wx);
		V = I + b*wx + c*(wx*wx);
	}
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	T.topLeftCorner<3,3>() = R;
	T.block<3,1>(0,3) = V*v;
	return T;
}

// Compute a dense normal map (in the optical frame) from a metric depth image.
cv::Mat computeNormals(
		const cv::Mat & depth,
		double fx, double fy, double cx, double cy,
		float minDepth, float maxDepth, float maxNeighborDepthDiff)
{
	cv::Mat normals = cv::Mat::zeros(depth.size(), CV_32FC3);
	for(int v=1; v<depth.rows-1; ++v)
	{
		const float * dC = depth.ptr<float>(v);
		const float * dU = depth.ptr<float>(v-1);
		const float * dD = depth.ptr<float>(v+1);
		cv::Vec3f * nRow = normals.ptr<cv::Vec3f>(v);
		for(int u=1; u<depth.cols-1; ++u)
		{
			float d = dC[u];
			if(d <= minDepth || (maxDepth>0.0f && d>maxDepth))
			{
				continue;
			}
			float dl = dC[u-1];
			float dr = dC[u+1];
			float du = dU[u];
			float dd = dD[u];
			if(dl<=minDepth || dr<=minDepth || du<=minDepth || dd<=minDepth)
			{
				continue;
			}
			// reject across depth discontinuities
			if(std::fabs(dl-dr) > maxNeighborDepthDiff || std::fabs(du-dd) > maxNeighborDepthDiff)
			{
				continue;
			}
			// neighbor 3D points (optical frame)
			Eigen::Vector3d pl(((u-1)-cx)*dl/fx, (v-cy)*dl/fy, dl);
			Eigen::Vector3d pr(((u+1)-cx)*dr/fx, (v-cy)*dr/fy, dr);
			Eigen::Vector3d pu((u-cx)*du/fx, ((v-1)-cy)*du/fy, du);
			Eigen::Vector3d pd((u-cx)*dd/fx, ((v+1)-cy)*dd/fy, dd);
			Eigen::Vector3d n = (pr-pl).cross(pd-pu);
			double norm = n.norm();
			if(norm < 1e-9)
			{
				continue;
			}
			n /= norm;
			// orient towards the camera (optical z+ points forward, viewing ray ~ +z)
			if(n.z() > 0.0)
			{
				n = -n;
			}
			nRow[u] = cv::Vec3f((float)n.x(), (float)n.y(), (float)n.z());
		}
	}
	return normals;
}

inline float depthAt(const cv::Mat & depth, int u, int v)
{
	if(u < 0 || v < 0 || u >= depth.cols || v >= depth.rows)
	{
		return 0.0f;
	}
	return depth.ptr<float>(v)[u];
}

// Per-correspondence cached data for the two-pass IRLS accumulation.
struct Corr
{
	Eigen::Vector3d Y;   // warped point in target optical frame
	double rp;           // photometric residual (filled after optional normalization)
	double iFrom, iTo;   // raw reference / warped-target intensities
	double gx, gy;       // target image gradient at warped pixel
	bool hasPhoto;
	Eigen::Vector3d n;   // target normal
	double rg;           // geometric (point-to-plane) residual
	bool hasGeo;
};

double robustScale(std::vector<double> & absResiduals)
{
	if(absResiduals.empty())
	{
		return 0.0;
	}
	size_t mid = absResiduals.size()/2;
	std::nth_element(absResiduals.begin(), absResiduals.begin()+mid, absResiduals.end());
	double median = absResiduals[mid];
	// 1.4826 * MAD assuming approximately zero-centered residuals
	double sigma = 1.4826 * median;
	return sigma;
}

} // anonymous namespace

//////////////////////////////////////////////////////////////////////////////
// Registration implementation
//////////////////////////////////////////////////////////////////////////////
Transform RegistrationDense::computeTransformationImpl(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess,
			RegistrationInfo & info) const
{
	UDEBUG("Dense refinement: guess=%s", guess.prettyPrint().c_str());

	// We can only refine an existing estimate.
	if(guess.isNull())
	{
		UDEBUG("Dense refinement skipped: no initial guess provided.");
		return guess;
	}

	const SensorData & dataFrom = fromSignature.sensorData();
	const SensorData & dataTo = toSignature.sensorData();

	// Resolve a single camera model (mono RGB-D or stereo-left) for both frames.
	auto getModel = [](const SensorData & data, CameraModel & model)->bool
	{
		if(data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForProjection())
		{
			model = data.cameraModels()[0];
			return true;
		}
		if(data.stereoCameraModels().size() == 1 && data.stereoCameraModels()[0].left().isValidForProjection())
		{
			model = data.stereoCameraModels()[0].left();
			return true;
		}
		return false;
	};

	CameraModel modelFrom, modelTo;
	if(!getModel(dataFrom, modelFrom) || !getModel(dataTo, modelTo))
	{
		UDEBUG("Dense refinement skipped: a single valid camera model is required on both frames.");
		return guess;
	}

	cv::Mat imageFrom = dataFrom.imageRaw();
	cv::Mat imageTo = dataTo.imageRaw();
	cv::Mat depthFromRaw = dataFrom.depthRaw();
	cv::Mat depthToRaw = dataTo.depthRaw();
	if(imageFrom.empty() || imageTo.empty() || depthFromRaw.empty() || depthToRaw.empty())
	{
		UWARN("Dense refinement skipped: requires raw image and depth on both frames "
			  "(from: image=%d depth=%d, to: image=%d depth=%d). Make sure depth data is "
			  "available (RGB-D input).",
			  imageFrom.empty()?0:1, depthFromRaw.empty()?0:1,
			  imageTo.empty()?0:1, depthToRaw.empty()?0:1);
		return guess;
	}

	// Full-resolution intensity + metric depth.
	cv::Mat intensityFromFull = toIntensity(imageFrom);
	cv::Mat intensityToFull = toIntensity(imageTo);
	cv::Mat depthFromFull = toDepthMeters(depthFromRaw);
	cv::Mat depthToFull = toDepthMeters(depthToRaw);

	// Depth maps must match their intensity image resolution.
	if(depthFromFull.size() != intensityFromFull.size())
	{
		cv::resize(depthFromFull, depthFromFull, intensityFromFull.size(), 0, 0, cv::INTER_NEAREST);
	}
	if(depthToFull.size() != intensityToFull.size())
	{
		cv::resize(depthToFull, depthToFull, intensityToFull.size(), 0, 0, cv::INTER_NEAREST);
	}

	const int W = intensityFromFull.cols;
	const int H = intensityFromFull.rows;
	if(W < 8 || H < 8 || intensityToFull.cols != W || intensityToFull.rows != H)
	{
		// Different image sizes between frames are not supported by the projective warp.
		UDEBUG("Dense refinement skipped: incompatible image sizes "
			   "(from=%dx%d, to=%dx%d).", W, H, intensityToFull.cols, intensityToFull.rows);
		return guess;
	}

	// Base (level 0) intrinsics.
	const double fxFrom0 = modelFrom.fx();
	const double fyFrom0 = modelFrom.fy();
	const double cxFrom0 = modelFrom.cx();
	const double cyFrom0 = modelFrom.cy();
	const double fxTo0 = modelTo.fx();
	const double fyTo0 = modelTo.fy();
	const double cxTo0 = modelTo.cx();
	const double cyTo0 = modelTo.cy();

	// Build the coarse-to-fine pyramid (index 0 = finest/full resolution).
	std::vector<DenseLevel> pyramid(_pyramidLevels);
	for(int l=0; l<_pyramidLevels; ++l)
	{
		DenseLevel & lvl = pyramid[l];
		int newW = std::max(1, W >> l);
		int newH = std::max(1, H >> l);
		double sx = (double)newW / (double)W;
		double sy = (double)newH / (double)H;

		if(l == 0)
		{
			lvl.intensityFrom = intensityFromFull;
			lvl.intensityTo = intensityToFull;
			lvl.depthFrom = depthFromFull;
			lvl.depthTo = depthToFull;
		}
		else
		{
			cv::resize(intensityFromFull, lvl.intensityFrom, cv::Size(newW,newH), 0, 0, cv::INTER_LINEAR);
			cv::resize(intensityToFull, lvl.intensityTo, cv::Size(newW,newH), 0, 0, cv::INTER_LINEAR);
			cv::resize(depthFromFull, lvl.depthFrom, cv::Size(newW,newH), 0, 0, cv::INTER_NEAREST);
			cv::resize(depthToFull, lvl.depthTo, cv::Size(newW,newH), 0, 0, cv::INTER_NEAREST);
		}
		lvl.width = newW;
		lvl.height = newH;

		// Scaled intrinsics (pixel-center convention: c' = (c+0.5)*s - 0.5).
		lvl.fxFrom = fxFrom0*sx;
		lvl.fyFrom = fyFrom0*sy;
		lvl.cxFrom = (cxFrom0+0.5)*sx - 0.5;
		lvl.cyFrom = (cyFrom0+0.5)*sy - 0.5;
		lvl.fxTo = fxTo0*sx;
		lvl.fyTo = fyTo0*sy;
		lvl.cxTo = (cxTo0+0.5)*sx - 0.5;
		lvl.cyTo = (cyTo0+0.5)*sy - 0.5;

		// Target image gradients (intensity/pixel). Sobel-3 has a factor of 8.
		cv::Sobel(lvl.intensityTo, lvl.gradXTo, CV_32F, 1, 0, 3, 1.0/8.0);
		cv::Sobel(lvl.intensityTo, lvl.gradYTo, CV_32F, 0, 1, 3, 1.0/8.0);

		// Target normals (for the point-to-plane term).
		lvl.normalsTo = computeNormals(
				lvl.depthTo, lvl.fxTo, lvl.fyTo, lvl.cxTo, lvl.cyTo,
				_minDepth, _maxDepth, _maxCorrespondenceDepthDiff);

		lvl.step = (l==0)?_decimation:1;
	}

	// Local transforms (optical -> base frame).
	Eigen::Matrix4d Lf = modelFrom.localTransform().toEigen4d();
	Eigen::Matrix4d Lt = modelTo.localTransform().toEigen4d();
	Eigen::Matrix4d LtInv = Lt.inverse();

	// W maps from-optical -> to-optical. W = Lt^{-1} * guess^{-1} * Lf.
	Eigen::Matrix4d Tguess = guess.toEigen4d();
	Eigen::Matrix4d Wopt = LtInv * Tguess.inverse() * Lf;

	const double minDepth = _minDepth;
	const double maxDepth = _maxDepth;
	const double huberK = _huberThreshold;
	const double photoW = _photometricWeight;
	const double geoW = _geometricWeight;

	int totalUsedFinest = 0;

	// Coarse-to-fine optimization.
	for(int l=_pyramidLevels-1; l>=0; --l)
	{
		const DenseLevel & lvl = pyramid[l];
		const Eigen::Matrix3d Rw = Wopt.topLeftCorner<3,3>();
		const Eigen::Vector3d tw = Wopt.block<3,1>(0,3);
		Eigen::Matrix3d R = Rw;
		Eigen::Vector3d t = tw;

		for(int iter=0; iter<_iterations; ++iter)
		{
			std::vector<Corr> corrs;
			corrs.reserve((size_t)(lvl.width/lvl.step)*(lvl.height/lvl.step));
			std::vector<double> absPhoto;
			std::vector<double> absGeo;
			// Accumulators for optional global affine illumination normalization (ZNCC-style).
			double sumFrom=0.0, sumFrom2=0.0, sumTo=0.0, sumTo2=0.0;
			int photoCount=0;

			for(int v=0; v<lvl.height; v+=lvl.step)
			{
				const float * depthRow = lvl.depthFrom.ptr<float>(v);
				const float * intenRow = lvl.intensityFrom.ptr<float>(v);
				for(int u=0; u<lvl.width; u+=lvl.step)
				{
					double d = depthRow[u];
					if(d <= minDepth || (maxDepth>0.0 && d>maxDepth))
					{
						continue;
					}
					// from-optical 3D point
					Eigen::Vector3d X(
							(u - lvl.cxFrom)*d/lvl.fxFrom,
							(v - lvl.cyFrom)*d/lvl.fyFrom,
							d);
					// warp to target optical frame
					Eigen::Vector3d Y = R*X + t;
					if(Y.z() <= minDepth)
					{
						continue;
					}
					double up = lvl.fxTo*Y.x()/Y.z() + lvl.cxTo;
					double vp = lvl.fyTo*Y.y()/Y.z() + lvl.cyTo;

					Corr c;
					c.Y = Y;
					c.hasPhoto = false;
					c.hasGeo = false;

					// photometric residual
					double iTo;
					if(sampleBilinear(lvl.intensityTo, up, vp, iTo))
					{
						double gx, gy;
						if(sampleBilinear(lvl.gradXTo, up, vp, gx) &&
						   sampleBilinear(lvl.gradYTo, up, vp, gy))
						{
							// pixel selection: enough texture (in the source image)
							double gMag = std::sqrt(gx*gx + gy*gy);
							if(gMag >= _minGradient || _minGradient <= 0.0f)
							{
								double iFrom = intenRow[u];
								c.iFrom = iFrom;
								c.iTo = iTo;
								c.rp = iTo - iFrom; // raw (may be replaced by normalized below)
								c.gx = gx;
								c.gy = gy;
								c.hasPhoto = true;
								// stats for illumination normalization
								sumFrom += iFrom;  sumFrom2 += iFrom*iFrom;
								sumTo   += iTo;    sumTo2   += iTo*iTo;
								++photoCount;
							}
						}
					}

					// geometric point-to-plane residual (projective association)
					if(geoW > 0.0)
					{
						int uu = (int)(up+0.5);
						int vv = (int)(vp+0.5);
						float dMeas = depthAt(lvl.depthTo, uu, vv);
						if(dMeas > minDepth && (maxDepth<=0.0 || dMeas<=maxDepth) &&
						   std::fabs(dMeas - Y.z()) <= _maxCorrespondenceDepthDiff)
						{
							const cv::Vec3f & nv = lvl.normalsTo.ptr<cv::Vec3f>(vv)[uu];
							if(nv[0]!=0.0f || nv[1]!=0.0f || nv[2]!=0.0f)
							{
								Eigen::Vector3d Q(
										(uu - lvl.cxTo)*dMeas/lvl.fxTo,
										(vv - lvl.cyTo)*dMeas/lvl.fyTo,
										dMeas);
								Eigen::Vector3d n((double)nv[0], (double)nv[1], (double)nv[2]);
								c.n = n;
								c.rg = n.dot(Y - Q);
								c.hasGeo = true;
								absGeo.push_back(std::fabs(c.rg));
							}
						}
					}

					if(c.hasPhoto || c.hasGeo)
					{
						corrs.push_back(c);
					}
				}
			}

			if(corrs.size() < 6)
			{
				UDEBUG("Dense refinement level %d iter %d: too few correspondences (%d), stopping level.",
						l, iter, (int)corrs.size());
				break;
			}

			// Photometric residuals with optional global affine illumination invariance
			// (ZNCC-style: zero-mean + unit-std normalization of the sampled intensities).
			// This makes the photometric term invariant to a global gain/bias change
			// (e.g. different exposure/lighting between two revisits) at near-zero cost.
			// When disabled (or too few points), it reduces exactly to the raw residual.
			double muFrom=0.0, muTo=0.0, sFrom=1.0, sTo=1.0;
			if(_photoIlluminationInvariant && photoCount >= 20)
			{
				muFrom = sumFrom/photoCount;
				muTo   = sumTo/photoCount;
				double varFrom = sumFrom2/photoCount - muFrom*muFrom;
				double varTo   = sumTo2/photoCount - muTo*muTo;
				// floor std at 1 intensity level (low-texture -> graceful fallback to bias-only)
				sFrom = varFrom > 1.0 ? std::sqrt(varFrom) : 1.0;
				sTo   = varTo   > 1.0 ? std::sqrt(varTo)   : 1.0;
			}
			const double invSTo = 1.0/sTo;
			const double invSFrom = 1.0/sFrom;
			for(size_t i=0; i<corrs.size(); ++i)
			{
				Corr & c = corrs[i];
				if(c.hasPhoto)
				{
					c.rp = (c.iTo - muTo)*invSTo - (c.iFrom - muFrom)*invSFrom;
					c.gx *= invSTo;
					c.gy *= invSTo;
					absPhoto.push_back(std::fabs(c.rp));
				}
			}

			// Robust per-term scales (auto-balances photometric vs geometric units).
			double sigmaPhoto = robustScale(absPhoto);
			double sigmaGeo = robustScale(absGeo);
			if(sigmaPhoto < 1e-3) sigmaPhoto = 1e-3;     // intensity floor
			if(sigmaGeo < 1e-4) sigmaGeo = 1e-4;         // meters floor
			double invVarPhoto = 1.0/(sigmaPhoto*sigmaPhoto);
			double invVarGeo = 1.0/(sigmaGeo*sigmaGeo);

			Eigen::Matrix<double,6,6> Hm = Eigen::Matrix<double,6,6>::Zero();
			Eigen::Matrix<double,6,1> bm = Eigen::Matrix<double,6,1>::Zero();

			for(size_t i=0; i<corrs.size(); ++i)
			{
				const Corr & c = corrs[i];
				const Eigen::Vector3d & Y = c.Y;
				// d(Y)/d(xi) = [ I | -skew(Y) ]   (3x6, left perturbation in target optical frame)
				Eigen::Matrix<double,3,6> dYdxi;
				dYdxi.leftCols<3>() = Eigen::Matrix3d::Identity();
				dYdxi.rightCols<3>() = -skew(Y);

				if(c.hasPhoto && photoW > 0.0)
				{
					double invZ = 1.0/Y.z();
					double invZ2 = invZ*invZ;
					// projective Jacobian (2x3)
					Eigen::Matrix<double,2,3> Jpi;
					Jpi << lvl.fxTo*invZ, 0.0, -lvl.fxTo*Y.x()*invZ2,
						   0.0, lvl.fyTo*invZ, -lvl.fyTo*Y.y()*invZ2;
					Eigen::Matrix<double,1,2> gradI;
					gradI << c.gx, c.gy;
					Eigen::Matrix<double,1,6> Jp = gradI * Jpi * dYdxi;
					double e = c.rp/sigmaPhoto;
					double wH = (std::fabs(e) <= huberK)?1.0:(huberK/std::fabs(e));
					double w = photoW * wH * invVarPhoto;
					Hm.noalias() += w * (Jp.transpose()*Jp);
					bm.noalias() += w * c.rp * Jp.transpose();
				}

				if(c.hasGeo && geoW > 0.0)
				{
					Eigen::Matrix<double,1,6> Jg = c.n.transpose() * dYdxi;
					double e = c.rg/sigmaGeo;
					double wH = (std::fabs(e) <= huberK)?1.0:(huberK/std::fabs(e));
					double w = geoW * wH * invVarGeo;
					Hm.noalias() += w * (Jg.transpose()*Jg);
					bm.noalias() += w * c.rg * Jg.transpose();
				}
			}

			// Levenberg-style tiny damping for numerical stability.
			for(int d=0; d<6; ++d)
			{
				Hm(d,d) += 1e-9;
			}

			Eigen::Matrix<double,6,1> dxi = Hm.ldlt().solve(-bm);
			if(!dxi.allFinite())
			{
				UDEBUG("Dense refinement level %d iter %d: non-finite update, stopping level.", l, iter);
				break;
			}

			// W <- exp(dxi) * W  (left perturbation in target optical frame)
			Eigen::Matrix4d Wcur = Eigen::Matrix4d::Identity();
			Wcur.topLeftCorner<3,3>() = R;
			Wcur.block<3,1>(0,3) = t;
			Eigen::Matrix4d Wnew = expSE3(dxi) * Wcur;
			R = Wnew.topLeftCorner<3,3>();
			t = Wnew.block<3,1>(0,3);

			if(l == 0)
			{
				totalUsedFinest = (int)corrs.size();
			}

			if(dxi.norm() < _convergenceEps)
			{
				break;
			}
		}

		Wopt.topLeftCorner<3,3>() = R;
		Wopt.block<3,1>(0,3) = t;
	}

	// Recover the refined base-frame transform: T = Lf * W^{-1} * Lt^{-1}.
	Eigen::Matrix4d Tref = Lf * Wopt.inverse() * LtInv;
	if(!Tref.allFinite())
	{
		UWARN("Dense refinement produced a non-finite transform, keeping the input guess.");
		return guess;
	}
	Transform refined = Transform::fromEigen4d(Tref);

	// Safety guards: never let the dense step move the estimate too far from the guess
	// (protects against divergence on low-texture / unreliable-depth data).
	Transform correction = guess.inverse() * refined;
	Eigen::Matrix4d Cm = correction.toEigen4d();
	double corrTrans = Cm.block<3,1>(0,3).norm();
	Eigen::Matrix3d Rc = Cm.topLeftCorner<3,3>();
	Eigen::AngleAxisd aa(Rc);
	double corrAngle = std::fabs(aa.angle());

	if((_maxTranslation > 0.0f && corrTrans > _maxTranslation) ||
	   (_maxRotation > 0.0f && corrAngle > _maxRotation))
	{
		UWARN("Dense refinement rejected: correction too large (translation=%.3fm > %.3f or "
			  "rotation=%.3frad > %.3f). Keeping the input guess.",
			  corrTrans, _maxTranslation, corrAngle, _maxRotation);
		return guess;
	}

	if(totalUsedFinest < 6)
	{
		UDEBUG("Dense refinement under-constrained at finest level (%d points), keeping guess.", totalUsedFinest);
		return guess;
	}

	UDEBUG("Dense refinement applied: correction translation=%.4fm rotation=%.4frad, points=%d, result=%s",
			corrTrans, corrAngle, totalUsedFinest, refined.prettyPrint().c_str());

	// Keep the covariance estimated upstream (Vis/ICP). The dense step refines the
	// transform mean; the upstream covariance remains a valid (conservative) weight.
	return refined;
}

}
