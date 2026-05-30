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

#ifndef REGISTRATIONDENSE_H_
#define REGISTRATIONDENSE_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/CameraModel.h>

namespace rtabmap {

/**
 * Dense RGB-D registration refinement.
 *
 * Given an initial transform guess (typically produced by RegistrationVis and/or
 * RegistrationIcp upstream in the pipeline), this refines it by minimizing a
 * combined cost over all valid-depth pixels:
 *   - a photometric (direct) term: intensity difference between the reference
 *     image and the warped target image, and
 *   - a geometric point-to-plane term: distance, along the target surface normal,
 *     between the warped reference 3D point and the measured target surface.
 *
 * The optimization is a robust (Huber IRLS) Gauss-Newton on the SE(3) relative
 * camera pose, performed coarse-to-fine over an image pyramid. It is designed for
 * RGB-D data with dense depth maps (e.g., a stereo network depth such as StereoNet).
 *
 * This registration always requires an initial guess; if none is available, or if
 * the refinement is implausible (moves too far from the guess) or under-constrained,
 * the input guess is returned unchanged so that the loop closure is never made worse.
 */
class RTABMAP_CORE_EXPORT RegistrationDense : public Registration
{
public:
	// take ownership of child
	RegistrationDense(const ParametersMap & parameters = ParametersMap(), Registration * child = 0);
	virtual ~RegistrationDense();

	virtual void parseParameters(const ParametersMap & parameters);

protected:
	virtual Transform computeTransformationImpl(
			Signature & from,
			Signature & to,
			Transform guess,
			RegistrationInfo & info) const;
	virtual bool isImageRequiredImpl() const {return true;}
	virtual bool canUseGuessImpl() const {return true;}

private:
	int _iterations;
	int _pyramidLevels;
	int _decimation;
	float _minDepth;
	float _maxDepth;
	float _minGradient;
	float _geometricWeight;
	float _photometricWeight;
	float _huberThreshold;
	float _convergenceEps;
	float _maxCorrespondenceDepthDiff;
	float _maxTranslation;
	float _maxRotation;
};

}

#endif /* REGISTRATIONDENSE_H_ */
