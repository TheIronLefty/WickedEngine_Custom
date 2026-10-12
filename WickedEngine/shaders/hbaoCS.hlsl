// GTAO / visibility-bitmask AO replacement for Wicked Engine's HBAO compute shader.
//
// References:
//	"Practical Realtime Strategies for Accurate Indirect Occlusion", Jimenez et al. 2016 (GTAO)
//	"Screen Space Indirect Lighting with Visibility Bitmask", Therrien et al. 2023 (bitmask)
//	Intel XeGTAO (MIT) - slice/horizon/inner-integral structure follows it closely
//
// Two integrators share the same slice/sampling setup:
//	GTAO_USE_VISIBILITY_BITMASK = 0 : classic GTAO, one horizon per side (assumes infinitely thick occluders)
//	GTAO_USE_VISIBILITY_BITMASK = 1 : each depth sample marks the sectors [front, front + thickness] of a 32-bit
//	                                  mask per slice. Thin objects no longer shadow everything behind them.
//	GTAO_BITMASK_SOLID_ANGLE_WEIGHTING = 1 weights every sector by its cosine-weighted solid angle
//	(the same integrand GTAO integrates analytically), 0 uses a plain popcount.
//
// Drop-in notes:
//	- Same bindings as the old hbaoCS.hlsl: push constants `postprocess`, depth via texture_lineardepth (mip 1,
//	  like the original), output in u0, and the hbao_* macros from ShaderInterop_Postprocess.h.
//	- Single pass (no separable H/V split, no groupshared cache).
//	  GTAO_COMPAT_HBAO_DISPATCH = 1 keeps the existing two-dispatch C++ code working unchanged:
//	  the "horizontal" dispatch exits immediately and the "vertical" dispatch does the whole job.
//	  Set it to 0 once you change the C++ to a single 8x8-tile dispatch (see the bottom of the file).
//	- Output is "visibility" (1 = unoccluded) with pow(visibility, hbao_power) applied, same as before.

#include "globals.hlsli"
#include "ShaderInterop_Postprocess.h"

PUSHCONSTANT(postprocess, PostProcess);

RWTexture2D<unorm float> output : register(u0);

// ------------------------------------------------------------------------------------------------
// Tunables
// ------------------------------------------------------------------------------------------------

// Total depth taps per pixel = SLICE_COUNT * STEPS_PER_SIDE * 2 (the old HBAO did ~64 over two passes)
#define GTAO_SLICE_COUNT 2
#define GTAO_STEPS_PER_SIDE 8

// World-space (view-space) radius of influence in meters
#define GTAO_RADIUS 2.0
// Fraction of the radius over which samples fade out (XeGTAO default 0.615)
#define GTAO_FALLOFF_RANGE 0.615
// >1 concentrates samples near the center pixel
#define GTAO_SAMPLE_DISTRIBUTION_POWER 2.0
// Samples closer than this many pixels are skipped (avoids self-occlusion from the pixel's own neighborhood)
#define GTAO_MIN_PIXEL_OFFSET 1.3
// Clamp on the screen-space search radius, in AO-resolution pixels (keeps close-up geometry cheap)
#define GTAO_MAX_PIXEL_RADIUS 128.0

// Visibility bitmask integrator (see header)
#define GTAO_USE_VISIBILITY_BITMASK 1
#define GTAO_BITMASK_SOLID_ANGLE_WEIGHTING 1
// Assumed thickness of every depth-buffer sample along the view ray, in meters.
// Too small -> holes between samples (under-occlusion on large surfaces, since a few taps can't cover all 32 sectors).
// Too large -> converges to classic GTAO behavior (thin objects occlude everything behind them).
#define GTAO_THICKNESS 0.35

// 1: noise changes every frame (needs temporal accumulation / blur downstream to look clean)
// 0: noise pattern is static (stable, but a fixed dither pattern is visible without a spatial blur)
#define GTAO_TEMPORAL_NOISE 1

// 1: keep the existing HBAO two-dispatch C++ path working. 0: single 8x8 dispatch.
#define GTAO_COMPAT_HBAO_DISPATCH 1

// Uncomment to use the G-buffer normals instead of normals reconstructed from depth.
// UNVERIFIED: assumes texture_normal_roughness is valid when this pass runs and that GetCamera().view
// is the world->view matrix with the same axis convention as hbao_uv_to_view_A/B.
//#define GTAO_USE_GBUFFER_NORMALS

// ------------------------------------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------------------------------------

// Linear view depth in meters. Mip 1, same as the original HBAO (AO runs on the half-res depth).
inline float gtao_load_depth(int2 pixel)
{
	return texture_lineardepth.Load(uint3(pixel, 1)) * GetCamera().z_far;
}

// View-space position of a pixel center. Same formula as the original hbaoCS:
//	xy = (A * uv + B) * z, z = linear depth.
// Off-screen taps return a point extremely far behind the shaded pixel, which the distance falloff then discards
// (zero contribution to the horizon / an empty bitmask slab). Clamping to the edge pixel instead is wrong here:
// the edge pixel is not on the slice's line anymore, and at the top of the screen it often lands on sky or far
// geometry, which produces bogus horizons. The normal reconstruction is unaffected because its "smaller depth
// difference" neighbor choice automatically avoids the off-screen side.
#define GTAO_OFFSCREEN_DEPTH 1.0e6
inline float3 gtao_fetch_view_position(int2 pixel)
{
	[branch]
	if (any(pixel < int2(0, 0)) || any(pixel >= int2(postprocess.resolution)))
		return float3(0, 0, GTAO_OFFSCREEN_DEPTH);

	const float z = gtao_load_depth(pixel);
	const float2 uv = (pixel + 0.5) * postprocess.resolution_rcp;
	return float3((hbao_uv_to_view_A * uv + hbao_uv_to_view_B) * z, z);
}

#ifdef GTAO_USE_GBUFFER_NORMALS
inline float3 gtao_view_normal(int2 pixel, float3 P)
{
	const float2 uv = (pixel + 0.5) * postprocess.resolution_rcp;
	const float3 N_world = decode_normal(texture_normal_roughness.SampleLevel(sampler_point_clamp, uv, 0));
	return normalize(mul((float3x3)GetCamera().view, N_world));
}
#else
// Normal from depth: per axis, pick the neighbor with the smaller depth difference so that normals
// don't smear across depth discontinuities (same idea as the original's "smaller tangent" selection).
inline float3 gtao_view_normal(int2 pixel, float3 P)
{
	const float3 PL = gtao_fetch_view_position(pixel + int2(-1, 0));
	const float3 PR = gtao_fetch_view_position(pixel + int2(1, 0));
	const float3 PT = gtao_fetch_view_position(pixel + int2(0, -1));
	const float3 PB = gtao_fetch_view_position(pixel + int2(0, 1));

	// Both vectors point towards increasing screen x / y respectively:
	const float3 dx = (abs(PR.z - P.z) < abs(PL.z - P.z)) ? (PR - P) : (P - PL);
	const float3 dy = (abs(PB.z - P.z) < abs(PT.z - P.z)) ? (PB - P) : (P - PT);

	float3 N = normalize(cross(dx, dy));

	// Winding depends on the handedness of the A/B convention, so just face the viewer:
	if (dot(N, P) > 0)
		N = -N;
	return N;
}
#endif // GTAO_USE_GBUFFER_NORMALS

// ------------------------------------------------------------------------------------------------
// Visibility bitmask
// ------------------------------------------------------------------------------------------------
#if GTAO_USE_VISIBILITY_BITMASK

#define GTAO_SECTOR_COUNT 32

// Marks the sectors covered by one depth sample treated as a slab [front, front + thickness] along the view ray.
//	Sector space: the 32 sectors span the hemisphere around the projected normal, theta in [n - PI/2, n + PI/2],
//	theta being the angle from the view vector V inside the slice plane (positive = +pixel_dir side).
//	delta          : sample position - shaded position (view space)
//	len            : length(delta)
//	fade           : distance falloff in [0,1]; collapses the slab so that samples fade out at the edge of the radius
//	negative_side  : sample was taken in -pixel_dir (angles are negative)
uint gtao_sample_mask(float3 delta, float len, float fade, float3 V, float n, bool negative_side)
{
	const float front_cos = dot(delta, V) / len;
	const float3 delta_back = delta - V * GTAO_THICKNESS; // push the sample away from the camera
	const float back_cos = dot(delta_back, V) / max(length(delta_back), 1e-5);

	const float front_angle = acosFast(clamp(front_cos, -1, 1));
	float back_angle = acosFast(clamp(back_cos, -1, 1)); // >= front_angle
	back_angle = lerp(front_angle, back_angle, fade);

	// Occupied angular interval, ordered (min, max):
	float2 h = negative_side ? float2(-back_angle, -front_angle) : float2(front_angle, back_angle);

	// Map [n - PI/2, n + PI/2] -> [0, 1]; anything outside the normal's hemisphere is clipped away:
	h = saturate((h - n + PI * 0.5) / PI);

	const uint first = (uint) (h.x * GTAO_SECTOR_COUNT);
	const uint count = (uint) ceil((h.y - h.x) * GTAO_SECTOR_COUNT);
	const uint run = count >= GTAO_SECTOR_COUNT ? 0xFFFFFFFFu : ((1u << count) - 1u);
	return first >= GTAO_SECTOR_COUNT ? 0u : (run << first);
}

// Slice visibility from the final bitmask (before multiplying by the projected normal length).
//	Fully unoccluded returns cos(n) + n*sin(n), which is exactly what the analytic GTAO inner integral gives
//	when both horizons sit on the tangent plane, so both integrators have the same brightness scale.
float gtao_bitmask_visibility(uint bitmask, float n)
{
	float sin_n, cos_n;
	sincos(n, sin_n, cos_n);
	const float total = cos_n + n * sin_n;

	[branch]
	if (bitmask == 0u)
		return total;
	[branch]
	if (bitmask == 0xFFFFFFFFu)
		return 0;

#if GTAO_BITMASK_SOLID_ANGLE_WEIGHTING
	// Weight of a sector [a, b] = integral of cos(theta - n) * |sin(theta)| over it, i.e. its cosine-weighted
	// solid angle in the slice parameterization (|sin(theta)| is the solid angle element, cos(theta - n) the
	// Lambert term relative to the projected normal).
	//	Antiderivative from 0:  Fs(theta) = sign(theta) * (cos(n) + 2*theta*sin(n) - cos(2*theta - n)) / 4
	//	Sector k starts at theta_k = n - PI/2 + k*PI/32, where -cos(2*theta_k - n) = cos(n + k*PI/16),
	//	so the cosine term advances by a fixed rotation of PI/16 per sector (no trig in the loop).
	const float sector_angle = PI / GTAO_SECTOR_COUNT;
	const float rot_cos = cos(2 * sector_angle);
	const float rot_sin = sin(2 * sector_angle);

	float c = cos_n; // cos(n + k*PI/16)
	float s = sin_n; // sin(n + k*PI/16)
	float theta = n - PI * 0.5;

	float F = 0.25 * (cos_n + 2 * theta * sin_n + c);
	float Fs_prev = theta < 0 ? -F : F;

	float visible = 0;
	[unroll]
	for (uint k = 0; k < GTAO_SECTOR_COUNT; ++k)
	{
		const float c_next = c * rot_cos - s * rot_sin;
		const float s_next = s * rot_cos + c * rot_sin;
		c = c_next;
		s = s_next;
		theta += sector_angle;

		F = 0.25 * (cos_n + 2 * theta * sin_n + c);
		const float Fs = theta < 0 ? -F : F;

		if (((bitmask >> k) & 1u) == 0u)
			visible += Fs - Fs_prev;
		Fs_prev = Fs;
	}
	return visible;
#else
	// Uniform sectors: unoccluded fraction scaled by the same total so brightness matches the weighted version
	return total * (1.0 - (float)countbits(bitmask) / GTAO_SECTOR_COUNT);
#endif // GTAO_BITMASK_SOLID_ANGLE_WEIGHTING
}

#endif // GTAO_USE_VISIBILITY_BITMASK

// ------------------------------------------------------------------------------------------------
// GTAO
// ------------------------------------------------------------------------------------------------

// Returns visibility in [0,1], 1 = fully unoccluded
float gtao(int2 pixel)
{
	const float3 P = gtao_fetch_view_position(pixel);

	// Sky / far plane, same threshold as the original shader:
	[branch]
	if (P.z >= GetCamera().z_far - 0.99)
		return 1;

	const float3 V = normalize(-P);
	const float3 N = gtao_view_normal(pixel, P);

	// Screen-space direction (in pixels) that corresponds to +x / +y of view space.
	// Derived from A so this stays correct regardless of whether view-space Y is up or down.
	const float2 view_to_pixel_sign = sign(hbao_uv_to_view_A);

	// Size of one AO-resolution pixel in view-space meters at this depth, and the resulting search radius in pixels:
	const float pixel_size_view = abs(hbao_uv_to_view_A.x) * P.z * postprocess.resolution_rcp.x;
	const float screen_radius = min(GTAO_RADIUS / pixel_size_view, GTAO_MAX_PIXEL_RADIUS);
	const float min_s = GTAO_MIN_PIXEL_OFFSET / screen_radius;

	// Distance falloff (linear fade over the outer part of the radius):
	const float falloff_range = GTAO_RADIUS * GTAO_FALLOFF_RANGE;
	const float falloff_mul = -1.0 / falloff_range;
	const float falloff_add = (GTAO_RADIUS - falloff_range) / falloff_range + 1.0;

	// Noise from globals.hlsli. IGN expects pixel coordinates.
	// The second value uses transposed coordinates so it isn't just a shifted copy of the first.
#if GTAO_TEMPORAL_NOISE
	const uint frame = GetFrameCount();
#else
	const uint frame = 0;
#endif
	const float noise_slice = InterleavedGradientNoise(float2(pixel), frame);
	const float noise_sample = InterleavedGradientNoise(float2(pixel.yx), frame);

	float visibility = 0;

	[unroll]
	for (uint slice = 0; slice < GTAO_SLICE_COUNT; ++slice)
	{
		const float phi = (slice + noise_slice) / GTAO_SLICE_COUNT * PI;
		float sin_phi, cos_phi;
		sincos(phi, sin_phi, cos_phi);

		const float2 omega = float2(cos_phi, sin_phi); // slice direction in view-space XY
		const float2 pixel_dir = omega * view_to_pixel_sign; // same direction in pixel space

		// Slice plane setup:
		const float3 direction_vec = float3(omega, 0);
		const float3 ortho_direction_vec = direction_vec - dot(direction_vec, V) * V;
		const float3 axis_vec = normalize(cross(ortho_direction_vec, V));

		// Normal projected into the slice plane, as an angle n relative to V:
		const float3 projected_normal = N - axis_vec * dot(N, axis_vec);
		const float projected_normal_len = length(projected_normal);
		const float sign_norm = sign(dot(ortho_direction_vec, projected_normal));
		const float cos_norm = saturate(dot(projected_normal, V) / max(projected_normal_len, 1e-5));
		const float n = sign_norm * acosFastPositive(cos_norm);
		float sin_n, cos_n_unused;
		sincos(n, sin_n, cos_n_unused);

#if GTAO_USE_VISIBILITY_BITMASK
		uint bitmask = 0;
#else
		// Lowest possible horizons = the tangent plane: cos(n + PI/2) = -sin(n), cos(n - PI/2) = sin(n)
		const float low_horizon_cos0 = -sin_n;	// side 0: +pixel_dir
		const float low_horizon_cos1 = sin_n;	// side 1: -pixel_dir
		float horizon_cos0 = low_horizon_cos0;
		float horizon_cos1 = low_horizon_cos1;
#endif // GTAO_USE_VISIBILITY_BITMASK

		[unroll]
		for (uint step_index = 0; step_index < GTAO_STEPS_PER_SIDE; ++step_index)
		{
			float s = (step_index + noise_sample) / GTAO_STEPS_PER_SIDE;
			s = pow(s, GTAO_SAMPLE_DISTRIBUTION_POWER) + min_s;

			// Whole-pixel offsets so the point-fetched depth is exactly at a pixel center:
			const int2 offset = int2(round(pixel_dir * (s * screen_radius)));

			const float3 delta0 = gtao_fetch_view_position(pixel + offset) - P;
			const float3 delta1 = gtao_fetch_view_position(pixel - offset) - P;
			const float len0 = max(length(delta0), 1e-5);
			const float len1 = max(length(delta1), 1e-5);

			const float fade0 = saturate(len0 * falloff_mul + falloff_add);
			const float fade1 = saturate(len1 * falloff_mul + falloff_add);

#if GTAO_USE_VISIBILITY_BITMASK
			bitmask |= gtao_sample_mask(delta0, len0, fade0, V, n, false);
			bitmask |= gtao_sample_mask(delta1, len1, fade1, V, n, true);
#else
			// cos of the horizon angle (relative to V) for each sample, faded to the "no occluder" horizon with distance:
			float shc0 = dot(delta0, V) / len0;
			float shc1 = dot(delta1, V) / len1;
			shc0 = lerp(low_horizon_cos0, shc0, fade0);
			shc1 = lerp(low_horizon_cos1, shc1, fade1);

			horizon_cos0 = max(horizon_cos0, shc0);
			horizon_cos1 = max(horizon_cos1, shc1);
#endif // GTAO_USE_VISIBILITY_BITMASK
		}

#if GTAO_USE_VISIBILITY_BITMASK
		visibility += projected_normal_len * gtao_bitmask_visibility(bitmask, n);
#else
		// Horizon angles, clamped to the hemisphere around the projected normal:
		float h0 = -acosFast(clamp(horizon_cos1, -1, 1));
		float h1 = acosFast(clamp(horizon_cos0, -1, 1));
		h0 = n + clamp(h0 - n, -PI * 0.5, PI * 0.5);
		h1 = n + clamp(h1 - n, -PI * 0.5, PI * 0.5);

		// Analytic cosine-weighted integral over each arc (Eq. 10 in the paper):
		const float iarc0 = (cos_norm + 2 * h0 * sin_n - cos(2 * h0 - n)) * 0.25;
		const float iarc1 = (cos_norm + 2 * h1 * sin_n - cos(2 * h1 - n)) * 0.25;

		visibility += projected_normal_len * (iarc0 + iarc1);
#endif // GTAO_USE_VISIBILITY_BITMASK
	}

	return saturate(visibility / GTAO_SLICE_COUNT);
}

// ------------------------------------------------------------------------------------------------
// Entry point
// ------------------------------------------------------------------------------------------------

#if GTAO_COMPAT_HBAO_DISPATCH

// The old C++ dispatches this shader twice: once with hbao_direction = (1,0) and once with (0,1),
// using 1D thread groups that walk along a line. GTAO needs no separable passes, so the horizontal
// dispatch does nothing and the vertical one computes the final result directly.
[numthreads(POSTPROCESS_HBAO_THREADCOUNT, 1, 1)]
void main(uint3 Gid : SV_GroupID, uint groupIndex : SV_GroupIndex)
{
	const bool horizontal = hbao_direction.y == 0;
	if (horizontal)
		return;

	const uint2 pixel = uint2(Gid.x, Gid.y * POSTPROCESS_HBAO_THREADCOUNT + groupIndex);
	if (pixel.x >= postprocess.resolution.x || pixel.y >= postprocess.resolution.y)
		return;

	output[pixel] = pow(saturate(gtao(int2(pixel))), hbao_power);
}

#else

// Single dispatch: Dispatch((res.x + 7) / 8, (res.y + 7) / 8, 1), writing straight into the final AO texture.
[numthreads(8, 8, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
	const uint2 pixel = DTid.xy;
	if (pixel.x >= postprocess.resolution.x || pixel.y >= postprocess.resolution.y)
		return;

	output[pixel] = pow(saturate(gtao(int2(pixel))), hbao_power);
}

#endif // GTAO_COMPAT_HBAO_DISPATCH
