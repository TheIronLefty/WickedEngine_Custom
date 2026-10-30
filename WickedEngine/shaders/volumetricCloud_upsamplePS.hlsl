#include "globals.hlsli"
#include "ShaderInterop_Postprocess.h"

PUSHCONSTANT(postprocess, PostProcess);

Texture2D<float4> cloud_current : register(t0);
Texture2D<float2> cloud_depth_current : register(t1); // no longer used for edge detection, kept so the bindings stay unchanged

// Relative linear depth difference (fraction of the full-res pixel's depth) at which a low-res
// cloud texel stops contributing to the pixel.
static const float DEPTH_REL_TOLERANCE = 0.1;

// Bilateral, depth-aware bilinear upsample of the half-res cloud result.
// The earlier version had two problems that show up as aliasing on silhouettes:
//	1) it picked the nearest low-res texel (pixel / 2) and applied the same weights to all four
//	   full-res pixels of a 2x2 block, so the edge looked like 2x2 pixel stair steps,
//	   and it only took the edge-aware path above a very loose 20% depth error;
//	2) the low-res reference depth came from the reprojected (history-filtered) cloud depth,
//	   which does not match the depth mip that the clouds were actually rendered against.
// Here both the full-res and the low-res depths are read from the same depth buffer (mip 0 / mip 1,
// the mip the cloud render + reproject passes use), and the 4 nearest low-res texels are
// weighted by true bilinear weights multiplied by a depth-similarity weight.
float4 main(float4 pos : SV_Position, float2 uv : TEXCOORD) : SV_Target
{
	const int2 pixel = int2(pos.xy);

	const float2 lowres_size = postprocess.params0.xy;
	const int2 lowres_max = int2(lowres_size) - 1;

	// Depth of the full-res pixel (sky resolves to z_far):
	const float z_high = compute_lineardepth(texture_depth.Load(int3(pixel, 0)));

	// Bilinear footprint in low-res space:
	const float2 lowres_pos = (float2(pixel) + 0.5) * 0.5 - 0.5; // full-res pixel center in low-res texel space
	const int2 base = int2(floor(lowres_pos));
	const float2 f = lowres_pos - float2(base);

	const float bilinear_weight[4] = {
		(1 - f.x) * (1 - f.y),
		f.x * (1 - f.y),
		(1 - f.x) * f.y,
		f.x * f.y,
	};
	static const int2 offsets[4] = { int2(0, 0), int2(1, 0), int2(0, 1), int2(1, 1) };

	float4 color = 0;
	float weight_sum = 0; // Note: weights need full precision on Nvidia Vulkan!

	[unroll]
	for (int i = 0; i < 4; ++i)
	{
		const int2 coord = clamp(base + offsets[i], 0, lowres_max);

		// Same depth mip that the cloud render / reproject passes sampled:
		const float z_low = compute_lineardepth(texture_depth.Load(int3(coord, 1)));

		const float rel_diff = abs(z_low - z_high) / max(z_high, 0.0001);
		const float depth_weight = max(0.0001, 1.0 - saturate(rel_diff / DEPTH_REL_TOLERANCE));

		const float w = bilinear_weight[i] * depth_weight;
		color += cloud_current.Load(int3(coord, 0)) * w;
		weight_sum += w;
	}

	return weight_sum > 0 ? color / weight_sum : 0;
}
