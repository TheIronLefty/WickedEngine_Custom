#include "globals.hlsli"
#include "ShaderInterop_Postprocess.h"
// Bloom prefilter: cuts out bright parts and downsamples. Temporally stable variant.
//
// What changed compared to the original (same bindings, same push constants, drop-in):
//	1. 13-tap downsample ("Next Generation Post Processing in Call of Duty: Advanced Warfare", Jimenez 2014)
//	   instead of a 4-tap box. The footprint is wider and overlaps between neighboring output pixels, so
//	   sub-pixel motion no longer makes energy jump between output pixels (shimmering).
//	2. Karis average: each of the five overlapping 4-tap boxes is weighted by 1/(1 + luma). A single
//	   super-bright texel (specular sparkle, aliased highlight) can no longer dominate an output pixel and
//	   pop in and out from frame to frame ("fireflies"). The weights are normalized, so uniformly bright areas
//	   keep their full brightness.
//	3. Soft-knee threshold instead of a hard max(color - threshold, 0). Pixels crossing the threshold fade
//	   in smoothly rather than popping on. Same per-channel behavior as the original, so the color of the
//	   bloom stays the same; with BLOOM_SOFT_KNEE = 0 this is exactly the original threshold.
//	4. Every tap is clamped before averaging, so inf / huge fp16 values can't turn into NaN through the weights.
//
// Assumption (same one the original makes): bloom.resolution_rcp is the reciprocal resolution of the OUTPUT
// texture. Tap offsets are expressed in output pixels, so the filter is designed for a 2x downsample from
// input to output (outer taps land 2 input texels away). With a bigger ratio the taps get sparser.

PUSHCONSTANT(bloom, Bloom);

// 1: Karis-weighted (anti-firefly) combine of the five boxes, 0: plain CoD weights
#define BLOOM_KARIS_AVERAGE 1
// Soft knee width as a fraction of bloom.threshold. 0 = hard threshold like before, 0.5 = smooth transition.
// Note: the knee starts letting light through below the threshold (from threshold * (1 - knee)),
// so larger values make the glow reach further out around bright areas.
#define BLOOM_SOFT_KNEE 0.25
// Scales all 13 tap offsets. 1.0 = full Call of Duty footprint (outer taps 2 input texels away),
// 0.5 = footprint about the same as the original 4-tap box. Lower = tighter bloom.
#define BLOOM_FILTER_SCALE 1.0
// Per-tap clamp after exposure is applied (the original clamps the filtered result to 10 as well, kept below)
#define BLOOM_TAP_CLAMP 100.0
// Upper limit of the filtered color before the threshold, same as the original
#define BLOOM_MAX_COLOR 10.0

inline float3 bloom_tap(Texture2D<half4> tex, float2 uv, float exposure)
{
	return min(tex.SampleLevel(sampler_linear_clamp, uv, 0).rgb * exposure, BLOOM_TAP_CLAMP);
}

inline float bloom_karis_weight(float3 color)
{
	return rcp(1.0 + dot(color, float3(0.2126, 0.7152, 0.0722)));
}

[numthreads(POSTPROCESS_BLOCKSIZE, POSTPROCESS_BLOCKSIZE, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
	Texture2D<half4> texture_input = bindless_textures_half4[descriptor_index(bloom.texture_input)];

	float exposure = bloom.exposure;

	[branch]
	if (bloom.buffer_input_luminance >= 0)
	{
		exposure *= bindless_buffers[descriptor_index(bloom.buffer_input_luminance)].Load <
		float > (LUMINANCE_BUFFER_OFFSET_EXPOSURE);
	}

	const float2 uv = (DTid.xy + 0.5) * bloom.resolution_rcp;
	const float2 px = bloom.resolution_rcp * BLOOM_FILTER_SCALE; // tap offset unit: one output pixel in UV space, scaled

	// 13 taps, offsets in output pixels:
	//	a(-1,-1)  b(0,-1)  c(1,-1)
	//	   j(-.5,-.5) k(.5,-.5)
	//	d(-1, 0)  e(0, 0)  f(1, 0)
	//	   l(-.5, .5) m(.5, .5)
	//	g(-1, 1)  h(0, 1)  i(1, 1)
	const float3 a = bloom_tap(texture_input, uv + px * float2(-1, -1), exposure);
	const float3 b = bloom_tap(texture_input, uv + px * float2(0, -1), exposure);
	const float3 c = bloom_tap(texture_input, uv + px * float2(1, -1), exposure);
	const float3 d = bloom_tap(texture_input, uv + px * float2(-1, 0), exposure);
	const float3 e = bloom_tap(texture_input, uv, exposure);
	const float3 f = bloom_tap(texture_input, uv + px * float2(1, 0), exposure);
	const float3 g = bloom_tap(texture_input, uv + px * float2(-1, 1), exposure);
	const float3 h = bloom_tap(texture_input, uv + px * float2(0, 1), exposure);
	const float3 i = bloom_tap(texture_input, uv + px * float2(1, 1), exposure);
	const float3 j = bloom_tap(texture_input, uv + px * float2(-0.5, -0.5), exposure);
	const float3 k = bloom_tap(texture_input, uv + px * float2(0.5, -0.5), exposure);
	const float3 l = bloom_tap(texture_input, uv + px * float2(-0.5, 0.5), exposure);
	const float3 m = bloom_tap(texture_input, uv + px * float2(0.5, 0.5), exposure);

	// Five overlapping 4-tap boxes. Base weights: center box 0.5, four corner boxes 0.125 each (sum = 1)
	const float3 box0 = (j + k + l + m) * 0.25;
	const float3 box1 = (a + b + d + e) * 0.25;
	const float3 box2 = (b + c + e + f) * 0.25;
	const float3 box3 = (d + e + g + h) * 0.25;
	const float3 box4 = (e + f + h + i) * 0.25;

	float3 color;
#if BLOOM_KARIS_AVERAGE
	const float w0 = 0.5 * bloom_karis_weight(box0);
	const float w1 = 0.125 * bloom_karis_weight(box1);
	const float w2 = 0.125 * bloom_karis_weight(box2);
	const float w3 = 0.125 * bloom_karis_weight(box3);
	const float w4 = 0.125 * bloom_karis_weight(box4);
	color = (box0 * w0 + box1 * w1 + box2 * w2 + box3 * w3 + box4 * w4) * rcp(w0 + w1 + w2 + w3 + w4);
#else
	color = box0 * 0.5 + (box1 + box2 + box3 + box4) * 0.125;
#endif // BLOOM_KARIS_AVERAGE

	color = min(color, BLOOM_MAX_COLOR); // avoid incredibly large values overly dominating bloom (high speculars)

	// Soft-knee threshold, applied per channel like the original max(color - threshold, 0):
	const float knee = bloom.threshold * BLOOM_SOFT_KNEE;
	float3 soft = clamp(color - bloom.threshold + knee, 0, 2 * knee);
	soft = soft * soft / (4 * knee + 1e-5);
	color = max(soft, color - bloom.threshold);

	bindless_rwtextures[descriptor_index(bloom.texture_output)][DTid.xy] = float4(color, 1);
}
