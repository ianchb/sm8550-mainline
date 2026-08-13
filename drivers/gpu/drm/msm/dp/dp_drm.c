// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/string_choices.h>
#include <linux/math64.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "dp_audio.h"
#include "dp_drm.h"

static const struct drm_bridge_funcs msm_dp_bridge_ops;

static struct drm_bridge_state *
msm_dp_bridge_atomic_duplicate_state(struct drm_bridge *bridge)
{
	struct msm_dp_bridge_state *state;

	state = kmemdup(bridge->base.state, sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_bridge_duplicate_state(bridge, &state->base);

	return &state->base;
}

static void msm_dp_bridge_atomic_destroy_state(struct drm_bridge *bridge,
					       struct drm_bridge_state *state)
{
	kfree(to_msm_dp_bridge_state(state));
}

static struct drm_bridge_state *
msm_dp_bridge_atomic_reset(struct drm_bridge *bridge)
{
	struct msm_dp_bridge_state *state;

	state = kzalloc_obj(*state);
	if (!state)
		return NULL;

	__drm_atomic_helper_bridge_reset(bridge, &state->base);

	return &state->base;
}

static bool msm_dp_dsc_bpc_supported(const u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE],
				     u8 bpc)
{
	u8 supported_bpc[3];
	int count, i;

	count = drm_dp_dsc_sink_supported_input_bpcs(dsc_dpcd, supported_bpc);
	for (i = 0; i < count; i++)
		if (supported_bpc[i] == bpc)
			return true;

	return false;
}

static int msm_dp_dsc_slice_count(const u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE],
				  const struct drm_display_mode *mode)
{
	static const u8 slice_counts[] = { 1, 2, 4, 8, 12, 16, 20, 24 };
	u32 supported = drm_dp_dsc_sink_slice_count_mask(dsc_dpcd, false);
	u8 throughput;
	int max_slice_width = drm_dp_dsc_sink_max_slice_width(dsc_dpcd);
	int max_throughput;
	int min_slices;
	int i;

	if (mode->clock <= 340000)
		min_slices = 1;
	else if (mode->clock <= 680000)
		min_slices = 2;
	else if (mode->clock <= 1360000)
		min_slices = 4;
	else if (mode->clock <= 3200000)
		min_slices = 8;
	else if (mode->clock <= 4800000)
		min_slices = 12;
	else if (mode->clock <= 6400000)
		min_slices = 16;
	else if (mode->clock <= 8000000)
		min_slices = 20;
	else if (mode->clock <= 9600000)
		min_slices = 24;
	else
		return -EINVAL;

	throughput = dsc_dpcd[DP_DSC_PEAK_THROUGHPUT - DP_DSC_SUPPORT] &
		     DP_DSC_THROUGHPUT_MODE_0_MASK;
	if (!throughput || throughput == DP_DSC_THROUGHPUT_MODE_0_MASK)
		return -EINVAL;

	max_throughput = drm_dp_dsc_sink_max_slice_throughput(dsc_dpcd,
							      mode->clock, true);
	if (!max_slice_width || !max_throughput)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(slice_counts); i++) {
		int count = slice_counts[i];
		int slice_width;

		if (count < min_slices || !(supported & BIT(count - 1)))
			continue;
		if (mode->hdisplay % count)
			continue;

		slice_width = mode->hdisplay / count;
		if (slice_width >= max_slice_width)
			continue;
		if (DIV_ROUND_UP(mode->clock, count) > max_throughput)
			continue;

		return count;
	}

	return -EINVAL;
}

static u16 msm_dp_dsc_slice_height(u16 pic_height)
{
	if (!(pic_height % 108))
		return 108;
	if (!(pic_height % 16))
		return 16;
	if (!(pic_height % 12))
		return 12;

	return 15;
}

static void msm_dp_dsc_calc_dp_params(struct msm_dp_dsc_config *dsc,
				      u8 lane_count)
{
	const struct drm_dsc_config *cfg = &dsc->drm;
	u32 total_bytes = cfg->slice_chunk_size * cfg->slice_count;
	u32 eoc_bytes = cfg->slice_chunk_size % lane_count;
	u32 dummy_bytes = eoc_bytes ?
		(lane_count - eoc_bytes) * cfg->slice_count : 0;
	u32 pclk_per_line = DIV_ROUND_UP(total_bytes, 6);
	u32 last_pclk = (cfg->pic_width / 2) % 3;
	u32 last_ack = pclk_per_line - cfg->pic_width / 6;
	u32 required_pclk = 0;
	u32 remainder = 1;
	u32 accumulated = 0;

	while (accumulated < last_ack) {
		u32 start;

		required_pclk++;
		start = remainder >= 1 ? remainder : remainder + 3;
		remainder = start - 1;
		if (remainder < 1)
			accumulated++;
	}

	dsc->extra_width = required_pclk > last_pclk ?
			   required_pclk - last_pclk : 0;
	dsc->extra_dto_cycles = pclk_per_line - 1;
	dsc->bytes_per_slice = cfg->slice_chunk_size;
	dsc->eol_byte_num = ALIGN(total_bytes, 3) - total_bytes;
	dsc->overhead_num = total_bytes + lane_count * cfg->slice_count +
			    dummy_bytes;
	dsc->overhead_den = total_bytes;
	dsc->be_in_lane = 10;
}

int msm_dp_dsc_compute_config(struct msm_dp_dsc_config *dsc,
			      const u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE],
			      const struct drm_display_mode *mode,
			      u8 max_bpc, u8 lane_count)
{
	struct drm_dsc_config *cfg = &dsc->drm;
	u8 bpc;
	u8 line_buf_depth;
	u8 revision;
	int slice_count;
	int ret;

	memset(dsc, 0, sizeof(*dsc));

	if (!mode || !lane_count || !drm_dp_sink_supports_dsc(dsc_dpcd) ||
	    !drm_dp_dsc_sink_supports_format(dsc_dpcd, DP_DSC_RGB))
		return -EOPNOTSUPP;

	if (max_bpc >= 10 && msm_dp_dsc_bpc_supported(dsc_dpcd, 10))
		bpc = 10;
	else if (max_bpc >= 8 && msm_dp_dsc_bpc_supported(dsc_dpcd, 8))
		bpc = 8;
	else
		return -EOPNOTSUPP;

	revision = dsc_dpcd[DP_DSC_REV - DP_DSC_SUPPORT];
	cfg->dsc_version_major = (revision & DP_DSC_MAJOR_MASK) >>
				 DP_DSC_MAJOR_SHIFT;
	cfg->dsc_version_minor = (revision & DP_DSC_MINOR_MASK) >>
				 DP_DSC_MINOR_SHIFT;
	if (cfg->dsc_version_major != 1 ||
	    (cfg->dsc_version_minor != 1 && cfg->dsc_version_minor != 2)) {
		cfg->dsc_version_major = 1;
		cfg->dsc_version_minor = 1;
	}

	slice_count = msm_dp_dsc_slice_count(dsc_dpcd, mode);
	if (slice_count < 0)
		return slice_count;

	cfg->pic_width = mode->hdisplay;
	cfg->pic_height = mode->vdisplay;
	cfg->slice_count = slice_count;
	cfg->slice_width = mode->hdisplay / slice_count;
	cfg->slice_height = msm_dp_dsc_slice_height(mode->vdisplay);
	cfg->bits_per_component = bpc;
	cfg->bits_per_pixel = 8 << 4;
	cfg->block_pred_enable =
		dsc_dpcd[DP_DSC_BLK_PREDICTION_SUPPORT - DP_DSC_SUPPORT] &
		DP_DSC_BLK_PREDICTION_IS_SUPPORTED;
	line_buf_depth = drm_dp_dsc_sink_line_buf_depth(dsc_dpcd);
	if (!line_buf_depth)
		return -EINVAL;
	cfg->line_buf_depth = line_buf_depth;

	cfg->simple_422 = false;
	cfg->convert_rgb = true;
	cfg->vbr_enable = false;
	drm_dsc_set_const_params(cfg);
	drm_dsc_set_rc_buf_thresh(cfg);
	ret = drm_dsc_setup_rc_params(cfg, DRM_DSC_1_1_PRE_SCR);
	if (ret)
		return ret;

	cfg->initial_scale_value = drm_dsc_initial_scale_value(cfg);
	ret = drm_dsc_compute_rc_parameters(cfg);
	if (ret)
		return ret;

	dsc->enabled = true;
	msm_dp_dsc_calc_dp_params(dsc, lane_count);

	return 0;
}

static u64 msm_dp_dsc_mode_rate(const struct msm_dp_dsc_config *dsc, int clock)
{
	u64 rate = (u64)clock * drm_dsc_get_bpp_int(&dsc->drm);

	rate = DIV_ROUND_UP_ULL(rate * dsc->overhead_num, dsc->overhead_den);

	return DIV_ROUND_UP_ULL(rate * 100000, 97582);
}

struct drm_dsc_config *msm_dp_bridge_get_dsc_config(struct drm_encoder *encoder,
						    struct drm_atomic_commit *state)
{
	struct drm_bridge *bridge __free(drm_bridge_put) =
		drm_bridge_chain_get_first_bridge(encoder);
	struct drm_bridge_state *bridge_state;
	struct msm_dp_bridge_state *msm_state;

	if (!bridge)
		return NULL;

	bridge_state = state ? drm_atomic_get_new_bridge_state(state, bridge) :
			       drm_priv_to_bridge_state(bridge->base.state);
	if (!bridge_state)
		return NULL;

	msm_state = to_msm_dp_bridge_state(bridge_state);

	return msm_state->dsc.enabled ? &msm_state->dsc.drm : NULL;
}

int msm_dp_bridge_disable_dsc(struct drm_encoder *encoder,
			      struct drm_atomic_commit *state)
{
	struct drm_bridge *bridge __free(drm_bridge_put) =
		drm_bridge_chain_get_first_bridge(encoder);
	struct drm_bridge_state *bridge_state;
	struct msm_dp_bridge_state *msm_state;
	struct msm_dp *dp;
	struct drm_connector *connector;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	u64 mode_rate, link_rate;
	u32 max_bpp;
	u32 mode_clock;

	if (!bridge || bridge->funcs != &msm_dp_bridge_ops)
		return -EOPNOTSUPP;

	bridge_state = drm_atomic_get_new_bridge_state(state, bridge);
	if (!bridge_state)
		return -EINVAL;
	msm_state = to_msm_dp_bridge_state(bridge_state);
	if (!msm_state->dsc.enabled)
		return -EOPNOTSUPP;

	crtc = drm_atomic_get_new_crtc_for_encoder(state, encoder);
	if (!crtc)
		return -EINVAL;
	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state)
		return -EINVAL;
	connector = drm_atomic_get_new_connector_for_encoder(state, encoder);
	if (!connector)
		return -EINVAL;

	dp = to_dp_bridge(bridge)->msm_dp_display;
	max_bpp = connector->display_info.bpc * 3;
	if (!max_bpp)
		max_bpp = 24;
	mode_clock = crtc_state->adjusted_mode.clock;
	msm_state->bpp = msm_dp_display_get_mode_bpp(dp, max_bpp, mode_clock);
	if (!msm_state->bpp)
		return -ENOSPC;

	mode_rate = (u64)mode_clock * msm_state->bpp;
	link_rate = (u64)msm_dp_display_get_link_rate(dp) *
		msm_dp_display_get_lane_count(dp) * 8;
	if (msm_dp_display_fec_capable(dp))
		link_rate = div64_u64(link_rate * 97582, 100000);
	if (mode_rate > link_rate)
		return -ENOSPC;

	memset(&msm_state->dsc, 0, sizeof(msm_state->dsc));

	return 0;
}

static int msm_dp_bridge_atomic_check(struct drm_bridge *bridge,
				      struct drm_bridge_state *bridge_state,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct drm_connector_state *old_conn_state;
	struct msm_dp_bridge_state *msm_state =
		to_msm_dp_bridge_state(bridge_state);
	struct msm_dp *dp = to_dp_bridge(bridge)->msm_dp_display;
	u64 link_rate;
	u32 max_bpp;
	u32 mode_clock;

	memset(&msm_state->dsc, 0, sizeof(msm_state->dsc));
	msm_state->bpp = 0;

	old_conn_state =
		drm_atomic_get_old_connector_state(conn_state->state,
						   conn_state->connector);
	if (crtc_state && conn_state->crtc &&
	    drm_mode_is_420_only(&conn_state->connector->display_info,
				 &crtc_state->adjusted_mode) &&
	    (conn_state->colorspace != DRM_MODE_COLORIMETRY_DEFAULT ||
	     conn_state->hdr_output_metadata))
		return -EINVAL;
	if (old_conn_state && crtc_state && conn_state->crtc &&
	    (!drm_connector_atomic_hdr_metadata_equal(old_conn_state, conn_state) ||
	     old_conn_state->colorspace != conn_state->colorspace))
		crtc_state->mode_changed = true;

	if (conn_state->crtc && crtc_state && crtc_state->active) {
		max_bpp = conn_state->connector->display_info.bpc * 3;
		if (!max_bpp)
			max_bpp = 24;
		if (msm_dp_display_check_video_test(dp))
			max_bpp = msm_dp_display_get_test_bpp(dp);

		mode_clock = crtc_state->adjusted_mode.clock;
		msm_state->bpp = msm_dp_display_get_mode_bpp(dp, max_bpp, mode_clock);
		link_rate = (u64)msm_dp_display_get_link_rate(dp) *
			msm_dp_display_get_lane_count(dp) * 8;
		if (!dp->is_edp && msm_dp_display_fec_capable(dp) &&
		    !msm_dp_display_check_video_test(dp) &&
		    !drm_mode_is_420_only(&conn_state->connector->display_info,
					  &crtc_state->adjusted_mode) &&
		    !msm_dp_dsc_compute_config(&msm_state->dsc,
					       msm_dp_display_get_dsc_dpcd(dp),
					       &crtc_state->adjusted_mode,
					       min_t(u32, max_bpp / 3, 10),
					       msm_dp_display_get_lane_count(dp)) &&
		    msm_dp_dsc_mode_rate(&msm_state->dsc,
					 crtc_state->adjusted_mode.clock) <= link_rate)
			msm_state->bpp =
				msm_state->dsc.drm.bits_per_component * 3;
		else
			memset(&msm_state->dsc, 0, sizeof(msm_state->dsc));
	}

	return 0;
}

static void msm_dp_attach_metadata_properties(struct drm_connector *connector)
{
	u32 colorspaces = BIT(DRM_MODE_COLORIMETRY_BT2020_RGB) |
			  BIT(DRM_MODE_COLORIMETRY_DCI_P3_RGB_D65);

	if (drm_mode_create_dp_colorspace_property(connector, colorspaces))
		return;

	drm_connector_attach_colorspace_property(connector);
	drm_connector_attach_hdr_output_metadata_property(connector);
}

/**
 * msm_dp_bridge_get_modes - callback to add drm modes via drm_mode_probed_add()
 * @bridge: Poiner to drm bridge
 * @connector: Pointer to drm connector structure
 * Returns: Number of modes added
 */
static int msm_dp_bridge_get_modes(struct drm_bridge *bridge, struct drm_connector *connector)
{
	int rc = 0;
	struct msm_dp *dp;

	if (!connector)
		return 0;

	dp = to_dp_bridge(bridge)->msm_dp_display;

	/* pluggable case assumes EDID is read when HPD */
	rc = msm_dp_display_get_modes(dp);
	if (rc <= 0) {
		DRM_ERROR("failed to get DP sink modes, rc=%d\n", rc);
		return rc;
	} else {
		drm_dbg_dp(connector->dev, "No sink connected\n");
	}
	return rc;
}

static void msm_dp_bridge_debugfs_init(struct drm_bridge *bridge, struct dentry *root)
{
	struct msm_dp *dp = to_dp_bridge(bridge)->msm_dp_display;

	msm_dp_display_debugfs_init(dp, root, false);
}

static const struct drm_bridge_funcs msm_dp_bridge_ops = {
	.atomic_duplicate_state = msm_dp_bridge_atomic_duplicate_state,
	.atomic_destroy_state   = msm_dp_bridge_atomic_destroy_state,
	.atomic_reset           = msm_dp_bridge_atomic_reset,
	.atomic_enable          = msm_dp_bridge_atomic_enable,
	.atomic_disable         = msm_dp_bridge_atomic_disable,
	.atomic_post_disable    = msm_dp_bridge_atomic_post_disable,
	.mode_set     = msm_dp_bridge_mode_set,
	.mode_valid   = msm_dp_bridge_mode_valid,
	.get_modes    = msm_dp_bridge_get_modes,
	.detect       = msm_dp_bridge_detect,
	.atomic_check = msm_dp_bridge_atomic_check,
	.hpd_enable   = msm_dp_bridge_hpd_enable,
	.hpd_disable  = msm_dp_bridge_hpd_disable,
	.hpd_notify   = msm_dp_bridge_hpd_notify,
	.debugfs_init = msm_dp_bridge_debugfs_init,

	.dp_audio_prepare = msm_dp_audio_prepare,
	.dp_audio_shutdown = msm_dp_audio_shutdown,
};

static int msm_edp_bridge_atomic_check(struct drm_bridge *drm_bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct msm_dp *dp = to_dp_bridge(drm_bridge)->msm_dp_display;

	if (WARN_ON(!conn_state))
		return -ENODEV;

	conn_state->self_refresh_aware = dp->psr_supported;

	if (!conn_state->crtc || !crtc_state)
		return 0;

	if (crtc_state->self_refresh_active && !dp->psr_supported)
		return -EINVAL;

	return 0;
}

static void msm_edp_bridge_atomic_enable(struct drm_bridge *drm_bridge,
					 struct drm_atomic_commit *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct msm_dp_bridge *msm_dp_bridge = to_dp_bridge(drm_bridge);
	struct msm_dp *dp = msm_dp_bridge->msm_dp_display;

	/*
	 * Check the old state of the crtc to determine if the panel
	 * was put into psr state previously by the msm_edp_bridge_atomic_disable.
	 * If the panel is in psr, just exit psr state and skip the full
	 * bridge enable sequence.
	 */
	crtc = drm_atomic_get_new_crtc_for_encoder(state,
						   drm_bridge->encoder);
	if (!crtc)
		return;

	old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);

	if (old_crtc_state && old_crtc_state->self_refresh_active) {
		msm_dp_display_set_psr(dp, false);
		return;
	}

	msm_dp_bridge_atomic_enable(drm_bridge, state);
}

static void msm_edp_bridge_atomic_disable(struct drm_bridge *drm_bridge,
					  struct drm_atomic_commit *atomic_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state = NULL, *old_crtc_state = NULL;
	struct msm_dp_bridge *msm_dp_bridge = to_dp_bridge(drm_bridge);
	struct msm_dp *dp = msm_dp_bridge->msm_dp_display;

	crtc = drm_atomic_get_old_crtc_for_encoder(atomic_state,
						   drm_bridge->encoder);
	if (!crtc)
		goto out;

	new_crtc_state = drm_atomic_get_new_crtc_state(atomic_state, crtc);
	if (!new_crtc_state)
		goto out;

	old_crtc_state = drm_atomic_get_old_crtc_state(atomic_state, crtc);
	if (!old_crtc_state)
		goto out;

	/*
	 * Set self refresh mode if current crtc state is active.
	 *
	 * If old crtc state is active, then this is a display disable
	 * call while the sink is in psr state. So, exit psr here.
	 * The eDP controller will be disabled in the
	 * msm_edp_bridge_atomic_post_disable function.
	 *
	 * We observed sink is stuck in self refresh if psr exit is skipped
	 * when display disable occurs while the sink is in psr state.
	 */
	if (new_crtc_state->self_refresh_active) {
		msm_dp_display_set_psr(dp, true);
		return;
	} else if (old_crtc_state->self_refresh_active) {
		msm_dp_display_set_psr(dp, false);
		return;
	}

out:
	msm_dp_bridge_atomic_disable(drm_bridge, atomic_state);
}

static void msm_edp_bridge_atomic_post_disable(struct drm_bridge *drm_bridge,
					       struct drm_atomic_commit *atomic_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state = NULL;

	crtc = drm_atomic_get_old_crtc_for_encoder(atomic_state,
						   drm_bridge->encoder);
	if (!crtc)
		return;

	new_crtc_state = drm_atomic_get_new_crtc_state(atomic_state, crtc);
	if (!new_crtc_state)
		return;

	/*
	 * Self refresh mode is already set in msm_edp_bridge_atomic_disable.
	 */
	if (new_crtc_state->self_refresh_active)
		return;

	msm_dp_bridge_atomic_post_disable(drm_bridge, atomic_state);
}

/**
 * msm_edp_bridge_mode_valid - callback to determine if specified mode is valid
 * @bridge: Pointer to drm bridge structure
 * @info: display info
 * @mode: Pointer to drm mode structure
 * Returns: Validity status for specified mode
 */
static enum drm_mode_status msm_edp_bridge_mode_valid(struct drm_bridge *bridge,
					  const struct drm_display_info *info,
					  const struct drm_display_mode *mode)
{
	struct msm_dp *dp;
	int mode_pclk_khz = mode->clock;

	dp = to_dp_bridge(bridge)->msm_dp_display;

	if (!dp || !mode_pclk_khz || !dp->connector) {
		DRM_ERROR("invalid params\n");
		return -EINVAL;
	}

	if (msm_dp_wide_bus_available(dp))
		mode_pclk_khz /= 2;

	if (mode_pclk_khz > DP_MAX_PIXEL_CLK_KHZ)
		return MODE_CLOCK_HIGH;

	/*
	 * The eDP controller currently does not have a reliable way of
	 * enabling panel power to read sink capabilities. So, we rely
	 * on the panel driver to populate only supported modes for now.
	 */
	return MODE_OK;
}

static void msm_edp_bridge_debugfs_init(struct drm_bridge *bridge, struct dentry *root)
{
	struct msm_dp *dp = to_dp_bridge(bridge)->msm_dp_display;

	msm_dp_display_debugfs_init(dp, root, true);
}

static const struct drm_bridge_funcs msm_edp_bridge_ops = {
	.atomic_enable = msm_edp_bridge_atomic_enable,
	.atomic_disable = msm_edp_bridge_atomic_disable,
	.atomic_post_disable = msm_edp_bridge_atomic_post_disable,
	.mode_set = msm_dp_bridge_mode_set,
	.mode_valid = msm_edp_bridge_mode_valid,
	.atomic_reset = msm_dp_bridge_atomic_reset,
	.atomic_duplicate_state = msm_dp_bridge_atomic_duplicate_state,
	.atomic_destroy_state = msm_dp_bridge_atomic_destroy_state,
	.atomic_check = msm_edp_bridge_atomic_check,
	.debugfs_init = msm_edp_bridge_debugfs_init,
};

int msm_dp_bridge_init(struct msm_dp *msm_dp_display, struct drm_device *dev,
		   struct drm_encoder *encoder, bool yuv_supported)
{
	int rc;
	struct msm_dp_bridge *msm_dp_bridge;
	struct drm_bridge *bridge;

	msm_dp_bridge = devm_drm_bridge_alloc(dev->dev, struct msm_dp_bridge, bridge,
					      msm_dp_display->is_edp ? &msm_edp_bridge_ops :
					      &msm_dp_bridge_ops);
	if (IS_ERR(msm_dp_bridge))
		return PTR_ERR(msm_dp_bridge);

	msm_dp_bridge->msm_dp_display = msm_dp_display;

	bridge = &msm_dp_bridge->bridge;
	bridge->type = msm_dp_display->connector_type;
	bridge->ycbcr_420_allowed = yuv_supported;

	/*
	 * Many ops only make sense for DP. Why?
	 * - Detect/HPD are used by DRM to know if a display is _physically_
	 *   there, not whether the display is powered on / finished initting.
	 *   On eDP we assume the display is always there because you can't
	 *   know until power is applied. If we don't implement the ops DRM will
	 *   assume our display is always there.
	 * - Currently eDP mode reading is driven by the panel driver. This
	 *   allows the panel driver to properly power itself on to read the
	 *   modes.
	 */
	if (!msm_dp_display->is_edp) {
		bridge->ops =
			DRM_BRIDGE_OP_DP_AUDIO |
			DRM_BRIDGE_OP_DETECT |
			DRM_BRIDGE_OP_HPD |
			DRM_BRIDGE_OP_MODES;
		bridge->hdmi_audio_dev = &msm_dp_display->pdev->dev;
		bridge->hdmi_audio_max_i2s_playback_channels = 8;
		bridge->hdmi_audio_dai_port = -1;
	}

	rc = devm_drm_bridge_add(dev->dev, bridge);
	if (rc) {
		DRM_ERROR("failed to add bridge, rc=%d\n", rc);

		return rc;
	}

	rc = drm_bridge_attach(encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (rc) {
		DRM_ERROR("failed to attach bridge, rc=%d\n", rc);

		return rc;
	}

	if (msm_dp_display->next_bridge) {
		rc = drm_bridge_attach(encoder,
					msm_dp_display->next_bridge, bridge,
					DRM_BRIDGE_ATTACH_NO_CONNECTOR);
		if (rc < 0) {
			DRM_ERROR("failed to attach panel bridge: %d\n", rc);
			return rc;
		}
	}

	msm_dp_display->bridge = bridge;

	return 0;
}

/* connector initialization */
struct drm_connector *msm_dp_drm_connector_init(struct msm_dp *msm_dp_display,
					    struct drm_encoder *encoder)
{
	struct drm_connector *connector = NULL;

	connector = drm_bridge_connector_init(msm_dp_display->drm_dev, encoder);
	if (IS_ERR(connector))
		return connector;

	if (!msm_dp_display->is_edp) {
		drm_connector_attach_dp_subconnector_property(connector);
		msm_dp_attach_metadata_properties(connector);
	}

	return connector;
}
