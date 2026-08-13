/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _DP_DRM_H_
#define _DP_DRM_H_

#include <linux/types.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_bridge.h>

#include "msm_drv.h"
#include "dp_display.h"

struct msm_dp_bridge {
	struct drm_bridge bridge;
	struct msm_dp *msm_dp_display;
};

#define to_dp_bridge(x)     container_of((x), struct msm_dp_bridge, bridge)

struct msm_dp_bridge_state {
	struct drm_bridge_state base;
	struct msm_dp_dsc_config dsc;
	u32 bpp;
};

#define to_msm_dp_bridge_state(x) \
	container_of(x, struct msm_dp_bridge_state, base)

struct drm_connector *msm_dp_drm_connector_init(struct msm_dp *msm_dp_display,
					    struct drm_encoder *encoder);
int msm_dp_bridge_init(struct msm_dp *msm_dp_display, struct drm_device *dev,
		   struct drm_encoder *encoder,
		   bool yuv_supported);
struct drm_dsc_config *msm_dp_bridge_get_dsc_config(struct drm_encoder *encoder,
						    struct drm_atomic_commit *state);
int msm_dp_bridge_disable_dsc(struct drm_encoder *encoder,
			      struct drm_atomic_commit *state);
int msm_dp_dsc_compute_config(struct msm_dp_dsc_config *dsc,
			      const u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE],
			      const struct drm_display_mode *mode,
			      u8 max_bpc, u8 lane_count);

enum drm_connector_status msm_dp_bridge_detect(struct drm_bridge *bridge,
					       struct drm_connector *connector);
void msm_dp_bridge_atomic_enable(struct drm_bridge *drm_bridge,
				 struct drm_atomic_commit *state);
void msm_dp_bridge_atomic_disable(struct drm_bridge *drm_bridge,
				  struct drm_atomic_commit *state);
void msm_dp_bridge_atomic_post_disable(struct drm_bridge *drm_bridge,
				       struct drm_atomic_commit *state);
enum drm_mode_status msm_dp_bridge_mode_valid(struct drm_bridge *bridge,
					  const struct drm_display_info *info,
					  const struct drm_display_mode *mode);
void msm_dp_bridge_mode_set(struct drm_bridge *drm_bridge,
			const struct drm_display_mode *mode,
			const struct drm_display_mode *adjusted_mode);
void msm_dp_bridge_hpd_enable(struct drm_bridge *bridge);
void msm_dp_bridge_hpd_disable(struct drm_bridge *bridge);
void msm_dp_bridge_hpd_notify(struct drm_bridge *bridge,
			      struct drm_connector *connector,
			      enum drm_connector_status status);

#endif /* _DP_DRM_H_ */
