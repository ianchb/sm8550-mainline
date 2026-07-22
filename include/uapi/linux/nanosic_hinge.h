/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_NANOSIC_HINGE_H
#define _UAPI_LINUX_NANOSIC_HINGE_H

#include <linux/types.h>

#define NANOSIC_HINGE_ABI_VERSION	1

#define NANOSIC_HINGE_SAMPLE_ATTACHED	(1U << 0)
#define NANOSIC_HINGE_SAMPLE_VALID	(1U << 1)

#define NANOSIC_HINGE_CONTROL_ENABLE	(1U << 0)

struct nanosic_hinge_sample {
	__u16 version;
	__u16 flags;
	__s16 x;
	__s16 y;
	__s16 z;
	__s16 reserved;
};

struct nanosic_hinge_control {
	__u16 version;
	__u16 flags;
};

#endif /* _UAPI_LINUX_NANOSIC_HINGE_H */
