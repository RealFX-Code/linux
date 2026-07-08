// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 chiffathefox <chiffathefoxx@gmail.com>
 *
 * EXYNOS9820 PMU (Power Management Unit) support
 */

#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>

#include "exynos-pmu.h"

const struct exynos_pmu_data exynos9820_pmu_data = {
	.pmu_cpuhp = true,
};
