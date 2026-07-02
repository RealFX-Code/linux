/*
 * Broadcom BCM470X / BCM5301X ARM platform code.
 *
 * Copyright 2013 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include <asm/mach/arch.h>
#include <asm/siginfo.h>
#include <asm/signal.h>

#define FSR_EXTERNAL		(1 << 12)
#define FSR_READ		(0 << 10)
#define FSR_IMPRECISE		0x0406

static const char *const bcm5301x_dt_compat[] __initconst = {
	"brcm,bcm4708",
	NULL,
};

static int bcm5301x_abort_handler(unsigned long addr, unsigned int fsr,
				  struct pt_regs *regs)
{
	/*
	 * We want to ignore aborts forwarded from the PCIe bus that are
	 * expected and shouldn't really be passed by the PCIe controller.
	 * The biggest disadvantage is the same FSR code may be reported when
	 * reading non-existing APB register and we shouldn't ignore that.
	 */
	pr_info("bcm5301x_abort_handler();\n");
	if ((fsr == (FSR_EXTERNAL | FSR_READ | FSR_IMPRECISE)) ||
		((fsr & 0x1c0f) == 0x1c06)) {
		pr_info("Hit an abort we don't want to handle!\n");
		pr_info(" >> fsr = 0x%x\n", fsr);
		pr_info(" >> fsr & 0x1c0f = 0x%x\n", fsr & 0x1c0f);
		return 0;
	}
	pr_info("Hit an abort, but we didn't ignore it! Maddness awaits!\n");
	return 1;
}

static void __init bcm5301x_map_io(void)
{
	pr_info("bcm5301x_map_io();\n");
    hook_fault_code(16 + 6, bcm5301x_abort_handler, SIGBUS, BUS_OBJERR,
        "imprecise external abort");
	pr_info("bcm5301x_map_io(); done!\n");
}

static void __init bcm5301x_init_early(void)
{
	hook_fault_code(16 + 6, bcm5301x_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
}

DT_MACHINE_START(BCM5301X, "BCM5301X")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.map_io		= bcm5301x_map_io,
	.dt_compat	= bcm5301x_dt_compat,
	.init_early	= bcm5301x_init_early,
MACHINE_END
