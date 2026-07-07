// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos FlexPMU (Active Power Management) debug interface.
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The FlexPMU firmware running on the APM (Active Power Management)
 * co-processor mirrors a snapshot of its internal power-sequencer state
 * into a small window of the APM SRAM. This driver exposes that snapshot
 * through debugfs so the CPU/cluster power state, low-power-mode counters
 * and MIF (memory interface) request accounting can be inspected from the
 * application processor.
 *
 * The window is a table of fixed 16-byte lines. Each line holds an 8-byte
 * firmware-private header followed by an 8-byte data payload, read either
 * as eight bytes or two 32-bit words depending on the field.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Geometry of the FlexPMU debug data window. */
#define FLEXPMU_LINE_SIZE	16	/* bytes per data line */
#define FLEXPMU_DATA_OFF	8	/* payload offset within a line */

#define BUF_MAX_LINE		10
#define BUF_LINE_SIZE		255
#define BUF_SIZE		(BUF_MAX_LINE * BUF_LINE_SIZE)

/* RTC tick counter used for MIF request accounting. */
#define RTC_CURTICCNT_0		0x90
#define RTC_TICK_TO_US		976	/* 1024 Hz: 1 tick = 976.5625 us */

/* Print format for a single field. */
enum flexpmu_print_type {
	NO_PRINT = 0,
	DEC_PRINT,
	HEX_PRINT,
};

/* debugfs files exposed under the flexpmu-dbg directory. */
enum flexpmu_fid {
	FID_CPU_STATUS,
	FID_SEQ_STATUS,
	FID_CUR_SEQ,
	FID_SW_FLAG,
	FID_SEQ_COUNT,
	FID_MIF_ALWAYS_ON,
	FID_LPM_COUNT,
	FID_APM_REQ_INFO,
	FID_CP_MIF_REQ,
	FID_AUD_MIF_REQ,
	FID_VTS_MIF_REQ,
	FID_AP_MIF_REQ,
	FID_LOG_STOP,
	FID_MAX
};

static const char * const flexpmu_file_name[FID_MAX] = {
	[FID_CPU_STATUS]	= "cpu_status",
	[FID_SEQ_STATUS]	= "seq_status",
	[FID_CUR_SEQ]		= "cur_sequence",
	[FID_SW_FLAG]		= "sw_flag",
	[FID_SEQ_COUNT]		= "seq_count",
	[FID_MIF_ALWAYS_ON]	= "mif_always_on",
	[FID_LPM_COUNT]		= "lpm_count",
	[FID_APM_REQ_INFO]	= "apm_req_info",
	[FID_CP_MIF_REQ]	= "mif_cp_active_time",
	[FID_AUD_MIF_REQ]	= "mif_aud_active_time",
	[FID_VTS_MIF_REQ]	= "mif_vts_active_time",
	[FID_AP_MIF_REQ]	= "mif_ap_active_time",
	[FID_LOG_STOP]		= "log_stop",
};

/*
 * Index of each data line in the debug window. These values are the
 * firmware ABI: they select the 16-byte line to read and must not be
 * reordered.
 */
enum flexpmu_did {
	DID_CPU_STATUS,		/* 0 */
	DID_SEQ_STATUS,
	DID_CUR_SEQ0,
	DID_CUR_SEQ1,
	DID_PWR_MODE0,		/* 4 */
	DID_PWR_MODE1,
	DID_PWR_MODE2,
	DID_PWR_MODE3,
	DID_PWR_MODE4,
	DID_PWR_MODE5,
	DID_SW_FLAG,
	DID_IRQ_STATUS,		/* 11 */
	DID_IRQ_DATA,
	DID_IPC_AP_STATUS,
	DID_IPC_AP_RXDATA,
	DID_IPC_AP_TXDATA,
	DID_SOC_COUNT,
	DID_MIF_COUNT,
	DID_IPC_VTS0,
	DID_IPC_VTS1,
	DID_LOCAL_PWR,
	DID_MIF_ALWAYS_ON,	/* 21 */
	DID_AP_COUNT_SLEEP,
	DID_MIF_COUNT_SLEEP,
	DID_AP_COUNT_SICD,
	DID_MIF_COUNT_SICD,
	DID_CUR_PMD,
	DID_CPU_INFORM01,
	DID_CPU_INFORM23,
	DID_CPU_INFORM45,
	DID_CPU_INFORM67,
	DID_INT_REG01,
	DID_INT_REG02,
	DID_INT_REG03,
	DID_INT_REG04,
	DID_INT_REG05,
	DID_INT_REG06,
	DID_INT_REG07,
	DID_INT_REG08,
	DID_INT_REG09,
	DID_INT_REG10,
	DID_INT_REG11,
	DID_MIFCP0,
	DID_MIFCP1,
	DID_MIFAUD0,
	DID_MIFAUD1,
	DID_MIFVTS0,
	DID_MIFVTS1,
	DID_MIFAP0,
	DID_MIFAP1,
	DID_LOG_STOP,
	DID_MAX
};

#define MIF_MASTER_MAX		4

static const char * const flexpmu_master_name[MIF_MASTER_MAX] = {
	"MIF_CP",
	"MIF_AUD",
	"MIF_VTS",
	"MIF_AP",
};

struct flexpmu_print_arg {
	const char *prefix;
	enum flexpmu_print_type type;
};

struct flexpmu_dbg {
	struct device *dev;
	void __iomem *base;		/* FlexPMU debug window in APM SRAM */
	void __iomem *rtc;		/* APM RTC tick counter (optional) */
	struct dentry *root;
	struct flexpmu_dbg_file {
		struct flexpmu_dbg *dbg;
		enum flexpmu_fid fid;
	} files[FID_MAX];
};

static void __iomem *flexpmu_line(struct flexpmu_dbg *dbg, enum flexpmu_did did)
{
	return dbg->base + FLEXPMU_LINE_SIZE * did + FLEXPMU_DATA_OFF;
}

/* Emit up to @count fields of a line, each 32 bits wide. */
static ssize_t print_words(struct flexpmu_dbg *dbg, enum flexpmu_did did,
			   const struct flexpmu_print_arg *args, int count,
			   char *buf, ssize_t len, int *arg_idx)
{
	void __iomem *line = flexpmu_line(dbg, did);
	int i;

	for (i = 0; i < count; i++, (*arg_idx)++) {
		const struct flexpmu_print_arg *a = &args[*arg_idx];
		u32 data = readl(line + i * 4);

		if (a->type == DEC_PRINT)
			len += scnprintf(buf + len, BUF_SIZE - len, "%s : %d\n",
					 a->prefix, data);
		else if (a->type == HEX_PRINT)
			len += scnprintf(buf + len, BUF_SIZE - len, "%s : 0x%x\n",
					 a->prefix, data);
	}

	return len;
}

/* Emit the eight bytes of a line's payload as separate fields. */
static ssize_t print_bytes(struct flexpmu_dbg *dbg, enum flexpmu_did did,
			   const struct flexpmu_print_arg *args, char *buf,
			   ssize_t len, int *arg_idx)
{
	void __iomem *line = flexpmu_line(dbg, did);
	int i;

	for (i = 0; i < 8; i++, (*arg_idx)++) {
		const struct flexpmu_print_arg *a = &args[*arg_idx];
		u8 data = readb(line + i);

		if (a->type == DEC_PRINT)
			len += scnprintf(buf + len, BUF_SIZE - len, "%s : %d\n",
					 a->prefix, data);
		else if (a->type == HEX_PRINT)
			len += scnprintf(buf + len, BUF_SIZE - len, "%s : 0x%x\n",
					 a->prefix, data);
	}

	return len;
}

static ssize_t flexpmu_cpu_status_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "CL0_CPU0", DEC_PRINT },
		{ "CL0_CPU1", DEC_PRINT },
		{ "CL0_CPU2", DEC_PRINT },
		{ "CL0_CPU3", DEC_PRINT },
		{ "CL1_CPU0", DEC_PRINT },
		{ "CL1_CPU1", DEC_PRINT },
		{ "CL1_CPU2", DEC_PRINT },
		{ "CL1_CPU3", DEC_PRINT },
	};
	int arg_idx = 0;

	return print_bytes(dbg, DID_CPU_STATUS, args, buf, 0, &arg_idx);
}

static ssize_t flexpmu_seq_status_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "SOC seq", DEC_PRINT },
		{ "MIF seq", DEC_PRINT },
		{ },
		{ },
		{ "nonCPU CL0", DEC_PRINT },
		{ "nonCPU CL1", DEC_PRINT },
		{ },
	};
	int arg_idx = 0;

	return print_bytes(dbg, DID_SEQ_STATUS, args, buf, 0, &arg_idx);
}

static ssize_t flexpmu_cur_sequence_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "UP Sequence", DEC_PRINT },
		{ "DOWN Sequence", DEC_PRINT },
		{ "Access Type", DEC_PRINT },
		{ "Seq Index", DEC_PRINT },
	};
	int arg_idx = 0;
	ssize_t len;

	len = print_words(dbg, DID_CUR_SEQ0, args, 2, buf, 0, &arg_idx);
	return print_words(dbg, DID_CUR_SEQ1, args, 2, buf, len, &arg_idx);
}

static ssize_t flexpmu_sw_flag_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "Hotplug out flag", HEX_PRINT },
		{ "Reset-Release flag", HEX_PRINT },
		{ },
		{ },
		{ "CHUB ref_count", DEC_PRINT },
		{ },
		{ "MIF req_Master", HEX_PRINT },
		{ "MIF req_count", DEC_PRINT },
	};
	int arg_idx = 0;

	return print_bytes(dbg, DID_SW_FLAG, args, buf, 0, &arg_idx);
}

static ssize_t flexpmu_seq_count_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "Early Wakeup", DEC_PRINT },
		{ "SOC sequence", DEC_PRINT },
		{ },
		{ "MIF sequence", DEC_PRINT },
	};
	int arg_idx = 0;
	ssize_t len;

	len = print_words(dbg, DID_SOC_COUNT, args, 2, buf, 0, &arg_idx);
	return print_words(dbg, DID_MIF_COUNT, args, 2, buf, len, &arg_idx);
}

static ssize_t flexpmu_mif_always_on_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ },
		{ "MIF always on", DEC_PRINT },
	};
	int arg_idx = 0;

	return print_words(dbg, DID_MIF_ALWAYS_ON, args, 2, buf, 0, &arg_idx);
}

static ssize_t flexpmu_lpm_count_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ "[SLEEP] Early wakeup", DEC_PRINT },
		{ "[SLEEP] SOC seq down", DEC_PRINT },
		{ },
		{ "[SLEEP] MIF seq down", DEC_PRINT },
		{ "[SICD] Early wakeup", DEC_PRINT },
		{ "[SICD] SOC seq down", DEC_PRINT },
		{ },
		{ "[SICD] MIF seq down", DEC_PRINT },
	};
	int arg_idx = 0;
	ssize_t len;

	len = print_words(dbg, DID_AP_COUNT_SLEEP, args, 2, buf, 0, &arg_idx);
	len = print_words(dbg, DID_MIF_COUNT_SLEEP, args, 2, buf, len, &arg_idx);
	len = print_words(dbg, DID_AP_COUNT_SICD, args, 2, buf, len, &arg_idx);
	return print_words(dbg, DID_MIF_COUNT_SICD, args, 2, buf, len, &arg_idx);
}

static ssize_t flexpmu_log_stop_read(struct flexpmu_dbg *dbg, char *buf)
{
	static const struct flexpmu_print_arg args[] = {
		{ },
		{ "log stopped", DEC_PRINT },
	};
	int arg_idx = 0;

	return print_words(dbg, DID_LOG_STOP, args, 2, buf, 0, &arg_idx);
}

static ssize_t flexpmu_apm_req_info_read(struct flexpmu_dbg *dbg, char *buf)
{
	u64 curr_tick;
	ssize_t len;
	int i;

	if (!dbg->rtc)
		return scnprintf(buf, BUF_SIZE, "This node is not supported.\n");

	curr_tick = readl(dbg->rtc + RTC_CURTICCNT_0);
	len = scnprintf(buf, BUF_SIZE, "%s: %llu\n", "curr_time",
			curr_tick * RTC_TICK_TO_US);
	len += scnprintf(buf + len, BUF_SIZE - len,
			 "%8s   %32s %32s %32s %32s\n", "Master",
			 "active_since(us ago)", "last_rel_time(us ago)",
			 "total_req_time(us)", "req_count");

	for (i = 0; i < MIF_MASTER_MAX; i++) {
		void __iomem *req0 = flexpmu_line(dbg, DID_MIFCP0 + i * 2);
		void __iomem *req1 = flexpmu_line(dbg, DID_MIFCP1 + i * 2);
		u32 active_req_tick = readl(req0);
		u32 last_rel_tick = readl(req0 + 4);
		u32 total_count = readl(req1);
		u64 total_time_us = (u64)readl(req1 + 4) * RTC_TICK_TO_US;
		u64 active_since_us = 0;
		u64 last_rel_us = 0;

		if (last_rel_tick > 0)
			last_rel_us = (curr_tick - last_rel_tick) * RTC_TICK_TO_US;

		if (active_req_tick) {
			active_since_us =
				(curr_tick - active_req_tick) * RTC_TICK_TO_US;
			total_time_us += active_since_us;
		}

		len += scnprintf(buf + len, BUF_SIZE - len,
				 "%8s : %32llu %32llu %32llu %32u\n",
				 flexpmu_master_name[i], active_since_us,
				 last_rel_us, total_time_us, total_count);
	}

	return len;
}

/* Total time (us) a single MIF master has held its request active. */
static ssize_t flexpmu_master_mif_req_read(struct flexpmu_dbg *dbg,
					   enum flexpmu_fid fid, char *buf)
{
	int idx = fid - FID_CP_MIF_REQ;
	void __iomem *req0 = flexpmu_line(dbg, DID_MIFCP0 + idx * 2);
	void __iomem *req1 = flexpmu_line(dbg, DID_MIFCP1 + idx * 2);
	u32 active_req;
	u64 total_time;

	if (!dbg->rtc)
		return scnprintf(buf, BUF_SIZE, "This node is not supported.\n");

	active_req = readl(req0);
	total_time = readl(req1 + 4);

	if (active_req)
		total_time += readl(dbg->rtc + RTC_CURTICCNT_0) - active_req;

	return scnprintf(buf, BUF_SIZE, "%llu\n", total_time * RTC_TICK_TO_US);
}

static ssize_t flexpmu_dbg_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct flexpmu_dbg_file *f = file->private_data;
	struct flexpmu_dbg *dbg = f->dbg;
	char buf[BUF_SIZE] = { 0 };
	ssize_t len;

	switch (f->fid) {
	case FID_CPU_STATUS:
		len = flexpmu_cpu_status_read(dbg, buf);
		break;
	case FID_SEQ_STATUS:
		len = flexpmu_seq_status_read(dbg, buf);
		break;
	case FID_CUR_SEQ:
		len = flexpmu_cur_sequence_read(dbg, buf);
		break;
	case FID_SW_FLAG:
		len = flexpmu_sw_flag_read(dbg, buf);
		break;
	case FID_SEQ_COUNT:
		len = flexpmu_seq_count_read(dbg, buf);
		break;
	case FID_MIF_ALWAYS_ON:
		len = flexpmu_mif_always_on_read(dbg, buf);
		break;
	case FID_LPM_COUNT:
		len = flexpmu_lpm_count_read(dbg, buf);
		break;
	case FID_APM_REQ_INFO:
		len = flexpmu_apm_req_info_read(dbg, buf);
		break;
	case FID_CP_MIF_REQ:
	case FID_AUD_MIF_REQ:
	case FID_VTS_MIF_REQ:
	case FID_AP_MIF_REQ:
		len = flexpmu_master_mif_req_read(dbg, f->fid, buf);
		break;
	case FID_LOG_STOP:
		len = flexpmu_log_stop_read(dbg, buf);
		break;
	default:
		return -EINVAL;
	}

	if (len < 0)
		return len;

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

/*
 * A one-shot control byte is written into the second payload word (offset
 * 0xC) of the corresponding line, which the firmware polls.
 */
static ssize_t flexpmu_dbg_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct flexpmu_dbg_file *f = file->private_data;
	struct flexpmu_dbg *dbg = f->dbg;
	enum flexpmu_did did;
	bool val;
	char buf[8];
	ssize_t ret;

	switch (f->fid) {
	case FID_MIF_ALWAYS_ON:
		did = DID_MIF_ALWAYS_ON;
		break;
	case FID_LOG_STOP:
		did = DID_LOG_STOP;
		break;
	default:
		return -EINVAL;
	}

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (ret < 0)
		return ret;

	if (buf[0] == '0')
		val = false;
	else if (buf[0] == '1')
		val = true;
	else
		return -EINVAL;

	writel(val, dbg->base + FLEXPMU_LINE_SIZE * did + FLEXPMU_DATA_OFF + 4);

	return ret;
}

static const struct file_operations flexpmu_dbg_fops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= flexpmu_dbg_read,
	.write		= flexpmu_dbg_write,
	.llseek		= default_llseek,
};

static void flexpmu_dbg_remove_root(void *data)
{
	debugfs_remove_recursive(data);
}

static int flexpmu_dbg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct flexpmu_dbg *dbg;
	struct resource res;
	int i, ret;

	dbg = devm_kzalloc(dev, sizeof(*dbg), GFP_KERNEL);
	if (!dbg)
		return -ENOMEM;

	dbg->dev = dev;

	/*
	 * The debug window lives inside the APM SRAM, which is already claimed
	 * by the mmio-sram node, so map it without reserving the region.
	 */
	ret = of_address_to_resource(dev->of_node, 0, &res);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get data window\n");

	dbg->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!dbg->base)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to map data window\n");

	/* The RTC tick counter is optional; without it MIF timing is absent. */
	if (!of_address_to_resource(dev->of_node, 1, &res)) {
		dbg->rtc = devm_ioremap(dev, res.start, resource_size(&res));
		if (!dbg->rtc)
			return dev_err_probe(dev, -ENOMEM,
					     "failed to map RTC\n");
	}

	dbg->root = debugfs_create_dir("flexpmu-dbg", NULL);
	ret = devm_add_action_or_reset(dev, flexpmu_dbg_remove_root, dbg->root);
	if (ret)
		return ret;

	for (i = 0; i < FID_MAX; i++) {
		dbg->files[i].dbg = dbg;
		dbg->files[i].fid = i;
		debugfs_create_file(flexpmu_file_name[i], 0644, dbg->root,
				    &dbg->files[i], &flexpmu_dbg_fops);
	}

	platform_set_drvdata(pdev, dbg);

	return 0;
}

static const struct of_device_id flexpmu_dbg_match[] = {
	{ .compatible = "samsung,exynos9820-flexpmu-dbg" },
	{ }
};
MODULE_DEVICE_TABLE(of, flexpmu_dbg_match);

static struct platform_driver flexpmu_dbg_driver = {
	.probe	= flexpmu_dbg_probe,
	.driver	= {
		.name		= "exynos-flexpmu-dbg",
		.of_match_table	= flexpmu_dbg_match,
	},
};
module_platform_driver(flexpmu_dbg_driver);

MODULE_DESCRIPTION("Exynos FlexPMU debug interface");
MODULE_LICENSE("GPL");
