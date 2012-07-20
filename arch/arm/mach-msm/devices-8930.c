/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/ion.h>
#include <mach/msm_iomap.h>
#include <mach/irqs-8930.h>
#include <mach/rpm.h>
#include <mach/msm_dcvs.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/board.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "rpm_log.h"
#include "rpm_stats.h"

#ifdef CONFIG_MSM_MPM
#include "mpm.h"
#endif

struct msm_rpm_platform_data msm8930_rpm_data __initdata = {
	.reg_base_addrs = {
		[MSM_RPM_PAGE_STATUS] = MSM_RPM_BASE,
		[MSM_RPM_PAGE_CTRL] = MSM_RPM_BASE + 0x400,
		[MSM_RPM_PAGE_REQ] = MSM_RPM_BASE + 0x600,
		[MSM_RPM_PAGE_ACK] = MSM_RPM_BASE + 0xa00,
	},
	.irq_ack = RPM_APCC_CPU0_GP_HIGH_IRQ,
	.irq_err = RPM_APCC_CPU0_GP_LOW_IRQ,
	.ipc_rpm_reg = MSM_APCS_GCC_BASE + 0x008,
	.ipc_rpm_val = 4,
	.target_id = {
		MSM_RPM_MAP(8930, NOTIFICATION_CONFIGURED_0, NOTIFICATION, 4),
		MSM_RPM_MAP(8930, NOTIFICATION_REGISTERED_0, NOTIFICATION, 4),
		MSM_RPM_MAP(8930, INVALIDATE_0, INVALIDATE, 8),
		MSM_RPM_MAP(8960, TRIGGER_TIMED_TO, TRIGGER_TIMED, 1),
		MSM_RPM_MAP(8960, TRIGGER_TIMED_SCLK_COUNT, TRIGGER_TIMED, 1),
		MSM_RPM_MAP(8930, RPM_CTL, RPM_CTL, 1),
		MSM_RPM_MAP(8930, CXO_CLK, CXO_CLK, 1),
		MSM_RPM_MAP(8930, PXO_CLK, PXO_CLK, 1),
		MSM_RPM_MAP(8930, APPS_FABRIC_CLK, APPS_FABRIC_CLK, 1),
		MSM_RPM_MAP(8930, SYSTEM_FABRIC_CLK, SYSTEM_FABRIC_CLK, 1),
		MSM_RPM_MAP(8930, MM_FABRIC_CLK, MM_FABRIC_CLK, 1),
		MSM_RPM_MAP(8930, DAYTONA_FABRIC_CLK, DAYTONA_FABRIC_CLK, 1),
		MSM_RPM_MAP(8930, SFPB_CLK, SFPB_CLK, 1),
		MSM_RPM_MAP(8930, CFPB_CLK, CFPB_CLK, 1),
		MSM_RPM_MAP(8930, MMFPB_CLK, MMFPB_CLK, 1),
		MSM_RPM_MAP(8930, EBI1_CLK, EBI1_CLK, 1),
		MSM_RPM_MAP(8930, APPS_FABRIC_CFG_HALT_0,
				APPS_FABRIC_CFG_HALT, 2),
		MSM_RPM_MAP(8930, APPS_FABRIC_CFG_CLKMOD_0,
				APPS_FABRIC_CFG_CLKMOD, 3),
		MSM_RPM_MAP(8930, APPS_FABRIC_CFG_IOCTL,
				APPS_FABRIC_CFG_IOCTL, 1),
		MSM_RPM_MAP(8930, APPS_FABRIC_ARB_0, APPS_FABRIC_ARB, 6),
		MSM_RPM_MAP(8930, SYS_FABRIC_CFG_HALT_0,
				SYS_FABRIC_CFG_HALT, 2),
		MSM_RPM_MAP(8930, SYS_FABRIC_CFG_CLKMOD_0,
				SYS_FABRIC_CFG_CLKMOD, 3),
		MSM_RPM_MAP(8930, SYS_FABRIC_CFG_IOCTL,
				SYS_FABRIC_CFG_IOCTL, 1),
		MSM_RPM_MAP(8930, SYSTEM_FABRIC_ARB_0,
				SYSTEM_FABRIC_ARB, 20),
		MSM_RPM_MAP(8930, MMSS_FABRIC_CFG_HALT_0,
				MMSS_FABRIC_CFG_HALT, 2),
		MSM_RPM_MAP(8930, MMSS_FABRIC_CFG_CLKMOD_0,
				MMSS_FABRIC_CFG_CLKMOD, 3),
		MSM_RPM_MAP(8930, MMSS_FABRIC_CFG_IOCTL,
				MMSS_FABRIC_CFG_IOCTL, 1),
		MSM_RPM_MAP(8930, MM_FABRIC_ARB_0, MM_FABRIC_ARB, 11),
		MSM_RPM_MAP(8930, PM8038_S1_0, PM8038_S1, 2),
		MSM_RPM_MAP(8930, PM8038_S2_0, PM8038_S2, 2),
		MSM_RPM_MAP(8930, PM8038_S3_0, PM8038_S3, 2),
		MSM_RPM_MAP(8930, PM8038_S4_0, PM8038_S4, 2),
		MSM_RPM_MAP(8930, PM8038_S5_0, PM8038_S5, 2),
		MSM_RPM_MAP(8930, PM8038_S6_0, PM8038_S6, 2),
		MSM_RPM_MAP(8930, PM8038_L1_0, PM8038_L1, 2),
		MSM_RPM_MAP(8930, PM8038_L2_0, PM8038_L2, 2),
		MSM_RPM_MAP(8930, PM8038_L3_0, PM8038_L3, 2),
		MSM_RPM_MAP(8930, PM8038_L4_0, PM8038_L4, 2),
		MSM_RPM_MAP(8930, PM8038_L5_0, PM8038_L5, 2),
		MSM_RPM_MAP(8930, PM8038_L6_0, PM8038_L6, 2),
		MSM_RPM_MAP(8930, PM8038_L7_0, PM8038_L7, 2),
		MSM_RPM_MAP(8930, PM8038_L8_0, PM8038_L8, 2),
		MSM_RPM_MAP(8930, PM8038_L9_0, PM8038_L9, 2),
		MSM_RPM_MAP(8930, PM8038_L10_0, PM8038_L10, 2),
		MSM_RPM_MAP(8930, PM8038_L11_0, PM8038_L11, 2),
		MSM_RPM_MAP(8930, PM8038_L12_0, PM8038_L12, 2),
		MSM_RPM_MAP(8930, PM8038_L13_0, PM8038_L13, 2),
		MSM_RPM_MAP(8930, PM8038_L14_0, PM8038_L14, 2),
		MSM_RPM_MAP(8930, PM8038_L15_0, PM8038_L15, 2),
		MSM_RPM_MAP(8930, PM8038_L16_0, PM8038_L16, 2),
		MSM_RPM_MAP(8930, PM8038_L17_0, PM8038_L17, 2),
		MSM_RPM_MAP(8930, PM8038_L18_0, PM8038_L18, 2),
		MSM_RPM_MAP(8930, PM8038_L19_0, PM8038_L19, 2),
		MSM_RPM_MAP(8930, PM8038_L20_0, PM8038_L20, 2),
		MSM_RPM_MAP(8930, PM8038_L21_0, PM8038_L21, 2),
		MSM_RPM_MAP(8930, PM8038_L22_0, PM8038_L22, 2),
		MSM_RPM_MAP(8930, PM8038_L23_0, PM8038_L23, 2),
		MSM_RPM_MAP(8930, PM8038_L24_0, PM8038_L24, 2),
		MSM_RPM_MAP(8930, PM8038_L25_0, PM8038_L25, 2),
		MSM_RPM_MAP(8930, PM8038_L26_0, PM8038_L26, 2),
		MSM_RPM_MAP(8930, PM8038_L27_0, PM8038_L27, 2),
		MSM_RPM_MAP(8930, PM8038_CLK1_0, PM8038_CLK1, 2),
		MSM_RPM_MAP(8930, PM8038_CLK2_0, PM8038_CLK2, 2),
		MSM_RPM_MAP(8930, PM8038_LVS1, PM8038_LVS1, 1),
		MSM_RPM_MAP(8930, PM8038_LVS2, PM8038_LVS2, 1),
		MSM_RPM_MAP(8930, NCP_0, NCP, 2),
		MSM_RPM_MAP(8930, CXO_BUFFERS, CXO_BUFFERS, 1),
		MSM_RPM_MAP(8930, USB_OTG_SWITCH, USB_OTG_SWITCH, 1),
		MSM_RPM_MAP(8930, HDMI_SWITCH, HDMI_SWITCH, 1),
		MSM_RPM_MAP(8930, QDSS_CLK, QDSS_CLK, 1),
		MSM_RPM_MAP(8930, VOLTAGE_CORNER, VOLTAGE_CORNER, 1),
	},
	.target_status = {
		MSM_RPM_STATUS_ID_MAP(8930, VERSION_MAJOR),
		MSM_RPM_STATUS_ID_MAP(8930, VERSION_MINOR),
		MSM_RPM_STATUS_ID_MAP(8930, VERSION_BUILD),
		MSM_RPM_STATUS_ID_MAP(8930, SUPPORTED_RESOURCES_0),
		MSM_RPM_STATUS_ID_MAP(8930, SUPPORTED_RESOURCES_1),
		MSM_RPM_STATUS_ID_MAP(8930, SUPPORTED_RESOURCES_2),
		MSM_RPM_STATUS_ID_MAP(8930, RESERVED_SUPPORTED_RESOURCES_0),
		MSM_RPM_STATUS_ID_MAP(8930, SEQUENCE),
		MSM_RPM_STATUS_ID_MAP(8930, RPM_CTL),
		MSM_RPM_STATUS_ID_MAP(8930, CXO_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, PXO_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, APPS_FABRIC_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, SYSTEM_FABRIC_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, MM_FABRIC_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, DAYTONA_FABRIC_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, SFPB_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, CFPB_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, MMFPB_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, EBI1_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, APPS_FABRIC_CFG_HALT),
		MSM_RPM_STATUS_ID_MAP(8930, APPS_FABRIC_CFG_CLKMOD),
		MSM_RPM_STATUS_ID_MAP(8930, APPS_FABRIC_CFG_IOCTL),
		MSM_RPM_STATUS_ID_MAP(8930, APPS_FABRIC_ARB),
		MSM_RPM_STATUS_ID_MAP(8930, SYS_FABRIC_CFG_HALT),
		MSM_RPM_STATUS_ID_MAP(8930, SYS_FABRIC_CFG_CLKMOD),
		MSM_RPM_STATUS_ID_MAP(8930, SYS_FABRIC_CFG_IOCTL),
		MSM_RPM_STATUS_ID_MAP(8930, SYSTEM_FABRIC_ARB),
		MSM_RPM_STATUS_ID_MAP(8930, MMSS_FABRIC_CFG_HALT),
		MSM_RPM_STATUS_ID_MAP(8930, MMSS_FABRIC_CFG_CLKMOD),
		MSM_RPM_STATUS_ID_MAP(8930, MMSS_FABRIC_CFG_IOCTL),
		MSM_RPM_STATUS_ID_MAP(8930, MM_FABRIC_ARB),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S1_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S1_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S2_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S2_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S3_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S3_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S4_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_S4_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L1_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L1_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L2_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L2_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L3_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L3_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L4_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L4_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L5_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L5_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L6_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L6_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L7_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L7_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L8_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L8_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L9_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L9_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L10_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L10_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L11_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L11_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L12_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L12_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L13_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L13_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L14_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L14_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L15_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L15_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L16_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L16_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L17_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L17_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L18_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L18_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L19_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L19_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L20_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L20_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L21_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L21_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L22_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L22_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L23_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L23_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L24_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L24_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L25_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_L25_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_CLK1_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_CLK1_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_CLK2_0),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_CLK2_1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_LVS1),
		MSM_RPM_STATUS_ID_MAP(8930, PM8038_LVS2),
		MSM_RPM_STATUS_ID_MAP(8930, NCP_0),
		MSM_RPM_STATUS_ID_MAP(8930, NCP_1),
		MSM_RPM_STATUS_ID_MAP(8930, CXO_BUFFERS),
		MSM_RPM_STATUS_ID_MAP(8930, USB_OTG_SWITCH),
		MSM_RPM_STATUS_ID_MAP(8930, HDMI_SWITCH),
		MSM_RPM_STATUS_ID_MAP(8930, QDSS_CLK),
		MSM_RPM_STATUS_ID_MAP(8930, VOLTAGE_CORNER),
	},
	.target_ctrl_id = {
		MSM_RPM_CTRL_MAP(8930, VERSION_MAJOR),
		MSM_RPM_CTRL_MAP(8930, VERSION_MINOR),
		MSM_RPM_CTRL_MAP(8930, VERSION_BUILD),
		MSM_RPM_CTRL_MAP(8930, REQ_CTX_0),
		MSM_RPM_CTRL_MAP(8930, REQ_SEL_0),
		MSM_RPM_CTRL_MAP(8930, ACK_CTX_0),
		MSM_RPM_CTRL_MAP(8930, ACK_SEL_0),
	},
	.sel_invalidate = MSM_RPM_8930_SEL_INVALIDATE,
	.sel_notification = MSM_RPM_8930_SEL_NOTIFICATION,
	.sel_last = MSM_RPM_8930_SEL_LAST,
	.ver = {3, 0, 0},
};

struct platform_device msm8930_rpm_device = {
	.name   = "msm_rpm",
	.id     = -1,
};

static struct msm_rpm_log_platform_data msm_rpm_log_pdata = {
	.phys_addr_base = 0x0010C000,
	.reg_offsets = {
		[MSM_RPM_LOG_PAGE_INDICES] = 0x00000080,
		[MSM_RPM_LOG_PAGE_BUFFER]  = 0x000000A0,
	},
	.phys_size = SZ_8K,
	.log_len = 4096,		  /* log's buffer length in bytes */
	.log_len_mask = (4096 >> 2) - 1,  /* length mask in units of u32 */
};

struct platform_device msm8930_rpm_log_device = {
	.name	= "msm_rpm_log",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_log_pdata,
	},
};

static struct msm_rpmstats_platform_data msm_rpm_stat_pdata = {
	.phys_addr_base = 0x0010D204,
	.phys_size = SZ_8K,
};

struct platform_device msm8930_rpm_stat_device = {
	.name = "msm_rpm_stat",
	.id = -1,
	.dev = {
		.platform_data = &msm_rpm_stat_pdata,
	},
};

static int msm8930_LPM_latency = 1000; /* >100 usec for WFI */

struct platform_device msm8930_cpu_idle_device = {
	.name   = "msm_cpu_idle",
	.id     = -1,
	.dev = {
		.platform_data = &msm8930_LPM_latency,
	},
};

static struct msm_dcvs_freq_entry msm8930_freq[] = {
	{ 384000, 166981,  345600},
	{ 702000, 213049,  632502},
	{1026000, 285712,  925613},
	{1242000, 383945, 1176550},
	{1458000, 419729, 1465478},
	{1512000, 434116, 1546674},

};

static struct msm_dcvs_core_info msm8930_core_info = {
	.freq_tbl = &msm8930_freq[0],
	.core_param = {
		.max_time_us = 100000,
		.num_freq = ARRAY_SIZE(msm8930_freq),
	},
	.algo_param = {
		.slack_time_us = 58000,
		.scale_slack_time = 0,
		.scale_slack_time_pct = 0,
		.disable_pc_threshold = 1458000,
		.em_window_size = 100000,
		.em_max_util_pct = 97,
		.ss_window_size = 1000000,
		.ss_util_pct = 95,
		.ss_iobusy_conv = 100,
	},
};

struct platform_device msm8930_msm_gov_device = {
	.name = "msm_dcvs_gov",
	.id = -1,
	.dev = {
		.platform_data = &msm8930_core_info,
	},
};

struct platform_device msm_bus_8930_sys_fabric = {
	.name  = "msm_bus_fabric",
	.id    =  MSM_BUS_FAB_SYSTEM,
};
struct platform_device msm_bus_8930_apps_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_APPSS,
};
struct platform_device msm_bus_8930_mm_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_MMSS,
};
struct platform_device msm_bus_8930_sys_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_SYSTEM_FPB,
};
struct platform_device msm_bus_8930_cpss_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_CPSS_FPB,
};

/* MSM Video core device */
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors vidc_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors vidc_venc_vga_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 54525952,
		.ib  = 436207616,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 72351744,
		.ib  = 289406976,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 1000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 1000000,
	},
};
static struct msm_bus_vectors vidc_vdec_vga_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 40894464,
		.ib  = 327155712,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 48234496,
		.ib  = 192937984,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 2000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 2000000,
	},
};
static struct msm_bus_vectors vidc_venc_720p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 163577856,
		.ib  = 1308622848,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 219152384,
		.ib  = 876609536,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 3500000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 3500000,
	},
};
static struct msm_bus_vectors vidc_vdec_720p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 121634816,
		.ib  = 973078528,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 155189248,
		.ib  = 620756992,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 7000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 7000000,
	},
};
static struct msm_bus_vectors vidc_venc_1080p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 372244480,
		.ib  = 2560000000U,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 501219328,
		.ib  = 2560000000U,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 5000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 5000000,
	},
};
static struct msm_bus_vectors vidc_vdec_1080p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 222298112,
		.ib  = 2560000000U,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 330301440,
		.ib  = 2560000000U,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 700000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 10000000,
	},
};

static struct msm_bus_paths vidc_bus_client_config[] = {
	{
		ARRAY_SIZE(vidc_init_vectors),
		vidc_init_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_vga_vectors),
		vidc_venc_vga_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_vga_vectors),
		vidc_vdec_vga_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_720p_vectors),
		vidc_venc_720p_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_720p_vectors),
		vidc_vdec_720p_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_1080p_vectors),
		vidc_venc_1080p_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_1080p_vectors),
		vidc_vdec_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata vidc_bus_client_data = {
	vidc_bus_client_config,
	ARRAY_SIZE(vidc_bus_client_config),
	.name = "vidc",
};
#endif

#define MSM_VIDC_BASE_PHYS 0x04400000
#define MSM_VIDC_BASE_SIZE 0x00100000

static struct resource apq8930_device_vidc_resources[] = {
	{
		.start	= MSM_VIDC_BASE_PHYS,
		.end	= MSM_VIDC_BASE_PHYS + MSM_VIDC_BASE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VCODEC_IRQ,
		.end	= VCODEC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct msm_vidc_platform_data apq8930_vidc_platform_data = {
#ifdef CONFIG_MSM_BUS_SCALING
	.vidc_bus_client_pdata = &vidc_bus_client_data,
#endif
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.memtype = ION_CP_MM_HEAP_ID,
	.enable_ion = 1,
#else
	.memtype = MEMTYPE_EBI1,
	.enable_ion = 0,
#endif
	.disable_dmx = 0,
	.disable_fullhd = 0,
};

struct platform_device apq8930_msm_device_vidc = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(apq8930_device_vidc_resources),
	.resource = apq8930_device_vidc_resources,
	.dev = {
		.platform_data = &apq8930_vidc_platform_data,
	},
};

struct platform_device *vidc_device[] __initdata = {
	&apq8930_msm_device_vidc
};

void __init msm8930_add_vidc_device(void)
{
	if (cpu_is_msm8627()) {
		struct msm_vidc_platform_data *pdata;
		pdata = (struct msm_vidc_platform_data *)
			apq8930_msm_device_vidc.dev.platform_data;
		pdata->disable_fullhd = 1;
	}
	platform_add_devices(vidc_device, ARRAY_SIZE(vidc_device));
}
