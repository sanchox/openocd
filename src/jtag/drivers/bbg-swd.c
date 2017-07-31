/***************************************************************************
 *   Copyright (C) 2016  Flying Stone Technology                           *
 *   Author: NIIBE Yutaka <gniibe@fsij.org>                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM 	 0

extern struct jtag_interface *jtag_interface;
static void bbg_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk);
static int bbg_swd_switch_seq(enum swd_special_seq seq);

static void pru_request_cmd(uint32_t *p)
{
	/* Wakeup the PRU0 which sleeps.  */
	prussdrv_pru_send_event(ARM_PRU0_INTERRUPT);

	/* Wait PRU0 response.  */
	prussdrv_pru_wait_event(PRU_EVTOUT_0);
	prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
	if ((p[0] & 0xff) == 4 || (p[0] & 0xff) == 5 || (p[0] & 0xff) == 7)
		LOG_DEBUG("BBD-SWD: command execution (%08x:%08x)", p[0], p[1]);
	else
		LOG_DEBUG("BBD-SWD: command execution (%08x)", p[0]);
}

static int queued_retval;

#define PRU_SWD_PROGRAM_PATH PKGDATADIR "/bbg-swd/pru-swd.bin"

static uint32_t *pru_data_ram;
static tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

static int bbg_swd_open(void)
{
	int r;

	LOG_DEBUG("bbg_swd_init");

	/* Initialize the PRUSS driver.  */
	prussdrv_init();

	/* Open PRU interrupt to Host.  */
	r = prussdrv_open(PRU_EVTOUT_0);
	if (r < 0) {
		LOG_ERROR("prussdrv_open open failed: %d", r);
		return ERROR_FAIL;
	}

	/* Initialize PRU interrupt controller.  */
	prussdrv_pruintc_init(&pruss_intc_initdata);

	/* Initialize PRU memory access from Host.  */
	r = prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, (void **)&pru_data_ram);
	if (r < 0) {
		prussdrv_exit();
		LOG_ERROR("prussdrv_map_prumem failed: %d", r);
		return ERROR_FAIL;
	}

	/* Execute example on PRU */
	LOG_DEBUG("Executing PRU-SWU program on PRUSS");
	r = prussdrv_exec_program(PRU_NUM, PRU_SWD_PROGRAM_PATH);
	if (r < 0) {
		prussdrv_exit();
		LOG_ERROR("prussdrv_exec_program failed: %d", r);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}


static int bbg_swd_close(void)
{
	/* Disable PRU.  */
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit();
	return ERROR_OK;
}


static void bbg_swd_gpio_srst(int on);

static bool swd_mode;

static int bbg_swd_interface_init(void)
{
	int retval;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (swd_mode) {
		retval = bbg_swd_open();
		if (retval != ERROR_OK)
			return retval;
	}

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			bbg_swd_gpio_srst(0);
			LOG_INFO("Connecting under reset");
		}
	}

	LOG_INFO("BBG-SWD: Interface ready");

	return ERROR_OK;
}

static int bbg_swd_interface_quit(void)
{
	bbg_swd_close();
	return ERROR_OK;
}

static int bbg_swd_swd_init(void)
{
	swd_mode = true;
	return ERROR_OK;
}

enum {
	CMD_HALT = 0,
	CMD_BLINK,
	CMD_GPIO_OUT,
	CMD_GPIO_IN,
	CMD_SIG_IDLE,
	CMD_SIG_GEN,
	CMD_READ_REG,
	CMD_WRITE_REG
};

#define BBG_SWS_RESULT 16

/* 
 * Signal patterns are defined in:
 *	ARM Debug Interface Architecture Specification (ADI)
 */
static const uint8_t seq_jtag_to_swd[] = { 0xde, 0xf9 };
static const int seq_jtag_to_swd_len = 16;
static const uint8_t seq_swd_to_seq[] = { 0x3c, 0xe7 };
static const int seq_swd_to_jtag_len = 16;

static void bbg_swd_idle(int count)
{
	pru_data_ram[0] = CMD_SIG_IDLE;
	pru_data_ram[1] = count;
	pru_request_cmd(pru_data_ram);
}

static void bbg_swd_gpio_srst(int signal)
{
	pru_data_ram[0] = CMD_GPIO_OUT;
	pru_data_ram[1] = (1 << 15);
	pru_data_ram[2] = signal ? 1 : 0;
	pru_request_cmd(pru_data_ram);
}

static int bbg_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_DEBUG("bbg_swd_switch_seq");

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		pru_data_ram[0] = CMD_SIG_GEN | (swd_seq_line_reset_len << 8);
		memcpy (&pru_data_ram[1], swd_seq_line_reset, (swd_seq_line_reset_len+7)/8);
		pru_request_cmd(pru_data_ram);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		pru_data_ram[0] = CMD_SIG_GEN | ((swd_seq_jtag_to_swd_len)<< 8);
		memcpy (&pru_data_ram[1], swd_seq_jtag_to_swd, (swd_seq_jtag_to_swd_len+7)/8);
		pru_request_cmd(pru_data_ram);
		bbg_swd_idle(8);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		pru_data_ram[0] = CMD_SIG_GEN | (swd_seq_swd_to_jtag_len << 8);
		memcpy (&pru_data_ram[1], swd_seq_swd_to_jtag, (swd_seq_swd_to_jtag_len+7)/8);
		pru_request_cmd(pru_data_ram);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}


static void bbg_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	int ack;
	int parity;
	uint32_t data;
	uint32_t delay = 0;

	LOG_DEBUG("bbg_swd_read_reg");
	assert(cmd & SWD_CMD_RnW);
	assert(ap_delay_clk < 256);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip bbg_swd_read_reg because queued_retval=%d", queued_retval);
		return;
	}

	if (cmd & SWD_CMD_APnDP)
		delay = ap_delay_clk;

	cmd |= 0x81;
	pru_data_ram[0] = CMD_READ_REG | (cmd << 8) | (delay << 24);
	pru_request_cmd(pru_data_ram);
	ack = pru_data_ram[BBG_SWS_RESULT] & 0x07;
	parity = (pru_data_ram[BBG_SWS_RESULT] & 0x80000000) != 0;
	data = pru_data_ram[BBG_SWS_RESULT+1];

	LOG_DEBUG("%s %s %s reg %X = %08" PRIx32,
		  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
		  cmd & SWD_CMD_APnDP ? "AP" : "DP",
		  cmd & SWD_CMD_RnW ? "read" : "write",
		  (cmd & SWD_CMD_A32) >> 1,
		  data);

	if (ack != SWD_ACK_OK) {
		queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
	} else if (parity != parity_u32(data)) {
		LOG_DEBUG("Wrong parity detected (%d)", parity);
		queued_retval = ERROR_FAIL;
	} else if (value) {
		*value = data;
		queued_retval = ERROR_OK;
	}
}

static void bbg_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	int ack;
	int parity = parity_u32(value);
	uint32_t delay = 0;

	LOG_DEBUG("bbg_swd_write_reg");
	assert(!(cmd & SWD_CMD_RnW));
	assert(ap_delay_clk < 256);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip bbg_swd_write_reg because queued_retval=%d", queued_retval);
		return;
	}

	if (cmd & SWD_CMD_APnDP)
		delay = ap_delay_clk;

	cmd |= 0x81;
	pru_data_ram[0] = CMD_WRITE_REG | (cmd << 8) | (parity << 16) | (delay << 24);
	pru_data_ram[1] = value;
	pru_request_cmd(pru_data_ram);
	ack = pru_data_ram[BBG_SWS_RESULT] & 0x07;

	LOG_DEBUG("%s %s %s reg %X = %08" PRIx32,
		  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
		  cmd & SWD_CMD_APnDP ? "AP" : "DP",
		  cmd & SWD_CMD_RnW ? "read" : "write",
		  (cmd & SWD_CMD_A32) >> 1,
		  value);

	if (ack != SWD_ACK_OK)
		queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
}

static int bbg_swd_run_queue(void)
{
	int retval;

	LOG_DEBUG("bbg_swd_run_queue");
	bbg_swd_idle(8);
	retval = queued_retval;
	queued_retval = ERROR_OK;
	LOG_DEBUG("SWD queue return value: %02x", retval);
	return retval;
}

const struct swd_driver bbg_swd = {
	.init = bbg_swd_swd_init,
	.switch_seq = bbg_swd_switch_seq,
	.read_reg = bbg_swd_read_reg,
	.write_reg = bbg_swd_write_reg,
	.run = bbg_swd_run_queue,
};


static const char * const bbg_swd_transport[] = { "swd", NULL };


COMMAND_HANDLER(bbg_swd_handle_hello_command)
{
	puts("Hello!");
	return ERROR_OK;
}

static const struct command_registration bbg_swd_command_handlers[] = {
	{
		.name = "bbg_swd_hello",
		.handler = &bbg_swd_handle_hello_command,
		.mode = COMMAND_CONFIG,
		.help = "hello command for BBG-SWD",
		.usage = "<cmd>",
	},
	COMMAND_REGISTRATION_DONE
};

static void bbg_swd_execute_reset(struct jtag_command *cmd)
{
	bbg_swd_gpio_srst(cmd->cmd.reset->srst ? 0: 1);
}

static void bbg_swd_execute_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static void bbg_swd_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RESET:
			bbg_swd_execute_reset(cmd);
			break;
		case JTAG_SLEEP:
			bbg_swd_execute_sleep(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
	}
}

static int bbg_swd_interface_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		bbg_swd_execute_command(cmd);
		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int bbg_swd_interface_speed(int speed)
{
	return ERROR_OK;
}

static int bbg_swd_interface_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int bbg_swd_interface_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

struct jtag_interface bbg_swd_interface = {
	.name = "bbg-swd",
	.commands = bbg_swd_command_handlers,
	.swd = &bbg_swd,
	.transports = bbg_swd_transport,

	.execute_queue = bbg_swd_interface_execute_queue,
	.speed = bbg_swd_interface_speed,
	.speed_div = bbg_swd_interface_speed_div,
	.khz = bbg_swd_interface_khz,

	.init = bbg_swd_interface_init,
	.quit = bbg_swd_interface_quit,
};
/*
 * Local Variables:
 * c-file-style: "linux"
 * End:
 */
