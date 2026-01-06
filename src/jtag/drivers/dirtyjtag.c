/***************************************************************************
 *   Copyright (C) 2020 Jean THOMAS                                        *
 *   pub0@git.jeanthomas.me                                                *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


/* project specific includes */
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include <helper/bits.h>
#include "libusb_helper.h"


/**
 * USB settings
 */
static  unsigned int dirtyjtag_ep_write = 0x01;
static  unsigned int dirtyjtag_ep_read = 0x82;
#define DIRTYJTAG_USB_TIMEOUT LIBUSB_TIMEOUT_MS
static const uint16_t dirtyjtag_vid = 0x1209;
static const uint16_t dirtyjtag_pid = 0xC0CA;

enum dirtyJtagCmd
{
    CMD_STOP = 0x00,
    CMD_INFO = 0x01,
    CMD_FREQ = 0x02,
    CMD_XFER = 0x03,
    CMD_SETSIG = 0x04,
    CMD_GETSIG = 0x05,
    CMD_CLK = 0x06
};

/* Modifiers applicable only to DirtyJTAG2 */
enum CommandModifier
{
    EXTEND_LENGTH = 0x40,
    NO_READ = 0x80,
	READOUT = 0x80
};

struct version_specific
{
    uint8_t no_read;   // command modifer for xfer no read
    uint16_t max_bits; // max bit count that can be transferred
};

static struct version_specific dirtyjtag_v_options[3] = {{0, 240}, {0, 240}, {NO_READ, 496}};
static int dirtyjtag_version;
enum dirtyJtagSig
{
    SIG_TCK = (1 << 1),
    SIG_TDI = (1 << 2),
    SIG_TDO = (1 << 3),
    SIG_TMS = (1 << 4),
    SIG_TRST = (1 << 5),
    SIG_SRST = (1 << 6)
};

static struct libusb_device_handle *usb_handle;

/**
 * Utils

static int min(int a, int b)
{
	return (a < b) ? a : b;
}
*/
static unsigned char swap_bits(unsigned char x)
{
	const unsigned char lut[16] = {
		0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};

	return lut[x & 0xF] << 4 | lut[x >> 4];
}

/*
 * DirtyJTAG command buffer code
 */
#define DIRTYJTAG_BUFFER_SIZE 64
static const size_t dirtyjtag_buffer_size = DIRTYJTAG_BUFFER_SIZE;
static uint8_t dirtyjtag_buffer[DIRTYJTAG_BUFFER_SIZE];
static size_t dirtyjtag_buffer_use;

struct dirtyjtag_scan_info
{
	int bits_to_read;
	uint8_t *read_buffer;
	uint8_t *read_buffer_start;
	int bit_index;
	bool last_bit;
	struct scan_command *scan_cmd;
	struct dirtyjtag_scan_info *next;
};

static struct dirtyjtag_scan_info *dirtyjtag_scan_queue;
static struct dirtyjtag_scan_info *dirtyjtag_scan_tail;
static struct dirtyjtag_scan_info *dirtyjtag_scan_free_list;
/* allocate from the free list or heap alloc*/
static struct dirtyjtag_scan_info *allocate_scan_info()
{
	struct dirtyjtag_scan_info *scan_info;
	if (dirtyjtag_scan_free_list)
	{
		scan_info = dirtyjtag_scan_free_list;
		dirtyjtag_scan_free_list = dirtyjtag_scan_free_list->next;
	}
	else
	{
		scan_info = (struct dirtyjtag_scan_info *)malloc(sizeof(struct dirtyjtag_scan_info));
		if (scan_info == NULL)
		{
			LOG_ERROR("dirtyjtag: unable to allocate scan info");
			return NULL;
		}
	}		
	scan_info->next = NULL;
	if (dirtyjtag_scan_queue == NULL)
		dirtyjtag_scan_queue = scan_info;
	else
		dirtyjtag_scan_tail->next = scan_info;
	dirtyjtag_scan_tail = scan_info;
	return scan_info;
}

/* delete the head of the queue*/
static void release_scan_info_head(struct dirtyjtag_scan_info *scan_info)
{
	assert(dirtyjtag_scan_queue == scan_info);
	dirtyjtag_scan_queue = scan_info->next;
	if (dirtyjtag_scan_tail == scan_info)
		dirtyjtag_scan_tail = NULL;
	scan_info->next = dirtyjtag_scan_free_list;
	dirtyjtag_scan_free_list = scan_info;
}


static int dirtyjtag_buffer_flush(void)
{
	size_t sent = 0;
	int res = ERROR_OK;

	if (dirtyjtag_buffer_use == 0)
	{
		return res;
	}
	size_t to_send = dirtyjtag_buffer_use;
	if (dirtyjtag_buffer_use < dirtyjtag_buffer_size)
	{
    	dirtyjtag_buffer[dirtyjtag_buffer_use] = CMD_STOP;
		to_send += 1;
	}

    res = jtag_libusb_bulk_write(usb_handle, dirtyjtag_ep_write, (char *)dirtyjtag_buffer,
								 to_send, DIRTYJTAG_USB_TIMEOUT, (int*)&sent);
	assert(res == ERROR_OK);
	assert(sent == to_send);

	dirtyjtag_buffer_use = 0;
	res = ERROR_OK;
	/* process all pending scan read */
	struct dirtyjtag_scan_info *scan_info = dirtyjtag_scan_queue;
	while (scan_info)
	{
		struct dirtyjtag_scan_info *next = scan_info->next;
		if (scan_info->last_bit || (scan_info->bits_to_read > 0))
		{
			uint8_t tdo_buf[64];
			int read = 0;
			int bytes_to_read = (scan_info->bits_to_read + 7) / 8;
			res = jtag_libusb_bulk_read(usb_handle, dirtyjtag_ep_read,
										(char *)tdo_buf, bytes_to_read+!!scan_info->last_bit, DIRTYJTAG_USB_TIMEOUT, &read);
			assert(res == ERROR_OK);
			assert(read >= bytes_to_read);
			for (int i = 0; i < bytes_to_read; i++)
			{
				scan_info->read_buffer[i] = swap_bits(tdo_buf[i]);
			}
			scan_info->read_buffer += bytes_to_read;
			scan_info->bit_index += scan_info->bits_to_read;
			scan_info->bits_to_read = 0;
			if (scan_info->last_bit)
			{
				scan_info->read_buffer[-1] = (scan_info->read_buffer[-1] & ~(1 << (scan_info->bit_index % 8))) | (!!tdo_buf[bytes_to_read] << (scan_info->bit_index % 8));
				res = jtag_read_buffer(scan_info->read_buffer_start, scan_info->scan_cmd);
				if (res != ERROR_OK)
					res = ERROR_JTAG_QUEUE_FAILED;
				free(scan_info->read_buffer_start);
				scan_info->read_buffer = NULL;
				scan_info->read_buffer_start = NULL;
				scan_info->last_bit = false;
				scan_info->bit_index = 0;
				scan_info->scan_cmd = NULL;
				if (next == NULL)
				{
					release_scan_info_head(scan_info);
				}
			}
		}
		if (next)
			release_scan_info_head(scan_info);
		scan_info = next;
	}
	return res;
}

static struct dirtyjtag_scan_info* prepare_for_scan_input(uint8_t *_read_buffer, struct scan_command *cmd)
{
	struct dirtyjtag_scan_info *scan_info = allocate_scan_info();
	if (scan_info == NULL)
		return NULL;
	scan_info->bits_to_read = 0;
	scan_info->read_buffer_start = _read_buffer;
	scan_info->read_buffer = scan_info->read_buffer_start;
	scan_info->bit_index = 0;
	scan_info->last_bit = false;
	scan_info->scan_cmd = cmd;
	return scan_info;
}

static uint8_t* dirtyjtag_get_buffer_ptr_for_length(size_t length)
{
	assert(length <= dirtyjtag_buffer_size);
	if ((dirtyjtag_buffer_use + length) > dirtyjtag_buffer_size)
	{
		dirtyjtag_buffer_flush();
	}
	uint8_t* ptr = &dirtyjtag_buffer[dirtyjtag_buffer_use];
	dirtyjtag_buffer_use += length;
	return ptr;
}


static void dirtyjtag_buffer_append(const uint8_t *command, size_t length)
{
	assert(command != NULL);
	uint8_t* ptr = dirtyjtag_get_buffer_ptr_for_length(length);
	memcpy(ptr, command, length);
}

/**
 * Add one TCK/TMS/TDI sample to send buffer.
 */
static void dirtyjtag_write(int tck, int tms, int tdi)
{
	uint8_t command[] = {
		CMD_SETSIG,
		SIG_TCK | SIG_TMS | SIG_TDI,
		(tck ? SIG_TCK : 0) | (tms ? SIG_TMS : 0) | (tdi ? SIG_TDI : 0)};
	dirtyjtag_buffer_append(command, sizeof(command) / sizeof(command[0]));
}

/**
 * Read TDO pin
 */
static void dirtyjtag_get_last_bit(struct dirtyjtag_scan_info *scan_info)
{
	uint8_t command[] = {
		CMD_GETSIG};
	dirtyjtag_buffer_append(command, sizeof(command) / sizeof(command[0]));
	scan_info->last_bit = true;
}

/**
 * Send bunch of clock pulses
 */
static void dirtyjtag_clk(int num_cycles, int tms, int tdi)
{
	uint8_t command[] = {
		CMD_CLK,
		(tms ? SIG_TMS : 0) | (tdi ? SIG_TDI : 0),
		0};

	/*
	 * We can only do 255 clock pulses in one command, so we need
	 * to send multiple clock commands.
	 */
	while (num_cycles > 0)
	{
		command[2] = MIN(255, num_cycles);
		num_cycles -= MIN(255, num_cycles);
		dirtyjtag_buffer_append(command, sizeof(command) / sizeof(command[0]));
	}
}

/**
 * Control /TRST and /SYSRST pins.
 * Perform immediate bitbang transaction.
 */
static int dirtyjtag_reset(int trst, int srst)
{
	uint8_t command[] = {
		CMD_SETSIG,
		SIG_TRST | SIG_SRST,
		(trst ? 0 : SIG_TRST) | (srst ? 0 :SIG_SRST)};

	LOG_DEBUG("dirtyjtag_reset(%d,%d)", trst, srst);
	dirtyjtag_buffer_append(command, sizeof(command) / sizeof(command[0]));
	dirtyjtag_buffer_flush();
	return ERROR_OK;
}

static int dirtyjtag_speed(int divisor)
{
	uint8_t command[] = {
		CMD_FREQ,
		divisor >> 8,
		divisor};

	dirtyjtag_buffer_append(command, sizeof(command) / sizeof(command[0]));
	dirtyjtag_buffer_flush();
	return ERROR_OK;
}
static int dirtyjtag_getversion(void)
{
	int actual_length;
	int res;
	uint8_t buf[] = {CMD_INFO,
					 CMD_STOP};
	uint8_t rx_buf[64];
	res = jtag_libusb_bulk_write(usb_handle, dirtyjtag_ep_write, (char *)buf,
								 2, DIRTYJTAG_USB_TIMEOUT, &actual_length);
	assert(res == ERROR_OK);
	if (res)
	{
		LOG_ERROR("dirtyjtag_getversion: usb bulk write failed");
		return ERROR_JTAG_INIT_FAILED;
	}
	do
	{
		res = jtag_libusb_bulk_read(usb_handle, dirtyjtag_ep_read,
									(char *)rx_buf, 64, DIRTYJTAG_USB_TIMEOUT, &actual_length);
		if (res)
		{
			LOG_ERROR("dirtyjtag_getversion: usb bulk read failed");
			return ERROR_JTAG_INIT_FAILED;
		}
	} while (actual_length == 0);
	if (strncmp("DJTAG", (char *)rx_buf, 5))
	{
		LOG_INFO("dirtyJtag version unknown");
		dirtyjtag_version = 0;
		return ERROR_JTAG_INIT_FAILED;
	}
	int lversion = atoi((char *)&rx_buf[5]);
	dirtyjtag_version = (lversion >= 2) ? 2 : 1;
	LOG_INFO("dirtyjtag version %d", dirtyjtag_version);
	return ERROR_OK;
}
static int dirtyjtag_init(void)
{
	uint16_t avids[] = {dirtyjtag_vid, 0};
	uint16_t apids[] = {dirtyjtag_pid, 0};
	int res;
	if (jtag_libusb_open(avids, apids, NULL, &usb_handle, NULL)!= ERROR_OK)
	{
		LOG_ERROR("dirtyjtag not found: vid=%04x, pid=%04x\n",
				  dirtyjtag_vid, dirtyjtag_pid);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (jtag_libusb_choose_interface(usb_handle, 
	    &dirtyjtag_ep_read, &dirtyjtag_ep_write, -1, -1, 0, LIBUSB_TRANSFER_TYPE_BULK))
	{
		LOG_ERROR("unable to claim interface");
		return ERROR_JTAG_INIT_FAILED;
	}
	res = dirtyjtag_getversion();
	if (res != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;
	dirtyjtag_buffer_use = 0;
	return ERROR_OK;
}

static int dirtyjtag_quit(void)
{
	if (libusb_release_interface(usb_handle, 0) != 0)
	{
		LOG_ERROR("usb release interface failed");
	}

	jtag_libusb_close(usb_handle);
	/* delete all scan_info in free list */
	struct dirtyjtag_scan_info *scan_info = dirtyjtag_scan_free_list;
	while (scan_info)
	{
		struct dirtyjtag_scan_info *next = scan_info->next;
		free(scan_info);
		scan_info = next;
	}

	return ERROR_OK;
}

static int dirtyjtag_speed_div(int divisor, int *khz)
{
	*khz = divisor;
	return ERROR_OK;
}

static int dirtyjtag_khz(int khz, int *divisor)
{
	if (khz == 0)
	{
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*divisor = (khz < 16000) ? khz : 16000;
	*divisor = max(5, *divisor);
	return ERROR_OK;
}

static const struct command_registration dirtyjtag_command_handlers[] = {
	COMMAND_REGISTRATION_DONE};

static void syncbb_state_move(enum tap_state state, int skip)
{
	int i = 0, tms = 0, prev_tms = -1;
	int clk_count = 0;

	// don't do anything if we are already in the right state; but do execute always the TAP_RESET
	if (tap_get_state() == state && state != TAP_RESET)
		return;

	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), state);
	int tms_count = tap_get_tms_path_len(tap_get_state(), state);
	for (i = skip; i < tms_count; i++)
	{
		tms = (tms_scan >> i) & 1;
		if (tms != prev_tms && clk_count > 0)
		{
			dirtyjtag_clk(clk_count, prev_tms, 0);
			clk_count = 0;
		}
		clk_count++;
		prev_tms = tms;
	}
	if (clk_count > 0)
	{
		dirtyjtag_clk(clk_count, prev_tms, 0);
	}

	tap_set_state(state);
}

/**
 * Clock a bunch of TMS transitions, to change the JTAG
 * state machine.
 */
static int syncbb_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	int tms = 0, prev_tms = -1;
	int clk_count = 0;
	for (unsigned i = 0; i < num_bits; i++)
	{
		tms = ((bits[i / 8] >> (i % 8)) & 1);
		if (tms != prev_tms && clk_count > 0)
		{
			dirtyjtag_clk(clk_count, prev_tms, 0);
			clk_count = 0;
		}
		clk_count++;
		prev_tms = tms;
	}
	if (clk_count > 0)
	{
		dirtyjtag_clk(clk_count, prev_tms, 0);
	}

	return ERROR_OK;
}

static int syncbb_path_move(struct pathmove_command *cmd)
{
	unsigned int state_count;
	int tms = 0, prev_tms = -1;
	int clk_count = 0;
	for (state_count = 0; state_count < cmd->num_states; state_count++)
	{
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
		{
			tms = 0;
		}
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
		{
			tms = 1;
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					  tap_state_name(tap_get_state()),
					  tap_state_name(cmd->path[state_count]));
			return ERROR_JTAG_TRANSITION_INVALID;
		}

		if (tms != prev_tms && clk_count > 0)
		{
			dirtyjtag_clk(clk_count, prev_tms, 0);
			clk_count = 0;
		}
		clk_count++;
		prev_tms = tms;
		tap_set_state(cmd->path[state_count]);
	}

	if (clk_count > 0)
	{
		dirtyjtag_clk(clk_count, prev_tms, 0);
	}

	return ERROR_OK;
}

static void syncbb_runtest(int num_cycles, enum tap_state state)
{
	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		syncbb_state_move(TAP_IDLE, 0);
	}

	dirtyjtag_clk(num_cycles, 0, 0);

    /* finish in end_state */
	syncbb_state_move(state, 0);
}

/**
 * CMD_XFER:
 *   Read TDO
 *   Set TDI
 *   Set TCK high
 *	 Set TCK low
 *
 * Bitbang:
 *   Read TDO
 *   Set TDI, TMS, TCK low
 *   Set TCK high
 *
 */
static int syncbb_scan(struct scan_command *cmd)
{
	uint8_t *buffer = NULL;
	int retval = ERROR_OK;
	int scan_size = jtag_build_buffer(cmd, &buffer);
	enum scan_type type = jtag_scan_type(cmd);
	int sent_bits, i, sent_bytes;
	size_t buffer_pos = 0;
	uint8_t xfer_header[2] = {
							 CMD_XFER,
							 0};
	int pos_last_byte = (scan_size - 1) / 8;
	int pos_last_bit = (scan_size - 1) % 8;
	bool last_bit = !!(buffer[pos_last_byte] & (1 << pos_last_bit));
	struct dirtyjtag_scan_info *scan_info = NULL;

	assert(scan_size > 0);
	scan_size--;

	if (cmd->ir_scan)
	{
		syncbb_state_move(TAP_IRSHIFT, 0);
	}
	else
	{
		syncbb_state_move(TAP_DRSHIFT, 0);
	}

	if (type == SCAN_OUT)
		xfer_header[0] |= dirtyjtag_v_options[dirtyjtag_version].no_read;
	if (!(xfer_header[0] & dirtyjtag_v_options[dirtyjtag_version].no_read))
	{
		scan_info = prepare_for_scan_input(buffer, cmd);
		if (scan_info == NULL)
		{
			free(buffer);
			return ERROR_OUTOFMEMORY;
		}
	}

	while (scan_size > 0)
	{
		sent_bits = MIN(dirtyjtag_v_options[dirtyjtag_version].max_bits, scan_size);
		assert(!(sent_bits & 0x7) || (sent_bits == scan_size));
		sent_bytes = (sent_bits + 7) / 8;
		if (sent_bits > 255)
		{
			xfer_header[0] |= EXTEND_LENGTH;
			xfer_header[1] = sent_bits - 256;
		}
		else
		{
			xfer_header[0] &= ~EXTEND_LENGTH;
			xfer_header[1] = sent_bits;
		}
		uint8_t *buf_ptr = dirtyjtag_get_buffer_ptr_for_length(sent_bytes + 2);

		buf_ptr[0] = xfer_header[0];
		buf_ptr[1] = xfer_header[1];
		uint8_t *lbuffer = &buffer[buffer_pos];
		if (type != SCAN_IN)
		{
			/* Set TDI bits */
			for (i = 0; i < sent_bytes; i++)
			{
				buf_ptr[i + 2] = swap_bits(lbuffer[i]);
			}
		}
		else
		{
			/* Set TDI to 0 */
			memset(&buf_ptr[2], 0, sent_bytes);
		}

		if (!dirtyjtag_v_options[dirtyjtag_version].no_read || (type != SCAN_OUT))
		{
			scan_info->bits_to_read += sent_bits;
		}

		scan_size -= sent_bits;
		buffer_pos += sent_bytes;
	}
	/* last bit */
	if (dirtyjtag_version == 1)
	{
		dirtyjtag_write(0, 1, last_bit);
		dirtyjtag_write(1, 1, last_bit);
		if (type != SCAN_OUT)
		{
			dirtyjtag_get_last_bit(scan_info);
		}
		dirtyjtag_write(0, 1, last_bit);
	}
	else
	{
		uint8_t xfer_last_bit[] = {
			CMD_CLK | ((type != SCAN_OUT) ? READOUT : 0x00),
			(last_bit ? SIG_TDI : 0x00) | SIG_TMS,
			1
		};
		dirtyjtag_buffer_append(xfer_last_bit, sizeof(xfer_last_bit) / sizeof(xfer_last_bit[0]));
		if (type != SCAN_OUT)
		{
			scan_info->last_bit = true;
		}
	}

	/* we *KNOW* the above loop transitioned out of
	* the shift state, so we skip the first state
	* and move directly to the end state.
	*/
	syncbb_state_move(cmd->end_state, 1);
	return retval;
}

static int syncbb_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue; /* currently processed command */
	int retval;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	while (retval == ERROR_OK && cmd)
	{
		switch (cmd->type)
		{
		case JTAG_RESET:
			LOG_DEBUG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

			if ((cmd->cmd.reset->trst == 1) ||
				(cmd->cmd.reset->srst &&
				 (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
			{
				tap_set_state(TAP_RESET);
			}
			dirtyjtag_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;

		case JTAG_RUNTEST:
			LOG_DEBUG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles,
						 tap_state_name(cmd->cmd.runtest->end_state));

			syncbb_runtest(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
			break;

		case JTAG_STABLECLOCKS:
			dirtyjtag_clk(cmd->cmd.stableclocks->num_cycles,
						  ((tap_get_state() == TAP_RESET) ? SIG_TMS : 0), 0);
			break;

		case JTAG_TLR_RESET: /* renamed from JTAG_STATEMOVE */
			LOG_DEBUG_IO("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));

			syncbb_state_move(cmd->cmd.statemove->end_state, 0);
			break;

		case JTAG_PATHMOVE:
			LOG_DEBUG_IO("pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
						 tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));

			retval = syncbb_path_move(cmd->cmd.pathmove);
			break;

		case JTAG_SCAN:
			LOG_DEBUG_IO("%s scan end in %s", (cmd->cmd.scan->ir_scan) ? "IR" : "DR",
						 tap_state_name(cmd->cmd.scan->end_state));


			retval = syncbb_scan(cmd->cmd.scan);

			break;

		case JTAG_SLEEP:
			retval = dirtyjtag_buffer_flush();
			jtag_sleep(cmd->cmd.sleep->us);
			break;

        case JTAG_TMS:
			retval = syncbb_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
			retval = ERROR_FAIL;
		}
		cmd = cmd->next;
	}
	if (retval != ERROR_OK)
		return retval;
	return dirtyjtag_buffer_flush();
}

static struct jtag_interface dirtyjtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = syncbb_execute_queue,
};

struct adapter_driver dirtyjtag_adapter_driver = {
	.name = "dirtyjtag",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = dirtyjtag_command_handlers,

	.init = dirtyjtag_init,
	.quit = dirtyjtag_quit,
	.reset = dirtyjtag_reset,
	.speed = dirtyjtag_speed,
	.khz = dirtyjtag_khz,
	.speed_div = dirtyjtag_speed_div,

	.jtag_ops = &dirtyjtag_interface,
};
