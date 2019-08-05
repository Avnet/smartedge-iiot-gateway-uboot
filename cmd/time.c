// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2011 The Chromium OS Authors.
 */

#include <common.h>
#include <command.h>
#include <uuid.h>
#include <tpm-v2.h>
#include <uboot_aes.h>
#include <stdio.h>
#include <fs.h>

static void report_time(ulong cycles)
{
	ulong minutes, seconds, milliseconds;
	ulong total_seconds, remainder;

	total_seconds = cycles / CONFIG_SYS_HZ;
	remainder = cycles % CONFIG_SYS_HZ;
	minutes = total_seconds / 60;
	seconds = total_seconds % 60;
	/* approximate millisecond value */
	milliseconds = (remainder * 1000 + CONFIG_SYS_HZ / 2) / CONFIG_SYS_HZ;

	printf("\ntime:");
	if (minutes)
		printf(" %lu minutes,", minutes);
	printf(" %lu.%03lu seconds\n", seconds, milliseconds);
}

static int do_time(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong cycles = 0;
	int retval = 0;
	int repeatable = 0;
	unsigned int random = 0;
	// secret key part
	uint8_t ScratchArea[] =
	  { 0x72, 0x55, 0x49, 0x59, 0x14, 0x50, 0xDC, 0x2C, 0x9B, 0x59, 0xB7,
	    0xA2, 0x75, 0x0D, 0x73, 0x87};
	uint8_t RandomStr[AES_EXPAND_KEY_LENGTH];
	uint8_t NewRandomStr[AES_EXPAND_KEY_LENGTH];
	uint8_t OldRandomStr[AES_EXPAND_KEY_LENGTH];
	uint8_t OldRandomAES[AES_EXPAND_KEY_LENGTH];
	uint8_t key_exp[AES_EXPAND_KEY_LENGTH];
	uint8_t ArgV3[16];
	unsigned char GuidBin[16];
	int OldCnt = 0;
	int FileSize = 0;
	char FileSizeStr[16];
	memset(RandomStr, 0, sizeof(RandomStr));
	memset(OldRandomStr, 0, sizeof(RandomStr));
	memset(key_exp, 0, sizeof(key_exp));
	
	// Prep key
	aes_expand_key(ScratchArea, key_exp);

	// Get new random value.
	cycles = get_timer(0);
	report_time(cycles);
	srand((int)cycles);
	random = rand();
	gen_rand_uuid(&GuidBin);
	random = random + GuidBin[16] + (GuidBin[15] << 8) + (GuidBin[14] << 16) + (GuidBin[13] << 24);

	memset(RandomStr, 0 , sizeof(RandomStr));
	sprintf(RandomStr, "%X", random);

	aes_encrypt(RandomStr, key_exp, NewRandomStr);

	
	// change TPM2_RH_PLATFORM authorizations
	tpm2_change_auth(TPM2_RH_PLATFORM, NewRandomStr, strlen(NewRandomStr), NULL, 0);
	// For security clear out all working values 
	memset(NewRandomStr, 0, sizeof(NewRandomStr));
	random = 0;
	cycles = 0;
	memset(RandomStr, 0, sizeof(RandomStr));
	memset(OldRandomStr, 0, sizeof(RandomStr));
	memset(key_exp, 0, sizeof(key_exp));
	memset(GuidBin, 0 , sizeof(GuidBin));
      return retval;
}


U_BOOT_CMD(time, CONFIG_SYS_MAXARGS, 0, do_time,
		"run commands and summarize execution time",
		"command [args...]\n");
