/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#ifdef CONFIG_S3C24X0

#include <asm/io.h>
#include <asm/arch/s3c24x0_cpu.h>
#include <div64.h>

DECLARE_GLOBAL_DATA_PTR;

#define TIMER_PREDIV_2 0
#define TIMER_PREDIV_4 1
#define TIMER_PREDIV_8 2
#define TIMER_PREDIV_16 3

/*
 * Timer is 16-bit
 * So we can have good (small) timer resolution OR good (big) overlap time
 * PCLK usualy 66 MHz
 * PCLK / 2 / 33 = 1 MHz, 1 uS @ tick, overlap after 65 mS
 * PCLK / 4 / 165 = 0.1 MHz ,10 uS @ tick, overlap after 650 mS
 * PCLK / 16 / 165 = 0.025 MHz, 40 uS @ tick, overlap after 2,5 S
 * ...etc
 */

#define TIMER_PRESCALER 165
#define TIMER_PREDIV TIMER_PREDIV_4 //2, 4, 8, 16

static inline unsigned long long tick_to_time(unsigned long long tick)
{
	tick *= CONFIG_SYS_HZ;
	do_div(tick, gd->timer_rate_hz);

	return tick;
}

static inline unsigned long long usec_to_tick(unsigned long long usec)
{
	usec *= gd->timer_rate_hz;
	do_div(usec, 1000000);

	return usec;
}

int timer_init(void)
{
	struct s3c24x0_timers *timers = s3c24x0_get_base_timers();
	ulong tmr;

	/* use PWM Timer 4 because it has no output */
	/* prescaler for Timer */
	writel(((TIMER_PRESCALER - 1) << 8) | (TIMER_PRESCALER - 1), &timers->tcfg0);
	/* PREDIV */
	writel((readl(&timers->tcfg1) & ~(0x0F << 16)) | (TIMER_PREDIV << 16), &timers->tcfg1);
	/* Reload value */
	writel(0xFFFF, &timers->tcntb4);
	/* auto load, manual update of Timer 4 */
	tmr = (readl(&timers->tcon) & ~0x0700000) | 0x0600000;
	writel(tmr, &timers->tcon);
	/* auto load, start Timer 4 */
	tmr = (tmr & ~0x0700000) | 0x0500000;
	writel(tmr, &timers->tcon);

	gd->lastinc = 0;
	gd->timer_rate_hz = get_PCLK() / (TIMER_PRESCALER * (1 << (TIMER_PREDIV + 1)));
	gd->tbu = gd->tbl = 0;

	return (0);
}

/*
 * macro to read the count-down 16 bit timer
 */
static inline ulong READ_TIMER16(void)
{
	struct s3c24x0_timers *timers = s3c24x0_get_base_timers();

	return (0xFFFF - ((readl(&timers->tcnto4) & 0xFFFF)));
}

static inline ulong READ_TIMER32(void)
{
	ulong now = READ_TIMER16();
	ulong tbl = gd->tbl;

	if (now >= (tbl & 0xFFFF))
		tbl = (tbl & 0xFFFF0000) | now;
	else
		tbl = ((tbl & 0xFFFF0000) | now) + 0x00010000;

	return tbl;
}

void udelay_masked(unsigned long usec)
{
	ulong tmo;
	ulong endtime;
	signed long diff;

	tmo = usec_to_tick(usec); /* convert usecs to ticks */

	endtime = get_ticks() + tmo;

	do {
			ulong now = get_ticks();
			diff = endtime - now;
	} while (diff >= 0);
}

/*
 * Get the current 64 bit timer tick count
 */
unsigned long long get_ticks(void)
{
	ulong now = READ_TIMER32();

	/* increment tbu if tbl has rolled over */
	if (now < gd->tbl)
		gd->tbu++;
	gd->tbl = now;
	return (((unsigned long long)gd->tbu) << 32) | gd->tbl;
}

void __udelay(unsigned long usec)
{
	unsigned long long start;
	unsigned long long tmo;

	start = get_ticks(); /* get current timestamp */
	tmo = usec_to_tick(usec); /* convert usecs to ticks */
	if (tmo == 0)
		tmo = 1;
	while ((get_ticks() - start) < tmo)
		; /* loop till time has passed */
}

/*
 * get_timer(base) can be used to check for timeouts or
 * to measure elasped time relative to an event:
 *
 * ulong start_time = get_timer(0) sets start_time to the current
 * time value.
 * get_timer(start_time) returns the time elapsed since then.
 *
 * The time is used in CONFIG_SYS_HZ units!
 */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

ulong get_timer_masked (void)
{
	return tick_to_time(get_ticks());
}

/*
 * Return the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return gd->timer_rate_hz;
}

void reset_cpu(ulong ignored)
{
	struct s3c24x0_watchdog *watchdog;

	watchdog = s3c24x0_get_base_watchdog();

	/* Disable watchdog */
	writel(0x0000, &watchdog->wtcon);

	/* Initialize watchdog timer count register */
	writel(0x0001, &watchdog->wtcnt);

	/* Enable watchdog timer; assert reset at timer timeout */
	writel(0x0021, &watchdog->wtcon);

	while (1)
		/* loop forever and wait for reset to happen */;

	/*NOTREACHED*/
}

#endif /* CONFIG_S3C24X0 */
