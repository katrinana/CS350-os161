/*
 * Copyright (c) 2000, 2001, 2002, 2003, 2004, 2005, 2008, 2009
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * sbrk
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <err.h>

#include "config.h"
#include "test.h"

static
int
try_sbrk(long val)
{
	void *rv;
	rv = sbrk(val);
	if (rv==(void *)-1) {
		return errno;
	}
	return 0;
}

static
void
enforce_sbrk(long val, const char *desc, int err)
{
	int e;

	e = try_sbrk(val);
	if (e != err && e==0) {
		warnx("FAILURE: sbrk(%s): no error", desc);
		return;
	}
	if (e != err) {
		errno = e;
		warn("FAILURE: sbrk(%s): wrong error", desc);
		return;
	}
	warnx("passed: sbrk(%s)", desc);
}

static
void
sbrk_bigpos(void)
{
	enforce_sbrk(4096*1024*256, "huge positive", ENOMEM);
}

static
void
sbrk_bigneg(void)
{
	enforce_sbrk(-4096*1024*256, "huge negative", EINVAL);
}

static
void
sbrk_neg(void)
{
	enforce_sbrk(-8192, "too-large negative", EINVAL);
}

static
void
sbrk_unalignedpos(void)
{
	switch (try_sbrk(17)) {
	    case 0:
	    case EINVAL:
		warnx("passed: sbrk(unaligned positive)");
		break;
	    default:
		warn("FAILURE: sbrk(unaligned positive): wrong error");
		break;
	}
}

static
void
sbrk_unalignedneg(void)
{
	switch (try_sbrk(-17)) {
	    case 0:
	    case EINVAL:
		warnx("passed: sbrk(unaligned negative)");
		break;
	    default:
		warn("FAILURE: sbrk(unaligned negative): wrong error");
		break;
	}
}

void
test_sbrk(void)
{
	sbrk_neg();
	sbrk_bigpos();
	sbrk_bigneg();
	sbrk_unalignedpos();
	sbrk_unalignedneg();
}

