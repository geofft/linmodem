/* 
 * V90 table generator
 * 
 * Copyright (c) 1999 Fabrice Bellard.
 *
 * This code is released under the GNU General Public License version
 * 2. Please read the file COPYING to know the exact terms of the
 * license.
 * 
 * This implementation is totally clean room. It was written by
 * reading the V90 specification and by using basic signal processing
 * knowledge.  
 */
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

/* from g711.c by SUN microsystems (unrestricted use) */

#define	SIGN_BIT	(0x80)		/* Sign bit for a A-law byte. */
#define	QUANT_MASK	(0xf)		/* Quantization field mask. */
#define	NSEGS		(8)		/* Number of A-law segments. */
#define	SEG_SHIFT	(4)		/* Left shift for segment number. */
#define	SEG_MASK	(0x70)		/* Segment field mask. */

#define	BIAS		(0x84)		/* Bias for linear code. */
/*
 * alaw2linear() - Convert an A-law value to 16-bit linear PCM
 *
 */
int
alaw2linear(a_val)
	unsigned char	a_val;
{
	int		t;
	int		seg;

	a_val ^= 0x55;

	t = (a_val & QUANT_MASK) << 4;
	seg = ((unsigned)a_val & SEG_MASK) >> SEG_SHIFT;
	switch (seg) {
	case 0:
		t += 8;
		break;
	case 1:
		t += 0x108;
		break;
	default:
		t += 0x108;
		t <<= seg - 1;
	}
	return ((a_val & SIGN_BIT) ? t : -t);
}

int
ulaw2linear(u_val)
	unsigned char	u_val;
{
	int		t;

	/* Complement to obtain normal u-law value. */
	u_val = ~u_val;

	/*
	 * Extract and bias the quantization bits. Then
	 * shift up by the segment number and subtract out the bias.
	 */
	t = ((u_val & QUANT_MASK) << 3) + BIAS;
	t <<= ((unsigned)u_val & SEG_MASK) >> SEG_SHIFT;

	return ((u_val & SIGN_BIT) ? (BIAS - t) : (t - BIAS));
}

int main(int argc, char **argv)
{
    int i;

    printf("/* THIS SOURCE CODE IS AUTOMATICALLY GENERATED - DO NOT MODIFY */\n");

    printf("/*\n"
           " * V90 tables\n"
           " * \n"
           " * Copyright (c) 1999 Fabrice Bellard.\n"
           " *\n"
           " * This code is released under the GNU General Public License version\n"
           " * 2. Please read the file COPYING to know the exact terms of the\n"
           " * license.\n"
           " */\n");

    printf("#include \"lm.h\"\n"
           "#include \"v90priv.h\"\n"
           "\n");
    
    printf("const s16 v90_ulaw_ucode_to_linear[128]= {\n");
    for(i=0;i<128;i++) {
        printf("%5d,", ulaw2linear(i ^ 0xff));
        if ((i & 7) == 7) printf("\n");
    }
    printf("};\n");

    printf("const s16 v90_alaw_ucode_to_linear[128]= {\n");
    for(i=0;i<128;i++) {
        printf("%5d,", alaw2linear(i ^ 0xd5));
        if ((i & 7) == 7) printf("\n");
    }
    printf("};\n");

    return 0;
}
