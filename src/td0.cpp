// TD0 (Teledisk) decoder — see td0.h.
//
// LZH unpacker and sector decoders ported from UnrealSpeccy wldr_td0.cpp.
// The LZH algorithm is the classic LZHUF (Haruyasu Yoshizaki / Haruhiko Okumura)
// adaptive-Huffman + LZSS used by Teledisk's "advanced compression".

#include "td0.h"
#include <string.h>

// ---------------------------------------------------------------- sectors

bool td0_decode_sector(const unsigned char *encData, unsigned encLen, unsigned secSize, unsigned char *out)
{
    if (encLen == 0)
        return false;

    memset(out, 0, secSize);

    const unsigned char *src = encData;
    const unsigned char *end = encData + encLen;
    unsigned char method = *src++;

    switch (method) {
        case 0: { // raw sector data
            unsigned n = (unsigned)(end - src);
            if (n > secSize) n = secSize;
            memcpy(out, src, n);
            return true;
        }
        case 1: { // repeated 2-byte pattern: count(2) + value(2)
            if (src + 4 > end) return false;
            unsigned n = src[0] | (src[1] << 8);
            unsigned short val = src[2] | (src[3] << 8);
            for (unsigned i = 0; i < n && (2 * i + 1) < secSize; i++) {
                out[2 * i]     = (unsigned char)(val & 0xFF);
                out[2 * i + 1] = (unsigned char)(val >> 8);
            }
            return true;
        }
        case 2: { // RLE block: stream of (literal | run) sub-blocks
            unsigned char *dst = out;
            unsigned char *dstEnd = out + secSize;
            while (src < end && dst < dstEnd) {
                unsigned char op = *src++;
                if (op == 0) {            // literal run: len(1) + bytes
                    if (src >= end) return false;
                    unsigned char s = *src++;
                    while (s-- && src < end && dst < dstEnd)
                        *dst++ = *src++;
                } else if (op == 1) {     // repeated fragment: count(1) + value(2)
                    if (src + 3 > end) return false;
                    unsigned char s = *src++;
                    unsigned short val = src[0] | (src[1] << 8);
                    src += 2;
                    while (s-- && (dst + 1) < dstEnd) {
                        *dst++ = (unsigned char)(val & 0xFF);
                        *dst++ = (unsigned char)(val >> 8);
                    }
                } else {
                    return false;         // malformed
                }
            }
            return true;
        }
        default:
            return false;
    }
}

// ------------------------------------------------------------- LZH unpack

namespace {

const unsigned char d_code[256] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
        0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
        0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
        0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
        0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
        0x0C, 0x0C, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0D,
        0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x0F, 0x0F,
        0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11,
        0x12, 0x12, 0x12, 0x12, 0x13, 0x13, 0x13, 0x13,
        0x14, 0x14, 0x14, 0x14, 0x15, 0x15, 0x15, 0x15,
        0x16, 0x16, 0x16, 0x16, 0x17, 0x17, 0x17, 0x17,
        0x18, 0x18, 0x19, 0x19, 0x1A, 0x1A, 0x1B, 0x1B,
        0x1C, 0x1C, 0x1D, 0x1D, 0x1E, 0x1E, 0x1F, 0x1F,
        0x20, 0x20, 0x21, 0x21, 0x22, 0x22, 0x23, 0x23,
        0x24, 0x24, 0x25, 0x25, 0x26, 0x26, 0x27, 0x27,
        0x28, 0x28, 0x29, 0x29, 0x2A, 0x2A, 0x2B, 0x2B,
        0x2C, 0x2C, 0x2D, 0x2D, 0x2E, 0x2E, 0x2F, 0x2F,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
};

const unsigned char d_len[256] = {
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
        0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
        0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
};

const int N = 4096;     // ring-buffer size
const int F = 60;       // lookahead buffer size
const int THRESHOLD = 2;

const int N_CHAR = (256 - THRESHOLD + F); // kinds of characters
const int T = (N_CHAR * 2 - 1);           // table size
const int R = (T - 1);                    // root position
const int MAX_FREQ = 0x8000;              // tree-rebuild threshold

// Decoder state. Bundled in a struct so all of it is constructed per call
// (no leftover state across two TD0 inserts) but lives on the stack only as
// pointers — the big arrays are static to keep the stack small.
struct LzhState {
    const unsigned char *packed_ptr;
    const unsigned char *packed_end;

    unsigned short freq[T + 1];
    short prnt[T + N_CHAR];
    short son[T];
    unsigned char text_buf[N + F - 1];

    int r;
    unsigned getbuf;
    unsigned char getlen;
};

static LzhState g_lzh; // ~10 KB; static keeps it off the stack (RP2350 SRAM ok)

int readChar(LzhState &st)
{
    if (st.packed_ptr < st.packed_end) return *st.packed_ptr++;
    return -1;
}

int GetBit(LzhState &st)
{
    int i;
    while (st.getlen <= 8) {
        if ((i = readChar(st)) == -1) i = 0;
        st.getbuf |= i << (8 - st.getlen);
        st.getlen += 8;
    }
    i = st.getbuf;
    st.getbuf <<= 1;
    st.getlen--;
    return (i >> 15) & 1;
}

int GetByte(LzhState &st)
{
    unsigned i;
    while (st.getlen <= 8) {
        if ((int)(i = readChar(st)) == -1) i = 0;
        st.getbuf |= i << (8 - st.getlen);
        st.getlen += 8;
    }
    i = st.getbuf;
    st.getbuf <<= 8;
    st.getlen -= 8;
    return (i >> 8) & 0xFF;
}

void StartHuff(LzhState &st)
{
    int i, j;
    st.getbuf = 0; st.getlen = 0;
    for (i = 0; i < N_CHAR; i++) {
        st.freq[i] = 1;
        st.son[i] = i + T;
        st.prnt[i + T] = i;
    }
    i = 0; j = N_CHAR;
    while (j <= R) {
        st.freq[j] = st.freq[i] + st.freq[i + 1];
        st.son[j] = i;
        st.prnt[i] = st.prnt[i + 1] = j;
        i += 2; j++;
    }
    st.freq[T] = 0xffff;
    st.prnt[R] = 0;
    for (i = 0; i < N - F; i++) st.text_buf[i] = ' ';
    st.r = N - F;
}

void reconst(LzhState &st)
{
    int i, j, k, f, l;
    j = 0;
    for (i = 0; i < T; i++) {
        if (st.son[i] >= T) {
            st.freq[j] = (st.freq[i] + 1) / 2;
            st.son[j] = st.son[i];
            j++;
        }
    }
    for (i = 0, j = N_CHAR; j < T; i += 2, j++) {
        k = i + 1;
        f = st.freq[j] = st.freq[i] + st.freq[k];
        for (k = j - 1; f < st.freq[k]; k--);
        k++;
        l = (j - k) * (int)sizeof(st.freq[0]);
        memmove(&st.freq[k + 1], &st.freq[k], l);
        st.freq[k] = (unsigned short)f;
        memmove(&st.son[k + 1], &st.son[k], l);
        st.son[k] = (short)i;
    }
    for (i = 0; i < T; i++)
        if ((k = st.son[i]) >= T) st.prnt[k] = i;
        else st.prnt[k] = st.prnt[k + 1] = i;
}

void update(LzhState &st, int c)
{
    int i, j, k, l;
    if (st.freq[R] == MAX_FREQ) reconst(st);
    c = st.prnt[c + T];
    do {
        k = ++st.freq[c];
        if (k > st.freq[l = c + 1]) {
            while (k > st.freq[++l]);
            l--;
            st.freq[c] = st.freq[l];
            st.freq[l] = (unsigned short)k;

            i = st.son[c];
            st.prnt[i] = l;
            if (i < T) st.prnt[i + 1] = l;

            j = st.son[l];
            st.son[l] = (short)i;

            st.prnt[j] = c;
            if (j < T) st.prnt[j + 1] = c;
            st.son[c] = (short)j;

            c = l;
        }
    } while ((c = st.prnt[c]) != 0);
}

int DecodeChar(LzhState &st)
{
    int c = st.son[R];
    while (c < T) c = st.son[c + GetBit(st)];
    c -= T;
    update(st, c);
    return c;
}

int DecodePosition(LzhState &st)
{
    int i, j, c;
    i = GetByte(st);
    c = (int)d_code[i] << 6;
    j = d_len[i];
    j -= 2;
    while (j--) i = (i << 1) + GetBit(st);
    return c | (i & 0x3f);
}

} // namespace

unsigned td0_unpack_lzh(const unsigned char *src, unsigned size, unsigned char *dst, unsigned dstCapacity)
{
    LzhState &st = g_lzh;
    st.packed_ptr = src;
    st.packed_end = src + size;

    int i, j, k, c;
    unsigned count = 0;
    StartHuff(st);

    while (st.packed_ptr < st.packed_end && count < dstCapacity) {
        c = DecodeChar(st);
        if (c < 256) {
            dst[count++] = (unsigned char)c;
            st.text_buf[st.r++] = (unsigned char)c;
            st.r &= (N - 1);
        } else {
            i = (st.r - DecodePosition(st) - 1) & (N - 1);
            j = c - 255 + THRESHOLD;
            for (k = 0; k < j && count < dstCapacity; k++) {
                c = st.text_buf[(i + k) & (N - 1)];
                dst[count++] = (unsigned char)c;
                st.text_buf[st.r++] = (unsigned char)c;
                st.r &= (N - 1);
            }
        }
    }
    return count;
}
