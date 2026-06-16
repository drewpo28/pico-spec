#include "Sftp.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "Debug.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <strings.h>

// SFTP v3 protocol constants (draft-ietf-secsh-filexfer-02).
enum {
    FXP_INIT = 1, FXP_VERSION = 2, FXP_OPEN = 3, FXP_CLOSE = 4, FXP_READ = 5,
    FXP_WRITE = 6, FXP_LSTAT = 7, FXP_FSTAT = 8, FXP_OPENDIR = 11, FXP_READDIR = 12,
    FXP_REMOVE = 13, FXP_MKDIR = 14, FXP_REALPATH = 16, FXP_STAT = 17,
    FXP_STATUS = 101, FXP_HANDLE = 102, FXP_DATA = 103, FXP_NAME = 104, FXP_ATTRS = 105,
};
enum { FXF_READ = 0x01, FXF_WRITE = 0x02, FXF_CREAT = 0x08, FXF_TRUNC = 0x10 };
enum { ATTR_SIZE = 0x01, ATTR_UIDGID = 0x02, ATTR_PERMISSIONS = 0x04, ATTR_ACMODTIME = 0x08,
       ATTR_EXTENDED = 0x80000000u };
enum { FX_OK = 0, FX_EOF = 1 };
#define S_IFDIR_MASK 0040000u
// Bytes per SFTP READ/WRITE. Kept modest: each READ reply is buffered in heap
// std::strings, and we run on a 12 KB alt-stack that leaves only ~20 KB heap —
// 8 KB chunks (×transient copies) overflowed it (OOM). 2 KB is plenty over a
// 115200-baud link and keeps per-op heap churn small.
static const uint32_t READ_CHUNK = 2048;

namespace {
struct Buf {
    std::string d;
    void u8(uint8_t v)   { d.push_back((char)v); }
    void u32(uint32_t v) { d.push_back((char)(v>>24)); d.push_back((char)(v>>16));
                           d.push_back((char)(v>>8)); d.push_back((char)v); }
    void u64(uint64_t v) { u32((uint32_t)(v>>32)); u32((uint32_t)v); }
    void raw(const void* p, size_t n) { d.append((const char*)p, n); }
    void str(const std::string& s) { u32((uint32_t)s.size()); raw(s.data(), s.size()); }
};
struct Reader {
    const uint8_t* p; size_t n, off;
    Reader(const std::string& s) : p((const uint8_t*)s.data()), n(s.size()), off(0) {}
    bool u32(uint32_t& v){ if (off+4>n) return false;
                           v=(p[off]<<24)|(p[off+1]<<16)|(p[off+2]<<8)|p[off+3]; off+=4; return true; }
    bool u64(uint64_t& v){ uint32_t hi,lo; if(!u32(hi)||!u32(lo)) return false; v=((uint64_t)hi<<32)|lo; return true; }
    bool str(std::string& s){ uint32_t l; if(!u32(l)) return false; if(off+l>n) return false;
                              s.assign((const char*)p+off,l); off+=l; return true; }
    // Parse an ATTRS structure, returning size & whether it's a directory.
    bool attrs(uint64_t& size, uint32_t& perms) {
        uint32_t f; size=0; perms=0;
        if (!u32(f)) return false;
        if (f & ATTR_SIZE)        { if(!u64(size)) return false; }
        if (f & ATTR_UIDGID)      { uint32_t a,b; if(!u32(a)||!u32(b)) return false; }
        if (f & ATTR_PERMISSIONS) { if(!u32(perms)) return false; }
        if (f & ATTR_ACMODTIME)   { uint32_t a,b; if(!u32(a)||!u32(b)) return false; }
        if (f & ATTR_EXTENDED)    { uint32_t c; if(!u32(c)) return false;
                                    for(uint32_t i=0;i<c;i++){ std::string s; if(!str(s)||!str(s)) return false; } }
        return true;
    }
};
} // namespace

Sftp::Sftp() : chan(-1), next_id(1), cur_dir("/"), up(false) {}
Sftp::~Sftp() { disconnect(); }

bool Sftp::readChan(uint8_t* buf, size_t need, uint32_t timeout_ms) {
    size_t got = 0;
    while (got < need) {
        int n = ssh.channelRecv(chan, buf + got, need - got, timeout_ms);
        if (n <= 0) return false;
        got += n;
    }
    return true;
}

bool Sftp::sendPacket(uint8_t type, const std::string& body) {
    Buf p; p.u32((uint32_t)(1 + body.size())); p.u8(type); p.raw(body.data(), body.size());
    return ssh.channelSend(chan, (const uint8_t*)p.d.data(), p.d.size()) == (int)p.d.size();
}

bool Sftp::recvPacket(uint8_t& out_type, std::string& out_body, uint32_t timeout_ms) {
    uint8_t hdr[5]; // length(4) + type(1)
    if (!readChan(hdr, 5, timeout_ms)) return false;
    uint32_t len = (hdr[0]<<24)|(hdr[1]<<16)|(hdr[2]<<8)|hdr[3];
    if (len < 1 || len > 65535 + 64) return false;
    out_type = hdr[4];
    // Read the body straight into out_body — no extra packet+substr copy (the
    // download path is heap-tight; see READ_CHUNK).
    out_body.resize(len - 1);
    if (len > 1 && !readChan((uint8_t*)out_body.data(), len - 1, timeout_ms)) return false;
    return true;
}

bool Sftp::connect(const char* host, uint16_t port, const char* user, const char* pass) {
    if (!ssh.connect(host, port, user, pass)) return false;
    chan = ssh.openSubsystem("sftp");
    if (chan < 0) { Debug::log("SFTP: openSubsystem failed"); return false; }

    // SSH_FXP_INIT(version=3).
    Buf init; init.u32(3);
    if (!sendPacket(FXP_INIT, init.d)) { Debug::log("SFTP: INIT send failed"); return false; }
    uint8_t t; std::string b;
    bool got = recvPacket(t, b);
    Debug::log("SFTP: INIT->VERSION got=%d type=%d len=%u", got, t, (unsigned)b.size());
    if (!got || t != FXP_VERSION) return false;
    up = true;

    // Resolve "." to an absolute home directory for the initial cwd.
    std::string home;
    bool rp = realpath(".", home);
    Debug::log("SFTP: realpath('.')=%d home=%s", rp, rp ? home.c_str() : "(none)");
    if (rp && !home.empty()) cur_dir = home;
    return true;
}

std::string Sftp::absPath(const std::string& name) const {
    if (!name.empty() && name[0] == '/') return name;
    std::string base = cur_dir;
    if (base.empty() || base.back() != '/') base += '/';
    return base + name;
}

bool Sftp::realpath(const std::string& path, std::string& resolved) {
    Buf req; req.u32(next_id++); req.str(path);
    if (!sendPacket(FXP_REALPATH, req.d)) return false;
    uint8_t t; std::string b;
    if (!recvPacket(t, b)) return false;
    if (t != FXP_NAME) return false;
    Reader r(b); uint32_t id, count;
    if (!r.u32(id) || !r.u32(count) || count < 1) return false;
    return r.str(resolved);
}

uint32_t Sftp::statSize(const std::string& path, bool& isDir, bool& ok) {
    ok = false; isDir = false;
    Buf req; req.u32(next_id++); req.str(path);
    if (!sendPacket(FXP_STAT, req.d)) return 0;
    uint8_t t; std::string b;
    if (!recvPacket(t, b) || t != FXP_ATTRS) return 0;
    Reader r(b); uint32_t sid; uint64_t sz; uint32_t perms;
    if (!r.u32(sid) || !r.attrs(sz, perms)) return 0; // FXP_ATTRS: u32 req-id, then ATTRS
    ok = true; isDir = (perms & S_IFDIR_MASK) != 0;
    return (uint32_t)sz;
}

bool Sftp::cwd(const std::string& path) {
    std::string target = absPath(path);
    std::string resolved;
    if (!realpath(target, resolved)) return false;
    bool isDir, ok;
    statSize(resolved, isDir, ok);
    if (!ok || !isDir) return false;
    cur_dir = resolved;
    return true;
}

bool Sftp::list(const std::string& path, std::vector<RemoteEntry>& out) {
    if (!up) return false;
    std::string dir = path.empty() ? cur_dir : absPath(path);

    Buf op; op.u32(next_id++); op.str(dir);
    if (!sendPacket(FXP_OPENDIR, op.d)) return false;
    uint8_t t; std::string b;
    if (!recvPacket(t, b) || t != FXP_HANDLE) return false;
    Reader hr(b); uint32_t hid; std::string handle;
    if (!hr.u32(hid) || !hr.str(handle)) return false; // FXP_HANDLE: u32 req-id, string handle

    for (;;) {
        Buf rd; rd.u32(next_id++); rd.str(handle);
        if (!sendPacket(FXP_READDIR, rd.d)) break;
        if (!recvPacket(t, b)) break;
        if (t == FXP_STATUS) break;       // SSH_FX_EOF → done
        if (t != FXP_NAME) break;
        Reader r(b); uint32_t id, count;
        if (!r.u32(id) || !r.u32(count)) break;
        for (uint32_t i = 0; i < count; i++) {
            std::string name, longname;
            uint64_t sz; uint32_t perms;
            if (!r.str(name) || !r.str(longname) || !r.attrs(sz, perms)) break;
            if (name == "." || name == "..") continue;
            RemoteEntry e; e.name = name; e.isDir = (perms & S_IFDIR_MASK) != 0; e.size = (uint32_t)sz;
            out.push_back(e);
        }
    }
    Buf cl; cl.u32(next_id++); cl.str(handle);
    sendPacket(FXP_CLOSE, cl.d);
    recvPacket(t, b); // status
    sortRemoteEntries(out);
    return true;
}

bool Sftp::get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) {
    if (!up) return false;
    std::string rpath = absPath(remote);
    bool isDir, sok; uint32_t total = statSize(rpath, isDir, sok);

    Buf op; op.u32(next_id++); op.str(rpath); op.u32(FXF_READ); op.u32(0); // attrs flags=0
    if (!sendPacket(FXP_OPEN, op.d)) return false;
    uint8_t t; std::string b;
    if (!recvPacket(t, b) || t != FXP_HANDLE) return false;
    Reader hr(b); uint32_t hid; std::string handle;
    if (!hr.u32(hid) || !hr.str(handle)) return false; // FXP_HANDLE: u32 req-id, string handle

    FIL* f = fopen2(localSdPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) { Buf cl; cl.u32(next_id++); cl.str(handle); sendPacket(FXP_CLOSE, cl.d); recvPacket(t,b); return false; }

    uint64_t off = 0; bool ok = true;
    for (;;) {
        Buf rd; rd.u32(next_id++); rd.str(handle); rd.u64(off); rd.u32(READ_CHUNK);
        if (!sendPacket(FXP_READ, rd.d)) { ok = false; break; }
        if (!recvPacket(t, b)) { ok = false; break; }
        if (t == FXP_STATUS) break; // EOF
        if (t != FXP_DATA) { ok = false; break; }
        // FXP_DATA body = u32 req-id, string data. Write straight out of `b` (no
        // extra copy): data length is at offset 4, payload at offset 8.
        if (b.size() < 8) { ok = false; break; }
        uint32_t dlen = ((uint8_t)b[4]<<24)|((uint8_t)b[5]<<16)|((uint8_t)b[6]<<8)|(uint8_t)b[7];
        if (dlen > b.size() - 8) dlen = (uint32_t)(b.size() - 8);
        UINT bw;
        if (f_write(f, b.data() + 8, dlen, &bw) != FR_OK || bw != dlen) { ok = false; break; }
        off += dlen;
        if (cb && !cb((uint32_t)off, total)) { ok = false; break; }
        if (dlen < READ_CHUNK) break; // short read → EOF
    }
    fclose2(f);
    Buf cl; cl.u32(next_id++); cl.str(handle); sendPacket(FXP_CLOSE, cl.d); recvPacket(t, b);
    return ok;
}

bool Sftp::put(const std::string& localSdPath, const std::string& remote, XferProgressCb cb) {
    if (!up) return false;
    FIL* f = fopen2(localSdPath.c_str(), FA_READ);
    if (!f) return false;
    uint32_t total = f_size(f);
    std::string rpath = absPath(remote);

    Buf op; op.u32(next_id++); op.str(rpath); op.u32(FXF_WRITE | FXF_CREAT | FXF_TRUNC); op.u32(0);
    if (!sendPacket(FXP_OPEN, op.d)) { fclose2(f); return false; }
    uint8_t t; std::string b;
    if (!recvPacket(t, b) || t != FXP_HANDLE) { fclose2(f); return false; }
    Reader hr(b); uint32_t hid; std::string handle;
    if (!hr.u32(hid) || !hr.str(handle)) { fclose2(f); return false; } // skip req-id

    uint64_t off = 0; bool ok = true;
    // Static (not stack): READ_CHUNK is 8 KB but PICO_STACK_SIZE is only 4 KB.
    // Safe because the OSD calls this single-threaded and non-reentrantly.
    static uint8_t chunk[READ_CHUNK];
    for (;;) {
        UINT br;
        if (f_read(f, chunk, sizeof(chunk), &br) != FR_OK) { ok = false; break; }
        if (br == 0) break;
        Buf wr; wr.u32(next_id++); wr.str(handle); wr.u64(off);
        wr.u32(br); wr.raw(chunk, br);
        if (!sendPacket(FXP_WRITE, wr.d)) { ok = false; break; }
        if (!recvPacket(t, b) || t != FXP_STATUS) { ok = false; break; }
        Reader r(b); uint32_t id, code;
        if (!r.u32(id) || !r.u32(code) || code != FX_OK) { ok = false; break; }
        off += br;
        if (cb && !cb((uint32_t)off, total)) { ok = false; break; }
    }
    fclose2(f);
    Buf cl; cl.u32(next_id++); cl.str(handle); sendPacket(FXP_CLOSE, cl.d); recvPacket(t, b);
    return ok;
}

void Sftp::disconnect() {
    if (up && chan >= 0) ssh.channelClose(chan);
    ssh.disconnect();
    up = false; chan = -1;
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
