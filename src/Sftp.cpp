#include "Sftp.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "Debug.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>

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
static const uint32_t READ_CHUNK = 8192; // bytes per SFTP READ/WRITE

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
    uint8_t lenb[4];
    if (!readChan(lenb, 4, timeout_ms)) return false;
    uint32_t len = (lenb[0]<<24)|(lenb[1]<<16)|(lenb[2]<<8)|lenb[3];
    if (len < 1 || len > 65535 + 64) return false;
    std::string pkt; pkt.resize(len);
    if (!readChan((uint8_t*)pkt.data(), len, timeout_ms)) return false;
    out_type = (uint8_t)pkt[0];
    out_body = pkt.substr(1);
    return true;
}

bool Sftp::connect(const char* host, uint16_t port, const char* user, const char* pass) {
    if (!ssh.connect(host, port, user, pass)) return false;
    chan = ssh.openSubsystem("sftp");
    if (chan < 0) return false;

    // SSH_FXP_INIT(version=3).
    Buf init; init.u32(3);
    if (!sendPacket(FXP_INIT, init.d)) return false;
    uint8_t t; std::string b;
    if (!recvPacket(t, b) || t != FXP_VERSION) return false;
    up = true;

    // Resolve "." to an absolute home directory for the initial cwd.
    std::string home;
    if (realpath(".", home) && !home.empty()) cur_dir = home;
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
    Reader r(b); uint64_t sz; uint32_t perms;
    if (!r.attrs(sz, perms)) return 0;
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
    Reader hr(b); std::string handle;
    if (!hr.str(handle)) return false;

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
    Reader hr(b); std::string handle; if (!hr.str(handle)) return false;

    FIL* f = fopen2(localSdPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) { Buf cl; cl.u32(next_id++); cl.str(handle); sendPacket(FXP_CLOSE, cl.d); recvPacket(t,b); return false; }

    uint64_t off = 0; bool ok = true;
    for (;;) {
        Buf rd; rd.u32(next_id++); rd.str(handle); rd.u64(off); rd.u32(READ_CHUNK);
        if (!sendPacket(FXP_READ, rd.d)) { ok = false; break; }
        if (!recvPacket(t, b)) { ok = false; break; }
        if (t == FXP_STATUS) break; // EOF
        if (t != FXP_DATA) { ok = false; break; }
        Reader r(b); uint32_t id; std::string data;
        if (!r.u32(id) || !r.str(data)) { ok = false; break; }
        UINT bw;
        if (f_write(f, data.data(), data.size(), &bw) != FR_OK || bw != data.size()) { ok = false; break; }
        off += data.size();
        if (cb && !cb((uint32_t)off, total)) { ok = false; break; }
        if (data.size() < READ_CHUNK) break; // short read → EOF
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
    Reader hr(b); std::string handle; if (!hr.str(handle)) { fclose2(f); return false; }

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
