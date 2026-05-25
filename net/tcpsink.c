/*
 * tcpsink -- a deterministic in-emulator TCP sink netdev backend.
 *
 * It runs on the GUEST's virtual clock: every guest TX frame is handled synchronously in
 * .receive and the reply is queued back to the guest NIC immediately. There is no second VM
 * and no real-time host peer, so there is no clock drift, no spurious RTO, and no frame loss
 * from flow-control mismatch -- the failures that make slirp and a two-VM socket rig unusable
 * for TCP throughput. It answers ARP, completes a passive open advertising a CONFIGURABLE MSS
 * (mss=, default 1460) and a large window, cumulatively ACKs all received data, and answers
 * FIN. The configurable MSS is what slirp can't do (slirp caps at 1460), enabling FDDI-sized
 * MSS experiments. It is a SINK only (receives + ACKs; sends no payload).
 *
 * Address defaults: IP 10.0.0.2, MAC 52:54:00:00:00:fe. The peer (guest) is learned from ARP.
 */
#include "qemu/osdep.h"
#include "net/net.h"
#include "qemu/main-loop.h"
#include "clients.h"
#include "qapi/error.h"

#define ETH_P_IP   0x0800
#define ETH_P_ARP  0x0806
#define TF_FIN 0x01
#define TF_SYN 0x02
#define TF_RST 0x04
#define TF_ACK 0x10
#define SINK_WIN 65535
#define SINK_ISS 0x1000

#define SINK_TXQ   64          /* pending-reply ring; replies are tiny (pure ACK/SYN-ACK/FIN) */
#define SINK_FRMAX 64

typedef struct TcpSinkState {
    NetClientState nc;
    uint16_t mss;
    uint8_t  mac[6];
    uint8_t  ip[4];
    /* single connection */
    uint8_t  peer_mac[6];
    uint8_t  peer_ip[4];
    uint16_t peer_port;
    uint16_t local_port;
    uint32_t rcv_nxt;
    uint32_t snd_nxt;
    int      open;
    uint64_t rx_bytes;
    /* deferred-reply ring: handlers run inside the el3's TX (re-entrant), so the el3 can't accept
     * a reply synchronously. Queue replies here and flush from a BH (main loop, with el3 idle). */
    uint8_t  txq[SINK_TXQ][SINK_FRMAX];
    int      txlen[SINK_TXQ];
    int      txhead, txtail, txcount;
    QEMUBH  *bh;
} TcpSinkState;

static void sink_emit(TcpSinkState *s, const uint8_t *f, int len)
{
    if (len > SINK_FRMAX || s->txcount >= SINK_TXQ) {
        return;
    }
    memcpy(s->txq[s->txhead], f, len);
    s->txlen[s->txhead] = len;
    s->txhead = (s->txhead + 1) % SINK_TXQ;
    s->txcount++;
    qemu_bh_schedule(s->bh);
}

static void sink_bh(void *opaque)
{
    TcpSinkState *s = opaque;
    while (s->txcount > 0) {
        qemu_send_packet(&s->nc, s->txq[s->txtail], s->txlen[s->txtail]);
        s->txtail = (s->txtail + 1) % SINK_TXQ;
        s->txcount--;
    }
}

static inline uint16_t rd16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static inline uint32_t rd32(const uint8_t *p)
{ return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline void wr32(uint8_t *p, uint32_t v)
{ p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

/* one's-complement sum over len bytes, folded; seed carries the TCP pseudo-header sum */
static uint16_t cksum16(const uint8_t *p, int len, uint32_t sum)
{
    int i;
    for (i = 0; i + 1 < len; i += 2) sum += rd16(p + i);
    if (i < len) sum += (uint16_t)p[i] << 8;
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return ~sum & 0xFFFF;
}

static void handle_arp(TcpSinkState *s, const uint8_t *buf, size_t size)
{
    const uint8_t *a = buf + 14;
    uint8_t f[60], *r;                                   /* padded to the 60-byte Ethernet minimum */
    if (size < 42 || rd16(a + 6) != 1) return;          /* need an ARP request */
    if (memcmp(a + 24, s->ip, 4) != 0) return;          /* not asking for our IP */
    memcpy(s->peer_mac, a + 8, 6);                       /* learn requester (sha/spa) */
    memcpy(s->peer_ip, a + 14, 4);

    memset(f, 0, sizeof f);
    memcpy(f + 0, s->peer_mac, 6);
    memcpy(f + 6, s->mac, 6);
    wr16(f + 12, ETH_P_ARP);
    r = f + 14;
    wr16(r + 0, 1); wr16(r + 2, ETH_P_IP); r[4] = 6; r[5] = 4; wr16(r + 6, 2);
    memcpy(r + 8, s->mac, 6);     memcpy(r + 14, s->ip, 4);
    memcpy(r + 18, s->peer_mac, 6); memcpy(r + 24, s->peer_ip, 4);
    sink_emit(s, f, 60);                                 /* pad: el3 drops sub-60-byte runts */
}

/* emit a pure (no-payload) segment: eth + ip(20) + tcp(20, or 24 with the MSS option on SYN) */
static void tcp_out(TcpSinkState *s, uint8_t flags, int with_mss)
{
    uint8_t f[60], *ip, *tcp;
    int tcplen = with_mss ? 24 : 20;
    int iplen  = 20 + tcplen;
    uint32_t sum;

    memset(f, 0, sizeof f);
    ip = f + 14; tcp = f + 34;
    memcpy(f + 0, s->peer_mac, 6);
    memcpy(f + 6, s->mac, 6);
    wr16(f + 12, ETH_P_IP);

    ip[0] = 0x45;
    wr16(ip + 2, iplen);
    ip[8] = 64; ip[9] = 6;                              /* TTL, proto TCP */
    memcpy(ip + 12, s->ip, 4);
    memcpy(ip + 16, s->peer_ip, 4);
    wr16(ip + 10, cksum16(ip, 20, 0));

    wr16(tcp + 0, s->local_port);
    wr16(tcp + 2, s->peer_port);
    wr32(tcp + 4, s->snd_nxt);
    wr32(tcp + 8, s->rcv_nxt);
    tcp[12] = (tcplen / 4) << 4;
    tcp[13] = flags;
    wr16(tcp + 14, SINK_WIN);
    if (with_mss) { tcp[20] = 2; tcp[21] = 4; wr16(tcp + 22, s->mss); }
    sum = rd16(s->ip) + rd16(s->ip + 2) + rd16(s->peer_ip) + rd16(s->peer_ip + 2) + 6 + tcplen;
    wr16(tcp + 16, cksum16(tcp, tcplen, sum));
    sink_emit(s, f, (14 + iplen < 60) ? 60 : 14 + iplen); /* pad: el3 drops sub-60-byte runts */
}

static void handle_tcp(TcpSinkState *s, const uint8_t *ip, int iplen)
{
    int ihl = (ip[0] & 0x0f) * 4;
    const uint8_t *tcp = ip + ihl;
    int seglen = iplen - ihl, doff, paylen;
    uint16_t sport, dport;
    uint32_t seq;
    uint8_t flags;

    if (seglen < 20) return;
    sport = rd16(tcp + 0); dport = rd16(tcp + 2);
    seq   = rd32(tcp + 4);
    doff  = (tcp[12] >> 4) * 4;
    flags = tcp[13];
    paylen = seglen - doff;
    if (paylen < 0) return;

    if (flags & TF_RST) { s->open = 0; return; }

    if ((flags & (TF_SYN | TF_ACK)) == TF_SYN) {        /* passive open */
        s->peer_port  = sport;
        s->local_port = dport;
        memcpy(s->peer_ip, ip + 12, 4);
        s->rcv_nxt  = seq + 1;                          /* their SYN consumes one seq */
        s->snd_nxt  = SINK_ISS;
        s->rx_bytes = 0;
        tcp_out(s, TF_SYN | TF_ACK, 1);                 /* SYN-ACK advertising our MSS */
        s->snd_nxt += 1;                                /* our SYN consumes one seq */
        s->open = 1;
        return;
    }
    if (!s->open || sport != s->peer_port || dport != s->local_port) return;

    if (paylen > 0) {
        if (seq == s->rcv_nxt) {                        /* in-order (no loss on this link) */
            s->rcv_nxt  += (uint32_t)paylen;
            s->rx_bytes += (uint64_t)paylen;
        }
        tcp_out(s, TF_ACK, 0);                          /* cumulative ACK (dup if out of order) */
    }
    if (flags & TF_FIN) {
        if (seq + (uint32_t)paylen == s->rcv_nxt) s->rcv_nxt += 1;
        tcp_out(s, TF_FIN | TF_ACK, 0);                 /* ack their FIN + send ours */
        s->snd_nxt += 1;
        s->open = 0;
    }
}

static ssize_t tcpsink_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    TcpSinkState *s = DO_UPCAST(TcpSinkState, nc, nc);
    uint16_t et;
    if (size < 14) return size;
    et = rd16(buf + 12);
    if (et == ETH_P_ARP) {
        handle_arp(s, buf, size);
    } else if (et == ETH_P_IP && size >= 34) {
        const uint8_t *ip = buf + 14;
        if (ip[9] == 6) {                               /* proto TCP */
            int iptot = rd16(ip + 2);
            if (iptot < 20 || 14 + iptot > (int)size) iptot = (int)size - 14;
            handle_tcp(s, ip, iptot);
        }
    }
    return size;
}

static void tcpsink_cleanup(NetClientState *nc)
{
    TcpSinkState *s = DO_UPCAST(TcpSinkState, nc, nc);
    if (s->bh) {
        qemu_bh_delete(s->bh);
        s->bh = NULL;
    }
}

static NetClientInfo net_tcpsink_info = {
    .type     = NET_CLIENT_DRIVER_TCPSINK,
    .size     = sizeof(TcpSinkState),
    .receive  = tcpsink_receive,
    .cleanup  = tcpsink_cleanup,
};

int net_init_tcpsink(const Netdev *netdev, const char *name,
                     NetClientState *peer, Error **errp)
{
    static const uint8_t defmac[6] = { 0x52, 0x54, 0x00, 0x00, 0x00, 0xfe };
    static const uint8_t defip[4]  = { 10, 0, 0, 2 };
    const NetdevTcpsinkOptions *opt;
    NetClientState *nc;
    TcpSinkState *s;

    assert(netdev->type == NET_CLIENT_DRIVER_TCPSINK);
    opt = &netdev->u.tcpsink;

    nc = qemu_new_net_client(&net_tcpsink_info, peer, "tcpsink", name);
    s = DO_UPCAST(TcpSinkState, nc, nc);
    s->mss = (opt->has_mss && opt->mss) ? opt->mss : 1460;
    memcpy(s->mac, defmac, 6);
    memcpy(s->ip, defip, 4);
    s->open = 0;
    s->txhead = s->txtail = s->txcount = 0;
    s->bh = qemu_bh_new(sink_bh, s);
    return 0;
}
