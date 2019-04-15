#ifndef BLOCK_OCSSD_H
#define BLOCK_OCSSD_H

#include "qemu/compiler.h"

#include "block/nvme.h"

#define OCSSD_MAGIC ('O' << 24 | 'C' << 16 | '2' << 8 | '0')

enum OcssdAdminCommands {
    OCSSD_ADM_CMD_GEOMETRY       = 0xe2,
};

enum OcssdIoCommands {
    OCSSD_CMD_VECT_RESET         = 0x90,
    OCSSD_CMD_VECT_WRITE         = 0x91,
    OCSSD_CMD_VECT_READ          = 0x92,
};

enum OcssdChunkStates {
    OCSSD_CHUNK_FREE     = 1 << 0,
    OCSSD_CHUNK_CLOSED   = 1 << 1,
    OCSSD_CHUNK_OPEN     = 1 << 2,
    OCSSD_CHUNK_OFFLINE  = 1 << 3,
};

#define OCSSD_CHUNK_RESETABLE \
    (OCSSD_CHUNK_FREE | OCSSD_CHUNK_CLOSED | OCSSD_CHUNK_OPEN)

enum OcssdChunkTypes {
    OCSSD_CHUNK_TYPE_SEQUENTIAL = 1 << 0,
    OCSSD_CHUNK_TYPE_RANDOM     = 1 << 1,
    OCSSD_CHUNK_TYPE_SHRINKED   = 1 << 4,
};

enum OcssdStatusCodes {
    OCSSD_LBAL_SGL_LENGTH_INVALID = 0x01c1,

    OCSSD_WRITE_NEXT_UNIT         = 0x02f0,
    OCSSD_CHUNK_EARLY_CLOSE       = 0x02f1,
    OCSSD_OUT_OF_ORDER_WRITE      = 0x02f2,
    OCSSD_OFFLINE_CHUNK           = 0x02c0,
    OCSSD_INVALID_RESET           = 0x02c1,
};

typedef struct OcssdChunkDescriptor {
    uint8_t  state;
    uint8_t  type;
    uint8_t  wear_index;
    uint8_t  rsvd[5];
    uint64_t slba;
    uint64_t cnlb;
    uint64_t wp;
} OcssdChunkDescriptor;

typedef struct OcssdRwCmd {
    uint16_t    opcode:8;
    uint16_t    fuse:2;
    uint16_t    rsvd1:4;
    uint16_t    psdt:2;
    uint16_t    cid;
    uint32_t    nsid;
    uint64_t    rsvd2;
    uint64_t    metadata;
    NvmeCmdDptr dptr;
    uint64_t    lbal;
    uint16_t    nlb;
    uint16_t    control;
    uint32_t    rsvd3;
    uint64_t    rsvd4;
} OcssdRwCmd;

typedef struct OcssdDmCmd {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t cid;
    uint32_t nsid;
    uint32_t rsvd1[8];
    uint64_t spba;
    uint32_t nlb;
    uint32_t rsvd2[3];
} OcssdDmCmd;

typedef struct OcssdIdGeo {
    uint16_t num_grp;
    uint16_t num_pu;
    uint32_t num_chk;
    uint32_t clba;
    uint8_t  rsvd[52];
} OcssdIdGeo;

typedef struct OcssdIdWrt {
    uint32_t ws_min;
    uint32_t ws_opt;
    uint32_t mw_cunits;
    uint8_t  rsvd[52];
} OcssdIdWrt;

typedef struct OcssdIdPerf {
    uint32_t trdt;
    uint32_t trdm;
    uint32_t tprt;
    uint32_t tprm;
    uint32_t tbet;
    uint32_t tbem;
    uint8_t  rsvd[40];
} OcssdIdPerf;

typedef struct OcssdIdLBAF {
    uint8_t grp_len;
    uint8_t pu_len;
    uint8_t chk_len;
    uint8_t sec_len;
    uint8_t rsvd[4];
} OcssdIdLBAF;

typedef struct OcssdFormatHeader {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    num_ns;
    uint32_t    md_size;
    uint64_t    sector_size;
    uint64_t    ns_size;
    OcssdIdLBAF lbaf;
    uint8_t     rsvd[4052];
} OcssdFormatHeader;

typedef struct OcssdIdentity {
    struct {
        uint8_t major;
        uint8_t minor;
    } ver;
    uint8_t     rsvd1[6];
    OcssdIdLBAF lbaf;
    uint32_t    mccap;
    uint8_t     rsvd2[12];
    uint8_t     wit;
    uint8_t     rsvd3[31];
    OcssdIdGeo  geo;
    OcssdIdWrt  wrt;
    OcssdIdPerf perf;
    uint8_t     rsvd4[3840];
} OcssdIdentity;

enum OcssdIdentityMccap {
    OCSSD_IDENTITY_MCCAP_MULTIPLE_RESETS = 0x1 << 1,

    /* OCSSD 2.0 spec de-facto extension */
    OCSSD_IDENTITY_MCCAP_EARLY_RESET = 0x1 << 2,
};

enum OcssdLogPage {
    OCSSD_CHUNK_INFO = 0xCA,
};

static inline void _ocssd_check_sizes(void)
{
    QEMU_BUILD_BUG_ON(sizeof(OcssdIdLBAF)          != 8);
    QEMU_BUILD_BUG_ON(sizeof(OcssdIdGeo)           != 64);
    QEMU_BUILD_BUG_ON(sizeof(OcssdIdWrt)           != 64);
    QEMU_BUILD_BUG_ON(sizeof(OcssdIdPerf)          != 64);
    QEMU_BUILD_BUG_ON(sizeof(OcssdRwCmd)           != 64);
    QEMU_BUILD_BUG_ON(sizeof(OcssdDmCmd)           != 64);
    QEMU_BUILD_BUG_ON(sizeof(OcssdIdentity)        != 4096);
    QEMU_BUILD_BUG_ON(sizeof(OcssdChunkDescriptor) != 32);
    QEMU_BUILD_BUG_ON(sizeof(OcssdFormatHeader)    != 4096);
}

#endif
