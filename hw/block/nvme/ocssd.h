/*
 * QEMU OpenChannel 2.0 Controller
 *
 * Copyright (c) 2019 Klaus Birkelund Jensen <klaus@birkelund.eu>
 * Copyright (c) 2019 CNEX Labs, Inc.
 *
 * Thank you to the following people for their contributions to the original
 * qemu-nvme (github.com/OpenChannelSSD/qemu-nvme) implementation from which
 * this code was originally derived.
 *
 *   Matias Bjørling <mb@lightnvm.io>
 *   Javier González <javier@javigon.com>
 *   Simon Andreas Frimann Lund <ocssd@safl.dk>
 *   Hans Holmberg <hans@owltronix.com>
 *   Jesper Devantier <contact@pseudonymous.me>
 *   Young Tack Jin <youngtack.jin@circuitblvd.com>
 *
 * This code is licensed under the GNU GPL v2 or later.
 */

#ifndef HW_NVME_OCSSD_H
#define HW_NVME_OCSSD_H

#include "hw/block/nvme/nvme.h"

#include "block/ocssd.h"

#define TYPE_OCSSD "ocssd"
#define OCSSD(obj) \
        OBJECT_CHECK(OcssdCtrl, (obj), TYPE_OCSSD)

#define OCSSD_MAX_CHUNK_NOTIFICATIONS 64

#define DEFINE_OCSSD_PROPERTIES(_state, _props) \
    DEFINE_PROP_UINT32("ws_min", _state, _props.ws_min, 4), \
    DEFINE_PROP_UINT32("ws_opt", _state, _props.ws_opt, 8), \
    DEFINE_PROP_UINT32("mw_cunits", _state, _props.mw_cunits, 32), \
    DEFINE_PROP_BOOL("early_reset", _state, _props.early_reset, true), \
    DEFINE_PROP_STRING("resetfail", _state, _props.resetfail_fname), \
    DEFINE_PROP_STRING("writefail", _state, _props.writefail_fname), \
    DEFINE_PROP_STRING("chunkinfo", _state, _props.chunkinfo_fname)

typedef struct OcssdParams {
    /* qemu configurable device characteristics */
    uint32_t ws_min;
    uint32_t ws_opt;
    uint32_t mw_cunits;
    bool     early_reset;

    char *chunkinfo_fname;
    char *resetfail_fname;
    char *writefail_fname;
} OcssdParams;

#define OCSSD_CMD_MAX_LBAS 64

typedef struct OcssdAddrF {
    uint64_t grp_mask;
    uint64_t pu_mask;
    uint64_t chk_mask;
    uint64_t sec_mask;
    uint8_t  grp_offset;
    uint8_t  pu_offset;
    uint8_t  chk_offset;
    uint8_t  sec_offset;
} OcssdAddrF;

typedef struct OcssdChunkAcctDescriptor {
    uint32_t pe_cycles;
} OcssdChunkAcctDescriptor;

typedef struct OcssdChunkAcct {
    uint64_t blk_offset;
    uint64_t size;
    OcssdChunkAcctDescriptor *descr;
} OcssdChunkAcct;

typedef struct OcssdChunkInfo {
    uint64_t blk_offset;
    uint64_t size;
    OcssdChunkDescriptor *descr;
} OcssdChunkInfo;

typedef struct OcssdNamespace {
    NvmeNamespace *ns;

    OcssdIdentity id;
    OcssdAddrF    addrf;

    /* reset and write fail error probabilities indexed by namespace */
    uint8_t *resetfail;
    uint8_t *writefail;

    /* derived values (convenience) */
    uint32_t chks_per_grp;
    uint32_t chks_total;
    uint32_t secs_per_chk;
    uint32_t secs_per_pu;
    uint32_t secs_per_grp;
    uint32_t secs_total;

    /* wear index tracking */
    uint8_t  wear_index_avg;
    uint64_t wear_index_total;

    OcssdChunkInfo info;
    OcssdChunkAcct acct;
} OcssdNamespace;

typedef struct OcssdCtrl {
    NvmeCtrl nvme;

    OcssdFormatHeader hdr;
    OcssdParams params;
    OcssdNamespace *namespaces;

    uint64_t notifications_count;
    uint16_t notifications_index;
    uint16_t notifications_max;
    OcssdChunkNotification notifications[OCSSD_MAX_CHUNK_NOTIFICATIONS];
} OcssdCtrl;

static inline void ocssd_ns_optimal_addrf(OcssdAddrF *addrf, OcssdIdLBAF *lbaf)
{
    addrf->sec_offset = 0;
    addrf->chk_offset = lbaf->sec_len;
    addrf->pu_offset  = lbaf->sec_len + lbaf->chk_len;
    addrf->grp_offset = lbaf->sec_len + lbaf->chk_len + lbaf->pu_len;

    addrf->grp_mask = ((1 << lbaf->grp_len) - 1) << addrf->grp_offset;
    addrf->pu_mask  = ((1 << lbaf->pu_len)  - 1) << addrf->pu_offset;
    addrf->chk_mask = ((1 << lbaf->chk_len) - 1) << addrf->chk_offset;
    addrf->sec_mask = ((1 << lbaf->sec_len) - 1) << addrf->sec_offset;
}

static inline bool ocssd_rw_is_write(NvmeRequest *req)
{
    return req->cmd.opcode == OCSSD_CMD_VECT_WRITE || nvme_rw_is_write(req);
}

static inline OcssdNamespace *_ons(OcssdCtrl *o, uint32_t nsid)
{
    if (unlikely(nsid == 0 || nsid > o->nvme.params.num_ns)) {
        return NULL;
    }

    return &o->namespaces[nsid - 1];
}

static inline bool _chk_wi_outside_threshold(OcssdNamespace *ons,
    OcssdChunkDescriptor *chk)
{
    return chk->wear_index < ons->wear_index_avg - ons->id.wit ||
        chk->wear_index > ons->wear_index_avg + ons->id.wit;
}

static inline uint8_t _calc_wi(OcssdCtrl *o, uint32_t pe_cycles)
{
    return (pe_cycles * 255) / o->hdr.pe_cycles;
}

/* _group returns the non-global sector number of the given lba */
static inline uint64_t _sectr(OcssdAddrF *addrf, uint64_t lba)
{
    return (lba & addrf->sec_mask) >> addrf->sec_offset;
}

/* _group returns the non-global chunk number of the given lba */
static inline uint64_t _chunk(OcssdAddrF *addrf, uint64_t lba)
{
    return (lba & addrf->chk_mask) >> addrf->chk_offset;
}

/* _group returns the non-global parallel unit number of the given lba */
static inline uint64_t _punit(OcssdAddrF *addrf, uint64_t lba)
{
    return (lba & addrf->pu_mask) >> addrf->pu_offset;
}

/* _group returns the group number of the given lba */
static inline uint64_t _group(OcssdAddrF *addrf, uint64_t lba)
{
    return (lba & addrf->grp_mask) >> addrf->grp_offset;
}

/*
 * _make_lba returns a physical lba given group, parallel unit, chunk and
 * sector number.
 */
static inline uint64_t _make_lba(OcssdAddrF *addrf, uint16_t group,
    uint16_t punit, uint32_t chunk, uint32_t sectr)
{
    return sectr << addrf->sec_offset
        | chunk << addrf->chk_offset
        | punit << addrf->pu_offset
        | group << addrf->grp_offset;
}

/* _valid verifies if the given lba is valid for the namespace geometry */
static inline int _valid(OcssdCtrl *o, OcssdNamespace *ons, uint64_t lba)
{
    OcssdIdGeo *geo = &ons->id.geo;
    OcssdAddrF *addrf = &ons->addrf;

    return _sectr(addrf, lba) < geo->clba &&
        _chunk(addrf, lba) < geo->num_chk &&
        _punit(addrf, lba) < geo->num_pu &&
        _group(addrf, lba) < geo->num_grp;
}

/* _idx returns the global sector number of the given lba */
static inline uint64_t _idx(OcssdCtrl *o, OcssdNamespace *ons,
    uint64_t lba)
{
    OcssdAddrF *addrf = &ons->addrf;

    return _sectr(addrf, lba) +
        _chunk(addrf, lba) * ons->secs_per_chk +
        _punit(addrf, lba) * ons->secs_per_pu +
        _group(addrf, lba) * ons->secs_per_grp;
}

/* _chk_idx returns the global chunk number of the given lba */
static inline uint64_t _chk_idx(OcssdCtrl *o, OcssdNamespace *ons,
    uint64_t lba)
{
    OcssdIdGeo *geo = &ons->id.geo;
    OcssdAddrF *addrf = &ons->addrf;

    return _chunk(addrf, lba) +
        _punit(addrf, lba) * geo->num_chk +
        _group(addrf, lba) * ons->chks_per_grp;
}

/* _vlba returns the nth lba in the vector lba list */
static inline uint64_t _vlba(NvmeRequest *req, uint16_t n)
{
    return req->nlb > 1 ? ((uint64_t *) req->slba)[n] : req->slba;
}

/* _addrfn translates from physical to logical address space */
static inline uint64_t _addrfn(NvmeCtrl *n, NvmeNamespace *ns, uint64_t lba)
{
    OcssdCtrl *o = OCSSD(n);
    OcssdNamespace *ons = &o->namespaces[ns->id - 1];

    return _idx(o, ons, lba);
}

#endif /* HW_NVME_OCSSD_H */
