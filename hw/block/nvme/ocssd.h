#ifndef HW_NVME_OCSSD_H
#define HW_NVME_OCSSD_H

#include "hw/block/nvme/nvme.h"

#include "block/ocssd.h"

#define TYPE_OCSSD "ocssd"
#define OCSSD(obj) \
        OBJECT_CHECK(OcssdCtrl, (obj), TYPE_OCSSD)

#define DEFINE_OCSSD_PROPERTIES(_state, _props) \
    DEFINE_PROP_UINT32("mccap", _state, _props.mccap, 0x4), \
    DEFINE_PROP_UINT32("ws_min", _state, _props.ws_min, 4), \
    DEFINE_PROP_UINT32("ws_opt", _state, _props.ws_opt, 8), \
    DEFINE_PROP_UINT32("mw_cunits", _state, _props.mw_cunits, 32), \
    DEFINE_PROP_STRING("resetfail", _state, _props.resetfail_fname), \
    DEFINE_PROP_STRING("writefail", _state, _props.writefail_fname), \
    DEFINE_PROP_STRING("chunkinfo", _state, _props.chunkinfo_fname), \
    DEFINE_PROP_UINT8("sgl_lbal", _state, _props.sgl_lbal, 0)

typedef struct OcssdParams {
    /* qemu configurable device characteristics */
    uint32_t mccap;
    uint32_t ws_min;
    uint32_t ws_opt;
    uint32_t mw_cunits;
    uint8_t  early_reset;
    uint8_t  sgl_lbal;

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

    /* chunk info log page */
    uint64_t              chunk_info_offset;
    uint64_t              chunk_info_size;
    OcssdChunkDescriptor *chunk_info;
} OcssdNamespace;

typedef struct OcssdCtrl {
    NvmeCtrl nvme;

    OcssdFormatHeader hdr;
    OcssdParams params;
    OcssdNamespace *namespaces;
} OcssdCtrl;

static inline bool ocssd_rw_is_write(NvmeRequest *req)
{
    return req->cmd_opcode == OCSSD_CMD_VECT_WRITE || nvme_rw_is_write(req);
}

static inline OcssdNamespace *_ons(OcssdCtrl *o, NvmeRequest *req)
{
    return &o->namespaces[req->ns->id - 1];
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
