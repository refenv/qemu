#ifndef HW_NVME_H
#define HW_NVME_H
#include "block/nvme.h"

#define DEFINE_NVME_PROPERTIES(_state, _props) \
    DEFINE_PROP_STRING("serial", _state, _props.serial), \
    DEFINE_PROP_UINT32("cmb_size_mb", _state, _props.cmb_size_mb, 0), \
    DEFINE_PROP_UINT32("num_queues", _state, _props.num_queues, 64), \
    DEFINE_PROP_UINT32("num_ns", _state, _props.num_ns, 1)

typedef struct NvmeParams {
    char     *serial;
    uint32_t num_queues;
    uint32_t num_ns;
    uint32_t cmb_size_mb;
} NvmeParams;

typedef struct NvmeAsyncEvent {
    QSIMPLEQ_ENTRY(NvmeAsyncEvent) entry;
    NvmeAerResult result;
} NvmeAsyncEvent;

typedef struct NvmeRequest {
    struct NvmeSQueue       *sq;
    BlockAIOCB              *aiocb;
    uint16_t                status;
    bool                    is_cmb;
    uint8_t                 cmd_opcode;
    NvmeCqe                 cqe;
    BlockAcctCookie         acct;
    QEMUSGList              qsg;
    QEMUIOVector            iov;
    QTAILQ_ENTRY(NvmeRequest)entry;
} NvmeRequest;

typedef struct NvmeSQueue {
    struct NvmeCtrl *ctrl;
    uint16_t    sqid;
    uint16_t    cqid;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    size;
    uint64_t    dma_addr;
    QEMUTimer   *timer;
    NvmeRequest *io_req;
    QTAILQ_HEAD(, NvmeRequest) req_list;
    QTAILQ_HEAD(, NvmeRequest) out_req_list;
    QTAILQ_ENTRY(NvmeSQueue) entry;
} NvmeSQueue;

typedef struct NvmeCQueue {
    struct NvmeCtrl *ctrl;
    uint8_t     phase;
    uint16_t    cqid;
    uint16_t    irq_enabled;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    vector;
    uint32_t    size;
    uint64_t    dma_addr;
    QEMUTimer   *timer;
    QTAILQ_HEAD(, NvmeSQueue) sq_list;
    QTAILQ_HEAD(, NvmeRequest) req_list;
} NvmeCQueue;

typedef struct NvmeNamespace {
    NvmeIdNs        id_ns;
    uint32_t        id;
    uint64_t        ns_blks;
    uint64_t        blk_offset;
} NvmeNamespace;

#define TYPE_NVME "nvme"
#define NVME(obj) \
        OBJECT_CHECK(NvmeCtrl, (obj), TYPE_NVME)

typedef struct NvmeCtrl {
    PCIDevice    parent_obj;
    MemoryRegion iomem;
    MemoryRegion ctrl_mem;
    NvmeBar      bar;
    BlockConf    conf;
    NvmeParams   params;

    time_t      start_time;
    uint32_t    page_size;
    uint16_t    page_bits;
    uint16_t    max_prp_ents;
    uint16_t    cqe_size;
    uint16_t    sqe_size;
    uint32_t    reg_size;
    uint32_t    max_q_ents;
    uint64_t    ns_size;
    uint8_t     outstanding_aers;
    uint32_t    cmbsz;
    uint32_t    cmbloc;
    uint8_t     *cmbuf;
    uint64_t    irq_status;

    QSIMPLEQ_HEAD(, NvmeAsyncEvent) aer_queue;
    QEMUTimer   *aer_timer;
    uint8_t     aer_mask;

    NvmeErrorLog    *elpes;
    NvmeRequest     **aer_reqs;
    NvmeNamespace   *namespaces;
    NvmeSQueue      **sq;
    NvmeCQueue      **cq;
    NvmeSQueue      admin_sq;
    NvmeCQueue      admin_cq;
    NvmeFeatureVal  features;
    NvmeIdCtrl      id_ctrl;
} NvmeCtrl;

static inline uint8_t nvme_ns_lbads(NvmeNamespace *ns)
{
    NvmeIdNs *id = &ns->id_ns;
    return id->lbaf[NVME_ID_NS_FLBAS_INDEX(id->flbas)].lbads;
}

static inline size_t nvme_ns_lbads_bytes(NvmeNamespace *ns)
{
    return 1 << nvme_ns_lbads(ns);
}

#endif /* HW_NVME_H */
