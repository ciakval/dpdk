/*
 *   BSD LICENSE
 *
 *   Copyright (C) Netcope Technologies, a.s.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Netcope Technologies, a.s. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ~~~~[ INCLUDES ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <rte_ethdev.h>
#include <rte_ethdev_vdev.h>
#include <rte_vdev.h>
#include <rte_atomic.h>

/* ~~~~[ TYPES ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
struct loopback_queue {
    struct rte_ring * ring;
    struct rte_mbuf * pkt;
    rte_atomic16_t pkt_status;
    volatile uint64_t rx_pkt;
    volatile uint64_t tx_pkt;
};

#define PKT_STATUS_EMPTY 0
#define PKT_STATUS_FULL 1
#define PKT_STATUS_HANDLING 2

struct pmd_internals {
    struct loopback_queue queue;

};

/* ~~~~[ STATIC DATA ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static const struct rte_eth_link default_pmd_link = {
    .link_speed = ETH_SPEED_NUM_100G,
    .link_autoneg = ETH_LINK_SPEED_FIXED,
    .link_duplex = ETH_LINK_FULL_DUPLEX,
    .link_status = ETH_LINK_DOWN
};

static struct ether_addr pmd_mac_addr = { .addr_bytes = {0} };

/* ~~~~[ RX / TX functions ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint16_t loopback_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
    struct loopback_queue * queue = q;

    if (nb_bufs == 0) return 0;

    if (rte_atomic16_cmpset((volatile uint16_t *)&queue->pkt_status.cnt, PKT_STATUS_FULL, PKT_STATUS_HANDLING) == 0) {
        RTE_LOG(INFO, PMD, "Cannot read packet, slot empty\n");
        return 0;
    }
    bufs[0] = queue->pkt;
    if (rte_atomic16_cmpset((volatile uint16_t *)&queue->pkt_status.cnt, PKT_STATUS_HANDLING, PKT_STATUS_EMPTY) == 0) {
        RTE_LOG(WARNING, PMD, "Someone is fiddling with slot status value, fix code\n");
        rte_atomic16_set(&queue->pkt_status, PKT_STATUS_EMPTY);
    }

    queue->rx_pkt++;

    return 1;
}

static uint16_t loopback_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
    struct loopback_queue * queue = q;

    if (nb_bufs == 0) return 0;

    if (rte_atomic16_cmpset((volatile uint16_t *)&queue->pkt_status.cnt, PKT_STATUS_EMPTY, PKT_STATUS_HANDLING) == 0) {
        RTE_LOG(INFO, PMD, "Cannot add packet, slot not empty\n");
        return 0;
    }

    queue->pkt = bufs[0];

    if (rte_atomic16_cmpset((volatile uint16_t *)&queue->pkt_status.cnt, PKT_STATUS_HANDLING, PKT_STATUS_FULL) == 0) {
        RTE_LOG(WARNING, PMD, "Someone is fiddling with slot atomic value, fix ur code\n");
        rte_atomic16_set(&queue->pkt_status, PKT_STATUS_FULL);
    }
    queue->tx_pkt++;

    return 1;
}


/* ~~~~[ ETHERNET DEVICE OPERATIONS ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void loopback_infos_get(struct rte_eth_dev *dev __rte_unused,
                    struct rte_eth_dev_info *dev_info)
{
    dev_info->driver_name = "loopback";
    dev_info->max_mac_addrs = 1;
    dev_info->max_rx_pktlen = UINT32_MAX;
    dev_info->max_rx_queues = 1;
    dev_info->max_tx_queues = 1;
}

static int loopback_configure(struct rte_eth_dev *dev __rte_unused)
{
    return 0;
}

static int loopback_start(struct rte_eth_dev *dev)
{
    dev->data->dev_link.link_status = ETH_LINK_UP;
    return 0;
}

static int loopback_rx_queue_setup(struct rte_eth_dev *dev,
                                   uint16_t rx_queue_id,
                                   uint16_t nb_rx_desc __rte_unused,
                                   unsigned int socket_id __rte_unused,
                                   const struct rte_eth_rxconf *rx_conf __rte_unused,
                                   struct rte_mempool *mb_pool __rte_unused)
{
    struct pmd_internals * internals = dev->data->dev_private;

    dev->data->rx_queues[rx_queue_id] = &internals->queue;

    return 0;
}

static int loopback_tx_queue_setup(struct rte_eth_dev *dev,
                                   uint16_t tx_queue_id,
                                   uint16_t nb_tx_desc __rte_unused,
                                   unsigned int socket_id __rte_unused,
                                   const struct rte_eth_txconf *tx_conf __rte_unused)
{
    struct pmd_internals * internals = dev->data->dev_private;

    dev->data->tx_queues[tx_queue_id] = &internals->queue;

    return 0;
}

static int loopback_link_update(struct rte_eth_dev *dev __rte_unused,
                                int wait_to_complete __rte_unused) { return 0; }


static void loopback_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
    struct pmd_internals *internals = dev->data->dev_private;

    stats->ipackets = internals->queue.rx_pkt;
    stats->opackets = internals->queue.tx_pkt;

}


struct eth_dev_ops loopback_pmd_ops = {
    .dev_configure = loopback_configure,
    .dev_infos_get = loopback_infos_get,
    .dev_start = loopback_start,
    .rx_queue_setup = loopback_rx_queue_setup,
    .tx_queue_setup = loopback_tx_queue_setup,
    .link_update = loopback_link_update,
    .stats_get = loopback_stats_get,
    NULL,
};

/* ~~~~[ PROBE functions ]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int loopback_probe(struct rte_vdev_device *dev)
{
    struct rte_eth_dev *eth_dev;
    struct rte_eth_dev_data *data;
    struct pmd_internals *internals;

    eth_dev = rte_eth_vdev_allocate(dev, sizeof(struct pmd_internals));
    if (!eth_dev) {
        return -ENOMEM;
    }

    data = eth_dev->data;
    internals = eth_dev->data->dev_private;

    rte_atomic16_init(&internals->queue.pkt_status);
    rte_atomic16_set(&internals->queue.pkt_status, PKT_STATUS_EMPTY);

    data->nb_rx_queues = 1;
    data->nb_tx_queues = 1;
    data->dev_link = default_pmd_link;
    data->mac_addrs = &pmd_mac_addr;
    eth_dev->dev_ops = &loopback_pmd_ops;

    eth_dev->rx_pkt_burst = loopback_rx;
    eth_dev->tx_pkt_burst = loopback_tx;
    return 0;
}

static int loopback_remove(struct rte_vdev_device *dev)
{
    if(dev){}
    return 0;
}

static struct rte_vdev_driver loopback_driver = {
    .probe = loopback_probe,
    .remove = loopback_remove
};

RTE_PMD_REGISTER_VDEV(loopback, loopback_driver);
