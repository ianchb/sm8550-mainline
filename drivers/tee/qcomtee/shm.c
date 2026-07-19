// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/genalloc.h>
#include <linux/mm.h>
#include <linux/sizes.h>

#include "qcomtee.h"

/*
 * Keep one maximum-sized inbound buffer available from boot.  Large QTEE
 * clients otherwise allocate high-order pages after the system has been
 * running for some time, when physical memory can be too fragmented even
 * though plenty of memory remains available overall.
 */
#define QCOMTEE_SHM_PREALLOC_SIZE SZ_4M

struct qcomtee_shm_pool {
	struct gen_pool *genpool;
	void *vaddr;
	phys_addr_t paddr;
	size_t size;
	u64 sec_world_id;
};

/**
 * define MAX_OUTBOUND_BUFFER_SIZE - Maximum size of outbound buffers.
 *
 * The size of outbound buffer depends on QTEE callback requests.
 */
#define MAX_OUTBOUND_BUFFER_SIZE SZ_4K

/**
 * define MAX_INBOUND_BUFFER_SIZE - Maximum size of the inbound buffer.
 *
 * The size of the inbound buffer depends on the user's requests,
 * specifically the number of IB and OB arguments. If an invocation
 * requires a size larger than %MAX_INBOUND_BUFFER_SIZE, the user should
 * consider using another form of shared memory with QTEE.
 */
#define MAX_INBOUND_BUFFER_SIZE SZ_4M

/**
 * qcomtee_msg_buffers_alloc() - Allocate inbound and outbound buffers.
 * @oic: context to use for the current invocation.
 * @u: array of arguments for the current invocation.
 *
 * It calculates the size of inbound and outbound buffers based on the
 * arguments in @u. It allocates the buffers from the teedev pool.
 *
 * Return: On success, returns 0. On error, returns < 0.
 */
int qcomtee_msg_buffers_alloc(struct qcomtee_object_invoke_ctx *oic,
			      struct qcomtee_arg *u)
{
	struct tee_context *ctx = oic->ctx;
	struct tee_shm *shm;
	size_t size;
	int i;

	/* Start offset in a message for buffer arguments. */
	size = qcomtee_msg_buffer_args(struct qcomtee_msg_object_invoke,
				       qcomtee_args_len(u));
	if (size > MAX_INBOUND_BUFFER_SIZE)
		return -EINVAL;

	/* Add size of IB arguments. */
	qcomtee_arg_for_each_input_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	/* Add size of OB arguments. */
	qcomtee_arg_for_each_output_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	shm = tee_shm_alloc_priv_buf(ctx, size);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	/* Allocate inbound buffer. */
	oic->in_shm = shm;
	shm = tee_shm_alloc_priv_buf(ctx, MAX_OUTBOUND_BUFFER_SIZE);
	if (IS_ERR(shm)) {
		tee_shm_free(oic->in_shm);

		return PTR_ERR(shm);
	}
	/* Allocate outbound buffer. */
	oic->out_shm = shm;

	oic->in_msg.addr = tee_shm_get_va(oic->in_shm, 0);
	oic->in_msg.size = tee_shm_get_size(oic->in_shm);
	oic->out_msg.addr = tee_shm_get_va(oic->out_shm, 0);
	oic->out_msg.size = tee_shm_get_size(oic->out_shm);
	/* QTEE assume unused buffers are zeroed. */
	memzero_explicit(oic->in_msg.addr, oic->in_msg.size);
	memzero_explicit(oic->out_msg.addr, oic->out_msg.size);

	return 0;
}

void qcomtee_msg_buffers_free(struct qcomtee_object_invoke_ctx *oic)
{
	tee_shm_free(oic->in_shm);
	tee_shm_free(oic->out_shm);
}

/* Dynamic shared memory pool based on tee_dyn_shm_alloc_helper(). */

static int qcomtee_shm_register(struct tee_context *ctx, struct tee_shm *shm,
				struct page **pages, size_t num_pages,
				unsigned long start)
{
	return qcom_tzmem_shm_bridge_create(shm->paddr, shm->size,
					    &shm->sec_world_id);
}

static int qcomtee_shm_unregister(struct tee_context *ctx, struct tee_shm *shm)
{
	qcom_tzmem_shm_bridge_delete(shm->sec_world_id);

	return 0;
}

static int pool_op_alloc(struct tee_shm_pool *pool, struct tee_shm *shm,
			 size_t size, size_t align)
{
	struct qcomtee_shm_pool *qpool = pool->private_data;
	struct genpool_data_align data = {
		.align = max_t(size_t, align, PAGE_SIZE),
	};
	unsigned long vaddr;
	size_t alloc_size = PAGE_ALIGN(size);

	if (qpool) {
		vaddr = gen_pool_alloc_algo(qpool->genpool, alloc_size,
					    gen_pool_first_fit_align,
					    &data);
		if (vaddr) {
			memset((void *)vaddr, 0, alloc_size);
			shm->kaddr = (void *)vaddr;
			shm->paddr = gen_pool_virt_to_phys(qpool->genpool,
							   vaddr);
			shm->size = alloc_size;
			shm->flags &= ~TEE_SHM_DYNAMIC;

			return 0;
		}
	}

	return tee_dyn_shm_alloc_helper(shm, size, align, qcomtee_shm_register);
}

static void pool_op_free(struct tee_shm_pool *pool, struct tee_shm *shm)
{
	struct qcomtee_shm_pool *qpool = pool->private_data;

	/* Dynamic allocations have a page array; preallocated chunks do not. */
	if (shm->pages) {
		tee_dyn_shm_free_helper(shm, qcomtee_shm_unregister);
		return;
	}

	gen_pool_free(qpool->genpool, (unsigned long)shm->kaddr, shm->size);
	shm->kaddr = NULL;
}

static void pool_op_destroy_pool(struct tee_shm_pool *pool)
{
	struct qcomtee_shm_pool *qpool = pool->private_data;

	if (qpool) {
		qcom_tzmem_shm_bridge_delete(qpool->sec_world_id);
		gen_pool_destroy(qpool->genpool);
		free_pages_exact(qpool->vaddr, qpool->size);
		kfree(qpool);
	}
	kfree(pool);
}

static const struct tee_shm_pool_ops pool_ops = {
	.alloc = pool_op_alloc,
	.free = pool_op_free,
	.destroy_pool = pool_op_destroy_pool,
};

struct tee_shm_pool *qcomtee_shm_pool_alloc(void)
{
	struct tee_shm_pool *pool;
	struct qcomtee_shm_pool *qpool;
	int ret = -ENOMEM;

	pool = kzalloc_obj(*pool);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	qpool = kzalloc_obj(*qpool);
	if (!qpool)
		goto err_free_pool;

	qpool->size = QCOMTEE_SHM_PREALLOC_SIZE;
	qpool->vaddr = alloc_pages_exact(qpool->size,
					 GFP_KERNEL | __GFP_ZERO |
					 __GFP_RETRY_MAYFAIL | __GFP_NOWARN);
	if (!qpool->vaddr) {
		pr_warn("failed to preallocate %zu-byte shared memory pool\n",
			qpool->size);
		kfree(qpool);
		qpool = NULL;
		goto use_dynamic_pool;
	}

	qpool->paddr = virt_to_phys(qpool->vaddr);
	qpool->genpool = gen_pool_create(PAGE_SHIFT, -1);
	if (!qpool->genpool)
		goto err_free_pages;

	ret = gen_pool_add_virt(qpool->genpool,
				(unsigned long)qpool->vaddr, qpool->paddr,
				qpool->size, -1);
	if (ret)
		goto err_destroy_genpool;

	ret = qcom_tzmem_shm_bridge_create(qpool->paddr, qpool->size,
					   &qpool->sec_world_id);
	if (ret)
		goto err_destroy_genpool;

	pr_info("preallocated %zu-byte shared memory pool\n", qpool->size);

use_dynamic_pool:
	pool->ops = &pool_ops;
	pool->private_data = qpool;

	return pool;

err_destroy_genpool:
	gen_pool_destroy(qpool->genpool);
err_free_pages:
	free_pages_exact(qpool->vaddr, qpool->size);
	kfree(qpool);
err_free_pool:
	kfree(pool);

	return ERR_PTR(ret);
}
