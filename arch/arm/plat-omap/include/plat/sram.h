int omap_sram_init(void);

void omap_map_sram(unsigned long start, unsigned long size,
			unsigned long skip, int cached);
void omap_sram_reset(void);

extern struct gen_pool *omap_gen_pool;

/*
 * Note that fncpy requires the SRAM address to be aligned to an 8-byte
 * boundary, so the min_alloc_order for the pool is set appropriately.
 */
#define omap_sram_push(funcp, size) ({					\
	typeof(&(funcp)) _res;						\
	size_t _sz = size;						\
	void *_sram = (void *) gen_pool_alloc(omap_gen_pool, _sz);	\
	_res = (_sram ? fncpy(_sram, &(funcp), _sz) : NULL);		\
	if (!_res)							\
		pr_err("Not enough space in SRAM\n");			\
	_res;								\
})
