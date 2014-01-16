#ifndef __MCUIO_PROTO_H__
#define __MCUIO_PROTO_H__

#ifdef __KERNEL__
#include <linux/types.h>

#define mcuio_ntohq(a) le64_to_cpu(a)
#define mcuio_htonq(a) cpu_to_le64(a)
#define mcuio_ntohl(a) le32_to_cpu(a)
#define mcuio_htonl(a) cpu_to_le32(a)
#define mcuio_ntohs(a) le16_to_cpu(a)
#define mcuio_htons(a) cpu_to_le16(a)

#endif /* __KERNEL__ */


#define mcuio_type_rdb			0
#define mcuio_type_wrb			1
#define mcuio_type_rdw			2
#define mcuio_type_wrw			3
#define mcuio_type_rddw			4
#define mcuio_type_wrdw			5
#define mcuio_type_rdq			6
#define mcuio_type_wrq			7

#define mcuio_error_bit			BIT(5)
#define mcuio_reply_bit			BIT(6)
#define mcuio_fill_data_bit		BIT(7)
#define mcuio_actual_type_mask		0x07

#define mask(b)				(BIT(b) - 1)

#define mcuio_addr_offset_bits		12
#define mcuio_addr_offset_shift		0
#define mcuio_addr_offset_mask		mask(mcuio_addr_offset_bits)

#define mcuio_addr_func_bits		5
#define mcuio_addr_func_shift		mcuio_addr_offset_bits
#define mcuio_addr_func_mask		mask(mcuio_addr_func_bits)
#define MCUIO_FUNCS_PER_DEV		BIT(mcuio_addr_func_bits)

#define mcuio_addr_dev_bits		4
#define mcuio_addr_dev_shift		(mcuio_addr_func_bits + \
					 mcuio_addr_offset_bits)
#define mcuio_addr_dev_mask		mask(mcuio_addr_dev_bits)
#define MCUIO_DEVS_PER_BUS		BIT(mcuio_addr_dev_bits)

#define mcuio_addr_bus_bits		3
#define mcuio_addr_bus_shift		(mcuio_addr_func_bits + \
					 mcuio_addr_offset_bits + \
					 mcuio_addr_dev_bits)
#define mcuio_addr_bus_mask		mask(mcuio_addr_bus_bits)

#define mcuio_addr_type_bits		8
#define mcuio_addr_type_shift		(mcuio_addr_func_bits + \
					 mcuio_addr_offset_bits + \
					 mcuio_addr_dev_bits + \
					 mcuio_addr_bus_bits)
#define mcuio_addr_type_mask		mask(mcuio_addr_type_bits)

struct mcuio_packet {
	uint32_t addr;
	uint32_t data[2];
	uint16_t crc;
	uint16_t dummy;
} __attribute__((packed));

static inline unsigned mcuio_packet_type(struct mcuio_packet *p)
{
	return mcuio_ntohl(p->addr) >> mcuio_addr_type_shift;
}

static inline void mcuio_set_packet_type(struct mcuio_packet *p, uint32_t t)
{
	p->addr &= (mcuio_addr_type_mask << mcuio_addr_type_shift);
	p->addr |= mcuio_htonl(t) << mcuio_addr_type_shift;
}

static inline unsigned mcuio_data_size(unsigned t)
{
	if (t & mcuio_fill_data_bit)
		return sizeof(uint64_t);
	return (1 << ((t & ~1) >> 1));
}

static inline unsigned mcuio_packet_data_size(struct mcuio_packet *p)
{
	return mcuio_data_size(mcuio_packet_type(p));
}

static inline int mcuio_type_is_read(unsigned t)
{
	return !(t & 0x1);
}

static inline int mcuio_packet_is_read(struct mcuio_packet *p)
{
	return mcuio_type_is_read(mcuio_packet_type(p));
}

static inline int mcuio_packet_is_write(struct mcuio_packet *p)
{
	return !mcuio_type_is_read(mcuio_packet_type(p));
}

static inline int mcuio_packet_is_reply(struct mcuio_packet *p)
{
	return (mcuio_packet_type(p) & mcuio_reply_bit);
}

static inline int mcuio_packet_is_fill_data(struct mcuio_packet *p)
{
	return (mcuio_packet_type(p) & mcuio_fill_data_bit);
}

static inline unsigned mcuio_packet_offset(struct mcuio_packet *p)
{
	return (mcuio_ntohl(p->addr) >> mcuio_addr_offset_shift) &
		mcuio_addr_offset_mask;
}

static inline unsigned mcuio_packet_func(struct mcuio_packet *p)
{
	return (mcuio_ntohl(p->addr) >> mcuio_addr_func_shift) &
		mcuio_addr_func_mask;
}

static inline unsigned mcuio_packet_dev(struct mcuio_packet *p)
{
	return (mcuio_ntohl(p->addr) >> mcuio_addr_dev_shift) &
		mcuio_addr_dev_mask;
}

static inline unsigned mcuio_packet_bus(struct mcuio_packet *p)
{
	return (mcuio_ntohl(p->addr) >> mcuio_addr_bus_shift) &
		mcuio_addr_bus_mask;
}

static inline void mcuio_packet_set_addr(struct mcuio_packet *p,
					 unsigned bus, unsigned dev,
					 unsigned func, unsigned offset,
					 unsigned type, int fill)
{
	p->addr = mcuio_htonl(((offset & mcuio_addr_offset_mask) <<
			       mcuio_addr_offset_shift) |
			      ((func & mcuio_addr_func_mask) <<
			       mcuio_addr_func_shift) |
			      ((dev & mcuio_addr_dev_mask) <<
			       mcuio_addr_dev_shift) |
			      ((bus & mcuio_addr_bus_mask) <<
			       mcuio_addr_bus_shift) |
			      (((type & mcuio_addr_type_mask) |
				(fill ? mcuio_fill_data_bit : 0)) <<
			       mcuio_addr_type_shift));
}

static inline int mcuio_packet_is_reply_to(struct mcuio_packet *p,
					   struct mcuio_packet *request)
{
	unsigned t = mcuio_packet_type(p);
	unsigned rt = mcuio_packet_type(request);
	unsigned o = mcuio_packet_offset(p);
	unsigned ro = mcuio_packet_offset(request);
	unsigned f = mcuio_packet_func(p);
	unsigned rf = mcuio_packet_func(request);
	unsigned d = mcuio_packet_dev(p);
	unsigned rd = mcuio_packet_dev(request);
	unsigned b = mcuio_packet_bus(p);
	unsigned rb = mcuio_packet_bus(request);

	pr_debug("type = 0x%02x, request type = 0x%02x\n", t, rt);
	return (t & mcuio_actual_type_mask) == rt && o == ro && f == rf &&
		d == rd && b == rb;
}


static inline void mcuio_packet_set_reply(struct mcuio_packet *p)
{
	mcuio_set_packet_type(p, mcuio_packet_type(p) | mcuio_reply_bit);
}

static inline int mcuio_packet_is_error(struct mcuio_packet *p)
{
	return mcuio_packet_type(p) & mcuio_error_bit;
}

static inline void mcuio_packet_set_error(struct mcuio_packet *p)
{
	mcuio_set_packet_type(p, mcuio_packet_type(p) | mcuio_error_bit);
}

static inline void mcuio_packet_set_fill_data(struct mcuio_packet *p)
{
	mcuio_set_packet_type(p, mcuio_packet_type(p) | mcuio_fill_data_bit);
}

static inline const char *mcuio_packet_type_to_str(int t)
{
	switch(t & mcuio_actual_type_mask) {
	case mcuio_type_rdb:
		return "mcuio_type_rdb";
	case mcuio_type_wrb:
		return "mcuio_type_wrb";
	case mcuio_type_rdw:
		return "mcuio_type_rdw";
	case mcuio_type_wrw:
		return "mcuio_type_wrw";
	case mcuio_type_rdq:
		return "mcuio_type_rdq";
	case mcuio_type_wrq:
		return "mcuio_type_wrq";
	}
	return "unknown";
}

struct mcuio_func_descriptor {
	uint32_t device_vendor;
	uint32_t rev_class;
} __attribute__((packed));

static inline uint16_t mcuio_get_vendor(struct mcuio_func_descriptor *d)
{
	return mcuio_ntohs(mcuio_ntohl(d->device_vendor) & 0xffff);
}

static inline uint16_t mcuio_get_device(struct mcuio_func_descriptor *d)
{
	return mcuio_ntohs(mcuio_ntohl(d->device_vendor) >> 16);
}

static inline uint32_t mcuio_get_class(struct mcuio_func_descriptor *d)
{
	return mcuio_ntohl(d->rev_class) >> 8;
}

static inline uint32_t mcuio_get_rev(struct mcuio_func_descriptor *d)
{
	return mcuio_ntohl(d->rev_class) & ((1 << 8) - 1);
}

#endif /* __MCUIO_PROTO_H__ */
