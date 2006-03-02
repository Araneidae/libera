// $Id: pool.h,v 1.5 2005/11/13 21:33:19 miha Exp $

//! \file pool.h
//! Declares memory pool interface for Data On Demand requests.

#if !defined(_POOL_H)
#define _POOL_H

#ifdef __cplusplus
extern "C" {
#endif

/** The number of memory blocks (objects) to allocate in advance.
 *  Memory will be allocated to accomodate OBJECT_COUNT requests
 *  with a single malloc call.
 */
#define OBJECT_COUNT	4

/** The size of a block in the memory pool.
 *  This is also the largest block that can be allocated on the pool.
 *  Requests of 'wrong' size are sent to standard malloc.
 */
#define CHUNK_SIZE	( 10240 * 8 * 4 )	// 10K samples, 32 bytes each

//--------------------------------------------------------------------------
// Public interface.

/** Private.
 *  Allocates from a memory pool. Requests of size larger than CHUNK_SIZE
 *  are routed to standard malloc.
 *  Returns a pointer to allocated memory or 0 if request fails.
 *  @param size The Number of bytes to allocate.
 */
void* pool_malloc( size_t size );

/** Private.
 *  Frees memory allocated with pool_malloc.
 *  @param p    A pointer to memory block allocated with pool_malloc.
 *  @param size The size of memory block in bytes.
 */
void pool_free( void *p, size_t size );

#ifdef __cplusplus
}
#endif
#endif	// _POOL_H
