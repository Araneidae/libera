// $Id: pool.c,v 1.8 2006/01/12 08:01:51 miha Exp $

//! \file pool.c
//! Implements memory pool for data-on-demand requests.

/*
CSPI - Control System Programming Interface
Copyright (C) 2004-2006 Instrumentation Technologies

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
or visit http://www.gnu.org
*/

/* TAB = 4 spaces. */

#include <stdlib.h>
#include <pthread.h>

#include "pool.h"

/** Private. Typedef representing a single byte. */
typedef unsigned char byte;

/* forward decl */
typedef struct Pool_object_t Pool_object;

/** Private.
 *  Represents a linked list of memory objects that constitute a pool.
 */
struct Pool_object_t {

    /** A memory block to use for allocations. */
	byte _unused[ CHUNK_SIZE ];

    /** A pointer to the next memory block. */
	Pool_object *next;
};

/** Private.
 *  Represents a memory pool of fixed-size objects (blocks).
 */
typedef struct {

	/** Protect from concurrent modifications. */
	pthread_mutex_t mutex;

	/** A pointer to the head of the free list. */
	Pool_object *head;
} Pool;

/** Instantiate one and only memory pool. */
Pool pool = { PTHREAD_MUTEX_INITIALIZER, 0, };

//--------------------------------------------------------------------------

void* pool_malloc( size_t size )
{
	if ( !size || size > CHUNK_SIZE ) return malloc( size );

	pthread_mutex_lock( &pool.mutex );
	Pool_object *p = pool.head;

	if ( p ) {

		pool.head = p->next;
	}
	else {

		// Allocate a block of memory to hold OBJECT_COUNT objects.
		Pool_object *new =
			(Pool_object *) malloc( OBJECT_COUNT * sizeof(Pool_object) );

		p = new;
		if ( !new ) goto cleanup;

		// Link objects together.
		// Skip the first, because we will return that to the caller.
		for (int i=1; i < OBJECT_COUNT-1; ++i) new[i].next = &new[i + 1];

		// Terminate the linked list.
		new[OBJECT_COUNT - 1].next = 0;

		// First object is returned to the caller. Set the free list head
		// to an object immediately following that one.
		pool.head = &new[1];
	}

cleanup:
	pthread_mutex_unlock( &pool.mutex );
	return p;
}

//--------------------------------------------------------------------------

void pool_free( void *p, size_t size )
{
	if ( !p ) return;

	if ( !size || size > CHUNK_SIZE ) {

		free( p );
		return;
	}

	Pool_object *obj =  (Pool_object *) p;
	pthread_mutex_lock( &pool.mutex );

	obj->next = pool.head;
	pool.head = obj;

	pthread_mutex_unlock( &pool.mutex );
}
