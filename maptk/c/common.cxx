/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Common C Interface Utilities Implementation
 */

#include "common.h"

#include <cstdlib>
#include <cstring>


/// Allocate a new maptk string structure
maptk_string_t* maptk_string_new(size_t length, char const* s)
{
  maptk_string_t* n =
    (maptk_string_t*)malloc(sizeof(maptk_string_t));
  n->length = length;
  // When length 0, this is just a 1 character string that is just the null
  // byte.
  n->str = (char*)malloc(sizeof(char) * (length+1));
  n->str[length] = 0;

  if( length && s )
  {
    strncpy( n->str, s, length );
  }
  return n;
}


/// Free an alocated string structure
void maptk_string_free( maptk_string_t *s )
{
  free(s->str);
  free(s);
}


/// Common function for freeing string lists
void maptk_common_free_string_list( size_t length,
                                    char **keys )
{
  for( unsigned int i = 0; i < length; i++ )
  {
    free(keys[i]);
  }
  free(keys);
}


void maptk_free_pointer( void *thing )
{
  if( thing )
  {
    free(thing);
  }
}


void maptk_free_double_pointer( size_t length, void **things )
{
  if( things )
  {
    for( size_t i=0; i < length; i++ )
    {
      if( things[i] )
      {
        free(things[i]);
      }
    }
    free(things);
  }
}
