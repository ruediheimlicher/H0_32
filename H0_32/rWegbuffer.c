//
//  rWegbuffer.c
//  Robot_32
//
//  Created by Ruedi Heimlicher on 30.07.2019.
//  Copyright © 2019 Ruedi Heimlicher. All rights reserved.
//

#include "rWegbuffer.h"
 // https://www.mikrocontroller.net/articles/FIFO


//
// Stellt 1 Byte in den Wegbuffer
//
// Returns:
//     BUFFER_FAIL       der Wegbuffer ist voll. Es kann kein weiteres Byte gespeichert werden
//     BUFFER_SUCCESS    das Byte wurde gespeichert
//

uint8_t wegbufferIn(struct wegposition pos)
{
   uint8_t next = ((wegbuffer.write + 1) & BUFFER_MASK);
   
   if (wegbuffer.read == next)
      return BUFFER_FULL; // voll
   
   wegbuffer.position[wegbuffer.write & BUFFER_MASK] = pos;
   
   // buffer.data[buffer.write & BUFFER_MASK] = byte; // absolut Sicher
   wegbuffer.write = next;
   
   return BUFFER_SUCCESS;
}


//
// Holt 1 Byte aus dem Wegbuffer, sofern mindestens eines abholbereit ist
//
// Returns:
//     BUFFER_FAIL       der Wegbuffer ist leer. Es kann kein Byte geliefert werden.
//     BUFFER_SUCCESS    1 Byte wurde geliefert
//

uint8_t wegbufferOut(struct wegposition *pos)
{
   if (wegbuffer.read == wegbuffer.write)
      return BUFFER_EMPTY;
   
   *pos = wegbuffer.position[wegbuffer.read];
   
   wegbuffer.read = (wegbuffer.read+1) & BUFFER_MASK;
   
   return BUFFER_SUCCESS;
}

uint16_t wegbufferCount(void)
{
   size_t n = sizeof(wegbuffer.position)/sizeof(wegbuffer.position[0]);
   return wegbuffer.write;
}

uint8_t wegbufferEmpty(void)
{
   if (wegbuffer.read == wegbuffer.write)
      return 1;
   else
      return 0;
}

uint8_t wegbufferFull(void)
{
   if ( ( wegbuffer.write + 1 == wegbuffer.read ) ||
       ( wegbuffer.read == 0 && wegbuffer.write + 1 == BUFFER_SIZE ) )
      return 1;
   else
      return 0;
}
 
uint8_t wegbufferClear(void)
{
   wegbuffer.read = 0;
   wegbuffer.write = 0;
   

}

