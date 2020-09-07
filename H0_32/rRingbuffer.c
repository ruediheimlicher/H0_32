//
//  rRingbuffer.c
//  Robot_32
//
//  Created by Ruedi Heimlicher on 30.07.2019.
//  Copyright © 2019 Ruedi Heimlicher. All rights reserved.
//

#include "rRingbuffer.h"
 // https://www.mikrocontroller.net/articles/FIFO


//
// Stellt 1 Byte in den Ringbuffer
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist voll. Es kann kein weiteres Byte gespeichert werden
//     BUFFER_SUCCESS    das Byte wurde gespeichert
//
uint8_t BufferIn(uint8_t byte)
{
   uint8_t next = ((ringbuffer.write + 1) & BUFFER_MASK);
   
   if (ringbuffer.read == next)
      return BUFFER_FULL; // voll
   
   ringbuffer.data[ringbuffer.write] = byte;
   // buffer.data[buffer.write & BUFFER_MASK] = byte; // absolut Sicher
   ringbuffer.write = next;
   
   return BUFFER_SUCCESS;
}

uint8_t ringbufferIn(struct robotposition pos)
{
   uint8_t next = ((ringbuffer.write + 1) & BUFFER_MASK);
   
   if (ringbuffer.read == next)
      return BUFFER_FULL; // voll
   
   ringbuffer.position[ringbuffer.write & BUFFER_MASK] = pos;
   
   // buffer.data[buffer.write & BUFFER_MASK] = byte; // absolut Sicher
   ringbuffer.write = next;
   
   return BUFFER_SUCCESS;
}


//
// Holt 1 Byte aus dem Ringbuffer, sofern mindestens eines abholbereit ist
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist leer. Es kann kein Byte geliefert werden.
//     BUFFER_SUCCESS    1 Byte wurde geliefert
//
uint8_t BufferOut(uint8_t *pByte)
{
   if (ringbuffer.read == ringbuffer.write)
      return BUFFER_FAIL;
   
   *pByte = ringbuffer.data[ringbuffer.read];
   
   ringbuffer.read = (ringbuffer.read+1) & BUFFER_MASK;
   
   return BUFFER_SUCCESS;
}

uint8_t ringbufferOut(struct robotposition *pos)
{
   if (ringbuffer.read == ringbuffer.write)
      return BUFFER_EMPTY;
   
   *pos = ringbuffer.position[ringbuffer.read];
   
   ringbuffer.read = (ringbuffer.read+1) & BUFFER_MASK;
   
   return BUFFER_SUCCESS;
}

uint16_t ringbufferCount(void)
{
   size_t n = sizeof(ringbuffer.position)/sizeof(ringbuffer.position[0]);
   return ringbuffer.write;
}

uint8_t ringbufferEmpty(void)
{
   if (ringbuffer.read == ringbuffer.write)
      return 1;
   else
      return 0;
}

uint8_t ringbufferFull(void)
{
   if ( ( ringbuffer.write + 1 == ringbuffer.read ) ||
       ( ringbuffer.read == 0 && ringbuffer.write + 1 == BUFFER_SIZE ) )
      return 1;
   else
      return 0;
}
 
uint8_t ringbufferClear(void)
{
   ringbuffer.read = 0;
   ringbuffer.write = 0;
   

}

