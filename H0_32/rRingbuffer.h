//
//  rRingbuffer.h
//  Robot_32
//
//  Created by Ruedi Heimlicher on 30.07.2019.
//  Copyright © 2019 Ruedi Heimlicher. All rights reserved.
//

#ifndef rRingbuffer_h
#define rRingbuffer_h

#include <stdio.h>

#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1
#define BUFFER_FULL  2
#define BUFFER_EMPTY 3
#define END_TASK  6

#define BUFFER_SIZE 1024 // muss 2^n betragen (8, 16, 32, 64 ...)
#define BUFFER_MASK (BUFFER_SIZE-1) // Klammern auf keinen Fall vergessen



struct robotposition
{
   uint16_t x;
   uint16_t y;
   uint16_t z;
   uint16_t hyp;
   uint16_t steps;
   uint16_t index;
   uint8_t task;
};

struct robotposition r;

struct Buffer {
   struct  robotposition position[BUFFER_SIZE];
   uint8_t data[BUFFER_SIZE];
   uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
   uint8_t write; // zeigt immer auf leeres Feld
} ringbuffer;



extern uint8_t BufferIn(uint8_t byte);
extern uint8_t BufferOut(uint8_t *pByte);

extern uint8_t ringbufferIn(struct robotposition pos);
extern uint8_t ringbufferOut(struct robotposition *pos);


#endif /* rRingbuffer_h */
