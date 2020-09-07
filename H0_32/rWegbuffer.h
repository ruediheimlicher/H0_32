//
//  rRingbuffer.h
//  Robot_32
//
//  Created by Ruedi Heimlicher on 30.07.2019.
//  Copyright © 2019 Ruedi Heimlicher. All rights reserved.
//

#ifndef rWegbuffer_h
#define rWegbuffer_h

#include <stdio.h>

#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1
#define BUFFER_FULL  2
#define BUFFER_EMPTY 3
#define END_TASK  6

#define BUFFER_SIZE 1024 // muss 2^n betragen (8, 16, 32, 64 ...)
#define BUFFER_MASK (BUFFER_SIZE-1) // Klammern auf keinen Fall vergessen



struct wegposition
{
   uint16_t rotwinkel;
   uint16_t winkel1;
   uint16_t winkel2;
   uint16_t index;
   uint16_t steps;
   uint8_t task;
   
};

struct wegposition w;

struct buffer {
   struct  wegposition position[BUFFER_SIZE];
   uint8_t data[BUFFER_SIZE];
   uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
   uint8_t write; // zeigt immer auf leeres Feld
} wegbuffer;




extern uint8_t wegbufferIn(struct wegposition pos);
extern uint8_t wegbufferOut(struct wegposition *pos);


#endif /* rRingbuffer_h */
