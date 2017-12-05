/*
 * audio_Protocol.h
 *
 *  Created on: Dec 5, 2016
 *      Author: Frederic St-Pierre
 */

#ifndef AUDIO_PROTOCOL_H_
#define AUDIO_PROTOCOL_H_

#include "link_Comm.h"
#include "cobs.h"

void (*protocol_function_ptrs[]) (void *);

#endif /* AUDIO_PROTOCOL_H_ */
