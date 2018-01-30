/* mbed Microcontroller Library
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MBED_MBED_RTX_H
#define MBED_MBED_RTX_H

#include <sys/types.h>

extern caddr_t Heap_Bank1_Start;
extern caddr_t Heap_Bank1_End;

#define HEAP_START      ((unsigned char *)&Heap_Bank1_Start)
#define HEAP_SIZE       (&Heap_Bank1_End - &Heap_Bank1_Start)

#ifdef TARGET_XMC4500_RELAXKIT

#ifndef OS_TASKCNT
#define OS_TASKCNT              10
#endif

#ifndef OS_MAINSTKSIZE
#define OS_MAINSTKSIZE          512
#endif

#ifndef OS_CLOCK
#define OS_CLOCK                120000000
#endif

#endif

#endif  // MBED_MBED_RTX_H
