// SPDX-License-Identifier: MIT
// Copyright (c) 2024-2026 Alex Conesa
// See LICENSE file for details.

#pragma once
#ifndef __cplusplus
#error UngulaEncoder requires a C++ compiler
#endif

// Ungula Encoder Library — magnetic / optical encoder drivers.
//
// Depend on UngulaCore and UngulaHal. Including those umbrellas first
// ensures the Arduino CLI discovers their include paths before our
// headers reach the compiler.
#include <ungula/core.h>
#include <ungula/hal.h>

// Chip-neutral interface — implemented by every concrete driver.
#include "ungula/encoder/i_encoder.h"
