//
// Copyright 2022 Ettus Research, A National Instruments Company
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: radio_ctrlport_regmap_utils.vh
// Description:
// The constants in this file are autogenerated by XmlParse.

//===============================================================================
// A numerically ordered list of registers and their HDL source files
//===============================================================================

  // DB_WINDOW          : 0x0 (rfdc_timing_control.v)
  // RFDC_TIMING_WINDOW : 0x8000 (rfdc_timing_control.v)

//===============================================================================
// RegTypes
//===============================================================================

//===============================================================================
// Register Group RADIO_CTRLPORT_WINDOWS
//===============================================================================

  // DB_WINDOW Window (from rfdc_timing_control.v)
  localparam DB_WINDOW = 'h0; // Window Offset
  localparam DB_WINDOW_SIZE = 'h8000;  // size in bytes

  // RFDC_TIMING_WINDOW Window (from rfdc_timing_control.v)
  localparam RFDC_TIMING_WINDOW = 'h8000; // Window Offset
  localparam RFDC_TIMING_WINDOW_SIZE = 'h8000;  // size in bytes
