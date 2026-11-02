# HippocamFLEX

FPGA prototype of a hippocampus-inspired architecture for context-aware sequence prediction, written in SystemVerilog and tested on a ZCU102 (Zynq UltraScale+ XCZU9EG) under PetaLinux. A three-stage pipeline recognises the event and context (DG/EC), predicts the next two events (CA3), and ranks them into timestamps (CA1). The design holds 64 events across 16 contexts and runs at 100 MHz.

- `hw/`: RTL, testbenches and constraints (Vivado 2023.2)
- `sw/`: board test driver and scripts
- `demo/`: card-recognition demo with a TCP server on the board and vision code on the laptop
- `docs/`: basic documents
