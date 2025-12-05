
# **Pipelined CPU with Forwarding**

## Description

This repository contains a hardware implementation of a **pipelined CPU** that supports **data hazard resolution using forwarding**. The design follows the 5-stage pipeline model:  
**Instruction Fetch (IF)** → **Instruction Decode (ID)** → **Execute (EX)** → **Memory (MEM)** → **Writeback (WB)**.

The architecture includes pipeline registers between each stage and incorporates hazard detection and a data forwarding unit so that instructions can proceed without stalls. Forwarding is used to send data from later stages to earlier stages when necessary, increasing pipeline throughput and decreasing stalls.

---
## Technologies

This project was developed using:

- Verilog
- Xilinx Vivado
- Standard CPU datapath design methodologies
- Waveform inspection for validation  
---
## Features

- Implements a **5-stage pipelined CPU**  
- Includes **hazard detection**  
- Uses **data forwarding** to resolve RAW hazards  
- Supports basic instruction types such as R-type, load, store, and branch  
- Simulation waveform demonstrates correct operation 
- Increased throughput and decreased stalls compared to non-pipelined designs

---
## The Process

1. **Datapath Design** — Built a 5-stage pipeline datapath with pipeline registers.
	
2. **Control Logic** — Implemented control signals routed to each of the 5 stages.
    
3. **Forwarding Unit** — Added logic to detect when data from later stages should be forwarded to the EX stage inputs.
    
4. **Hazard Detection Unit** — Created logic to stop load-use hazards by inserting stalls when forwarding cannot resolve them.
    
5. **Simulation Testing** — Ran test instructions and tested waveform results to identify errors.
    

---
## What I Learned

- How pipelining improves CPU throughput by processing multiple instructions at once
    
- How **data hazards** occur and how they affect the cycle time of an execution
    
- How **forwarding** and **hazard detection units** work to minimize pipeline stalls and execution time
    
- Practical experience with simulation tools, verilog programming, and waveform analysis
    

---
## How It Can Be Improved

-  **Extend Instruction support** – Add more instruction types and support for additional operations (ex. subtraction)
-  **Branch Prediction** – Implement basic static or dynamic prediction to reduce cycle time
- **Synthesis on Hardware** – Optimize and implement the design for FPGA hardware

---
## How to Run the Project

1. Open the project in Xilinx Vivado (PipelinedCPU → PipelinedCPU.xpr)
2. Click Run Simulation
3. Maximize Simulation window, set the timescale accordingly, and step to first cycle.

---
## Waveform Example
![[Pasted image 20251205131055.png]]
Figure 1: Waveform of example instructions in MIPS:
```mips
25: add $3, $1, $2
26: sub $4, $9, $3
27: or  $5, $3, $9
28: xor $6, $3, $9
29: and $7, $3, $9
```

