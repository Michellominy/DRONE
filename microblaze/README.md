## Objective

The goal of this setup is to:

1. Develop and compile a MicroBlaze firmware (`pid_mb.c`) for the PYNQ-Z2 board.
2. Deploy the compiled firmware binary (`pid_mb.bin`) into the PYNQ library directory.
3. Load and use the firmware from Python, controlling the PMOD peripherals for drone motor actuation.


## Step-by-Step Instructions

### 1. Place your firmware source
Copy or create your `pid_mb.c` in the same directory as `setup_pid_mb.sh` (e.g., inside your Jupyter project folder).

### 2. Run the setup script
From the PYNQ terminal:
```bash
./setup_pid_mb.sh
cd /home/xilinx/pynq/lib/pmod/pid_mb/Debug
make clean
make
```

This script will:

* Create the destination project directory under cd /home/xilinx/pynq/lib/pmod/pid_mb

* Copy all necessary files (pid_mb.c, linker script, makefiles)

* Compile the MicroBlaze firmware

If successful, it will generate:

* /home/xilinx/pynq/lib/pmod/pid_mb/Debug/pid_mb.elf
* /home/xilinx/pynq/lib/pmod/pid_mb.bin

