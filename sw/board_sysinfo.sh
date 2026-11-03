#!/bin/bash
OUT=/tmp/hippocampex_sysinfo.txt
exec > $OUT 2>&1

echo "=== BOOT.SCR ===" 
dtc -I dtb -O dts /boot/boot.scr 2>/dev/null || strings /boot/boot.scr

echo "=== SYSTEM.DTB (memory + chosen nodes) ==="
dtc -I dtb -O dts /boot/system.dtb 2>/dev/null | grep -A5 -E "memory@|chosen|bootargs|reserved-memory|cma"

echo "=== DEVICETREE MEMORY REG ==="
cat /sys/firmware/devicetree/base/memory@0/reg 2>/dev/null | xxd

echo "=== CHOSEN / BOOTARGS ==="
cat /sys/firmware/devicetree/base/chosen/bootargs 2>/dev/null

echo "=== DMA HEAP ==="
ls -la /dev/dma_heap/

echo "=== MEMINFO ==="
cat /proc/meminfo | grep -iE "cma|mem|huge"

echo "=== CMDLINE ==="
cat /proc/cmdline

echo "=== DONE: $OUT ==="