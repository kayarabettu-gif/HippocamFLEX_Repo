# Create pre-emption script that resets DMA before ZOCL touches it
cat > /home/petalinux/reset_dma.sh << 'EOF'
#!/bin/sh
# Reset both DMA channels to clean state
devmem 0xA0000000 32 0x4   # MM2S reset
sleep 0.1
devmem 0xA0000000 32 0x1   # MM2S RS=1
devmem 0xA0000030 32 0x4   # S2MM reset  
sleep 0.1
devmem 0xA0000030 32 0x1   # S2MM RS=1
echo "DMA reset complete"
devmem 0xA0000004 32       # verify MM2S DMASR
devmem 0xA0000034 32       # verify S2MM DMASR
EOF
chmod +x /home/petalinux/reset_dma.sh

# Unmask and reboot
sudo systemctl unmask zocl
sudo reboot