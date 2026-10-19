#!/bin/sh
# Build TcmKbd.efi (resident boot-service driver) for AArch64 UEFI.
# Needs: clang and lld-link (apt install clang lld).
set -e
clang --target=aarch64-unknown-windows -ffreestanding -fno-builtin -fshort-wchar \
      -fno-stack-protector -mgeneral-regs-only -O2 -Wall -Wextra -c src/TcmKbd.c -o TcmKbd.obj
lld-link /subsystem:efi_boot_service_driver /entry:efi_main /nodefaultlib \
         /machine:arm64 /dll /base:0 /align:4096 TcmKbd.obj /out:TcmKbd.efi
ls -l TcmKbd.efi
