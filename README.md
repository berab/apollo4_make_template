# Makefile Template for Apollo4 devices using JLink GDB Server and gdb-multiarch 
Device: Apollo4 Blue Lite EVB
## Make:
```bash
make clean all
```

## RUN JLink GDB Server:
```bash
JLinkGDBServerCLExe -singlerun -nogui -port 61234 -device AMAP42KL-KBR
```

## Connect GDB to the Server (.gdbinit is autocreated):
```bash
gdb-multiarch
```
