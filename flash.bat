ECHO OFF
SET param1=%1
ECHO %param1%

nrfjprog --recover

nrfjprog --eraseall
nrfjprog --memwr 0x10001080 --val %param1%
nrfjprog --program build\zephyr\merged.hex --verify
nrfjprog --reset