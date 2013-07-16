#!/bin/sh
echo /opt/tilera/TileraMDE-3.0.1.125620/tilepro/bin/tile-monitor --pci --hvc my.hvc --tile 8x8 --batch-mode --mkdir /opt/test --cd /opt/test --upload life life --here --hvx dataplane=0-61 -- life $@
/opt/tilera/TileraMDE-3.0.1.125620/tilepro/bin/tile-monitor --pci --hvc my.hvc --tile 8x8 --batch-mode --mkdir /opt/test --cd /opt/test --upload life life --here --hvx dataplane=0-61 -- life $@
