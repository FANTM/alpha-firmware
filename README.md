# FANTM alpha Firmware

## Build
Running `make` will build the entire project. If you update the packet schema, make sure to run `make update-schema` to update the C telemetry API.

## Snapshot
Each push to master should include the copying of your binary (dfu-package.zip) into the ${PRJ}/bin folder. This is dual purpose: first it ensures the firmware compiles prior to it becoming the new master, and second it gives the next user a comparable for their build in case they have trouble building or just need to flash quickly.