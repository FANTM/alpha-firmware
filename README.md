# FANTM alpha Firmware

## Install
First time installation can be a little bit tricky; there are a lot of moving parts and the nRF5 SDK is complex. Hopefully these instructions can continue to improve throughout the lifetime of this repository. 

### Initial Setup
Because there are multiple directories and paths to manage, it is helpful to manually create one parent folder. This parent will henceforth be our base path and refered to as `fantm/`.

You can create it by running `mkdir fantm` and then enter the directory with `cd fantm`.

All of the Windows installation instructions were tested with Powershell, *not* CMD.

### Downloading Source
Next we need to gather the SDK and the alpha firmware. Some of these downloads might take a while because the SDK is large. We'll do this by running the following:
 1. `git clone https://github.com/FANTM/nRF5-SDK`  
 2. `git clone https://github.com/FANTM/alpha-firmware`

### Install SDK
We've made a snapshot of the SDK version readily available for ease of install and versioning. Download and install it by running:
 
```
git clone https://github.com/FANTM/nRF5-SDK
cd nRF5-SDK
unzip nRF5SDK1702d674dde.zip
ls nRF5_SDK_17.0.2_d674dde    # If this doesn't exit successfully, something went wrong.
cd ..
```

Congratulations, you should now have the SDK installed.

### Install FANTM alpha Firmware
Now that the SDK is ready, we should get the firmware. The one wrinkle in this process is the integration with the SDK which will be elaborated on in the code snippet below:

```
git clone https://github.com/FANTM/alpha-firmware
cd alpha-firmware
# This is the tricky bit, it will fail on Windows, so if that's your environment skip to the end of this code block.
cp -r ./nRF5_SDK_17.0.2_d674dde ../nRF5-SDK/  # This overlays our plugins into the SDK.
cd ..
```

Windows does not appreciate this little trick, so you have to be a bit more manual with it. 

```
# Make sure you're in the firmware directory
cp .\nRF5_SDK_17.0.2_d674dde\components\boards\boards.h ..\nRF5_SDK_17.0.2_d674dde\components\boards\boards.h
cp .\nRF5_SDK_17.0.2_d674dde\components\boards\fantm-alpha.h ..\nRF5_SDK\nRF5_SDK_17.0.2_d674dde\components\boards\
```

### Final Configuration
Steps with Makefile to get variables set up, TODO

## Build
### Firmware

### Telemetry
Running `make` will build the entire project. If you update the packet schema, make sure to run `make update-schema` to update the C telemetry API.

## Snapshot
Each push to master should include the copying of your binary (dfu-package.zip) into the ${PRJ}/bin folder. This is dual purpose: first it ensures the firmware compiles prior to it becoming the new master, and second it gives the next user a comparable for their build in case they have trouble building or just need to flash quickly.
