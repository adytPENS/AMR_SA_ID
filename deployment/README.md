# USB deployment packaging

Copy the generated **[STUDICA_VMX_USB](STUDICA_VMX_USB/README.md)** folder to a flash drive.
On the target Studica Ubuntu 22.04 ARM64 VMX, log in as `vmx`, open that folder in a
terminal and run `bash INSTALL.sh`. Internet is required for system dependencies.

The installer is supplied for the target machine; generating this kit does not run
it or modify the development machine's system configuration.

To refresh the source snapshot after changing the program, run from this workspace:

```bash
python3 scripts/create_usb_deployment.py
bash deployment/STUDICA_VMX_USB/INSTALL.sh --check
```

The packager also requires `/home/vmx/YDLidar-SDK` and
`/home/vmx/ydlidar_ros2_ws/src`. Edit `templates/INSTALL.sh` and
`templates/README.md`, then regenerate; do not edit generated kit files directly
because its integrity hashes would no longer match.

Validation performed on the development host is not a substitute for a full
installation and hardware acceptance test on a spare VMX with the intended image.
