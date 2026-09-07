# Studica VMX USB deployment kit

## Start here

Copy this entire **STUDICA_VMX_USB** folder to your flash drive. Keep
`payload.tar.gz` compressed: it contains the robot program, configuration, maps,
Orbbec driver/SDK, YDLidar ROS driver, and YDLidar SDK source. The archive preserves
Linux permissions and symlinks even on FAT32/exFAT flash drives.

On the target Raspberry Pi / VMX:

1. Boot the Studica **Ubuntu 22.04 ARM64** image and log in as **vmx**.
2. Connect to the internet and insert the flash drive.
3. Open this folder in the file manager, right-click → **Open in Terminal**.
4. Run this single command:

   ```bash
   bash INSTALL.sh
   ```

Enter the `vmx` sudo password when requested. Do **not** use `sudo bash INSTALL.sh`.
Sudo may ask again during a long build. Keep the VMX powered and the terminal open
until **INSTALL COMPLETE** appears. Installation/build time depends on the SD card,
network, and packages already installed; allow substantial time before competition.

You can also run from any terminal using the actual USB path, quoted if it contains spaces:

```bash
bash "/media/vmx/YOUR_USB_NAME/STUDICA_VMX_USB/INSTALL.sh"
```

After completion, reboot or log out and back in, then reconnect the camera.
The robot does not start or move automatically. Running the program is a separate
step after checking the robot configuration.

## Requirements and scope

- Studica Ubuntu 22.04, ARM64 (`aarch64`), user/home `vmx` / `/home/vmx`.
- Working Studica VMX HAL installed by the matching Studica image/SDK.
- Sudo access and at least **8 GiB free** on the SD card; more may be needed for
  missing packages. A stable power supply is required during compilation.
- Internet access from the VMX to Ubuntu, ROS repositories, GitHub and rosdep.
  **This is an online installer carried on USB, not a fully offline package mirror.**
- Stop existing robot/camera processes before installation. Do not install while
  another build, apt installation, or software update is running.

The installer rejects other OS versions/architectures and missing VMX HAL before
modifying the system. It does not flash an OS, upgrade the distribution, install
VMX firmware, or copy this machine's proprietary HAL binaries. If the image differs,
check the [Studica image documentation](https://learn.studica.com/docs/ws/vmx/os-images).

To check host compatibility and package integrity without installation:

```bash
bash INSTALL.sh --check
```

This check does not test internet availability, installed ROS dependencies, or hardware.

## What is automatic

1. Verify bundled files, supported OS/user, VMX HAL presence, and free disk space.
2. Install build tools; configure the official ROS apt source if absent; install
   Humble, ROS/Python vision dependencies, navigation, Foxglove, RViz, and gamepad support.
3. Resolve source dependencies with rosdep.
4. Move existing `studica_ws`, `ydlidar_ros2_ws`, and `YDLidar-SDK` into a dated
   backup, then extract fresh project copies owned by `vmx` into `/home/vmx`.
5. Build Studica drivers, YDLidar SDK, YDLidar ROS driver, Orbbec, and studica_control
   with one build worker to limit RAM use.
6. Configure library lookup, a sudoers entry for the hardware executable, Orbbec
   udev rules, I2C access, and video/dialout group membership. The hardware executable
   runs as root because the current VMX HAL/pigpio integration requires it.
7. Enable SSH, repair ownership of an existing VS Code Server folder, and add a
   managed ROS environment block to `.bashrc`.
8. Check ROS package discovery, Python imports, and shared library resolution.

Workspace backups: `/home/vmx/studica-backups/DATE-TIME-PID/`.
Installation logs: `/home/vmx/studica-deploy-logs/`.
The backup also contains `.bashrc` and any replaced managed system configuration.
A staging folder `/home/vmx/.studica-deploy-stage.*` is retained for diagnosis.

Rerunning the same command creates another backup and fresh build. It does not merge
local edits from the old workspace: retrieve any desired changes from its backup.
System apt packages and vendor libraries are shared across installations; workspace
backup is not a complete OS rollback. There is no automatic rollback after failure.

## Connect VS Code from your laptop

Install **VS Code** and Microsoft's **Remote - SSH** extension on the laptop.
On VMX run `hostname -I`, then on the laptop test `ssh vmx@VMX_IP`.
In VS Code select **Remote-SSH: Connect to Host**, enter `vmx@VMX_IP`, and open
`/home/vmx/studica_ws`.

VS Code Server is installed automatically by the laptop's Remote SSH extension on
first connection, using the version matching that laptop. This kit prepares SSH but
cannot preselect the server version for an unknown laptop. See the
[official Remote SSH documentation](https://code.visualstudio.com/docs/remote/ssh).
If only the laptop has internet, set this in the laptop's VS Code settings:

```json
"remote.SSH.localServerDownload": "always"
```

Connect at least once before going offline; updating VS Code may require a new server.

## Test after reboot

### Camera first (no motor launch)

Open a terminal:

```bash
cd /home/vmx/studica_ws
lsusb
ros2 run orbbec_camera list_devices_node
bash scripts/start_orbbec_camera.sh
```

In another terminal:

```bash
ros2 topic list
```

Check the image/depth topic names that actually appear with `ros2 topic hz TOPIC_NAME`.
Stop with Ctrl+C. This snapshot targets **Orbbec Gemini E**, driver 1.5.22 and bundled
ARM64 SDK 1.10.37. A different camera may require different launch/configuration.
OpenCV windows need a graphical desktop; Remote SSH alone does not provide a display.

### Robot control

Check Titan CAN IDs, motor direction, servo ports, wheel/encoder calibration and YAML
settings under `src/studica_control/config/` against the target robot. Lift the wheels
for the first motor test and keep a stop method available. Then, in a terminal:

```bash
cd /home/vmx/studica_ws
bash scripts/start_full_keyboard.sh
```

See `studica_ws/docs/KEYBOARD_REMOTE_GUIDE.md` after extracting the payload, or
`/home/vmx/studica_ws/docs/KEYBOARD_REMOTE_GUIDE.md` after installation, for controls.
Robot operation still requires hardware testing; installation success alone does not
prove camera streaming, CAN communications, motor direction, or navigation correctness.

### LiDAR / mapping

YDLidar source is included and built automatically. The current `Tmini.yaml` selects
`/dev/ttyUSB0` at 230400 baud. Check `ls -l /dev/serial/by-id/` and adjust the port in
`/home/vmx/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml` if needed.
The installer does not guess which USB serial device is your LiDAR. Check your model
before using the included parameters. Then use the mapping/navigation instructions
in the project README. Existing maps are supplied as a snapshot and may not match
another venue or robot.

## If installation fails

Read the last error and the log path printed by the installer. Fix the reported
cause and rerun `bash INSTALL.sh`. A partially installed workspace is backed up on
the next run, just like an existing workspace.

| Error | Next action |
|---|---|
| Unsupported OS/user | Use Studica Ubuntu 22.04 ARM64 and log in as vmx. |
| VMX HAL missing | Restore the SDK/image matching your board; the kit does not supply HAL. |
| Checksum failure | Copy the entire original kit again; do not edit payload.tar.gz. |
| apt / DNS / download failure | Check internet, system date, repository keys, and the error; rerun. |
| apt wants removals/conflicting packages | Resolve the image's package conflict; initial apt steps refuse removals. |
| Sudo password requested | Enter the vmx password in the installer terminal. |
| Permission denied on USB | Use `bash INSTALL.sh`; do not execute `./INSTALL.sh` directly. |
| Build killed / out of space | Free SD space or close memory-heavy programs, then rerun. |
| Camera access denied | Reboot/log in again, reconnect camera, check video group and udev. |
| Camera not in lsusb | Check cable/data connection and camera power. |
| VS Code cannot connect | Verify ordinary SSH first; inspect Remote SSH Output on the laptop. |

For manual workspace recovery, stop robot processes, rename the failed new workspace,
and move the corresponding backup back to its original `/home/vmx/...` path. Review
system-library/configuration changes separately; restoring folders alone does not
undo apt or SDK installation.

## Package contents and provenance

- `INSTALL.sh`: installer and read-only `--check` mode.
- `payload.tar.gz`: project source/config/maps plus Orbbec and YDLidar sources.
- `MANIFEST.json`: snapshot date, source revisions and archive inventory count.
- `SHA256SUMS`: integrity checks for payload, installer, README and manifest.
- `README.md`: this guide.

The bundled application is the current working-tree snapshot, including local
modifications, not just the upstream Git commit. Existing application documentation
and comments retain their original languages; this deployment guide and installer
are in English. Vendor license files are retained in the payload.

ROS repository bootstrap follows the [official Humble repository setup](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst).
Package versions downloaded by apt may change; this kit does not lock all system packages.
