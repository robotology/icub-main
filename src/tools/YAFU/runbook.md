# YAFU — User Runbook (concise, actionable)

This runbook describes two ways to update Ethernet boards with YAFU:

- Method A — Parallel orchestrator (recommended for fleets): run `parallel.sh` which discovers IPs from a network XML and performs a 3‑phase update (prepare → program → restart) in parallel.
- Method B — Stepwise control: use `program.sh` to prepare and program explicit IPs, then `restart.sh` to bring them back to application mode. 

!!! Note: 

    Method B is a forced, user-driven workflow — it does not perform automatic firmware-version pre‑checks across targets. Use this when you need explicit control or to recover a single board. You may target one or multiple IP addresses; the scripts will act on the provided list.

All logs are written per‑IP under tools/YAFU/logs with dots replaced by underscores (e.g. 10_0_1_2.log).

!!! Prerequisites

    - Built executable: ../robotology-superbuild/build/src/ICUB/bin/YAFU (executable).
    - firmware.info.xml available and correct (see code or repo path).
    - Host can reach targets on UDP port 3333.
    - tools/YAFU/logs is writable.


## Runtime network XML resolution

!!! caution 
  
    `parallel.sh` resolves the network XML at runtime using YARP and the current robot name:
    ```sh
    NETWORK_XML="$(yarp resource --from "network.$YARP_ROBOT_NAME.xml" | grep '^\".*$' | sed 's/\"//g')"
    ```
    - That returns the effective network file used by the robot (often under an install tree such as:
    ../robotology-superbuild/build/install/share/ICUBcontrib/robots/).
    - If you prefer a fixed file, set the env var before running:
    ```sh
    export NETWORK_XML=/path/to/network.setupFU.xml
    ```

## Method A — Parallel orchestrator (single command)
What it does
- Reads `NETWORK_XML`, extracts Ethernet `<ataddress ip="...">` entries, and runs the three phases in parallel:
  1. prepare: discover, version pre-check, ensure maintenance (eUpdater)
  2. program: stream Intel HEX (PROG_START / PROG_DATA / PROG_END) to boards that need updating
  3. restart: def2run + RESTART for successfully programmed boards

Quick usage
```sh
cd ../robotology-superbuild/src/ICUB/src/tools/YAFU
chmod +x parallel.sh
./parallel.sh
```

Exit codes (summary)
- 0: all OK (or nothing needed)
- 2: nothing prepared for programming (all skipped or failed)
- 3: some programming failed
- 4: fatal orchestrator error

## Method B — Controlled per‑IP flow (program.sh + restart.sh)
What it does
- Operator supplies IPs. `program.sh` puts each IP into maintenance if needed and programs it. `restart.sh` runs def2run + RESTART for given IPs.

Typical workflow
```sh
cd ../robotology-superbuild/src/ICUB/src/tools/YAFU
chmod +x program.sh restart.sh
./program.sh 10.0.1.2 10.0.1.3
./restart.sh 10.0.1.2 10.0.1.3
```
Examples
- Single IP:
```sh
./program.sh 10.0.1.1
./restart.sh 10.0.1.2
```
- Multiple IPs:
```sh
./program.sh 10.0.1.1 10.0.1.2 10.0.1.3
./restart.sh 10.0.1.1 10.0.1.2 10.0.1.3
```

## Extra perk — use the YAFU binary directly (no shell script)

If you prefer to run single commands without the shell orchestrator scripts, you can invoke the `YAFU` executable directly from the `bin` folder. These commands map to the small helpers implemented in the code:

Notes:

- The `YAFU` binary sends UDP packets from the host using the host's source IP and ephemeral source port (or the interface forced by `LOCAL_IP`). Targets reply to that source socket. Running the binary directly behaves the same network-wise as the scripts. But this is single target not parallel or multi target approach

- `discover()`  -> `discover`
- `jump2updater()` -> `jump2updater` (request maintenance)
- `def2run_application()` -> `def2run_application` (set application mode)
- `restart()` -> `restart`
- `blink()` -> `blink`

Example (run from the repository root or change path as needed):

```sh
cd ../robotology-superbuild/build/src/ICUB/bin/
chmod +x ./YAFU

# Get full information about a board (DISCOVER):
./YAFU discover 10.0.1.2

# Put a board in maintenance (request jumper to updater):
./YAFU maintenance 10.0.1.2

# Put a board back to application mode (def2run):
./YAFU application 10.0.1.2

# Restart the board (RESTART):
./YAFU restart 10.0.1.2

# Blink the board LED (BLINK):
./YAFU blink 10.0.1.2
```





## Troubleshooting (concise)
- No DISCOVER replies:
  - Check cabling, IP, host firewall
- NETWORK_XML seems wrong:
  ```sh
  yarp resource --from "network.$YARP_ROBOT_NAME.xml" | sed 's/"//g'
  less "$(yarp resource --from "network.$YARP_ROBOT_NAME.xml" | sed 's/\"//g' | head -n1)"
  ls -l /usr/local/.../ICUBcontrib/robots/
  ```
  - If mismatched, set `NETWORK_XML` or fix the installed robot folder or `YARP_ROBOT_NAME`.
- Firmware file not found:
  - Verify the `<board type="...">` and `<file>` entries in your firmware.info.xml and that the file path resolves from the host.
- Board won't enter maintenance:
  - Try restart.sh (single IP) or power-cycle

## Wiring vs network‑file consistency
- The network XML must reflect physical wiring (IP addresses, CAN IDs, board assignments).
- If wiring changes, update the network XML and verify before running the updater.
Quick checks:
```sh
# show resolved network resource(s)
yarp resource --from "network.$YARP_ROBOT_NAME.xml" | sed 's/"//g'

# inspect the resolved file (first match)
less "$(yarp resource --from "network.$YARP_ROBOT_NAME.xml" | sed 's/\"//g' | head -n1)"

# list installed robot folders (where deployed network files live)
ls -l ../robotology-superbuild/build/install/share/ICUBcontrib/robots/
```
- Keep wiring diagrams and the exact network XML used for the run together for traceability:
  https://github.com/icub-tech-iit/electronics-wiring-public

## Logs and escalation
- Per‑IP logs: tools/YAFU/logs/*.log — include these plus the firmware.info.xml board entry when asking for help.

!!! Low-key:

    YAFU sends UDP messages from the host using the host's source IP and source port (or the interface forced by `LOCAL_IP`). Targets reply back to that source IP:port. The FirmwareUpdater GUI uses the well-known target port `3333` (each target IP is addressed on its configured destination port), so the GUI and YAFU operate on different directions/ports and do not conflict.

    In short: you can keep the FirmwareUpdater GUI open while running YAFU — the target devices reply to YAFU's source socket, and the GUI's use of the predefined port (`3333`) does not cause socket errors for YAFU. 

    Avoid manually triggering conflicting actions from the FirmwareUpdater GUI at the exact same time you issue a `program`/`restart` command to the same target to prevent overlapping operations on the device.


Enjoy the YAFU - Yet Another Firmware Updater !!!!