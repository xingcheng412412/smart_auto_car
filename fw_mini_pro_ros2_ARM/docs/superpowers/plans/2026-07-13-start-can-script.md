# Manual SocketCAN Start Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a manually executed Bash script that configures and starts a SocketCAN interface safely, defaulting to `can0` at 500000 bit/s.

**Architecture:** Keep CAN setup independent from ROS2. Validate all user input and interface existence before requesting sudo, then apply the standard down/configure/up sequence and verify the resulting kernel state.

**Tech Stack:** Bash, iproute2, SocketCAN, sudo

## Global Constraints

- Default interface is `can0`.
- Default bitrate is `500000` bit/s.
- Do not configure boot-time startup or launch ROS2 nodes.
- Return nonzero on invalid input, missing tools/interfaces, command failure, or failed final-state verification.
- This directory is not a Git repository, so changes cannot be committed.

---

### Task 1: Implement and Verify the CAN Start Script

**Files:**
- Create: `script/start_can.sh`
- Modify: `README.md`

**Interfaces:**
- Consumes: optional positional arguments `[interface] [bitrate]`, `ip`, and `sudo`
- Produces: an UP SocketCAN interface at the selected bitrate and clear status/error output

- [ ] **Step 1: Record failing pre-implementation checks**

Run:

```bash
test -x script/start_can.sh
```

Expected: FAIL because the script does not exist.

- [ ] **Step 2: Implement validation before privileged operations**

The script must use `set -Eeuo pipefail`, default positional parameters, check `ip` and `sudo`, validate the interface with `^[[:alnum:]_.-]+$`, validate bitrate with `^[1-9][0-9]*$`, and call `ip link show dev "$CAN_IFACE"` before any sudo command.

- [ ] **Step 3: Implement configuration and final-state checks**

Run these privileged commands in order:

```bash
sudo -v
sudo ip link set dev "$CAN_IFACE" down
sudo ip link set dev "$CAN_IFACE" type can bitrate "$CAN_BITRATE"
sudo ip link set dev "$CAN_IFACE" up
```

Read `ip -details link show dev "$CAN_IFACE"`; require the flags to contain `UP`, reject CAN states `STOPPED` and `BUS-OFF`, and require `bitrate $CAN_BITRATE`. Print the full final status on success.

- [ ] **Step 4: Verify syntax and non-privileged failures**

Run:

```bash
bash -n script/start_can.sh
script/start_can.sh 'bad/interface' 500000
script/start_can.sh can0 not-a-number
script/start_can.sh definitely_missing_can 500000
```

Expected: syntax check passes; each invocation returns nonzero with a specific validation/interface error before requesting sudo.

- [ ] **Step 5: Document usage**

Replace the repeated manual `ip link` setup in README with:

```bash
./script/start_can.sh
./script/start_can.sh can1 500000
```

Explain that the first argument is the interface, the second is bitrate, and sudo authentication is requested.

- [ ] **Step 6: Final verification**

Run syntax and failure checks again, confirm the file is executable, and confirm README contains both default and custom examples. Hardware success is verified separately on the target controller.
