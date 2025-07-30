# ros2_humble

> **Note:**  
> It is **strongly recommended** to use a **virtual machine** for this setup. The installation script has only been tested on **Ubuntu 22.04** running in a VM environment, and results on other systems are not guaranteed.

---

## Requirements

- **Operating System:** Ubuntu 22.04 (64-bit)
- **Required Software:**  
  - `sudo`
  - `git`
  - `whoami`
- **Privileges:** Access to the `sudo` command

### Install Prerequisites

On a fresh Ubuntu 22.04 VM, you may need to install `sudo` and `git`.  
The `whoami` command is included by default in Ubuntu coreutils, so you do **not** need to install it.

Install `sudo` (if not present):
```bash
apt update
apt install sudo
```

Install `git`:
```bash
sudo apt update
sudo apt install git
```

---

## Setup Instructions

1. **Clone this repository:**  
   You can clone this repository to your home directory:
   ```bash
   git clone https://github.com/Darkstar-blip/ros2_humble.git ~/ros2_humble
   cd ~/ros2_humble
   ```

2. **Run the setup script:**  
   From anywhere, you can run:
   ```bash
   ~/ros2_humble/setup.sh
   ```
   This script will:
   - Install ROS 2 Humble and its dependencies
   - Set up your environment for ROS 2 development
   - Download or prepare datasets as needed by the project
   - Run the navigation

   Follow the instructions provided by the setup script.

3. **Rebuild the package (if needed):**
   ```bash
   ~/ros2_humble/rebuild.sh
   ```

---

## Usage

- Run the setup script as described above:
  ```bash
  ~/ros2_humble/setup.sh
  ```
- If anything fails, use:
  ```bash
  ~/ros2_humble/undo_script.sh
  ```
  Fix the error, and then run the setup script again with:
  ```bash
  ~/ros2_humble/setup.sh
  ```

---

## Notes

- All commands assume you cloned the repository to your home directory as `~/ros2_humble`.
- If you cloned it elsewhere, adjust the script paths accordingly.
