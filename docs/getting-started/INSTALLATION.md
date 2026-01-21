# Installation Guide

This guide walks you through setting up your development environment for shulib.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Ubuntu / Linux Setup](#ubuntu--linux-setup)
- [Windows Setup](#windows-setup)
- [macOS Setup](#macos-setup)
- [VS Code Setup](#vs-code-setup)
- [Verifying Installation](#verifying-installation)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

Before starting, you'll need:

- A computer running Ubuntu (recommended), Windows, or macOS
- Internet connection for downloading tools
- About 2GB of free disk space
- Basic familiarity with command line / terminal

---

## Ubuntu / Linux Setup

Ubuntu is our recommended development environment. These instructions were tested on Ubuntu 24.

### Step 1: Install Python and pip
```bash
sudo apt update
sudo apt install python3 python3-pip
```

### Step 2: Install PROS CLI
```bash
# Install PROS CLI
pip install pros-cli --break-system-packages

# Add to PATH
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Reload shell configuration
source ~/.bashrc
```

### Step 3: Install ARM Toolchain

The ARM toolchain compiles code for the VEX V5 brain.
```bash
sudo apt install gcc-arm-none-eabi
```

### Step 4: Install Git
```bash
sudo apt install git
```

### Step 5: Clone the Repository
```bash
# Create projects directory (optional)
mkdir -p ~/projects
cd ~/projects

# Clone shulib
git clone https://github.com/n0es/shulib.git
cd shulib

# Switch to working branch
git checkout lodge
```

### Step 6: Test the Build
```bash
pros make
```

You should see output ending with `[OK] build/output.bin`.

---

## Windows Setup

### Option A: WSL2 (Recommended)

Windows Subsystem for Linux gives you a full Ubuntu environment.

1. Open PowerShell as Administrator
2. Run: `wsl --install`
3. Restart your computer
4. Open "Ubuntu" from Start menu
5. Follow the [Ubuntu / Linux Setup](#ubuntu--linux-setup) instructions above

### Option B: Native Windows with VS Code

1. **Install VS Code**
   - Download from [code.visualstudio.com](https://code.visualstudio.com/)
   - Run the installer

2. **Install PROS Extension**
   - Open VS Code
   - Press `Ctrl+Shift+X` (Extensions)
   - Search for "PROS"
   - Install **PROS** by Purdue ACM SIGBots
   - Restart VS Code
   - The extension will prompt to install the toolchain – click Yes

3. **Install Git**
   - Download from [git-scm.com](https://git-scm.com/download/win)
   - Run the installer (default options are fine)

4. **Clone the Repository**
   - Open Git Bash (installed with Git)
   - Run:
```bash
     cd ~
     git clone https://github.com/n0es/shulib.git
     cd shulib
     git checkout lodge
```

5. **Open in VS Code**
   - File → Open Folder → Select `shulib`
   - VS Code should recognize it as a PROS project

6. **Build**
   - Press `Ctrl+Shift+P`
   - Type "PROS: Build"
   - Press Enter

---

## macOS Setup

### Step 1: Install Homebrew
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Step 2: Install Dependencies
```bash
# Install Python
brew install python

# Install ARM toolchain
brew install --cask gcc-arm-embedded

# Install Git (if not already installed)
brew install git
```

### Step 3: Install PROS CLI
```bash
pip3 install pros-cli
```

### Step 4: Clone and Build
```bash
cd ~
git clone https://github.com/n0es/shulib.git
cd shulib
git checkout lodge
pros make
```

---

## VS Code Setup

VS Code is our recommended editor for all platforms.

### Required Extensions

Install these from the Extensions panel (`Ctrl+Shift+X`):

1. **PROS** – Build, upload, and terminal integration
2. **C/C++** (Microsoft) – IntelliSense, debugging, code navigation
3. **C/C++ Extension Pack** – Additional tools

### Recommended Extensions

4. **GitLens** – Enhanced Git integration
5. **Error Lens** – Inline error display
6. **Todo Tree** – Track TODOs in code
7. **Bracket Pair Colorizer** – Easier code reading

### Configuring IntelliSense

If you see red squiggles under `#include` statements:

1. Press `Ctrl+Shift+P`
2. Type "C/C++: Edit Configurations (UI)"
3. Under **Include path**, add:
```
   ${workspaceFolder}/**
   ${workspaceFolder}/include/**
```
4. Save and reload VS Code

### Useful Keybindings

| Action | Keybinding |
|--------|------------|
| Build | `Ctrl+Shift+B` (after setting up task) |
| Open Terminal | `` Ctrl+` `` |
| Go to Definition | `F12` |
| Find in Files | `Ctrl+Shift+F` |
| Command Palette | `Ctrl+Shift+P` |

---

## Verifying Installation

Run these commands to verify everything works:
```bash
# Check PROS CLI version
pros --version
# Expected: 3.5.x or higher

# Check ARM compiler
arm-none-eabi-g++ --version
# Expected: 10.x, 11.x, 12.x, or 13.x

# Check Git
git --version
# Expected: 2.x

# Navigate to project
cd ~/projects/shulib  # or wherever you cloned it

# Build the project
pros make
# Expected: Ends with [OK] build/output.bin

# Check project info
pros conduct info-project
# Expected: Shows project name, target, etc.
```

If all commands succeed, you're ready! Continue to [Quick Start](QUICK_START.md).

---

## Troubleshooting

### "pros: command not found"

Your PATH isn't configured correctly.

**Fix (Linux/macOS):**
```bash
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**Fix (Windows):** Reinstall the PROS VS Code extension and let it set up the toolchain.

### "arm-none-eabi-g++: command not found"

The ARM toolchain isn't installed or isn't in PATH.

**Fix (Ubuntu):**
```bash
sudo apt install gcc-arm-none-eabi
```

**Fix (macOS):**
```bash
brew install --cask gcc-arm-embedded
```

### Permission denied when uploading to robot

On Linux, you need udev rules for the V5 brain.

**Fix:**
```bash
# Download udev rules
sudo curl -o /etc/udev/rules.d/99-vex.rules \
  https://raw.githubusercontent.com/purduesigbots/pros-cli/develop/scripts/99-vex.rules

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Unplug and replug the V5 brain
```

### Build fails with "No such file or directory"

You're probably not in the project directory.

**Fix:**
```bash
cd ~/projects/shulib  # Navigate to project root
pros make
```

### IntelliSense shows errors but build succeeds

VS Code's C++ extension can't find headers, but the compiler can.

**Fix:**
1. Open Command Palette (`Ctrl+Shift+P`)
2. Type "C/C++: Edit Configurations (UI)"
3. Add to Include path:
```
   ${workspaceFolder}/**
   ${workspaceFolder}/include/**
```

### "multiple definition" linker errors

You might have duplicate function implementations.

**Fix:** Make sure functions are only defined in `.cpp` files, not `.hpp` files (unless they're `inline`).

### Git says "repository not found"

Check you have access to the repository and the URL is correct.

**Fix:**
```bash
# Verify URL
git remote -v

# If wrong, update it
git remote set-url origin https://github.com/n0es/shulib.git
```

---

## Next Steps

- [Quick Start](QUICK_START.md) – Build and run your first program
- [Project Structure](PROJECT_STRUCTURE.md) – Understand the codebase layout
- [First Build](FIRST_BUILD.md) – Detailed build and upload guide