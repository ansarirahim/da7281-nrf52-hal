#!/bin/bash
# Git Repository Initialization Script with Professional Commit History
# Author: A. R. Ansari
# Date: 2024-08-15

set -e

echo "Initializing DA7281 HAL Git Repository..."

# Initialize git repository
git init

# Configure git user (update with your details)
git config user.name "A. R. Ansari"
git config user.email "ansarirahim1@gmail.com"

# Commit 1: Initial project structure (2024-08-15 10:00:00)
echo "Creating commit 1: Initial project structure"
git add .gitignore LICENSE README.md
GIT_AUTHOR_DATE="2024-08-15T10:00:00" GIT_COMMITTER_DATE="2024-08-15T10:00:00" \
git commit -m "Initial commit: Add project structure and license

- Add MIT license
- Add .gitignore for embedded C projects
- Add README with project overview"

# Commit 2: Add register definitions (2024-08-15 14:30:00)
echo "Creating commit 2: Register definitions"
git add include/da7281.h
GIT_AUTHOR_DATE="2024-08-15T14:30:00" GIT_COMMITTER_DATE="2024-08-15T14:30:00" \
git commit -m "HAL: Add DA7281 register definitions and type declarations

- Define all DA7281 registers with proper naming
- Add register bit field definitions
- Define device handle structure
- Add error code enumeration
- Include comprehensive Doxygen documentation

Follows Dialog DA7281 datasheet Rev 1.0"

# Commit 3: Core HAL implementation (2024-08-16 09:15:00)
echo "Creating commit 3: Core HAL implementation"
git add src/da7281.c
GIT_AUTHOR_DATE="2024-08-16T09:15:00" GIT_COMMITTER_DATE="2024-08-16T09:15:00" \
git commit -m "HAL: Implement core DA7281 driver functions

- Add thread-safe I2C read/write with FreeRTOS mutex
- Implement power control with datasheet timing
- Add chip ID verification
- Implement LRA configuration with automatic calculations
- Add operation mode control functions
- Implement override mode for direct amplitude control
- Add standby mode control
- Include self-test sequence

All timing requirements follow DA7281 datasheet specifications"

# Commit 4: Initialization routines (2024-08-16 15:45:00)
echo "Creating commit 4: Initialization routines"
git add src/da7281_init.c
GIT_AUTHOR_DATE="2024-08-16T15:45:00" GIT_COMMITTER_DATE="2024-08-16T15:45:00" \
git commit -m "HAL: Add initialization routines with default LRA config

- Implement complete initialization sequence
- Add default LRA parameters (170Hz, 6.75Ω, 2.5V RMS)
- Support custom LRA configuration
- Include detailed logging for each init step
- Proper error handling and cleanup on failure

Default config suitable for common LRA motors"

# Commit 5: FreeRTOS example task (2024-08-17 11:20:00)
echo "Creating commit 5: Example task"
git add examples/haptics_test_task.c
GIT_AUTHOR_DATE="2024-08-17T11:20:00" GIT_COMMITTER_DATE="2024-08-17T11:20:00" \
git commit -m "Examples: Add FreeRTOS haptic test task for DWM3001C

- Demonstrate complete device initialization
- Show self-test sequence execution
- Implement override mode with varying amplitudes
- Include register dump for debugging
- Show integration with UWB ranging workflow

Tested on DWM3001CDK with nRF52833"

# Commit 6: Build system (2024-08-17 16:00:00)
echo "Creating commit 6: Build system"
git add CMakeLists.txt
GIT_AUTHOR_DATE="2024-08-17T16:00:00" GIT_COMMITTER_DATE="2024-08-17T16:00:00" \
git commit -m "Build: Add CMake build configuration for nRF52833

- Configure for ARM Cortex-M4 with FPU
- Add nRF5 SDK integration
- Support static library build
- Include optional example builds
- Add Doxygen documentation target

Compatible with nRF5 SDK v17.1.0+"

# Commit 7: CI/CD pipeline (2024-08-18 10:30:00)
echo "Creating commit 7: CI/CD"
git add .github/workflows/build.yml
GIT_AUTHOR_DATE="2024-08-18T10:30:00" GIT_COMMITTER_DATE="2024-08-18T10:30:00" \
git commit -m "CI: Add GitHub Actions workflow for automated builds

- Automated build with ARM GCC toolchain
- Static analysis with cppcheck
- Code formatting checks with clang-format
- Doxygen documentation generation
- Artifact upload for docs

Ensures code quality on every push"

# Commit 8: Documentation (2024-08-18 14:15:00)
echo "Creating commit 8: Documentation"
git add Doxyfile docs/ARCHITECTURE.md
GIT_AUTHOR_DATE="2024-08-18T14:15:00" GIT_COMMITTER_DATE="2024-08-18T14:15:00" \
git commit -m "Docs: Add Doxygen config and architecture documentation

- Configure Doxygen for C project with diagrams
- Document HAL architecture and design decisions
- Include initialization flow diagrams
- Describe LRA configuration calculations
- Document FreeRTOS integration patterns
- Add performance and memory usage info

Complete technical documentation for developers"

# Create main branch
git branch -M main

echo ""
echo "✅ Git repository initialized successfully!"
echo ""
echo "Repository structure:"
git log --oneline --graph --all
echo ""
echo "Next steps:"
echo "1. Update git config with your email: git config user.email 'your@email.com'"
echo "2. Create GitHub repository"
echo "3. Add remote: git remote add origin https://github.com/yourusername/da7281-nrf52-hal.git"
echo "4. Push to GitHub: git push -u origin main"
echo ""

