# Scenic-Habitat Interface

This folder contains the Scenic interface to Meta Habitat 3.0 simulator.

## Requirements
    - Any operating systems supporting Habitat and Scenic should work. Scenic currently supports all of MacOS, Windows, and Linux, and so does Habitat. This interface was developed and tested on Ubuntu 20.04.
    - `Python >= 3.9`. Either `conda` or `mamba` environment should work
    - The Python environment should NOT have `pygame` installed. Both Scenic and VerifAI installs `pygame` by default so it is important to uninstall it after either is installed, as noted below. `pygame` interferes with Habitat for unknown reasons, and if running Habitat gives an obscure `segmentation fault`, having `pygame` is likely the cause.
## Setup Instructions
    - If you don't have Scenic installed, please follow the instructions [here](https://docs.scenic-lang.org/en/latest/quickstart.html) ; note tht while the official Scenic docs uses `pyvenv` for its virtual environments, using `conda/mamba` environment also works and is recommended here for Habitat.
    - Install Habitat according to the instructions [here](https://github.com/facebookresearch/habitat-lab)
    - `pip` will likely give error messages about incompatible version requirments for `numpy`, `antlr`, and `omega-conf` between Scenic and Habitat. For these packages, use versions that Habitat requires.
    - Run `pip uninstall pygame` to uninstall `pygame`. Run this after installing VerifAI, too.
