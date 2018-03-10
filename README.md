# Udacity Flying Car Nanodegree

## Projects

### 1. Backyard Flyer
**Description:** Set up a state machine using event-driven programming to make [udacidrone](https://github.com/udacity/udacidrone) fly autonomously in a square shape. The pupose of this project is to get familiar with sending commands and receiving incoming data from the drone. The required task is to command the drone to fly a 10 meter box at a 3 meter altitude. Since communication with the drone is done using MAVLink, the code can potentially be used to control an PX4 quadcopter autopilot with minimal modification.

**Code**: [backyard_flyer.py](projects/backyard_flyer/backyard_flyer.py)

**Result**:

![backyard flyer](doc/gif/backyard_flyer.gif)

## Environment Setup
1. Download and install [miniconda3](https://conda.io/miniconda.html).
2. Clone the repository and then navigate to `FCND-Term1-Starter-Kit` submodule:
```bash
git clone --recursive https://github.com/pyadmell/flying-car-udacity.git

cd ext/udacity/FCND-Term1-Starter-Kit
```
3. Create the miniconda environment:
```bash
conda env create -f environment.yml
```
4. Verify the fcnd environment:
```bahs
conda info --envs
```
5. Clean up downloaded packages:
```bash
conda clean -tp
```
6. Activate `fcnd` conda environment:
```bash
source activate fcnd
```

