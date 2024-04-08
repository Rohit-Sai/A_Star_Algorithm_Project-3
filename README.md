# A_Star_Algorithm_Project-3
Implementation of A_Star Algorithm
---

# A* Pathfinding Project

This A* Pathfinding Project is designed to find the shortest path between two points in a 2D grid while avoiding obstacles. It visualizes the pathfinding process and outputs a video showing the exploration of nodes and the final path.

## Getting Started

To run this project, you will need Python installed on your computer. The project relies on several external libraries, including NumPy and OpenCV, to perform matrix operations and visualize the pathfinding process.

### Prerequisites

Before running the code, ensure you have the following Python libraries installed:
- `numpy`
- `opencv-python`
- `heapq` (usually included with Python)

You can install these libraries using pip, Python's package installer. Run the following commands in your terminal:

```bash
pip install numpy
pip install opencv-python
```

### Running the Code

To run the project, follow these steps:
1. Download the project files to a directory on your computer.
2. Open a terminal or command prompt.
3. Navigate to the directory containing the project files.
4. Run the script using Python:

```bash
python a_star_pathfinding.py
```

### User Inputs

When you run the script, you will be prompted to enter several pieces of information:
- **Clearance**: The minimum distance between the path and obstacles.
- **Radius**: The radius of the robot or point considered for pathfinding.
- **Step Size**: The size of each step the robot takes in the grid (1-10).
- **Start Node**: The starting coordinates and orientation of the robot in the format `x y theta`.
- **End Node**: The goal coordinates and orientation of the robot in the format `x y theta`.
- Enter the clearance: 5
- Enter the radius: 5
- Enter the step size(1-10): 5
- start node:50 50 30
- end node:1150 50 30

Please ensure the start and end nodes are not within obstacle spaces and are within the bounds of the grid.

### Output

After finding the path, the script will generate a video file (`A_star_rohit_suresh_phase1.mp4`) showing the explored nodes and the final path from the start to the goal node.

## Libraries/Dependencies Used

- **NumPy**: For efficient array operations and mathematical calculations.
- **OpenCV (cv2)**: For creating and manipulating the visualization of the pathfinding process.

## Authors

- Rohit Suresh, UID: 119283684
Feel free to contact me for any questions or feedback regarding this project.

---
