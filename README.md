# Steerable-Flexible-Probe

    https://github.com/tybarrett/Steerable-Flexible-Probe

Mahima Arora - marora1

Tyler Barrett - tbarret2

---

This repository provides an implementation of the algorithm described in the following paper: 

> C. Caborni, S. Y. Ko, E. De Momi, G. Ferrigno and F. R. y Baena, "Risk-based path planning for a steerable flexible probe for neurosurgical intervention," 2012 4th IEEE RAS & EMBS International Conference on Biomedical Robotics and Biomechatronics (BioRob), 2012, pp. 866-871, doi: 10.1109/BioRob.2012.6290859.

This includes the following:
- Implementation of RG-RRT
- The integration of brain components as obstacle maps
- The calculation of cost as described in the paper

Additionally, this includes original research which generates a tree from the goal node (the malignant mass) to any valid exit point on the skull.
Using this approach and through generating statistics regarding the anticipated cost for each solution, exiting through the closest point on the skull can result in a ten-fold reduction of cost.

## Dependencies

To run this software, the following will be required:
- Python 3
- OpenCV
- NumPy

Numpy is a dependency of OpenCV, so you will only need to install OpenCV to satisfy all requirements. Consider running the following command:

    pip install cv2

## Execution

To execute this code, simply execute the following command to run `main.py`:
    
    python main.py

This will execute 6 test cases in order. Each test case includes finding a suitable solution from the start point to the end point using RG-RRT. Afterwards, a solution is found using our original research.
After all test cases have been run, the software will generate an average cost for all of the "forward-searching" paths and an average cost for all the "reverse" paths. 

Feel free to execute just one test case by modifying the `TEST_CASES` constants on line 22 of `main.py`.
This will generate an output file named `output.avi` which shows each solution and the chosen solution based on the cost function.

## Input

The software does not accept any user input. 
Instead, it searches from start to goal point based on specific test cases. Feel free to modify these test cases as you see fit on line 22 of `main.py`.
Each test case is made of three elements: the start coordinate, the starting angle (in degrees) and the goal coordinate. 