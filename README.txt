EXPLANATION OF RRT

An RRT is a Rapidly-exploring-random-tree. 
It is a tree structure in which each child has a position in the plane, a parent node, and a list of child nodes.
The idea behind this RRT implementation is to incrementally build a tree by randomly selecting sampling points in the space and connecting them
to the closet point in the existing tree. This is how the tree "explores" the space.
At each step a random point in space is selected and the algorithm ensures that it is a valid point (it is not running into walls with this new point) and adds it to the nearest node if possible.
This is done using the nearest node function and if no nearest node is found it is added onto a new branch.

This algorithm uses goal biasing to steer the new nodes towards the end goal using the steer point function. This helps the tree find the goal faster.

Here is a google drive link to a video explaining the algotithm and walking through the code as it runs
https://drive.google.com/file/d/1ndhBLnB2yM37ZMZ8ruWV3h-PNU5WmqV6/view?usp=drive_link 

If you wish to run this code, set up a python virtual environment and ensure that numpy and matplotlib are installed.
Then change the directory reference to map.npy on line 140 to where you saved map.npy
Then run main.py and watch the algorithm work
