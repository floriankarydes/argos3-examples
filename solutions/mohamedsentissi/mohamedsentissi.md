# Evaluation

## Candidate

### Name

Mohamed Larbi Sentissi

### Message

Here are the controller files as promised. I also modified the diffusion_10.argos config file in order to tune the controller parameters without having to recompile everything each time.

The solution I implemented is based on a virtual force (This concept seems pretty common in the literature, though I didn't base my solution on any paper in particular). The average proximity vector computed from sensor readings was multiplied by -1 to get a repulsive force.

This repulsive force is then separated into X (parallel to wheel direction) and Y (orthogonal to wheel direction) components.
The effects are as follows :
- fX induces acceleration away from obstacle
- fY induces rotation away from obstacle (difference of speed between the wheels)
- fX also induces rotation away from obstacle if the robot is moving directly towards the obstacle

## Reviewer

### Score

#### Algorithm

- Originality: 5/5
- Simplicity: 5/5
- Performance: 5/5

#### Code

- Language: 5/5
- Architecture: 5/5
- Documentation: 4/5

### Comment

The solution developed make intelligent use of reverse gear and does not suffer the locks observed in the example. The code is clean and well documented. The formalism of the existing code could be better respected.
