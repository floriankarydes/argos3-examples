# Evaluation

## Candidate

### Name

Tansen Rahman

### Message

I noticed in the original implementation that the robot will collide with walls when heading straight towards them. It does not move to avoid them until it is very close, at which point it does turn, but towards the wall, and so still collides. A similar problem happens when two robots move straight towards each other.

For my implementation I solved these problems by using the fact that the robot can move in reverse. So instead of moving towards the obstacles, I have the robot rotate away from them, depending on their relative position considering both the x and y axis, as opposed to just the x axis in the original implementation. As a result, mechanically, the robot moves to avoid obstacles much like a car would.

I also changed the condition for avoiding obstacles to the maximum length of the sensors, instead of their average. This makes more sense for obstacle avoidance and also takes care of the niche case of heading exactly straight towards a corner of a rectangular obstacle.

I would've liked to add the behavior of moving towards a gap between two objects if a collision would not occur, however this risks causing collisions in other cases where the robot detects several objects, and handling those cases separately would make the code much longer and more complex.

## Review

### Score

#### Algorithm

- Originality: 4/5
- Simplicity: 5/5
- Performance: 5/5

#### Code

- Language: 5/5
- Architecture: 5/5
- Documentation: 5/5

### Comment

The solution provides a nice upgrade of the example without diverging from the initial algorithm paradigm. The code is clean, well documented.
