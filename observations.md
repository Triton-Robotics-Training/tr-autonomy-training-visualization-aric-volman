I noticed that in order to recreate the jump in x error, you have to rotate the target to the edge. The target rectangle overextends by a lot which increases the x error in SolvePnP. 

My computer wasn't powerful enough to get small increments but I noticed some oscillation between different errors (i.e. it seemed that the x, y, and z errors were oscillating a little bit or had some pattern). This is probably due to the consistent conditions of rotation (the target consistently rotated every time). 

Otherwise it worked well and the error seemed low enough (within 5 cm most of the time). I used a slightly different technique to compare closeness of the target to a plate by comparing only the X and Y coordinates. This is more robust because the Z tends to jump around and wiggle a bit.