# CS-E5520 Advanced Computer Graphics, Spring 2020
# Lehtinen / Aarnio, Kemppinen
#
# Assignment 1: Accelerated Ray Tracing

Student name: Nariadchikov Aleksandr
Student number: 843513

# Which parts of the assignment did you complete? Mark them 'done'.
# You can also mark non-completed parts as 'attempted' if you spent a fair amount of
# effort on them. If you do, explain the work you did in the problems/bugs section
# and leave your 'attempt' code in place (commented out if necessary) so we can see it.

R1 BVH construction and traversal (5p): done
        R2 BVH saving and loading (1p): done
              R3 Simple texturing (1p): done
             R4 Ambient occlusion (2p): done
         R5 Simple multithreading (1p): done
-----

# Did you do any extra credit work?

I've implemented SAH in my BVH builder. Task was quite challenging and I've spent a lot of attemps making it work.
I've decided to use simplified inner node heuristic, how it was described in "assn1.pdf".

I iterate by all 3 axis splitting current bounding box with plane within the 10% step.
After split I calculate SAH score (area of newly formed bounding boxes) of primitives stayed to the left of the split plane and to the right.
Best SAH score defines which splitIndex I would use for left and right subtrees separation.

By experiment "MAX_TRIS_PER_LEAF_SAH" constant was selected with value "10". For my hardware it gives best performance.

Besides SAH, I've implemented ObjectMedian and SpatialMedian split modes.
Depending on which split mode you pass to "Bvh" class constructor, bounding volume building type would be chosen (switch-case).


Comparison between hierarchies for "conference" scene (default view angle) with my traversal:

			Build Time |  Mrays/sec | Size (Mb) | MAX_TRIS_PER_LEAF
Spatial Median   	 < 1 sec	~0.74	   11.8		   3
Object Median    	 ~ 2 sec	~0.40	   10.9	   	   3
SAH		 	 ~ 6 sec	~2.23	    4.4	   	  10
Solution hierarchy	 Unknown	~1.86	    5.1	 	Unknown


Mrays/sec for same scene in your "reference_assignment1_64bit.exe": ~2.49 Mrays/sec
So with SAH my tracer's perfomance matches yours.
-----


# Are there any known problems/bugs remaining in your code?

There are no problems.
!!! However, be aware that in "intersectTriangles" function in "RayTracer.cpp" I iterate from "startPrim" to "endPrim" inclusive, because I built "indices_" array like that.
-----


# Did you collaborate with anyone in the class?

I've been attending exercise sessions 3 times. Lauri and Pauli have helped me to find some bugs and kindly explained some theory and how framework is working.
-----


# Any other comments you'd like to share about the assignment or the course so far?

I've enjoyed working on assignment.
I can say that every time I sit behind computer doing it I caught "flow" state, so it shows that task is interesting and challenging in the same time.