# 3D-Separating-Axis-Theorem
#####This initial commit is a code draft. I hope to commit a more complete example of 3D collision detection once I have the code ready to do so#####


This is an implementation of the Separating Axis Theorem working in 3D space and returning contact points and other required collision data, such as collision normal and penetration depth.
The code is nearly entirely uncommented, since its pretty much a rough draft at this point but the testing so far has been really positive. I'm finding it operates in good time, since its been somewhat optimized to escape the trap of O(n^3) time complexity that a brute force SAT implementation has. That said, more optimization could be done to revoke some of the unnecessary support point calculations, which are a moderately extensive part of the algorithm's execution time. At the time I ran into issues making the swap to a more optimized setup and decided to upload the current implementation to github for safe-keeping and public viewing. It is around a O(n^2) implementation currently and I'm pretty happy with that.

SAT in 3D is a subject I find is poorly documented outside of some science papers and thesis'. So I do hope it helps someone to see a working implementation to reference from.
