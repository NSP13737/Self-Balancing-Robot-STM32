---
onenote-id: 0-2faeae20416a4428ad231c6f53a51450!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
# Things I changed
 
Reduced `CTRL_N_STATE` from 5 to 4 to go from

![[Implementing LQR velo controller image 212cf493d22fa45a.png]]

to x = [phi, theta, p_theta, z]
 
Updated ctrl_mK_f32 to reflect new fit negatted K
 
Updated ctrl_Az_f32 based on matlab script (remove one zero to make 1x4 instead of 1x5)