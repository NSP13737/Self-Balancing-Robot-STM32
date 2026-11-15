---
onenote-id: 0-7399fe83fba64637a7b9452f0bd23b0d!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
Had to go into assignment 3  
Since linear model uses state space

As opposed to

When doing control setup  
controllerCartPole.controlInit_reference_integrator(balancingRobotLin.A(), balancingRobotLin.B(), C_r, D_r, Q_int, R, N_int, T, y_ref);   Which runs  
partial_N_ff = inv([obj.A obj.B; obj.C_r obj.D_r]);  
and  
obj.A_int = [obj.A zeros(4,1); obj.C_r 0]; %lec08 01 slide 16
 
The size of C_r and zeros(4,1) had to each be reduced by 1 so that sizes of matricies matched
 
We also had to reduce size of  
Q_int = diag([10 1 1 1 1]);  
N_int = zeros(5,1);  
x0 = [0; 3*pi/180; 0; 0];  
to  
Q_int = diag([10 1 1 1]);  
N_int = zeros(4,1);  
x0 = [0; 3*pi/180; 0];
 
for the same reason
   

And when analyzing outputs changed  
solved_phi_col = res.x(:,1);  
solved_theta_col = res.x(:,2);  
solved_dphi_col = res.x(:,3);  
solved_dtheta_col = res.x(:,4);
 
to  
solved_phi_col = res.x(:,1);  
solved_theta_col = res.x(:,2);  
solved_dp_phi_col = res.x(:,3);
   

Also removed velocity -\> torque control allocation and just have velocity straight in there
 
Results w/ alpha = 0  
and y_ref to control phi at 0.5
 ![[Modifying Assignment 3 image fc2f46ef4c7496da.png]]  

with alpha = 5*pi/180  
and y_ref set to phi to 0.5

![[Modifying Assignment 3 image a4000684010c1559.png]]   
--------------------------------------------
   

Realized that STM doesn't currently have reference tracking built in, therefore these are the sims with y_ref = 0
 
Alpha = 0
   
![[Modifying Assignment 3 image 708b400fee8e70f7.png]]   
Alpha = 5pi/180

![[Modifying Assignment 3 image 09f47129b481e81d.png]]  

# PIL Testing
 
For some reason it doesn't work (NOTE THAT THIS IS 20 SECOND SIM)
 ![[Modifying Assignment 3 image bb883c34e1638252.png]]  

Z STATE

![[Modifying Assignment 3 image 5c4af1649177e1d6.png]]   
This is compared to the z state of the matlab sim

![[Modifying Assignment 3 image 9a3b0d91f380b0f8.png]]   
FIXED PIL

![[Modifying Assignment 3 image 501b1b8c65a17b81.png]]  
![[Modifying Assignment 3 image 6f1d4937c96ffd8b.png]]

![[Modifying Assignment 3 - Ink.svg]]
