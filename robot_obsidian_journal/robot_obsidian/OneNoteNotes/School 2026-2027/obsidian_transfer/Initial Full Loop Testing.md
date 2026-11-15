---
onenote-id: 0-46548f74c289460a8bae9472c3b8dee9!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
control_loop_20261115_140646

![[Initial Full Loop Testing image 0346daf89c369977.png]]  

It looks like maybe my lqr is just going the wrong way?
 
I'll test this with 2 tests. One with the robot clearly starting at a pos angle, and one at a neg angle
 
TEST 1 pos angle

![[Initial Full Loop Testing image 62776ac891669cc8.png]]  

TEST 2 negative angle

![[Initial Full Loop Testing image 36c8d3b3cfde754f.png]]  

So in both cases the LQR control was negative  
Upon further inspection it looks like in the positive test the angle began far negative which is odd. So I'll do another positive test
 
It looks like it did this again?

![[Initial Full Loop Testing image dea29127f805752e.png]]

And the filtered angle agrees with this

![[Initial Full Loop Testing image 7538caafa8f34a17.png]]   
I'm going to do one test where I just oscillate the robot and give the data to AI and see if it can figure it out

![[Initial Full Loop Testing image 387535aa4dc66f6e.png]]  
![[Initial Full Loop Testing image f2fafa733f4ce32f.png]]  

This shows me that something about the kalman filter doesn't seem to be working properly. So I'm just going to test that by itself  
onboard_imu_kf_20261115_143859
 ![[Initial Full Loop Testing image c1dcb3cfe7522932.png]]  

This seems to be working pretty well, so now I'll try it with the motors running since maybe there is some noise?? But I'm also really confused why this estimate starts at like -2.5 rads as well
   

onboard_imu_kf_20261115_144148
 ![[Initial Full Loop Testing image 7b68e6021563cd9f.png]]  

Yeah that seems to be working fine which is weird. And the estimate didn't start way negative
 
Now I'll test by estimating the full robot state
 
state_estimate_20261115_144831

![[Initial Full Loop Testing image f8466f01313a32ed.png]]   
The angle state estimate here def is working (but is maybe negative relative to wheel position?  
state_estimate_20261115_144955
 ![[Initial Full Loop Testing image 30aba756df0569d1.png]]  

Nah it’s the right way
 
I found out that the data was trying to send too fast so it was getting overwritten or something so I'll fix that
 ![[Initial Full Loop Testing image 0c53673425630cf5.png]]  

Looks much better. Is now collecting the full 5 seconds and the KF seems to line up better
 
But control still sucks
 
I verified that fwd results in pos measured velocities
 
I verified that when setting a pos voltage, the motor went fwd, and vise versa  
One interesting thing I noticed was that when setting a voltage on a motor and then readin the voltage on it, the estimated voltage was about 0.5V greater in magnitude
 
I fixed the mistake causing the extra 0.5V magnitude when reading motor voltage
 
I think right now the angle kf is really dependent on gyro and slow to update so I'm just using raw vals for now