---
onenote-id: 0-ff7f276074ef4919b090a7558001961b!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
LQR K Value  
-4.492148056511271 -9.967726608934711 -22.355953248794705 -1.274592539852193  
Tm Value (in control allocator in motor class)  
0.0044
 
------------------
   

Kp: 1
   

Q_int =
 
10 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
      

R =
 
0.1000
 ![[Tuning System image fd37d7402c128aee.png]]  

Kp: 1
   

Q_int =
 
2 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
   

R =
 
0.1000

![[Tuning System image 639e7a801d71226d.png]]  

Kp: 0.5000
   

Q_int =
 
2 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
   

R =
 
0.1000

![[Tuning System image 1581de049ae99d5e.png]]  

Kp: 0.5000
   

Q_int =
 
2 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
   

R =
 
0.3000

![[Tuning System image e2b320eac4219b39.png]]  

Kp: 0.5000
   

Q_int =
 
2 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
   

R =
 
0.6000

![[Tuning System image 78b269a73d040f80.png]]  

Kp: 0.8000
   

Q_int =
 
2 0 0 0  
0 1 0 0  
0 0 1 0  
0 0 0 1
   

R =
 
0.6000
 ![[Tuning System image 0f9d5f2593620332.png]]      
# Implementing params on system in PIL

Attempt 1 with Kp and K changed

![[Tuning System image 7aac92ef705096ad.png]]  

Identified mistake when dividing Kp by Tm

![[Tuning System image 7e4a46d521de84d8.png]]  

AI Thought that x[2] was velocity so I fixed that by updating vel based on x[0]
 ![[Tuning System image 9b1d95a50cba2f67.png]]  

I was calculating velocity backwards in C code (NOW IT MATCHES)
 ![[Tuning System image a07d0644ee46667a.png]]

![[Tuning System - Ink.svg]]
