---
onenote-id: 0-ac7c66d204c84373af0319af6d3b1bdf!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
Want to ask if it is possible to do torque: present info on why this isn't working
 
If we did velocity, please lay out the path as if I'm stupid

- How do we do??
- Figure out tonight
      

```
return batt_voltage_read() * signed_dc / 100.0f;
```
   

TorqueID stuff
       
# My Data

![[Torque vs Velocity Presentation image f2613c8229786123.png]]        
![[Torque vs Velocity Presentation image d9c1ca429664ef13.png]]  
![[Torque vs Velocity Presentation image f0cb4369bbccf93b.png]]   
**How torque is reconstructed --\>** **Ki*N*current**
 
Ki = 6.2391e-04
 ![[Torque vs Velocity Presentation image 32ae1ae89cca8171.png]]      
# Good 80% DC data

![[Torque vs Velocity Presentation image 3087740e78258b00.png]]  

# Regular Implementation

Ki = 0.0024
 ![[Torque vs Velocity Presentation image a7cfcdda88c70a2c.png]]  

# AI Suggested Intercept Fitting (basically just accounts for one way friction)

Ki =0.0070
 ![[Torque vs Velocity Presentation image 918981dfb4f6cd8c.png]]   
# DOCUEMTATION For right motro
 
ki = 0.0048
 ![[Torque vs Velocity Presentation image ba87b14eae2068a2.png]]  

![[Torque vs Velocity Presentation image 302ecf18b24fab20.png]] ![[Torque vs Velocity Presentation image 3e4727ee4f7fc104.png]]

# Assignment Data

![[Torque vs Velocity Presentation image f5d7dff396948e3a.png]] ![[Torque vs Velocity Presentation image 19fd8c21eddf7477.png]]

Ki =0.0033

for vel control
 
esense: use flow source controller, so you get desired velocity
 
to get from velocity to voltage you need a PI controller
 
(I didn't understand some of the info here) error that we feed into the PI controller
 
how to tune the PI controller: need a relationship between angular vel and voltage
 
On Friday: we'll go through PI loop for velocity control
 
Action item for me is to so lower freq sinusoid

![[Torque vs Velocity Presentation - Ink.svg]]
