---
onenote-id: 0-ede79dc0769d4748a0cd9df73674d811!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
I realized that my motor params were not up to date with the freewheel spin test measured Jw and b coeffs
 
This is new output with no tuning
 ![[Fixing and improving sim image f4c786cee50b560c.png]]  

I want to see if my DC motor actually behaves somewhat similarly to the real one now
 
I discovered that my motor in sim is currently maxed out at ~1.2rad/sec because my Ko value was so high. So I divided it by 10 to get 0.031185
 
The good news is that it looks like in this simulated test my motors are much closer to what my real motors response looks like. (about 1.5 sec to get to steady state)