---
onenote-id: 0-eedaecfda47a4d05893a0e73e23c0b80!1-BC985D38C6EE9748!sfe254750458a4ca2a142f1061d7c5546
---
flowchart LR  
Loop([Loop]) --\> LQR[LQR]
 
LQR --\> ML["Measure left wheel vel"]  
LQR --\> MR["Measure right wheel vel"]
 
ML --\> CAL["Left: Control Allocation"]  
MR --\> CAR["Right: Control Allocation"]
 
CAL --\> VL["Voltage to motor control"]  
CAR --\> VR["Voltage to motor control"]
 ![[Implement between control allocation & motors image ba353eae1bd0a048.png]]   
Initial attempt w/ omega = 5 and Kp at 0.8  
right_SetVel_5.00_rad_s_20261114_144427
 ![[Implement between control allocation & motors image ee19d912904a9193.png]]   
I think Kp is too high since there is lag on velocity, so I will lower that and the time const respectively  
I tuned Kp way down to 0.1 to try and isolate it quickly
 
right_SetVel_5.00_rad_s_20261114_145022
 ![[Implement between control allocation & motors image 173f3ac3184b7879.png]]

And with same params on left  
left_SetVel_5.00_rad_s_20261114_145139

![[Implement between control allocation & motors image 7825c4ba21cf958d.png]]

An interesting note for this one is that in real life the wheel was going much further in one direction than the other, which I guess the plot kinda implies if you look close
 
Changing Kp to 0.01  
left_SetVel_5.00_rad_s_20261114_145518

![[Implement between control allocation & motors image a39f606b8c0170c9.png]]

right_SetVel_5.00_rad_s_20261114_145814

![[Implement between control allocation & motors image 0bf6368dc04c8aff.png]]   
I'm going to try making the update of velocity measurements more accurate by shrinking the window from 5 to 3
 
left_SetVel_5.00_rad_s_20261114_150323

![[Implement between control allocation & motors image fb4a7cae775886fd.png]]   
Now I'm going to try turning Kp back up to 0.1 but NOT changing to integral constant scalar  
left_SetVel_5.00_rad_s_20261114_150744

![[Implement between control allocation & motors image 382249b9c6228306.png]]   
That seemed to work really well (in the sense that I think it may have isolated that the integral scaling const was too high), so I will set Kp back to 0.8
 ![[Implement between control allocation & motors image 58138a04d47f71b4.png]]

WOW!!!!!!
 
Now I'm going to try turning the velo avging window back to 5 samples
 ![[Implement between control allocation & motors image 3b8f1086278b1276.png]]

It does look like that makes it take longer to reach a steady state so I'll put it back to 3
 
And just checking on right motor  
right_SetVel_5.00_rad_s_20261114_151321

![[Implement between control allocation & motors image 545d4b94fb1b5d87.png]]   
Now with some higher speeds at omega=15  
left_SetVel_15.00_rad_s_20261114_151535

![[Implement between control allocation & motors image f8b84644e900eb8a.png]]

right_SetVel_15.00_rad_s_20261114_151619

![[Implement between control allocation & motors image 2f37b2243f8fe27d.png]]  

Now I'll try with applying a small constant ad hoc load  
right_SetVel_15.00_rad_s_20261114_151803

![[Implement between control allocation & motors image eb01a7c35cc3c917.png]]   
left_SetVel_15.00_rad_s_20261114_151855

![[Implement between control allocation & motors image bf1b416f2d39797a.png]]  

Now testing in opposite direction, first with no load  
left_SetVel_-15.00_rad_s_20261114_152123

![[Implement between control allocation & motors image 719df3aab753e613.png]]

right_SetVel_-15.00_rad_s_20261114_152357

![[Implement between control allocation & motors image 843d2b1a5a6b5733.png]]   
Now switching from neg to pos (starting fwd at 75%dc)  
left_SetVel_-15.00_rad_s_20261114_153353

![[Implement between control allocation & motors image ea33d5fbffc054e0.png]]  

right_SetVel_-15.00_rad_s_20261114_153740

![[Implement between control allocation & motors image 7183c69605676a4d.png]]  

I'm going to increase Kp to 1.2 to see if we can get a faster response
 
right_SetVel_-15.00_rad_s_20261114_154235

![[Implement between control allocation & motors image b37bd92699ce1403.png]]  

Kp = 1.8
 
right_SetVel_-15.00_rad_s_20261114_154414

![[Implement between control allocation & motors image ce7687c4b4e5d1af.png]]

This doesn't seem to be doing much
 
I was worried that the control was just getting saturated w/ 100% duty cycle so I am showing that now as well

![[Implement between control allocation & motors image 33e5c252e72eac77.png]]  

left_SetVel_-15.00_rad_s_20261114_161509

![[Implement between control allocation & motors image 4b6111865e23473b.png]]  

This was because my motor was super low :(
 
left_SetVel_-15.00_rad_s_20261114_162134

![[Implement between control allocation & motors image fcf29e4447c9f359.png]]  

We don't seem to be peaking DC for that long so I think it's okay. I'm going to turn Kp back down to 0.8
 
This is with applying a load  
left_SetVel_-15.00_rad_s_20261114_162419

![[Implement between control allocation & motors image a9f851db8087b28b.png]]  

right_SetVel_-15.00_rad_s_20261114_163904

![[Implement between control allocation & motors image 128496e6316e213c.png]]