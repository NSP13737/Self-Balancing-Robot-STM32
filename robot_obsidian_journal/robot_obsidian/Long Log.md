I'm reverse calculating my motor Ko value based on the maximum velocity my wheel can achieve so that my sim is relatively accurate on that front.

Given
$\[ \omega_{\max} \approx \frac{12}{Ko \times 30}\ \text{rad/s}. \]$
My calculated Ko during sys id got me
$\[ \omega_{\max} \approx \frac{12}{0.31185 \times 30} = 1.28\ \text{rad/s}. \]$

But my measured max wheel vel is
```
Left: 31.098597 | Right: 27.925272
Left: 31.415930 | Right: 27.925272
Left: 31.415930 | Right: 28.242603
```

I'm also updating Vmax and Vmin to |11.2| bc this is what it is after voltage drop

Which means (assuming we take the min achievable velocity of ~27) my Ko should be 
11.2/(27*30) = Ko = 0.01382

This led to some CRAZY oscillating dynamics (note that I'm still just hard coding a target of +15rads/s)
![[Pasted image 20261115220552.png]]
I found that the lower I set Kp the better this behavior was
![[Pasted image 20261115221303.png]]
I think I'm just going to set Kp to a conservative 0.3 for now
![[Pasted image 20261115222950.png]]
So now I'll go back to doing the full control simulation
![[Pasted image 20261115223048.png]]
Yeah now matching the wheel velocity is just way too slow I think

Tuning Kp
![[Pasted image 20261115223333.png]]
I finally got something that works in simulation

Now I'm going to implement a min magnitude for voltage and see how it does from there, but lowkey just from looking at this I can tell it's not going to work... (I didn't end up doing that and decided it would be best to just figure out why the actual implementation is blowing up)

# Back to figuring out why robot is blowing up
Here's my initial starting point
![[Pasted image 20261115224817.png]]
I tried just tuning the integrator to 0 which didn't help
![[Pasted image 20261115224958.png]]
Now I'll try reversing the sign of the LQR even though it should be right...
![[Pasted image 20261115225344.png]]
WAYYYYY BETTER (wtf why is it negative....)

Now going to test on the ground (with me holding it)
![[Pasted image 20261115225543.png]]This looks good but just because I was holding it. It is..... in fact..... now going backwards..... meaning the negative sign wasn't the issue

I change ctrl_mk_3 to be 0 so that LQR doesn't track integral state
![[Pasted image 20261115230502.png]]
but no luck

Here I just force ctrl_mk_0 to be 0 so that the control of LQR basically doesn't care about the position of the motors
![[Pasted image 20261115231321.png]]
That seems to have done something at least. 
I'll try running it on the ground from here

I'm going to reimplement the kalman filter
Here I have it on the ground and am supporting it
![[Pasted image 20261115233320.png]]
It doesn't seem that far off...

I'm lowering the sensor noise anyway

# Seeing if I can get velocity control to be faster
It's obviously not correctly setting the velocity
![[Pasted image 20261115235320.png]]
Same on the left
![[Pasted image 20261115235437.png]]
Actually that was just bc the integrator kept its state from other things the robot was doing
![[Pasted image 20261115235730.png]]
Now it's fixed.

Trying to make it faster by adjusting Kp to 1.2
![[Pasted image 20261116000231.png]]

I mean this definitely seems to help

Testing some new K vals
![[Pasted image 20261116001543.png]]
It seems like it underactuates near low angle values so I might just try scaling the IMU gotten params by a little bit

