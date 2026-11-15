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

Now I'm going to implement a min magnitude for voltage and see how it does from there, but lowkey just from looking at this I can tell it's not going to work...



