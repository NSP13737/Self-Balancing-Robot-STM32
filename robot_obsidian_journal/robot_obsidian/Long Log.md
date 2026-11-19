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

#### IMU KF Improvement
- I figured out that my old val for accAngleVar derived in matlab was way too high (0.2492)
- This stemmed from the encoder data that I collected having a negative sign relative to the other data
- After fixing this dropped the angle variance significantly (0.0729) which will greatly speed up how quickly my angle estimate updates
- To compare, this is what the onboard data looked like previously
- ![[Pasted image 20261117221336.png]]
- Now you can see in onboard_imu_kf_20261117_222453 (the new data) that the angle is much less delayed
- ![[Pasted image 20261117222538.png]]

# Improving Simulation of DC Motors

- Main issue is that right now, in my simulation I am just setting the motor velocity which kind of defeats the whole purpose of running a simulation. It doesn't take into account Jm, or b
- I am going to try to verify this change by seeing if hardcoding a set value seems similar to what real motor does
- Initial implementation says my max motor vel is 0.4
- ![[Pasted image 20261118000203.png]]

to figure out why, I'm starting by looking at the current compared to some real world stuff
- In the sim we obviously see that steady state current is way too high (I matched Kp to real data below)
![[Pasted image 20261118000846.png]]
Compare that with this old measurement
- I avged a measured steady state sample here and got avg current of ~0.1667A
![[Pasted image 20261118000940.png]]

- This effect is being controlled by Td and Ki because to even get the motors running, Ki\*I_a needs to be greater than Td
- Td was never estimated and Ki could be very wrong because of static friction in the torque test
- I'm going to try raising my Ki to what I found online for my 12v 333rpm JGB37-520B motors
	- Ki = 0.21
- Manufacturers don't give a Td (static friction) value, but they give a no load amperage of 0.12 (which matches pretty well to my measured data), which when multiplied by Ki gived Td = 0.025
- So I will use these values
- Now I am getting a max wheel speed of abt 1.1
- ![[Pasted image 20261118002805.png]]
In better news the motor current seems to match what I see on the real robot
![[Pasted image 20261118002847.png]]
Tomorrow I will try back deriving some values from what I know the robot can actually do

#### Next day
- Ko and N are the predominant vars holding velocity of sim wheel back
	- V_in = obj.Ra\*I_a + obj.Ko\*obj.N\*omega_m;
- I figured out that in the electrical sysID script since we take velocity after the gear ratio, we can divide by the gear ratio to get the actual motor Ko
- This takes Ko from 0.31185 to 0.010395
- Now we get results similar to real life![[Pasted image 20261118215254.png]]
- Now trying full sim
- ![[Pasted image 20261118215458.png]]
- You can actually see how it oscillates back and forth how many robots do in real life
- I also want to check at what voltage in the sim the wheel begin to turn
- At 0.5V
- ![[Pasted image 20261118220242.png]]
- At 0.7V
- ![[Pasted image 20261118220351.png]]
- At 0.9V
- ![[Pasted image 20261118220421.png]]
- It looks like it is relatively simulating the robots static friction, so I'm not going to implement a minimum voltage to the controller
# Testing some different params for implementation
- This has a period of over 1 sec which is a little too long for me
![[Pasted image 20261118220654.png]]

- I'll just call this one good
- ![[Pasted image 20261118220832.png]]
- Or this
- ![[Pasted image 20261118221535.png]]


## Test on robot
- PIL LQR Test
- ![[Pasted image 20261119124009.png]]
- That last test was actually running the control allocation twice, but I fixed it![[Pasted image 20261119124628.png]]
- Open air test
- Isn't working bc angle cuts off immediately
- Running IMU log you can see imu is really low for first 3 samples
- ```
  0.000000,-2.356194,-0.000061,-1.000000
0.005000,-2.356194,-0.000061,-1.000000
0.010000,-2.356194,-0.000061,-1.000000
0.015000,-0.020962,-0.088075,-1.000000
0.020000,-0.029843,-0.387762,-1.000000
0.025000,-0.020070,-0.805615,-1.000000
0.030000,-0.026469,0.108949,-1.000000

  ```
- Now open air test
- Fell over really quick
- ![[Pasted image 20261119132059.png]]
- Doing another test just to see KF angle
- That file is somewhere \\_0_\/
- But I figured out that motor positions weren't being reset so encoder count was starting really high
- This next one is werid bc it just didn't really react
- ![[Pasted image 20261119133351.png]]
- ![[Pasted image 20261119133401.png]]