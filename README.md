Conductors 4580 Swerve Bot for 2025

Mk4 Swerve Modules w/Neo 1650s for both drive & turn; REV through-bore encorders


##### RADIO Stuff
4580BigToe

#### Competition Notes
1) The Algae Grabber, if not balanced around mid-point angle (.5), can overshoot past Pi or 0, causing the PID to go to max.  Consider some continuous input -Pi to Pi or something like that to eliminate this dependency
2) We noticed that the wheels kept "losing their offsets" after every couple matches.  Check out the encoders - it felt as though we could force the wheels to slip rotation while the encoders were stationary.  Consider a function in "TEST" mode that can disable the motors for easier re-alignment (i.e. so we don't have to disable motor voltage, then align manually, then update setpoins & re-enable motor voltage)
3) The elevators we torqing at high setpoints, we kept to levels 2 & 3 (out of 4)
4) (Steve) I think there's more we can get out of the drivebase.  I think we can go faster, and ramp up to high speed quicker.  Consider additional turning.  This is especially important for field relative.  We need to get a large test field and see how field relative works
5) Using Elastic was great
6) Need to watch the computer performance.  New laptop was good for programming, but couldn't handle the urgency of comps.  Consider at least an i5 (steve's laptop) or an i7 (old laptop) with 16 Gb RAM and solid state HD.  

#### Thoughts for Offseason
1) Limelight & vision processing:  Continue building on this bot.  Add April tag to the Reef setup and practice autos & alignment functions using vision processing
2) Simulation capability - add physical simulation capability into a simple bot program