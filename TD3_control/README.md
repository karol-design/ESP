# Technical demonstration 3 | Control and Steering

This is the code to be used during the third technical demonstration.

### Requirements (for the firmware) for TD3:
- [ ] Demonstration of **speed control** using the encoders. Does the wheel speed stay constant when the
buggy is held in the air and when friction is applied to the wheel? Does the speed stay const. when the buggy goes up/down a slope?
- [ ] Follow a **straight section** of white line on a flat surface. Marks to be awarded for smoothness of trajectory, repeatability of control and on speed.
- [ ] Follow a left-hand and right-hand **bend**. Marks to be awarded for smoothness and repeatability of control and on speed.
- [ ] Return - detect the **turnaround point** and either reverse or perform 180⁰ turn, to return along the track. Marks are awarded for smoothness and speed.
- [ ] Controlled stop - Come to a controlled stop at the **end of line**. Marks are awarded for smoothness and repeatability of control.

### Naming convention in the project
- macros - MACRO_NAME
- classes - ClassName
- objects - objectName
- member functions - functionName
- private data members - _variable_name
- variables - variable_name
- functions - function_name

### Contributing and testing requirements
- Commit to the main only the code that **compile without any warnings or errors**.
- To test, compile or flash the code use [online mBed compiler](https://www.ide.mbed.com/compiler)
