# OR - LV1 test functions
MATLAB/Octave functions for testing Lab1 functions

## Prerequisites
Test functions are developed and tested using (but should work on other version too): 
 - MATLAB R2019b
 - Octave 4.4.2

## Installation
Clone this repo in any folder of your choosing:
```
$ git clone https://github.com/larics/or-lv1-tests.git
```

or download .zip file from Github.

## Usage
Make sure to position your MATLAB/Octave session to the folder containing the test code. Paste your functions into the folder:
 - dk_nao_rarm.m
 - ik_nao_rarm.m
 - test_closest_q.m
 - test_transform_w.m

##### To test direct and inverse kinematics, run:
```
>> test_nao_rarm_kinematics(3, @dk_nao_rarm, @ik_nao_rarm);
```
This will call direct and inverse kinematics for 3 different values of each joint variable. You can generate a more comprehensive test by calling the function with larger number of values, but it may take longer to run. The sample output of the test function in case your functions are working as expected is:
```
 
--- Testing infeasible point ---
Configuration for infeasible point: <some numbers>
 
--- Testing critical point q = [0,0,0,0.035,0] ---
Configuration in critical point: <some numbers>
 
--- Testing critical point q = [pi/2,-1.2217,0,1.2217,pi/2] ---
Configuration in critical point: <some numbers>
 
--- Checking validity of direct kinematics ---
Max error in feasible workspace: 0
Error rate (all points): 0/<some number>
 
--- Checking validity of inverse kinematics ---
Max error in feasible workspace: <some number>
Error rate (all points): 0/<some number>
 
--- Uniformly testing the feasible workspace for consistency---
Max error in feasible workspace: <some number>
Error rate (all points): 0/<some number>
 
--- Checking initial conditions for direct kinematics ---
Looks like initial conditions are ok for direct kinematics.
If you have errors in inverse kinematics, make sure to 
return values that include inital conditions.
```

If your functions fail at critical points, you probably have some divisions by zero or something similar in the implementation. If you have error rate above zero for any of cases, there is something wrong with your functions. Or with the test, but most likely your functions are wrong. If you are sure that the test fails, raise an issue. 

##### To test closest_q, run:
```
>> test_closest_q(@closest_q);
```
Expected output:
```
--- Testing closest_q ---
closest_q error rate: 0%
```
If you have error rate >0, check that you are using infinity norm to calculate the distance between configurations. If you are sure your function works and the test fail, raise an issue. 


##### To test transform_w, run:
```
>> test_transform_w(@transform_w);

```
Expected output:
```
--- Testing transform_w ---
transform_w error rate: 0%
```
If you have error rate >0, try different combinations of inverting and multiplying order. If you are sure your function works, raise an issue. 
