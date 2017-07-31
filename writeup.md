# RoboND-Kinematics-Project

## Steps to complete the project:

1、Setup ROS Workspace.

2、Clone the project repository into the src directory https://github.com/udacity/RoboND-Kinematics-Project

3、Experiment with the forward_kinematics environment and get familiar with the robot 

4、Launch the pick and place demo (instructions here) to get familiar with various steps in the project

5、Perform Kinematic Analysis for the robot following the project rubric along with proper writeup.

6、Fill in the IK_server.py with your Inverse Kinematics code. Instructions on how to do the math behind IK can be found here and how to convert your IK math into code can be found here.

## kuka arm
![Alt text](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/1.jpg)

## Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

## Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

## Analyze each mechanical arm coordinate system
![Alt text7](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/7.jpg)

## Define Modified DH Transformation matrix

     s = {alpha0:   0.0,   a0:    0.0,    d1:  0.75, 

          alpha1: -pi/2,   a1:   0.35,    d2:   0.0,    q2:q2-pi/2,
     
          alpha2:   0.0,   a2:   1.25,    d3:   0.0,    
     
          alpha3: -pi/2,   a3: -0.054,    d4:   1.5,
     
          alpha4:  pi/2,   a4:    0.0,    d5:   0.0,
     
          alpha5: -pi/2,   a5:    0.0,    d6:   0.0,
     
          alpha6:   0.0,   a6:    0.0,    d7: 0.303,   q7:0.0}

## Create individual transformation matrices
            T0_1 = Matrix([[            cos(q1),           -sin(q1),           0,             a0],
                           [sin(q1)*cos(alpha0),cos(q1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1],
                           [sin(q1)*sin(alpha0),cos(q1)*sin(alpha0), cos(alpha0), cos(alpha0)*d1],
                           [                  0,                  0,           0,              1]])
            T0_1 = T0_1.subs(s)
        
            T1_2 = Matrix([[            cos(q2),           -sin(q2),           0,             a1],
                           [sin(q2)*cos(alpha1),cos(q2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2],
                           [sin(q2)*sin(alpha1),cos(q2)*sin(alpha1), cos(alpha1), cos(alpha1)*d2],
                           [                  0,                  0,           0,              1]])
            T1_2 = T1_2.subs(s)
            
            T2_3 = Matrix([[            cos(q3),             -sin(q3),             0,               a2],
                           [sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2),  -sin(alpha2),  -sin(alpha2)*d3],
                           [sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),   cos(alpha2),   cos(alpha2)*d3],
                           [                  0,                    0,             0,                1]])
            T2_3 = T2_3.subs(s)
        
            T0_3 = T0_1 * T1_2 * T2_3
 
##rotate axis

            R_y = Matrix([[ cos(-pi/2),        0, sin(-pi/2)],
                          [          0,        1,          0],
                          [-sin(-pi/2),        0, cos(-pi/2)]])

            R_z = Matrix([[ cos(pi),     -sin(pi),          0],
                          [ sin(pi),      cos(pi),          0],
                          [       0,            0,          1]])
            
            R_corr = R_z*R_y
         
# Calculate theta1
![Alt text2](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/2.jpg)

     theta1 = atan2(wy, wx)
     
# Calculate theta2
![Alt text3](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/3.jpg)

            #distance 
            dis_2_3=a2
            dis_2_4=sqrt((sqrt(wx**2+wy**2)-a1)**2+(wz-d1)**2)
            dis_3_4=sqrt(d4**2+a3**2)
            # theta2
            cos_theta2_1=(dis_2_4**2+dis_2_3**2-dis_3_4**2)/(2*dis_2_4*dis_2_3)
            sin_theta2_1=sqrt(1-cos_theta2_1**2)
            theta2_1 = atan2(sin_theta2_1,cos_theta2_1)
            
            sin_theta2_2=(wz-d1)/(dis_2_4)
            cos_theta2_2=sqrt(1-sin_theta2_2**2)
            theta2_2 = atan2(sin_theta2_2,cos_theta2_2)
            
            theta2 = pi/2-theta2_2-theta2_1 
            
# Calculate theta3
![Alt text4](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/4.jpg)

            theta3_1=atan2(a3,d4)
            cos_theta3_2 = (dis_2_3**2+dis_3_4**2-dis_2_4**2)/(2*dis_2_3*dis_3_4)
            sin_theta3_2 = sqrt(1-cos_theta3_2**2)
            theta3_2 = atan2(sin_theta3_2,cos_theta3_2)
            theta3 = pi/2 - theta3_2-theta3_1
            
# Calculate theta4 5 6
![Alt text5](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/5.jpg)
![Alt text6](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/6.jpg)
![Alt text7](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/7.jpg)

            theta4=atan2(new_R3_6[2,2],-new_R3_6[0,2])
            sin_theta5=sqrt(new_R3_6[2,2]**2+new_R3_6[0,2]**2)
            theta5=atan2(sin_theta5, new_R3_6[1,2])
            theta6=atan2(-new_R3_6[1,1],new_R3_6[1,0])
            
# result
![Alt text8](https://github.com/nnresearcher/RoboND-Kinematics-Project/blob/master/RoboND-Kinematics-Project/writeup_picture/8.jpg)
I think it is enough. 
