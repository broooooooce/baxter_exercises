#!/usr/bin/env python

# Baxter Exercises

# Bruce Stracener - University of Arkansas for Medical Sciences
# started 01/08/18

#   THE TRICEP EXERCISES ONLY WORK WITH COLLISION AVOIDANCE SUPPRESSED
#   USE rostopic pub /robot/limb/right/suppress_collision_avoidance std_msgs/Empty -r 10
#   AND rostopic pub /robot/limb/left/suppress_collision_avoidance std_msgs/Empty -r 10
#   OR use the suppress_collision_avoidance.py script

#       Local variables for all functions:
#           a = repetitions
#           b = speed multiplier
#           c = limb


import rospy                # ROS python API
import baxter_interface     # Baxter Python API

# initialize ROS node and register with master

rospy.init_node('Baxter_Exercises', anonymous=True)

# set poses 

# right bicep

r_bicep_curled = {
    'right_e0': 2.996631, 
    'right_e1': 2.615437, 
    'right_s0': 0.655009, 
    'right_s1': 1.046558, 
    'right_w0': 3.058374, 
    'right_w1': -0.506029, 
    'right_w2': 0.0617427}
r_bicep_uncurled = {
    'right_e0': 3.014272, 
    'right_e1': 1.13783, 
    'right_s0': 0.710617, 
    'right_s1': 1.047325, 
    'right_w0': 3.060292, 
    'right_w1': -0.062126, 
    'right_w2': 0.0590582}

# left bicep

l_bicep_curled = {
    'left_e0': -2.996631, 
    'left_e1': 2.615437, 
    'left_s0': -0.655009, 
    'left_s1': 1.046558, 
    'left_w0': -3.058374, 
    'left_w1': -0.506029, 
    'left_w2': 0.0617427}
l_bicep_uncurled = {
    'left_e0': -3.014272, 
    'left_e1': 1.13783, 
    'left_s0': -0.710617, 
    'left_s1': 1.047325, 
    'left_w0': -3.060292, 
    'left_w1': -0.062126, 
    'left_w2': 0.0590582}

# right tricep

r_tricep_extended = {
    'right_e0': 3.039, 
    'right_e1': 0.309, 
    'right_s0': 0.791, 
    'right_s1': 0.511, 
    'right_w0': 3.05, 
    'right_w1': -0.29 , 
    'right_w2': 0.094}
r_tricep_midpoint = {
    'right_e0': 3.05454 , 
    'right_e1': 1.278572, 
    'right_s0': 0.3225194, 
    'right_s1': -0.97983, 
    'right_w0': 3.0591412, 
    'right_w1': -1.2087768, 
    'right_w2': -0.13652429}
r_tricep_stretched = {
    'right_e0': 3.05453924387683, 
    'right_e1': 1.4016749449302968, 
    'right_s0': 0.051004861197190006, 
    'right_s1': -1.5297623407187289, 
    'right_w0': 3.0591411862404865, 
    'right_w1': -0.9825146946406075, 
    'right_w2': -0.06020874592450249}

# left tricep

l_tricep_extended = {
    'left_e0': -3.039, 
    'left_e1': 0.309, 
    'left_s0': -0.791, 
    'left_s1': 0.511, 
    'left_w0': -3.05, 
    'left_w1': -0.29 , 
    'left_w2': 0.094}
l_tricep_midpoint = {
    'left_e0': -3.05454 , 
    'left_e1': 1.278572, 
    'left_s0': -0.3225194, 
    'left_s1': -0.97983, 
    'left_w0': -3.0591412, 
    'left_w1': -1.2087768, 
    'left_w2': -0.13652429}
l_tricep_stretched = {
    'left_e0': -3.05453924387683, 
    'left_e1': 1.4016749449302968, 
    'left_s0': -0.051004861197190006, 
    'left_s1': -1.5297623407187289, 
    'left_w0': -3.0591411862404865, 
    'left_w1': -0.9825146946406075, 
    'left_w2': -0.06020874592450249}

# right deltoid/rotator cuff

r_deltoid_relaxed = {
    'right_e0': 0.3271214030165645, 
    'right_e1': 0.3336408213650775, 
    'right_s0': -0.6036214400329103, 
    'right_s1': 1.0538448012772792, 
    'right_w0': 2.462039164556089, 
    'right_w1': -0.26307770512234846, 
    'right_w2': 0.2876213977285151}
r_deltoid_midpoint = {
    'right_e0': 0.24658741165258025, 
    'right_e1': 0.03528155812136451, 
    'right_s0': -0.7781117546548761, 
    'right_s1': -0.0828980840765908, 
    'right_w0': 2.88158291004275, 
    'right_w1': -0.18867963690990588, 
    'right_w2': 0.21820876707670012}
r_deltoid_elevated = {
    'right_e0': 0.24658741165258025, 
    'right_e1': 0.03528155812136451, 
    'right_s0': -0.7781117546548761, 
    'right_s1': -0.8068738944277276, 
    'right_w0': 2.88158291004275,
    'right_w1': -0.18867963690990588, 
    'right_w2': 0.21820876707670012}
r_deltoid_overhead = {
    'right_e0': 0.31906800388016604, 
    'right_e1': -0.04947088040930459, 
    'right_s0': -0.692208830533293, 
    'right_s1': -1.5217089415823304, 
    'right_w0': 2.866626597360867, 
    'right_w1': 0.4126408319411763, 
    'right_w2': 0.022626216621309852}

# left deltoid/rotator cuff

l_deltoid_relaxed = {
    'left_e0': -0.3271214030165645, 
    'left_e1': 0.3336408213650775, 
    'left_s0': 0.6036214400329103, 
    'left_s1': 1.0538448012772792, 
    'left_w0': -2.462039164556089, 
    'left_w1': -0.26307770512234846, 
    'left_w2': 0.2876213977285151}
l_deltoid_midpoint = {
    'left_e0': -0.24658741165258025, 
    'left_e1': 0.03528155812136451, 
    'left_s0': 0.7781117546548761, 
    'left_s1': -0.0828980840765908, 
    'left_w0': -2.88158291004275, 
    'left_w1': -0.18867963690990588, 
    'left_w2': 0.21820876707670012}
l_deltoid_elevated = {
    'left_e0': -0.24658741165258025, 
    'left_e1': 0.03528155812136451, 
    'left_s0': 0.7781117546548761, 
    'left_s1': -0.8068738944277276, 
    'left_w0': -2.88158291004275,
    'left_w1': -0.18867963690990588, 
    'left_w2': 0.21820876707670012}
l_deltoid_overhead = {
    'left_e0': -0.31906800388016604, 
    'left_e1': -0.04947088040930459, 
    'left_s0': 0.692208830533293, 
    'left_s1': -1.5217089415823304, 
    'left_w0': -2.866626597360867, 
    'left_w1': 0.4126408319411763, 
    'left_w2': 0.022626216621309852}

# exercise functions

# neck stretch

def neck_stretch(a, b):
    head = baxter_interface.Head()
    for x in range (0,a):
        # wait at least half second for node to initialize before publishing 
        rospy.sleep(3 / b)
        head.command_nod()
        rospy.sleep(2 / b)
        head.set_pan(1.45, speed=0.05 * b, timeout=0)
        rospy.sleep(15 / b)
        head.command_nod()
        rospy.sleep(1 / b)
        head.set_pan(0.0, speed=0.05 * b, timeout=0)
        rospy.sleep(5 / b)
        head.command_nod()
        rospy.sleep(2 / b)
        head.set_pan(-1.45, speed=0.05 * b, timeout=0)
        rospy.sleep(15 / b)
        head.command_nod()
        rospy.sleep(1 / b)
        head.set_pan(0.0, speed=0.05 * b, timeout=0)
        rospy.sleep(5 / b)
    head.command_nod()
    rospy.sleep(3 / b)    

# bicep curl

def bicep_curl(a, b, c):
    if c == 'left' or c == 'both':
        limb = baxter_interface.Limb('left')    
        limb.set_joint_position_speed(0.2 * b)
        for x in range(0,a):
            limb.move_to_joint_positions(l_bicep_uncurled)
            rospy.sleep(2 / b)
            limb.move_to_joint_positions(l_bicep_curled)
            rospy.sleep(2 / b)             
        limb.move_to_joint_positions(l_bicep_uncurled)
    if c == 'right' or c == 'both':
        limb = baxter_interface.Limb('right')
        limb.set_joint_position_speed(0.2 * b)
        for x in range(0,a):         
            limb.move_to_joint_positions(r_bicep_uncurled)
            rospy.sleep(2 / b)
            limb.move_to_joint_positions(r_bicep_curled)
            rospy.sleep(2 / b)             
        limb.move_to_joint_positions(r_bicep_uncurled)
    rospy.sleep(2 / b)

# tricep stretch

def tricep_stretch(a, b, c):
    if c == 'left' or c == 'both':    
        limb = baxter_interface.Limb('left')
        for x in range (0,a):
            limb.set_joint_position_speed(0.15 * b)
            limb.move_to_joint_positions(l_tricep_extended)
            rospy.sleep(2 / b)    
            limb.move_to_joint_positions(l_tricep_midpoint)
            limb.set_joint_position_speed(0.075 * b)
            limb.move_to_joint_positions(l_tricep_stretched)
            rospy.sleep(2 / b)
        limb.move_to_joint_positions(l_tricep_extended) 
    if c == 'right' or c == 'both':
        limb = baxter_interface.Limb('right')
        for x in range (0,a):
            limb.set_joint_position_speed(0.15 * b)
            limb.move_to_joint_positions(r_tricep_extended)
            rospy.sleep(2 / b)    
            limb.move_to_joint_positions(r_tricep_midpoint)
            limb.set_joint_position_speed(0.075 * b)
            limb.move_to_joint_positions(r_tricep_stretched)
            rospy.sleep(2 / b)
        limb.move_to_joint_positions(r_tricep_extended) 
    rospy.sleep(2 / b)

# deltoid/rotator cuff stretch

def deltoid_stretch(a, b, c):
    if c == 'left' or c == 'both':
        limb = baxter_interface.Limb('left')    
        for x in range (0,a):
            limb.set_joint_position_speed(0.175 * b)
            limb.move_to_joint_positions(l_deltoid_relaxed)
            rospy.sleep(2.0 / b)
            limb.set_joint_position_speed(0.15 * b)
            limb.move_to_joint_positions(l_deltoid_midpoint)
            rospy.sleep(3.0 / b)
            limb.set_joint_position_speed(0.085 * b)
            limb.move_to_joint_positions(l_deltoid_elevated)
            limb.move_to_joint_positions(l_deltoid_overhead)
            rospy.sleep(3.0 / b)
            limb.set_joint_position_speed(0.1 * b)
            limb.move_to_joint_positions(l_deltoid_midpoint)
            rospy.sleep(2.0 / b)
        limb.move_to_joint_positions(l_deltoid_relaxed)
    if c == 'right' or c == 'both':
        limb = baxter_interface.Limb('right')
        for x in range (0,a):    
            limb.set_joint_position_speed(0.175 * b)
            limb.move_to_joint_positions(r_deltoid_relaxed)
            rospy.sleep(2.0 / b)
            limb.set_joint_position_speed(0.15 * b)
            limb.move_to_joint_positions(r_deltoid_midpoint)
            rospy.sleep(3.0 / b)
            limb.set_joint_position_speed(0.085 * b)
            limb.move_to_joint_positions(r_deltoid_elevated)
            limb.move_to_joint_positions(r_deltoid_overhead)
            rospy.sleep(3.0 / b)
            limb.set_joint_position_speed(0.1 * b)
            limb.move_to_joint_positions(r_deltoid_midpoint)
            rospy.sleep(2.0 / b)
        limb.move_to_joint_positions(r_deltoid_relaxed)
    rospy.sleep(2.0 / b)
 
# the part that actually does the stuff

print("\033[H\033[J")   # clears the screen
choices =[
    raw_input('Do neck stretchs (y/N)? '),
    raw_input('Do bicep curls (y/N)? '),
    raw_input('Do tricep stretches (y/N)? '),
    raw_input('Do deltoid stretches (y/N)? ')]   
if 'y' in choices[1:4]:
    arm = raw_input('Which arm(s) (left/right/both)? ')
if 'y' in choices:
    reps = int(raw_input('Repetitions for each exercise (1-10)? '))
    if reps < 1 or reps > 3:
        reps = 3
    smult = float(raw_input('Desired speed multiplier (0.25 - 3.00)? '))
    if smult < 0.25 or smult > 3.00:
        smult = 1
    if choices[0] == 'y':
        neck_stretch(reps, smult)
    if choices[1] == 'y':
        bicep_curl(reps, smult, arm)
    if choices[2] == 'y':
        tricep_stretch(reps, smult, arm)
    if choices[3] == 'y':
        deltoid_stretch(reps, smult, arm)

