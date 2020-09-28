import rospy
import pygame
import numpy as np
import sys
import time
from pkg_ta.msg import IMU_9_DOF

freq = 20
state = 'transition'

rospy.init_node('imu_calibration')

a_max = [-np.Inf, -np.Inf, -np.Inf]
a_min = [ np.Inf,  np.Inf,  np.Inf]
w_max = [-np.Inf, -np.Inf, -np.Inf]
w_min = [ np.Inf,  np.Inf,  np.Inf]

def callback(msg):
    global a_max, a_min
    global w_max, w_min

    if state == 'a_max_x':
        y = msg.accelerometer.x
        if a_max[0] <= y:
            a_max[0] = y
    elif state == 'a_max_y':
        y = msg.accelerometer.y
        if a_max[1] <= y:
            a_max[1] = y
    elif state == 'a_max_z':
        y = msg.accelerometer.z
        if a_max[2] <= y:
            a_max[2] = y

    elif state == 'a_min_x':
        y = msg.accelerometer.x
        if a_max[0] >= y:
            a_max[0] = y
    elif state == 'a_min_y':
        y = msg.accelerometer.y
        if a_max[1] >= y:
            a_max[1] = y
    elif state == 'a_min_z':
        y = msg.accelerometer.z
        if a_max[2] >= y:
            a_max[2] = y

    elif state == 'w_max_x':
        y = msg.gyroscope.x
        if w_max[0] <= y:
            w_max[0] = y
    elif state == 'w_max_y':
        y = msg.gyroscope.y
        if w_max[1] <= y:
            w_max[1] = y
    elif state == 'w_max_z':
        y = msg.gyroscope.z
        if w_max[2] <= y:
            w_max[2] = y

    elif state == 'w_min_x':
        y = msg.gyroscope.x
        if w_max[0] >= y:
            w_max[0] = y
    elif state == 'w_min_y':
        y = msg.gyroscope.y
        if w_max[1] >= y:
            w_max[1] = y
    elif state == 'w_min_z':
        y = msg.gyroscope.z
        if w_max[2] >= y:
            w_max[2] = y

rospy.Subscriber('/mpu9250_raw_data', IMU_9_DOF, callback)
r = rospy.Rate(freq) # Hz

print('')
print('Welcome ...')
print('This program is running for calibration purposes.')
print('For more information, please send an email to fadillahadam11@gmail.com')
print('')
print('WARNING: Please do the calibration step correctly !')
print('')

pygame.init()
screen = pygame.display.set_mode((800, 400))

pygame.display.set_caption('IMU Calibration')
background_colour = (255,255,255)
screen.fill(background_colour)
pygame.display.flip()

num_font = pygame.font.Font(None,36)

while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print('Exiting the program')
            pygame.quit()
            print('Thank you !')
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                state = 'a_max_x'
            elif event.key == pygame.K_w:
                state = 'a_max_y'
            elif event.key == pygame.K_e:
                state = 'a_max_z'
            elif event.key == pygame.K_r:
                state = 'a_min_x'
            elif event.key == pygame.K_t:
                state = 'a_min_y'
            elif event.key == pygame.K_y:
                state = 'a_min_z'
            elif event.key == pygame.K_a:
                state = 'w_max_x'
            elif event.key == pygame.K_s:
                state = 'w_max_y'
            elif event.key == pygame.K_d:
                state = 'w_max_z'
            elif event.key == pygame.K_f:
                state = 'w_min_x'
            elif event.key == pygame.K_g:
                state = 'w_min_y'
            elif event.key == pygame.K_h:
                state = 'w_min_z'
            elif event.key == pygame.K_RETURN:
                state = 'transition'

    screen.fill(background_colour)

    s_state = pygame.font.Font(None,50).render('State: ', True,(255,255,255),(0,0,0))
    screen.blit(s_state, (300,10))
    a_x = num_font.render('ax: ', True,(255,0,0),(255,255,255))
    a_y = num_font.render('ay: ', True,(255,0,0),(255,255,255))
    a_z = num_font.render('az: ', True,(255,0,0),(255,255,255))
    w_x = num_font.render('wx: ', True,(255,0,0),(255,255,255))
    w_y = num_font.render('wy: ', True,(255,0,0),(255,255,255))
    w_z = num_font.render('wz: ', True,(255,0,0),(255,255,255))
    screen.blit(a_x, (60,60))
    screen.blit(a_y, (60,90))
    screen.blit(a_z, (60,120))
    screen.blit(w_x, (60,200))
    screen.blit(w_y, (60,230))
    screen.blit(w_z, (60,260))

    s_state = pygame.font.Font(None,50).render(state, True,(255,255,0),(0,0,0))
    screen.blit(s_state, (425,10))

    a_x = num_font.render('{:.5f}/{:.5f} m/s2'.format(a_min[0],a_max[0]), True,(0,0,0),(255,255,255))
    a_y = num_font.render('{:.5f}/{:.5f} m/s2'.format(a_min[1],a_max[1]), True,(0,0,0),(255,255,255))
    a_z = num_font.render('{:.5f}/{:.5f} m/s2'.format(a_min[2],a_max[2]), True,(0,0,0),(255,255,255))
    w_x = num_font.render('{:.5f}/{:.5f} rad/s'.format(a_min[0],a_max[0]), True,(0,0,0),(255,255,255))
    w_y = num_font.render('{:.5f}/{:.5f} rad/s'.format(a_min[1],a_max[1]), True,(0,0,0),(255,255,255))
    w_z = num_font.render('{:.5f}/{:.5f} rad/s'.format(a_min[2],a_max[2]), True,(0,0,0),(255,255,255))
    screen.blit(a_x, (110,60))
    screen.blit(a_y, (110,90))
    screen.blit(a_z, (110,120))
    screen.blit(w_x, (110,200))
    screen.blit(w_y, (110,230))
    screen.blit(w_z, (110,260))

    # screen.fill(background_colour)
    pygame.display.update()
    # print('state: ' + state)
    # print('accel: {:.5f}/{:.5f} | {:.5f}/{:.5f} | {:.5f}/{:.5f}'.format(a_min[0],a_max[0],a_min[1],a_max[1],a_min[2],a_max[2]))
    # print('gyros: {:.5f}/{:.5f} | {:.5f}/{:.5f} | {:.5f}/{:.5f}'.format(w_min[0],w_max[0],w_min[1],w_max[1],w_min[2],w_max[2]))
    # print('-------------------------------------------------------------------')
    r.sleep()
