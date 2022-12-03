g = 9.8;

j_1_dh = [0 0 0 0];
joint_1  = Robot(j_1_dh,[0],[0]);
j_1 = joint_1.jacobians_analytical()
f1 = j_1'*[0 0 .33*g 0 0 0]'

j_1_mid_dh = [0 pi/2 56.05 0; 28.025 0 51.775 -pi/2];
joint_1_mid = Robot(j_1_mid_dh, [0;0],[0;0]);
j_1_mid  =joint_1_mid.jacobians_analytical();
f1_mid = j_1_mid'*[0 0 .225*g 0 0 0]'

j_2_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; -165.15 0 -105.55 0];
joint_2 = Robot(j_2_dh,[0;0;0], [0;0;0]);
j_2 = joint_2.jacobians_analytical();
f2 = j_2'*[0 0 .6*g 0 0 0]'

j_2_mid_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 0 -73.05 0; -127.05 0 30.5 0];
joint_2_mid = Robot(j_2_mid_dh,[0;0;0;0],[0;0;0;0]);
j_2_mid = joint_2_mid.jacobians_analytical();
f2_mid = j_2_mid'*[0 0 .169*g 0 0 0]'

j_3_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0;0 0 -67.025 0];
joint_3 = Robot(j_3_dh,[0;0;0;0],[0;0;0;0]);
j_3 = joint_3.jacobians_analytical();
f3 = j_3'*[0 0 .58*g 0 0 0]'

j_3_mid_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0;-127.5 0 30.5 0];
joint_3_mid = Robot(j_3_mid_dh,[0;0;0;0],[0;0;0;0]);
j_3_mid = joint_3_mid.jacobians_analytical();
f3_mid = j_3_mid'*[0 0 .14*g 0 0 0]'

j_4_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0; 0 0 60.83 0];
joint_4 = Robot(j_4_dh,[0;0;0;0],[0;0;0;0]);
j_4 = joint_4.jacobians_analytical();
f4 = j_4'*[0 0 .49*g 0 0 0]'

j_4_mid_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0; 0 pi/2 91 pi/2; 30.3 0 37.025 pi/2];
joint_4_mid = Robot(j_4_mid_dh,[0;0;0;0;0],[0;0;0;0;0]);
j_4_mid = joint_4_mid.jacobians_analytical();
f4_mid = j_4_mid'*[0 0 .054*g 0 0 0]'

j_5_dh  = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0; 0 pi/2 91 pi/2; 0 0 74.05 0];
joint_5 = Robot(j_5_dh,[0;0;0;0;0],[0;0;0;0;0]);
j_5 = joint_5.jacobians_analytical();
f5 = j_5'*[0 0 .4*g 0 0 0]'

j_5_mid_dh = [0 pi/2 56.05 0; 330.3 pi 0 0; 254.1 pi 0 0; 0 pi/2 91 pi/2; 0 0 143.9 0];
joint_5_mid = Robot(j_5_mid_dh, [0;0;0;0;0],[0;0;0;0;0]);
j_5_mid = joint_5_mid.jacobians_analytical()
f5_mid = j_5_mid'*[0 0 .03*g 0 0 0]'

final_torq = f1 + f1_mid + f2 + f2_mid + f3 + f3_mid + f4 + f4_mid + f5 + f5_mid



ee_dh = [0 pi/2 56.05 0; 330.3 pi 0 pi/2; 254.1 pi 0 pi/2; 0 pi/2 91 pi/2; 0 0 287.8 0];
ee = Robot(ee_dh, [0;0;0;0;0],[0;0;0;0;0]);
j_ee = ee.jacobians_analytical()
sqrt(det(j_ee*j_ee'))

