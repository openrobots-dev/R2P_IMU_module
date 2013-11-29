#%QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
#%
#%   q = quatern2euler(q)
#%
#%   Converts a quaternion orientation to ZYX Euler angles where phi is a
#%   rotation around X, theta around Y and psi around Z.
#%
#%   For more information see:
#%   http://www.x-io.co.uk/node/8#quaternions
#%
#%	Date          Author          Notes
#%	27/09/2011    SOH Madgwick    Initial release

# taken from madgwick_algorithm_matlab\quaternion_library\quatern2euler.m

from pyprocessing import *

phi = 0.0; theta = 0.0; psi = 0.0;
def quaternionToEuler( qt1, qt2, qt3, qt4):
    global phi; global theta; global psi;
    #R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    r11 = 2*(qt1*qt1)-1+2*(qt2*qt2);

    #R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    r21 = 2*(qt2*qt3-qt1*qt4);

    #R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    r31  = 2*(qt2*qt4+qt1*qt3);
    if(r31 > 1):
        print ("stop here")

    #R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    r32  = 2*(qt3*qt4-qt1*qt2);

    #R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    r33 = 2*(qt1*qt1)-1+2*(qt4*qt4);

    #phi = atan2(R(3,2,:), R(3,3,:) );
    phi = atan2(r32, r33 ); # phi = z
    #theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    theta = -atan(r31 / sqrt(1-r31*r31) ); # theta = y
    #psi = atan2(R(2,1,:), R(1,1,:) );
    psi = atan2(r21, r11); # psi = x

    #--------------------------------
    #psi = atan2(2 * qt2 * qt3 - 2 * qt1 * qt4, 2 * qt1*qt1 + 2 * qt2 * qt2 - 1) #// psi

    #theta = -asin(2 * qt2 * qt4 + 2 * qt1 * qt3) #// theta

    #phi = atan2(2 * qt3 * qt4 - 2 * qt1 * qt2, 2 * qt1 * qt1 + 2 * qt4 * qt4 - 1) #// phi