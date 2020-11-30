%% Animates the 2-link polishing robot
% Filename: animate.m
% Written By: Muhammad Saud Ul Hassan
% Dependencies: main.m
% Note: Run main.m to load L, psi, theta_1 and theta_2 into the work space,
% which are required for drawing the 2-link mechanism here

% Defining the lengths of the two links
l1 = L;  % Length of Link 1
l2 = 2*L;  % Length of Link 2

% Defining the coordinates of joint_1, joint_2 and the arm's end effector
joint1_x = 0;
joint1_y = 0;
joint2_x = l1*cos(theta_1);
joint2_y = l1*sin(theta_1);
endEffector_x = joint2_x + l2*cos(theta_2);
endEffector_y = joint2_y + l2*sin(theta_2);

% Maximum length reached by the arm (used to scale the plot axis later)
l_m = l1 + l2;

% set the frames per second for the animation
fps = 30;

% Setting up the video file to write the animation to
myVideo = VideoWriter('Polisher');
myVideo.FrameRate = fps;
open(myVideo) 

% Cycle through arm configrations. (Because of the sheer number of time
% points, the animation would be very slow if the arm was drawn at each
% time point. Therefore, only every 30th time point is used here)
for i = 1:30:length(theta_1)
        
    % Draw the solar panel slope
    plot([1.5*L (l_m+0.5)], [0 tan(psi)*(l_m+0.5)-1.5*tan(psi)*L], 'k-', 'LineWidth', 1);
    hold on;    
    
    % Draw a horizontal line passing through (0,0) as the reference
    plot([-(l_m+0.5) (l_m+0.5)], [0 0], 'k-', 'LineWidth', 0.5);
    
    % Draw the current arm configration
    plot([joint1_x joint2_x(i) endEffector_x(i)], [joint1_y joint2_y(i) endEffector_y(i)], 'r.-', 'MarkerSize', 20, 'LineWidth', 2);
    hold off;
    
    % plot settings
    axis equal;
    axis ([-0.5 (l_m+0.5) -0.5 (l_m+0.5)]);
    
    drawnow;
    pause(1/fps);
        
    % Write the frame to the video file
    frame = getframe(gcf);
    writeVideo(myVideo, frame);

end

close(myVideo)