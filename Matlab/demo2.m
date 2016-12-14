%Get Matlab to read text file
%output: graphs of each joint
% Matlab must point to the correct file l ocation

%Read Files

close all
%http://www.mathworks.com/help/matlab/ref/fscanf.html 
    fileID2 = fopen('numPoints.txt','r');
    formatSpec = '%f';
    numPoints=fscanf(fileID2,formatSpec, 1);
    fclose(fileID2);
    numPoints=numPoints'


%open the file for reading, and obtain the file identifier, fileID.
    fileID1 = fopen('Desired_Joints_XYZ.txt','r');
    % this file contains information in columns of 
    %[time x y z phi]
    
    
    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f %f'; 
    fileSize = [5 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    Location = fscanf(fileID1,formatSpec, fileSize)
    fclose(fileID1);   

    %Transpose the array so that * matches the orientation of the data in the file.
   Location = Location'


%% Plot data
	figure % new figure
    plot3(Location(:,2),Location(:,3),Location(:,4))
    title('Plotted Trajctory')
    ylim([-150 339])
    xlim([-150 339])
    zlim([0 125])
    ylabel('y-axis')
	xlabel('x-axis')
    zlabel('z-axis')
    
    
    %%
    
    
    %open the file for reading, and obtain the file identifier, fileID.
    fileID1 = fopen('Desired_Joints_thetaMM.txt','r');
    % this file contains information in columns of 
    %[time x y z phi]
    
    
    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f %f'; 
    fileSize = [5 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    Location = fscanf(fileID1,formatSpec, fileSize)
    fclose(fileID1);   

    %Transpose the array so that * matches the orientation of the data in the file.
   Location = Location'
    
    figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,Location(:,1),Location(:,2),'-*k')
	title(ax1,strcat('Theta ','1'))

	ylabel(ax1,strcat('Theta ','1'))
	xlabel(ax1,'time(sections)' )

	plot(ax2,Location(:,1),Location(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(seconds)' )

	plot(ax3,Location(:,1),Location(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(seconds)' )

     figure 
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,Location_desired(:,1), Location_desired(:,3),'k',Location_actual(:,1), Location_actual(:,3),'b')
	ylabel(ax1,strcat('Joint  ','2'))
	xlabel(ax1,'time(sec)' )

	plot(ax2,Location_desired(:,1),Velocity_desired(:,3),'k',Location_actual(:,1), Velocity_actual(:,3),'b')
    ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(sec)' )

	plot(ax3,Location_desired(:,1), Acceleration_desired(:,3),'k',Location_actual(:,1), Acceleraton_actual(:,3),'b')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(sec)' )

figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,Location_desired(:,1), Location_desired(:,4),'k',Location_actual(:,1), Location_actual(:,4),'b')
	ylabel(ax1,strcat('Joint ','3'))
	xlabel(ax1,'time(sec)' )

	plot(ax2,Location_desired(:,1),Velocity_desired(:,4),'k',Location_actual(:,1), Velocity_actual(:,4),'b')
    ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(sec)' )

	plot(ax3,Location_desired(:,1), Acceleration_desired(:,4),'k',Location_actual(:,1),  Acceleraton_actual(:,4),'b')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(sec)' )
     
figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,Location_desired(:,1), Location_desired(:,5),'k',Location_actual(:,1), Location_actual(:,5),'b')
	ylabel(ax1,strcat('Joint ','4'))
	xlabel(ax1,'time(sec)' )

	plot(ax2,Location_desired(:,1),Velocity_desired(:,5),'k',Location_actual(:,1), Velocity_actual(:,5),'b')
    ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(sec)' )

	plot(ax3,Location_desired(:,1), Acceleration_desired(:,5),'k',Location_actual(:,1), Acceleraton_actual(:,5),'b')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(sec)' )
      
