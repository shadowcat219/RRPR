
%output: graphs of each joint
% Matlab must point to the correct file l ocation
clc
clear
%Read Files

close all

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f %f'; 
    fileSize = [5 Inf];

%open the file for reading, and obtain the file identifier, fileID.
   fileID1 = fopen('Actual_Joints__thetaMM.txt','r');
    fileID1 = fopen('Desired_Joints_thetaMM.txt');
        Location_desired=fscanf(fileID1,formatSpec, fileSize);
    fclose(fileID1);
    fileID2 = fopen('Desired_Velocity_thetaMM.txt');
        Velocity_desired=fscanf(fileID2,formatSpec, fileSize);
    fclose(fileID2);
    fileID3 = fopen('Desired_Accel_thetaMM.txt');
    Acceleration_desired=fscanf(fileID3,formatSpec, fileSize);
    fclose(fileID3);
    fileID4 = fopen('Actual_Joints_thetaMM.txt');
        Location_actual = fscanf(fileID4,formatSpec, fileSize);
    fclose(fileID4);
    fileID5 = fopen('Actual_Velocity_thetaMM.txt');
        Velocity_actual=fscanf(fileID5,formatSpec, fileSize);
    fclose(fileID5);
    fileID6 = fopen('Actual_Accel_thetaMM.txt');
        Acceleraton_actual=fscanf(fileID6,formatSpec, fileSize);
    fclose(fileID6);
    


    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.


 %  Transpose the array so that * matches the orientation of the data in the file.
    Location_desired =Location_desired';
    Velocity_desired= Velocity_desired';
    Acceleration_desired= Acceleration_desired';
    Location_actual=Location_actual';
    Velocity_actual= Velocity_actual';
    Acceleraton_actual=Acceleraton_actual';

  %%
   figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,Location_desired(:,1), Location_desired(:,2),'k',Location_actual(:,1), Location_actual(:,2),'b')
	ylabel(ax1,strcat('Joint ','1'))
	xlabel(ax1,'time(sec)' )

	plot(ax2,Location_desired(:,1),Velocity_desired(:,2),'k',Location_actual(:,1), Velocity_actual(:,2),'b')
    ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(sec)' )
   
	plot(ax3,Location_desired(:,1), Acceleration_desired(:,2),'k',Location_actual(:,1), Acceleraton_actual(:,2),'b')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(sec)' )


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
      