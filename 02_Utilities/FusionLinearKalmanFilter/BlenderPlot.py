'''
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

The Linear Kalman Filter implementation is provided by LIKE, Friedrich Alexander Universität Erlangen, Germany
'''

import matplotlib
import matplotlib.pyplot as plt
import glob

# Set Matplotlib Font Style:
font = {'family' : 'sans-serif',
        'size'   : 22}

matplotlib.rc('font', **font)



for filename in glob.glob("./Blender_*.txt"):
    with open(filename, 'r') as f:
        lines = f.readlines()
        # first line contains the scalar values for RMS Errors
        rmse_values = lines[0].split() 
        rmse_gps_value_x,rmse_gps_value_y,rmse_gps_value = rmse_values[0],rmse_values[1],rmse_values[2]
        rmse_dso_value_x,rmse_dso_value_y,rmse_dso_value_z,rmse_dso_value = rmse_values[3],rmse_values[4],rmse_values[5],rmse_values[6]
        rmse_filtered_value_x,rmse_filtered_value_y,rmse_filtered_value_z,rmse_filtered_value = rmse_values[7],rmse_values[8],rmse_values[9],rmse_values[10]
        rmse_velocity_value_x,rmse_velocity_value_y,rmse_velocity_value_z,rmse_velocity_value = rmse_values[11],rmse_values[12],rmse_values[13],rmse_values[14]
        #remove first element from list
        lines.pop(0)
        
        frames = [ float(line.split()[0]) for line in lines]
        GT_x = [ float(line.split()[1]) for line in lines]
        GT_y = [ float(line.split()[2]) for line in lines]
        GT_z = [ float(line.split()[3]) for line in lines]
        GPS_x = [ float(line.split()[4]) for line in lines]
        GPS_y = [ float(line.split()[5]) for line in lines]
        DSO_x = [ float(line.split()[6]) for line in lines]
        DSO_y = [ float(line.split()[7]) for line in lines]
        DSO_z = [ float(line.split()[8]) for line in lines]
        KF_x = [ float(line.split()[9]) for line in lines]
        KF_y = [ float(line.split()[10]) for line in lines]
        KF_z = [ float(line.split()[11]) for line in lines]
        RMSE_GPS_x = [ float(line.split()[12]) for line in lines]
        RMSE_GPS_y = [ float(line.split()[13]) for line in lines]
        RMSE_DSO_x = [ float(line.split()[14]) for line in lines]
        RMSE_DSO_y = [ float(line.split()[15]) for line in lines]
        RMSE_DSO_z = [ float(line.split()[16]) for line in lines]
        RMSE_Kalman_x = [ float(line.split()[17]) for line in lines]
        RMSE_Kalman_y = [ float(line.split()[18]) for line in lines]
        RMSE_Kalman_z = [ float(line.split()[19]) for line in lines]
        GT_Velocity_x = [ float(line.split()[20]) for line in lines]
        GT_Velocity_y = [ float(line.split()[21]) for line in lines]
        GT_Velocity_z = [ float(line.split()[22]) for line in lines]
        VELOCITY_x = [ float(line.split()[23]) for line in lines]
        VELOCITY_y = [ float(line.split()[24]) for line in lines]
        VELOCITY_z = [ float(line.split()[25]) for line in lines]
        RMSE_Velocity_x = [ float(line.split()[26]) for line in lines]
        RMSE_Velocity_y = [ float(line.split()[27]) for line in lines]
        RMSE_Velocity_z = [ float(line.split()[28]) for line in lines]
        
    # Dissect filename to use as caption in plots:
    filename_parts = filename.replace(".txt", "").split("_")
    title = filename_parts[1]
    title = filename_parts[2]
    initx = filename_parts[3]
    inity = filename_parts[4]
    initz = filename_parts[5]
    gps_freq = filename_parts[6]
    sigma_p_gps = filename_parts[7]
    sigma_p_dso = filename_parts[8]
    sigma_x0 = filename_parts[9]
    sigma_v0 = filename_parts[10]
    sigma_wx = filename_parts[11]
    sigma_wv = filename_parts[12]
    
    caption = "$init_x=$" + initx + ", $init_y=$" + inity + ", $init_z=$" + initz + ", $GPS_{stride}=$" + gps_freq
    caption += ", $\sigma_{P,GPS}=$" + sigma_p_gps + ", $\sigma_{P,DSO}=$" + sigma_p_dso
    caption += ", $\sigma_{x_0}=$" + sigma_x0 + ", $\sigma_{v_0}=$" + sigma_v0
    caption += ", $\sigma_{w_x}=$" + sigma_wx + ", $\sigma_{w_v}=$" + sigma_wv
    caption_position = title + ': position plots\n(' + caption + ')'
    caption_velocity = title + ': velocity plots\n(' + caption + ')'
    caption_rmse = title + ': error plots\n(' + caption + ')'
    
    # Determine what kind of data we have here:
    gps, dso = False, False
    if title == "GPS":
        gps = True
    elif title == "DSO":
        dso = True
    else:
        gps, dso = True, True
    
    #############
    #
    # Plots for the printed version
    #
    #############
    
    
    figure = plt.figure(figsize=(16, 14))
    Error_plotSize = (2,6) # rows x cols (six because of flexibility in colspan)
    
    # 2D map
    ax1 = plt.subplot2grid(Error_plotSize, (0,0), colspan=6) # figError.add_subplot(3,3,7)
    
    # KalmanFilter
    ax2 = plt.subplot2grid(Error_plotSize, (1,0), colspan=2) # figError.add_subplot(3,3,7)
    ax3 = plt.subplot2grid(Error_plotSize, (1,2), colspan=2) # figError.add_subplot(3,3,8)
    ax4 = plt.subplot2grid(Error_plotSize, (1,4), colspan=2) # figError.add_subplot(3,3,9)

    
    
    # 2D positions
    #ax1.set_title('top view (x,y plane)')
    ax1.plot(GT_x, GT_y, 'k', label='reference')
    if gps:
        ax1.plot(GPS_x, GPS_y, '.r', label='GPS')
    if dso:
        ax1.plot(DSO_x, DSO_y, color='skyblue', marker='.', label='DSO')
    ax1.plot(KF_x, KF_y, 'g', label='prediction')
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.legend()
    
    

    # KalmanFilter x
    #ax2.set_title('error in x position')
    ax2.plot(RMSE_Kalman_x, 'g', label='prediction')
    ax2.set_xlabel('frame [k]')
    ax2.set_ylabel('x error [m]')
    ax2.legend()
    ax2.grid()
    # KalmanFilter y
    #ax3.set_title('error in y position')
    ax3.plot(RMSE_Kalman_y, 'g', label='prediction')
    ax3.set_xlabel('frame [k]')
    ax3.set_ylabel('y error [m]')
    ax3.legend()
    ax3.grid()
    # KalmanFilter z
    #ax4.set_title('error in z position')
    ax4.plot(RMSE_Kalman_z, 'g', label='prediction')
    ax4.set_xlabel('frame [k]')
    ax4.set_ylabel('z error [m]')
    ax4.legend()
    ax4.grid()

    figure.tight_layout() # WILL not work with different sizes of subplot
    plt.savefig(filename.replace(".txt", "_print") + ".pdf", format='pdf')
    # Prevents "out of memory" error, by clearing the current figure
    plt.close(figure)
    plt.clf()

    
    ################################# End of plots for printed version ##########################################
    
    
    
    
    
    #############
    #
    # More detailed plots follow now (not used in the written document), because too many plots
    #
    #############
    
    
    # Plots of position values
    ##############
    fig1 = plt.figure(figsize=(16, 9))
    ax1 = fig1.add_subplot(2,1,1)
    ax2 = fig1.add_subplot(2,3,4)
    ax3 = fig1.add_subplot(2,3,5)
    ax4 = fig1.add_subplot(2,3,6)

    ax1.set_title('top view (x,y plane)')
    ax1.plot(GT_x, GT_y, 'k', label='reference')
    if gps:
        ax1.plot(GPS_x, GPS_y, '.r', label='GPS')
    if dso:
        ax1.plot(DSO_x, DSO_y, color='skyblue', marker='.', label='DSO')
    ax1.plot(KF_x, KF_y, 'g', label='prediction')
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.legend()

    ax2.set_title('x position')
    ax2.plot(frames, GT_x, 'k', label='reference')
    if gps:
        ax2.plot(frames, GPS_x, '.r', label='GPS')
    if dso:
        ax2.plot(frames, DSO_x, color='skyblue', marker='.', label='DSO')
    ax2.plot(frames, KF_x, 'g', label='prediction')
    ax2.set_xlabel('frame [k]')
    ax2.set_ylabel('x [m]')
    ax2.legend()

    ax3.set_title('y position')
    ax3.plot(frames, GT_y, 'k', label='reference')
    if gps:
        ax3.plot(frames, GPS_y, '.r', label='GPS')
    if dso:
        ax3.plot(frames, DSO_y, color='skyblue', marker='.', label='DSO')
    ax3.plot(frames, KF_y, 'g', label='prediction')
    ax3.set_xlabel('frame [k]')
    ax3.set_ylabel('y [m]')
    ax3.legend()

    ax4.set_title('z position')
    ax4.plot(frames, GT_z, 'k', label='reference')
    if dso:
        ax4.plot(frames, DSO_z, color='skyblue', marker='.', label='DSO')
    ax4.plot(frames, KF_z, 'g', label='prediction')
    ax4.set_xlabel('frame [k]')
    ax4.set_ylabel('z [m]')
    ax4.legend()

    plt.tight_layout() # Make sure spacing between plots is re-calculated
    #plt.suptitle(caption_position, fontsize=16)
    #plt.subplots_adjust(wspace=0.1, top=0.85)
    plt.savefig(filename.replace(".txt", "_positions") + ".pdf", format='pdf')
    
    # Prevents "out of memory" error, by clearing the current figure
    plt.close(fig1)
    plt.clf()

    
    # Velocity Plots
    ##############
    fig12 = plt.figure(figsize=(16, 9))
    ax1 = fig12.add_subplot(3,1,1)
    ax2 = fig12.add_subplot(3,1,2)
    ax3 = fig12.add_subplot(3,1,3)
    
    # Velocity x
    ax1.set_title('velocity x')
    ax1.plot(GT_Velocity_x, 'k', label='reference')
    ax1.plot(VELOCITY_x, 'g', label='prediction')
    ax1.set_xlabel('frame [k]')
    ax1.set_ylabel('$v_x$ [m/s]')
    ax1.legend()
    # Velocity y
    ax2.set_title('velocity y')
    ax2.plot(GT_Velocity_y, 'k', label='reference')
    ax2.plot(VELOCITY_y, 'g', label='prediction')
    ax2.set_xlabel('frame [k]')
    ax2.set_ylabel('$v_y$ [m/s]')
    ax2.legend()
    # Velocity z
    ax3.set_title('velocity z')
    ax3.plot(GT_Velocity_z, 'k', label='reference')
    ax3.plot(VELOCITY_z, 'g', label='prediction')
    ax3.set_xlabel('frame [k]')
    ax3.set_ylabel('$v_z$ [m/s]')
    ax3.legend()
    
    plt.tight_layout() # Make sure spacing between plots is re-calculated
    #plt.suptitle(caption_velocity + "\nOverall velocity error: " + rmse_velocity_value + "m", fontsize=16)
    #plt.subplots_adjust(wspace=0.1, top=0.85)
    plt.savefig(filename.replace(".txt", "_velocities") + ".pdf", format='pdf')

    # Prevents "out of memory" error, by clearing the current figure
    plt.close(fig12)
    plt.clf()
    
        
    # RMSE Plots
    ##############
    fig2 = plt.figure(figsize=(16, 9))
    RMSE_plotSize = (3,6) # rows x cols (six because of flexibility in colspan)
    
    # GPS
    ax1 = plt.subplot2grid(RMSE_plotSize, (0,0), colspan=3) # fig2.add_subplot(3,2,1)
    ax2 = plt.subplot2grid(RMSE_plotSize, (0,3), colspan=3) # fig2.add_subplot(3,2,2)
    # DSO
    ax3 = plt.subplot2grid(RMSE_plotSize, (1,0), colspan=2) # fig2.add_subplot(3,3,4)
    ax4 = plt.subplot2grid(RMSE_plotSize, (1,2), colspan=2) # fig2.add_subplot(3,3,5)
    ax5 = plt.subplot2grid(RMSE_plotSize, (1,4), colspan=2) # fig2.add_subplot(3,3,6)
    # KalmanFilter
    ax6 = plt.subplot2grid(RMSE_plotSize, (2,0), colspan=2) # fig2.add_subplot(3,3,7)
    ax7 = plt.subplot2grid(RMSE_plotSize, (2,2), colspan=2) # fig2.add_subplot(3,3,8)
    ax8 = plt.subplot2grid(RMSE_plotSize, (2,4), colspan=2) # fig2.add_subplot(3,3,9)

    if gps:
        # GPS x
        #ax1.set_title('RMSE error = ' + rmse_gps_value_x + "m")
        ax1.plot(RMSE_GPS_x, 'r', label='GPS')
        ax1.set_xlabel('frame [k]')
        ax1.set_ylabel('x error [m]')
        ax1.legend()
        ax1.grid()
        # GPS y
        #ax2.set_title('RMSE error = ' + rmse_gps_value_y + "m")
        ax2.plot(RMSE_GPS_y, 'r', label='GPS')
        ax2.set_xlabel('frame [k]')
        ax2.set_ylabel('y error [m]')
        ax2.legend()
        ax2.grid()
    
    if dso:
        # DSO x
        #ax3.set_title('RMSE error = ' + rmse_dso_value_x + "m")
        ax3.plot(RMSE_DSO_x, color='skyblue', label='DSO')
        ax3.set_xlabel('frame [k]')
        ax3.set_ylabel('x error [m]')
        ax3.legend()
        ax3.grid()
        # DSO y
        #ax4.set_title('RMSE error = ' + rmse_dso_value_y + "m")
        ax4.plot(RMSE_DSO_y, color='skyblue', label='DSO')
        ax4.set_xlabel('frame [k]')
        ax4.set_ylabel('y error [m]')
        ax4.legend()
        ax4.grid()
        # DSO z
        #ax5.set_title('RMSE error = ' + rmse_dso_value_z + "m")
        ax5.plot(RMSE_DSO_z, color='skyblue', label='DSO')
        ax5.set_xlabel('frame [k]')
        ax5.set_ylabel('z error [m]')
        ax5.legend()
        ax5.grid()
    
    # KalmanFilter x
    #ax6.set_title('RMSE errors = ' + rmse_filtered_value_x + "m")
    ax6.plot(RMSE_Kalman_x, 'g', label='prediction')
    ax6.set_xlabel('frame [k]')
    ax6.set_ylabel('x error [m]')
    ax6.legend()
    ax6.grid()
    # KalmanFilter y
    #ax7.set_title('RMSE errors = ' + rmse_filtered_value_y + "m")
    ax7.plot(RMSE_Kalman_y, 'g', label='prediction')
    ax7.set_xlabel('frame [k]')
    ax7.set_ylabel('y error [m]')
    ax7.legend()
    ax7.grid()
    # KalmanFilter z
    #ax8.set_title('RMSE errors = ' + rmse_filtered_value_z + "m")
    ax8.plot(RMSE_Kalman_z, 'g', label='prediction')
    ax8.set_xlabel('frame [k]')
    ax8.set_ylabel('z error [m]')
    ax8.legend()
    ax8.grid()
    
    # TODO: Use "subplot2grid" instead of "add_subplot" to get the spacings right
    fig2.tight_layout() # WILL not work with different sizes of subplot
    cumulativeRMSE = " Prediction:" + str(rmse_filtered_value)+"m"
    if dso:
        cumulativeRMSE = " DSO:" + str(rmse_dso_value) + "m;" + cumulativeRMSE
    if gps:
        cumulativeRMSE = " GPS:" + str(rmse_gps_value) + "m;" + cumulativeRMSE
    
    #plt.suptitle(caption_rmse + "\nAccumulated:" + cumulativeRMSE, fontsize=16)
    #plt.subplots_adjust(wspace=0.1, top=0.85)
    plt.savefig(filename.replace(".txt", "_errors") + ".pdf", format='pdf')
    # Prevents "out of memory" error, by clearing the current figure
    plt.close(fig2)
    plt.clf()

    #plt.show()