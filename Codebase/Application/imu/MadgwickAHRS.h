//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __MadgwickAHRS_h
#define __MadgwickAHRS_h

void madgwick_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void madgwick_computeAngles(float *roll, float *pitch, float *yaw);

#endif
