/*
 * Bao cao: Bo uoc luong tu the dung thuat toan Madgwick de tinh quaternion va goc roll/pitch/yaw tu IMU.
 */

#include "ARHS.h"

#define ARHS_NORM_EPSILON 1.0e-12f

static uint8_t ARHS_InvSqrtSafe(float32_t i, float32_t *out)
{
	float32_t sqrt;

	if (!(i > ARHS_NORM_EPSILON)) {
		return 0U;
	}
	if (arm_sqrt_f32(i, &sqrt) != ARM_MATH_SUCCESS || !(sqrt > 0.0f)) {
		return 0U;
	}

	*out = 1.0f / sqrt;
	return 1U;
}

static void ARHS_ResetQuaternion(float32_t *q0, float32_t *q1, float32_t *q2, float32_t *q3)
{
	*q0 = 1.0f;
	*q1 = 0.0f;
	*q2 = 0.0f;
	*q3 = 0.0f;
}

void Norm_Q(float32_t *q){
    float32_t q_square = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    float32_t invert_q;

    if (!ARHS_InvSqrtSafe(q_square, &invert_q)) {
    	q[0] = 1.0f;
    	q[1] = 0.0f;
    	q[2] = 0.0f;
    	q[3] = 0.0f;
    	return;
    }

    q[0] *= invert_q;
    q[1] *= invert_q;
    q[2] *= invert_q;
    q[3] *= invert_q;
}

void EQ_ZYX(float32_t* ypr, float32_t *q){  
	  float32_t c1 = cosf(ypr[0] / 2);
	  float32_t s1 = sinf(ypr[0] / 2);
	  float32_t c2 = cosf(ypr[1] / 2);
	  float32_t s2 = sinf(ypr[1] / 2);
	  float32_t c3 = cosf(ypr[2] / 2);
	  float32_t s3 = sinf(ypr[2] / 2);
	  q[0] = c1*c2*c3+s1*s2*s3;
	  q[1] = c1*c2*s3-s1*s2*c3;
	  q[2] = c1*s2*c3+s1*c2*s3;
	  q[3] = s1*c2*c3-c1*s2*s3;
	  Norm_Q(q);
}


void QE_ZYX(float32_t *q,float32_t* ypr) {  
	  float32_t q0 = q[0];
	  float32_t q1 = q[1];
	  float32_t q2 = q[2];
	  float32_t q3 = q[3];

	  ypr[2] = atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
	  float32_t sinp = -2*(q1*q3-q0*q2);
	  if (sinp >= 1.0f) ypr[1] = (float)(M_PI/2.0);
	  else if (sinp <= -1.0f) ypr[1] = -(float)(M_PI/2.0);
	  else ypr[1] = asinf(sinp);
	  ypr[0] = atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
}
void Complimentary_Filter_Reset(Complimentary_Filter_t * imu){
	imu->q[0] = 1;
	for(int i = 0 ; i < 3 ; i ++){
		imu->alpha[i] = 0.98f;
		imu->Euler_Angle_Deg[i] = 0;
		imu->Euler_Angle_Rad[i] = 0;
		imu->predict_count = 0;
		imu->q[i + 1] = 0;
		imu->update_count = 0;
	}
	imu->status = Fusion_START;
	imu->Fusion_OK = 0;
}
void Complimentary_Filter_Predict(Complimentary_Filter_t * imu ,IMU_Data_t * imu_data){
	   float32_t a_square = imu_data->acc[0] * imu_data->acc[0] + imu_data->acc[1] * imu_data->acc[1] + imu_data->acc[2] * imu_data->acc[2];
	   float32_t a_norm_ = sqrt(a_square);
	   float32_t a_norm[3];
	   float32_t p = imu_data->w[0];
	   float32_t q = imu_data->w[1];
	   float32_t r = imu_data->w[2];
	   float32_t Phi_Acc,Theta_Acc;
	   float32_t phi,theta;
	   for(int i = 0 ; i < 3 ; i ++){
		a_norm[i] = imu_data->acc[i]/a_norm_;
	   }
		switch(imu->status){
		  case Fusion_START:
			  
			  Complimentary_Filter_Reset(imu);
				
				imu->Euler_Angle_Rad[0] = atan2f(-a_norm[1], -a_norm[2]);
				imu->Euler_Angle_Rad[1] = atan2f(a_norm[0],sqrtf(a_norm[1]*a_norm[1] + a_norm[2]*a_norm[2]));
				imu->Euler_Angle_Deg[0] = imu->Euler_Angle_Rad[0] * RAD_TO_DEG;
				imu->Euler_Angle_Deg[1] = imu->Euler_Angle_Rad[1] * RAD_TO_DEG;
			  imu->status = Fusion_RUN;
			  break;
		  case Fusion_RUN:
			  

			  Phi_Acc = atan2f(-a_norm[1], -a_norm[2]);
			  Theta_Acc = atan2f(a_norm[0],sqrtf(a_norm[1]*a_norm[1] + a_norm[2]*a_norm[2]));

			  phi = imu->Euler_Angle_Rad[0];
			  theta = imu->Euler_Angle_Rad[1];

			  float32_t phi_dot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
			  float32_t theta_dot = cos(phi) * q - sin(phi) * r;
			  float32_t psi_dot = (sin(phi) * q + cos(phi) * r)/cos(theta);

			  imu->Euler_Angle_Rad[0] += 0.5 * phi_dot * imu_data->dt;
			  imu->Euler_Angle_Rad[0] = imu->alpha[0] * imu->Euler_Angle_Rad[0] + (1.0 - imu->alpha[0]) * Phi_Acc;
			  imu->Euler_Angle_Rad[1] += 0.5 * theta_dot * imu_data->dt;
			  imu->Euler_Angle_Rad[1] = imu->alpha[1] * imu->Euler_Angle_Rad[1] + (1.0 - imu->alpha[1]) * Theta_Acc;
			  imu->Euler_Angle_Rad[2] += 0.5 * psi_dot * imu_data->dt;
              for(int i = 0 ; i < 3 ; i ++){
            	  imu->Euler_Angle_Deg[i] = imu->Euler_Angle_Rad[i] * RAD_TO_DEG;
              }
              imu->predict_count++;
              float32_t ypr[3] = {imu->Euler_Angle_Rad[2],imu->Euler_Angle_Rad[1],imu->Euler_Angle_Rad[0]};
              EQ_ZYX(ypr,imu->q);
              QE_ZYX(imu->q, imu->ypr);
              for(int i = 0 ; i < 3 ; i ++){
            	  imu->ypr[i] *= RAD_TO_DEG;
              }
              if(imu->update_count > 1000){
            	  imu->Fusion_OK = 1;
              }
			  break;
		  case Fusion_RESET :
			  Complimentary_Filter_Reset(imu);
			  break;
		  default:
		      break;

		}
}
void Complimentary_Filter_Update(Complimentary_Filter_t * imu ,MAG_DATA_t * mag){
	            float32_t mx = mag->mag_uT[0];
		        float32_t my = mag->mag_uT[1];
		        float32_t mz = mag->mag_uT[2];
		        float32_t mag_norm = sqrtf(mx*mx + my*my + mz*mz);
		        if (mag_norm < 1e-6f) mag_norm = 1.0f;  
		        mx /= mag_norm;
		        my /= mag_norm;
		        mz /= mag_norm;
		        float32_t cRoll = cosf(imu->Euler_Angle_Rad[0]), sRoll = sinf(imu->Euler_Angle_Rad[0]);
		        float32_t cPitch = cosf(imu->Euler_Angle_Rad[1]), sPitch = sinf(imu->Euler_Angle_Rad[1]);
		        float32_t mx2 = mx * cPitch + my * sRoll * sPitch + mz * cRoll * sPitch;
		        float32_t my2 = my * cRoll - mz * sRoll;
		        float32_t psi_mag = atan2f(-my2, mx2);  
		        if (psi_mag > (float)M_PI) psi_mag -= (float)(2.0 * M_PI);
		        else if (psi_mag < -(float)M_PI) psi_mag += (float)(2.0 * M_PI);

		        if(imu->update_count == 0){
		        	imu->Euler_Angle_Rad[2] = psi_mag;
		        	imu->Euler_Angle_Deg[2] = imu->Euler_Angle_Rad[2] * RAD_TO_DEG;
		        }
		        else{
					imu->Euler_Angle_Rad[2] = imu->alpha[2] * imu->Euler_Angle_Rad[2] + (1.0 - imu->alpha[2]) * psi_mag;
					if (imu->Euler_Angle_Rad[2] > (float)M_PI) imu->Euler_Angle_Rad[2] -= (float)(2.0 * M_PI);
					else if (imu->Euler_Angle_Rad[2] < -(float)M_PI) imu->Euler_Angle_Rad[2] += (float)(2.0 * M_PI);
					imu->Euler_Angle_Deg[2] = imu->Euler_Angle_Rad[2] * RAD_TO_DEG;
		        }
		        imu->update_count++;
		    	

}

float32_t invSqrt(float32_t i){
	float32_t recip;
    return ARHS_InvSqrtSafe(i, &recip) ? recip : 0.0f;
}
void ARHS_Predict(ARHS_t* arhs,IMU_Data_t * imu_data){
    if(arhs->fusion_count < 4000){
        arhs->B_madgwick = arhs->B_ARHS_Default * 100;
    }
    else{
        arhs->B_madgwick = arhs->B_ARHS_Default;
        arhs->Fusion_OK = 1;
    }
    arhs->fusion_count++;
	  float32_t recipNorm;
	  float32_t s0, s1, s2, s3;
	  float32_t qDot1, qDot2, qDot3, qDot4;
	  float32_t _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	  float32_t gx = imu_data->w[0];
	  float32_t gy = imu_data->w[1];
	  float32_t gz = imu_data->w[2];
	  float32_t ax = -imu_data->acc[0];
	  float32_t ay = -imu_data->acc[1];
	  float32_t az = -imu_data->acc[2];

	  float32_t q0 = arhs->q[0]; float32_t q1 = arhs->q[1];
	  float32_t q2 = arhs->q[2]; float32_t q3 = arhs->q[3];

	  
	  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	  
	  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	    if (!ARHS_InvSqrtSafe(ax * ax + ay * ay + az * az, &recipNorm)) {
	    	goto arhs_predict_integrate;
	    }
	    ax *= recipNorm;
	    ay *= recipNorm;
	    az *= recipNorm;
	    
	    _2q0 = 2.0f * q0;
	    _2q1 = 2.0f * q1;
	    _2q2 = 2.0f * q2;
	    _2q3 = 2.0f * q3;
	    _4q0 = 4.0f * q0;
	    _4q1 = 4.0f * q1;
	    _4q2 = 4.0f * q2;
	    _8q1 = 8.0f * q1;
	    _8q2 = 8.0f * q2;
	    q0q0 = q0 * q0;
	    q1q1 = q1 * q1;
	    q2q2 = q2 * q2;
	    q3q3 = q3 * q3;
	    
	    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
	    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
	    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
	    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
	    if (ARHS_InvSqrtSafe(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3, &recipNorm)) {
		    s0 *= recipNorm;
		    s1 *= recipNorm;
		    s2 *= recipNorm;
		    s3 *= recipNorm;

		    qDot1 -= arhs->B_madgwick * s0;
		    qDot2 -= arhs->B_madgwick * s1;
		    qDot3 -= arhs->B_madgwick * s2;
		    qDot4 -= arhs->B_madgwick * s3;
	    }
	  }
arhs_predict_integrate:

	  q0 += qDot1 * imu_data->dt;
	  q1 += qDot2 * imu_data->dt;
	  q2 += qDot3 * imu_data->dt;
	  q3 += qDot4 * imu_data->dt;

	  if (ARHS_InvSqrtSafe(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, &recipNorm)) {
		  q0 *= recipNorm;
		  q1 *= recipNorm;
		  q2 *= recipNorm;
		  q3 *= recipNorm;
	  }
	  else {
		  ARHS_ResetQuaternion(&q0, &q1, &q2, &q3);
	  }
	  arhs->q[0] = q0; arhs->q[1] = q1;
	  arhs->q[2] = q2; arhs->q[3] = q3;
	  
	  QE_ZYX(arhs->q,arhs->RPY_RAD);
	  for(int i = 0 ; i <3 ; i ++){
		  arhs->RPY_DEG[i] = arhs->RPY_RAD[i] * RAD_TO_DEG;
	  }
	  
	   
	     q0q0 = q0 * q0;
	     q1q1 = q1 * q1;
	     q2q2 = q2 * q2;
	     q3q3 = q3 * q3;
		float32_t  q0q1 = q0 * q1;
		float32_t  q0q2 = q0 * q2;
		float32_t  q0q3 = q0 * q3;
		float32_t  q1q2 = q1 * q2;
		float32_t  q1q3 = q1 * q3;
		float32_t  q2q3 = q2 * q3;
	    
	     float32_t R_data[9];
	     float32_t g = 9.81;
	     R_data[0] = q0q0 + q1q1 - q2q2 - q3q3; 
	     R_data[1] = 2.0f * (q1q2 - q0q3);      
	     R_data[2] = 2.0f * (q1q3 + q0q2);      
	     R_data[3] = 2.0f * (q1q2 + q0q3);      
	     R_data[4] = q0q0 - q1q1 + q2q2 - q3q3; 
	     R_data[5] = 2.0f * (q2q3 - q0q1);      
	     R_data[6] = 2.0f * (q1q3 - q0q2);      
	     R_data[7] = 2.0f * (q2q3 + q0q1);      
	     R_data[8] = q0q0 - q1q1 - q2q2 + q3q3; 
	    arhs->acc_linear_fixed_frame[0] = -((-imu_data->acc[0]) * R_data[0] + (-imu_data->acc[1]) * R_data[1] + (-imu_data->acc[2]) * R_data[2]);
	    arhs->acc_linear_fixed_frame[1] = -((-imu_data->acc[1]) * R_data[4] + (-imu_data->acc[0]) * R_data[3] + (-imu_data->acc[2]) * R_data[5]);
	    arhs->acc_linear_fixed_frame[2] = -(-g + (-imu_data->acc[2]) * R_data[8] + (-imu_data->acc[0]) * R_data[6] + (-imu_data->acc[1]) * R_data[7]);

	    for(int i = 0 ; i < 3 ; i ++){
          arhs->vel_fixed_frame[i] += arhs->acc_linear_fixed_frame[i] * imu_data->dt;
	    }
}

void ARHS_Update(ARHS_t* arhs,IMU_Data_t * imu_data,MAG_DATA_t * mag){
      if(arhs->fusion_count < 4000){
    	  arhs->B_madgwick = arhs->B_ARHS_Default * 100;
      }
      else{
    	  arhs->B_madgwick = arhs->B_ARHS_Default;
      }
	  float32_t recipNorm;
	  float32_t s0, s1, s2, s3;
	  float32_t qDot1, qDot2, qDot3, qDot4;
	  float32_t hx, hy;
	  float32_t _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	  float32_t dt = imu_data->dt;

	  float32_t gx = imu_data->w[0];
	  float32_t gy = imu_data->w[1];
	  float32_t gz = imu_data->w[2];
	  float32_t ax = -imu_data->acc[0];
	  float32_t ay = -imu_data->acc[1];
	  float32_t az = -imu_data->acc[2];

	  float32_t q0 = arhs->q[0]; float32_t q1 = arhs->q[1];
	  float32_t q2 = arhs->q[2]; float32_t q3 = arhs->q[3];

  	  float32_t mx = mag->mag_uT[0];
  	  float32_t my = mag->mag_uT[1];
  	  float32_t mz = -mag->mag_uT[2];

	  
	  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		  ARHS_Predict(arhs,imu_data);
	    return;
	  }

	  
	  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	  
	  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	    if (!ARHS_InvSqrtSafe(ax * ax + ay * ay + az * az, &recipNorm)) {
	    	goto arhs_update_integrate;
	    }
	    ax *= recipNorm;
	    ay *= recipNorm;
	    az *= recipNorm;

	    if (!ARHS_InvSqrtSafe(mx * mx + my * my + mz * mz, &recipNorm)) {
	    	goto arhs_update_integrate;
	    }
	    mx *= recipNorm;
	    my *= recipNorm;
	    mz *= recipNorm;
	    
	    _2q0mx = 2.0f * q0 * mx;
	    _2q0my = 2.0f * q0 * my;
	    _2q0mz = 2.0f * q0 * mz;
	    _2q1mx = 2.0f * q1 * mx;
	    _2q0 = 2.0f * q0;
	    _2q1 = 2.0f * q1;
	    _2q2 = 2.0f * q2;
	    _2q3 = 2.0f * q3;
	    _2q0q2 = 2.0f * q0 * q2;
	    _2q2q3 = 2.0f * q2 * q3;
	    q0q0 = q0 * q0;
	    q0q1 = q0 * q1;
	    q0q2 = q0 * q2;
	    q0q3 = q0 * q3;
	    q1q1 = q1 * q1;
	    q1q2 = q1 * q2;
	    q1q3 = q1 * q3;
	    q2q2 = q2 * q2;
	    q2q3 = q2 * q3;
	    q3q3 = q3 * q3;
	    
	    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	    _2bx = sqrtf(hx * hx + hy * hy);
	    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	    _4bx = 2.0f * _2bx;
	    _4bz = 2.0f * _2bz;
	    
	    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	    if (ARHS_InvSqrtSafe(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3, &recipNorm)) {
		    s0 *= recipNorm;
		    s1 *= recipNorm;
		    s2 *= recipNorm;
		    s3 *= recipNorm;

		    qDot1 -= arhs->B_madgwick * s0;
		    qDot2 -= arhs->B_madgwick * s1;
		    qDot3 -= arhs->B_madgwick * s2;
		    qDot4 -= arhs->B_madgwick * s3;
	    }
	  }
arhs_update_integrate:

	  q0 += qDot1 * dt;
	  q1 += qDot2 * dt;
	  q2 += qDot3 * dt;
	  q3 += qDot4 * dt;

	  if (ARHS_InvSqrtSafe(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, &recipNorm)) {
		  q0 *= recipNorm;
		  q1 *= recipNorm;
		  q2 *= recipNorm;
		  q3 *= recipNorm;
	  }
	  else {
		  ARHS_ResetQuaternion(&q0, &q1, &q2, &q3);
	  }
	  arhs->q[0] = q0; arhs->q[1] = q1;
	  arhs->q[2] = q2; arhs->q[3] = q3;


	  QE_ZYX(arhs->q,arhs->RPY_RAD);
	  for(int i = 0 ; i <3 ; i ++){
		  arhs->RPY_DEG[i] = arhs->RPY_RAD[i] * RAD_TO_DEG;
	  }
	   
	     q0q0 = q0 * q0;
	     q1q1 = q1 * q1;
	     q2q2 = q2 * q2;
	     q3q3 = q3 * q3;
	     q0q1 = q0 * q1;
	     q0q2 = q0 * q2;
	     q0q3 = q0 * q3;
	     q1q2 = q1 * q2;
	     q1q3 = q1 * q3;
	     q2q3 = q2 * q3;
	    
	     float32_t R_data[9];
	     float32_t g = 9.81;
	     R_data[0] = q0q0 + q1q1 - q2q2 - q3q3; 
	     R_data[1] = 2.0f * (q1q2 - q0q3);      
	     R_data[2] = 2.0f * (q1q3 + q0q2);      
	     R_data[3] = 2.0f * (q1q2 + q0q3);      
	     R_data[4] = q0q0 - q1q1 + q2q2 - q3q3; 
	     R_data[5] = 2.0f * (q2q3 - q0q1);      
	     R_data[6] = 2.0f * (q1q3 - q0q2);      
	     R_data[7] = 2.0f * (q2q3 + q0q1);      
	     R_data[8] = q0q0 - q1q1 - q2q2 + q3q3; 
	    arhs->acc_linear_fixed_frame[0] = -((-imu_data->acc[0]) * R_data[0] + (-imu_data->acc[1]) * R_data[1] + (-imu_data->acc[2]) * R_data[2]);
	    arhs->acc_linear_fixed_frame[1] = -((-imu_data->acc[1]) * R_data[4] + (-imu_data->acc[0]) * R_data[3] + (-imu_data->acc[2]) * R_data[5]);
	    arhs->acc_linear_fixed_frame[2] = -(-g + (-imu_data->acc[2]) * R_data[8] + (-imu_data->acc[0]) * R_data[6] + (-imu_data->acc[1]) * R_data[7]);

	    for(int i = 0 ; i < 3 ; i ++){
           arhs->vel_fixed_frame[i] += arhs->acc_linear_fixed_frame[i] * dt;
	    }
	    arhs->fusion_count++;
}


