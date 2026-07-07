/*
 * Bao cao: Xu ly hieu chuan gyro va accelerometer de giam sai so cam bien truoc khi bay.
 */

#include "IMU_CALIBRATE.h"
#include <string.h>
#include <math.h>

void RESET_IMU_CALIBRATE(IMU_CALIBRATE_t *imu) {
    if (!imu) return;
    
    memset(imu->acc_positive, 0, sizeof(imu->acc_positive));
    memset(imu->acc_negative, 0, sizeof(imu->acc_negative));
    memset(imu->S, 0, sizeof(imu->S));
    memset(imu->bias, 0, sizeof(imu->bias));
    
    memset(imu->acc_x, 0, sizeof(imu->acc_x));
    memset(imu->acc_y, 0, sizeof(imu->acc_y));
    memset(imu->acc_z, 0, sizeof(imu->acc_z));
    memset(imu->M, 0, sizeof(imu->M));
    memset(imu->b, 0, sizeof(imu->b));
    
    imu->sum_x = imu->sum_y = imu->sum_z = 0;
    imu->num_cal = 4000;
    imu->count = 0;
    
    memset(imu->axis_done, 0, sizeof(imu->axis_done));
    imu->axis_done_count = 0;
    imu->status = START_CALI;
    imu->sample = Stop;
    imu->sample_gyro = Stop;
    imu->sum_gyro_x = imu->sum_gyro_y = imu->sum_gyro_z = 0;
    imu->count_gyro = 0;
    imu->num_cal_gyro = 1000;
}

IMU_CALIBRATE_AXIS_t IMU_DetectAxis(float32_t* acc) {
    float32_t abs_x = fabsf(acc[0]);
    float32_t abs_y = fabsf(acc[1]);
    float32_t abs_z = fabsf(acc[2]);

    if(abs_x >= abs_y && abs_x >= abs_z)
        return (acc[0] > 0) ? X_P : X_N;
    else if(abs_y >= abs_x && abs_y >= abs_z)
        return (acc[1] > 0) ? Y_P : Y_N;
    else
        return (acc[2] > 0) ? Z_P : Z_N;
}
void IMU_Sampling(IMU_CALIBRATE_t * imu, float32_t* acc) {
    switch(imu->sample) {
        case Start:
            imu->axis = IMU_DetectAxis(acc);
            imu->sum_x = imu->sum_y = imu->sum_z = 0;
            imu->count = 0;
            imu->sample = Running;
            break;
        case Running:
            if(imu->count < imu->num_cal) {
                imu->count++;
                imu->sum_x += acc[0];
                imu->sum_y += acc[1];
                imu->sum_z += acc[2];
            } else {
                imu->sum_x /= imu->count;
                imu->sum_y /= imu->count;
                imu->sum_z /= imu->count;

                uint8_t ax = imu->axis;
                if(imu->axis_done[ax] == 0) {
                    imu->axis_done_count++;
                }
                
                imu->acc_x[ax] = imu->sum_x;
                imu->acc_y[ax] = imu->sum_y;
                imu->acc_z[ax] = imu->sum_z;
                
                switch(ax) {
                    case X_P: imu->acc_positive[0] = imu->sum_x; break;
                    case X_N: imu->acc_negative[0] = imu->sum_x; break;
                    case Y_P: imu->acc_positive[1] = imu->sum_y; break;
                    case Y_N: imu->acc_negative[1] = imu->sum_y; break;
                    case Z_P: imu->acc_positive[2] = imu->sum_z; break;
                    case Z_N: imu->acc_negative[2] = imu->sum_z; break;
                }
                imu->axis_done[ax] = 1;
                imu->sample = Stop;
                imu->sum_x = imu->sum_y = imu->sum_z = 0;
                imu->count = 0;
                if(imu->axis_done_count >= 6) {
                    imu->status = Caculate_Calibrate;
                }
            }
            break;

        case Stop:
            break;
    }
}
void IMU_PROCESS_CALIBRATE(IMU_CALIBRATE_t * imu, float32_t* acc,  float32_t* gyro) {
    switch(imu->status){
        case START_CALI:
            imu->sample = Start;
            imu->status = CALIBARTE;
            break;
        case CALIBARTE:
            IMU_Sampling(imu, acc);
            break;
        case Caculate_Calibrate:
            
        	IMU_CalcCrossAxisFrom6Face(imu->acc_x,imu->acc_y,imu->acc_z,imu->M,imu->b);
        	IMU_CalcSimpleCalibrate(imu->acc_positive,imu->acc_negative,imu->S,imu->bias);
        	imu->status = CALIBRATE_OKAY;
             break;
        case CALIBRATE_OKAY:
            
            break;

        case STOP_CALI:
            break;

        case RESET_CALI:
            RESET_IMU_CALIBRATE(imu);
            break;
    }
    switch(imu->sample_gyro){
    case Start:
                imu->sum_gyro_x = imu->sum_gyro_y = imu->sum_gyro_z = 0;
                imu->offset_gyro[0] = imu->offset_gyro[1] = imu->offset_gyro[2] = 0;
                imu->count_gyro = 0;
                imu->sample_gyro = Running;
                break;
            case Running:
                if(imu->count_gyro < imu->num_cal_gyro) {
                    imu->count_gyro++;
                    imu->sum_gyro_x += gyro[0];
                    imu->sum_gyro_y += gyro[1];
                    imu->sum_gyro_z += gyro[2];
                } else {
                    imu->sum_gyro_x /= imu->count_gyro;
                    imu->sum_gyro_y /= imu->count_gyro;
                    imu->sum_gyro_z /= imu->count_gyro;
                    
                    imu->offset_gyro[0] = imu->sum_gyro_x;
                    imu->offset_gyro[1] = imu->sum_gyro_y;
                    imu->offset_gyro[2] = imu->sum_gyro_z;

                    imu->sample_gyro = Stop;
                    imu->sum_gyro_x = imu->sum_gyro_y = imu->sum_gyro_z = 0;
                    imu->count_gyro = 0;
                }
                break;

            case Stop:
                break;
    }
}
void IMU_CalcSimpleCalibrate(float32_t acc_positive[3], float32_t acc_negative[3],float32_t S[3], float32_t bias[3]){
    for(int i=0;i<3;i++){
        
        bias[i] = (acc_positive[i] + acc_negative[i]) / 2.0f;
        
        S[i] = 2.0f * G_Scale / (acc_positive[i] - acc_negative[i]);
    }
}

void IMU_ApplySimpleCalibrate(float32_t raw[3], float32_t S[3], float32_t bias[3], float32_t out[3]){
    for(int i=0;i<3;i++){
        out[i] = (raw[i] - bias[i]) * S[i];
    }
}


void IMU_CalcCrossAxisFrom6Face(float32_t acc_x[6], float32_t acc_y[6], float32_t acc_z[6],
                                float32_t M[9], float32_t b[3]) {
    
    float32_t z[NUM_EQ];
    int idx = 0;
    for (int i = 0; i < NUM_POS; i++) {
        z[idx++] = acc_x[i];
        z[idx++] = acc_y[i];
        z[idx++] = acc_z[i];
    }

    
    float32_t ref_a[NUM_POS * 3] = {
        G_SCALE, 0.0f, 0.0f,    
        -G_SCALE, 0.0f, 0.0f,   
        0.0f, G_SCALE, 0.0f,    
        0.0f, -G_SCALE, 0.0f,   
        0.0f, 0.0f, G_SCALE,    
        0.0f, 0.0f, -G_SCALE    
    };

    
    float32_t x[NUM_PARAMS];
    arm_fill_f32(0.0f, x, NUM_PARAMS);
    x[0] = x[4] = x[8] = 1.0f;

    
    float32_t delta = 1e-6f;
    int max_iter = 20;
    arm_status status;

    for (int iter = 0; iter < max_iter; iter++) {
        
        float32_t h[NUM_EQ];
        float32_t A_data[9];
        arm_copy_f32(x, A_data, 9);
        arm_matrix_instance_f32 matA;
        arm_mat_init_f32(&matA, 3, 3, A_data);

        float32_t b_paper[3];
        arm_copy_f32(&x[9], b_paper, 3);
        arm_matrix_instance_f32 matB;
        arm_mat_init_f32(&matB, 3, 1, b_paper);

        for (int i = 0; i < NUM_POS; i++) {
            float32_t ai[3];
            arm_copy_f32(&ref_a[i*3], ai, 3);
            arm_matrix_instance_f32 matAi;
            arm_mat_init_f32(&matAi, 3, 1, ai);

            float32_t hat_ai[3];
            arm_matrix_instance_f32 matHat;
            arm_mat_init_f32(&matHat, 3, 1, hat_ai);
            arm_mat_mult_f32(&matA, &matAi, &matHat);
            arm_mat_add_f32(&matHat, &matB, &matHat);

            arm_copy_f32(hat_ai, &h[i*3], 3);
        }

        
        float32_t p[NUM_EQ];
        arm_matrix_instance_f32 matH, matZ, matP;
        arm_mat_init_f32(&matH, NUM_EQ, 1, h);
        arm_mat_init_f32(&matZ, NUM_EQ, 1, z);
        arm_mat_init_f32(&matP, NUM_EQ, 1, p);
        arm_mat_sub_f32(&matH, &matZ, &matP);

        
        float32_t residual_sq = 0.0f;
        arm_dot_prod_f32(p, p, NUM_EQ, &residual_sq);
        if (residual_sq < delta) break;

        
        float32_t J_data[NUM_EQ * NUM_PARAMS] = {0};
        arm_matrix_instance_f32 matJ;
        arm_mat_init_f32(&matJ, NUM_EQ, NUM_PARAMS, J_data);
        for (int i = 0; i < NUM_POS; i++) {
            int row = i * 3;
            float32_t ax = ref_a[i*3 + 0];
            float32_t ay = ref_a[i*3 + 1];
            float32_t az = ref_a[i*3 + 2];

            
            J_data[row * NUM_PARAMS + 0] = ax;
            J_data[row * NUM_PARAMS + 1] = ay;
            J_data[row * NUM_PARAMS + 2] = az;
            J_data[row * NUM_PARAMS + 9] = 1.0f;

            
            J_data[(row + 1) * NUM_PARAMS + 3] = ax;
            J_data[(row + 1) * NUM_PARAMS + 4] = ay;
            J_data[(row + 1) * NUM_PARAMS + 5] = az;
            J_data[(row + 1) * NUM_PARAMS + 10] = 1.0f;

            
            J_data[(row + 2) * NUM_PARAMS + 6] = ax;
            J_data[(row + 2) * NUM_PARAMS + 7] = ay;
            J_data[(row + 2) * NUM_PARAMS + 8] = az;
            J_data[(row + 2) * NUM_PARAMS + 11] = 1.0f;
        }

        
        float32_t JT_data[NUM_PARAMS * NUM_EQ];
        arm_matrix_instance_f32 matJT;
        arm_mat_init_f32(&matJT, NUM_PARAMS, NUM_EQ, JT_data);
        arm_mat_trans_f32(&matJ, &matJT);

        
        float32_t JTJ_data[NUM_PARAMS * NUM_PARAMS];
        arm_matrix_instance_f32 matJTJ;
        arm_mat_init_f32(&matJTJ, NUM_PARAMS, NUM_PARAMS, JTJ_data);
        arm_mat_mult_f32(&matJT, &matJ, &matJTJ);

        
        float32_t invJTJ_data[NUM_PARAMS * NUM_PARAMS];
        arm_matrix_instance_f32 matInvJTJ;
        arm_mat_init_f32(&matInvJTJ, NUM_PARAMS, NUM_PARAMS, invJTJ_data);
        status = arm_mat_inverse_f32(&matJTJ, &matInvJTJ);
        if (status != ARM_MATH_SUCCESS) {
            
            arm_fill_f32(0.0f, M, 9);
            M[0] = M[4] = M[8] = 1.0f;
            arm_fill_f32(0.0f, b, 3);
            return;
        }

        
        float32_t JTp_data[NUM_PARAMS];
        arm_matrix_instance_f32 matJTp;
        arm_mat_init_f32(&matJTp, NUM_PARAMS, 1, JTp_data);
        arm_mat_mult_f32(&matJT, &matP, &matJTp);

        
        float32_t dx[NUM_PARAMS];
        arm_matrix_instance_f32 matDx;
        arm_mat_init_f32(&matDx, NUM_PARAMS, 1, dx);
        arm_mat_mult_f32(&matInvJTJ, &matJTp, &matDx);
        arm_scale_f32(dx, -1.0f, dx, NUM_PARAMS);

        
        arm_add_f32(x, dx, x, NUM_PARAMS);

        
        float32_t norm_dx;
        arm_rms_f32(dx, NUM_PARAMS, &norm_dx);
        if (norm_dx < delta) break;
    }

    
    float32_t A_data[9];
    arm_copy_f32(x, A_data, 9);
    arm_matrix_instance_f32 matA;
    arm_mat_init_f32(&matA, 3, 3, A_data);

    float32_t invA_data[9];
    arm_matrix_instance_f32 matInvA;
    arm_mat_init_f32(&matInvA, 3, 3, invA_data);
    status = arm_mat_inverse_f32(&matA, &matInvA);
    if (status != ARM_MATH_SUCCESS) {
        
        arm_fill_f32(0.0f, M, 9);
        M[0] = M[4] = M[8] = 1.0f;
        arm_fill_f32(0.0f, b, 3);
        return;
    }
    arm_copy_f32(invA_data, M, 9);

    float32_t b_paper[3];
    arm_copy_f32(&x[9], b_paper, 3);
    arm_matrix_instance_f32 matBpaper;
    arm_mat_init_f32(&matBpaper, 3, 1, b_paper);

    float32_t temp[3];
    arm_matrix_instance_f32 matTemp;
    arm_mat_init_f32(&matTemp, 3, 1, temp);
    arm_mat_mult_f32(&matInvA, &matBpaper, &matTemp);

    b[0] = -temp[0];
    b[1] = -temp[1];
    b[2] = -temp[2];
}

void IMU_ApplyCrossAxis(float32_t raw[3], float32_t M[9], float32_t b[3], float32_t out[3]){
    float32_t x = raw[0];
    float32_t y = raw[1];
    float32_t z = raw[2];

    out[0] = M[0]*x + M[1]*y + M[2]*z + b[0];
    out[1] = M[3]*x + M[4]*y + M[5]*z + b[1];
    out[2] = M[6]*x + M[7]*y + M[8]*z + b[2];
}


void Mag_ApplyCalibration(MagCal_Simple_t* c,MAG_DATA_t* raw,MAG_DATA_t* out){
    for (int i = 0; i < 3; i++)
    {
        float x = raw->mag_uT[i] - c->offset[i];
        x *= c->scale[i];

        
        out->mag_uT[i] = x * (1.0f / c->S);
    }
    
}
void MagCal_Update(MagCal_Simple_t* c, MAG_DATA_t* raw , MAG_DATA_t* out){
    switch (c->state){
    case MAG_CAL_IDLE:
        
    	c->offset[0] = -565.980103;
    	c->offset[1] = 371.411591;
    	c->offset[2] = 266.549927;
    	c->scale[0] = 0.99188453;
    	c->scale[1] = 0.985158622;
    	c->scale[2] = 1.02380025;
    	c->state = MAG_CAL_DONE;
        break;
    case MAG_CAL_START:
        
        for (int i = 0; i < 3; i++) {
            c->min[i] =  1e9f;
            c->max[i] = -1e9f;
        }
        c->samples = 0;
        c->state = MAG_CAL_COLLECTING;
        break;

    case MAG_CAL_COLLECTING:
        
        for (int i = 0; i < 3; i++) {
            if (raw->mag_uT[i] < c->min[i]) c->min[i] = raw->mag_uT[i];
            if (raw->mag_uT[i] > c->max[i]) c->max[i] = raw->mag_uT[i];
        }
        c->samples++;
        
        if (c->samples >= c->samples_target) {
            c->state = MAG_CAL_COMPUTE;
        }
        break;

    case MAG_CAL_COMPUTE:
    {
        float radius[3];
        
        for (int i = 0; i < 3; i++) {
            c->offset[i] = (c->max[i] + c->min[i]) * 0.5f;
            radius[i]    = (c->max[i] - c->min[i]) * 0.5f;
        }
        float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
        
        for (int i = 0; i < 3; i++) {
            c->scale[i] = avg_radius / radius[i];
        }
        c->state = MAG_CAL_DONE;
    }
    break;
    case MAG_CAL_DONE:
        
    	Mag_ApplyCalibration(c, raw, out);
        break;
    }
    if(c->state != MAG_CAL_DONE){
    	for(int i = 0 ; i < 3 ;i ++){
    	  out->mag_uT[i] = 0;
    	}
    }
}
