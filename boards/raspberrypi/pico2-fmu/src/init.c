#include <nuttx/config.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <hardware/rp2350_gpio.h>
#include <px4_platform_common/px4_config.h>
#include <sys/mount.h>
#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <math.h>
#include <syslog.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include "board_config.h"


// ---- EKF2 STATE ESTIMATION ----
typedef struct {
    float q[4];        // Quaternion [w, x, y, z]
    float P[16];       // Covariance matrix (4x4)
    float gyro_bias[3]; // Gyroscope bias
} ekf2_state_t;

// Matrix operations for EKF
static void matrix_multiply_4x4(const float *A, const float *B, float *C) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4 + j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                C[i*4 + j] += A[i*4 + k] * B[k*4 + j];
            }
        }
    }
}

static void quaternion_normalize(float *q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

static void quaternion_multiply(const float *q1, const float *q2, float *q_out) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quaternion_to_euler(const float *q, float *roll, float *pitch, float *yaw) {
    *roll = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
    *pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
    *yaw = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
}

// ---- CORE 1 TASKS ----
static int imu_core1_task(int argc, char *argv[])
{
    const float dt = 0.0005f; // 2kHz loop
    
    // EKF2 state initialization
    ekf2_state_t ekf = {
        .q = {1.0f, 0.0f, 0.0f, 0.0f}, // Identity quaternion
        .P = {1.0f, 0.0f, 0.0f, 0.0f,  // Initial covariance
              0.0f, 1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 0.0f, 1.0f},
        .gyro_bias = {0.0f, 0.0f, 0.0f}
    };
    
    // Process and measurement noise covariance
    const float Q_gyro = 1e-4f;    // Gyro process noise
    const float R_accel = 0.1f;    // Accelerometer measurement noise
    
    int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
    int gyro_sub  = orb_subscribe(ORB_ID(sensor_gyro));
    int attitude_pub = orb_advertise(ORB_ID(vehicle_attitude));
    
    struct sensor_accel_s accel = {0};
    struct sensor_gyro_s  gyro  = {0};
    struct vehicle_attitude_s attitude = {0};
    
    bool initialized = false;
    
    while (1) {
        bool updated = false;
        orb_check(accel_sub, &updated);
        
        if (updated) {
            orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);
            orb_copy(ORB_ID(sensor_gyro),  gyro_sub,  &gyro);
            
            // ---- EKF2 PREDICTION STEP ----
            float omega[3] = {
                gyro.x - ekf.gyro_bias[0],
                gyro.y - ekf.gyro_bias[1], 
                gyro.z - ekf.gyro_bias[2]
            };
            
            // Quaternion derivative: q_dot = 0.5 * q * omega
            float q_omega[4] = {0.0f, omega[0], omega[1], omega[2]};
            float q_dot[4];
            quaternion_multiply(ekf.q, q_omega, q_dot);
            
            for (int i = 0; i < 4; i++) {
                q_dot[i] *= 0.5f;
            }
            
            // State prediction: q_pred = q + q_dot * dt
            float q_pred[4];
            for (int i = 0; i < 4; i++) {
                q_pred[i] = ekf.q[i] + q_dot[i] * dt;
            }
            quaternion_normalize(q_pred);
            
            // Jacobian F = I + 0.5*dt*Omega(omega)
            float F[16] = {
                1.0f, -0.5f*dt*omega[0], -0.5f*dt*omega[1], -0.5f*dt*omega[2],
                0.5f*dt*omega[0], 1.0f, 0.5f*dt*omega[2], -0.5f*dt*omega[1],
                0.5f*dt*omega[1], -0.5f*dt*omega[2], 1.0f, 0.5f*dt*omega[0],
                0.5f*dt*omega[2], 0.5f*dt*omega[1], -0.5f*dt*omega[0], 1.0f
            };
            
            // Covariance prediction: P_pred = F * P * F^T + Q
            float FT[16], P_pred[16];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    FT[i*4 + j] = F[j*4 + i];
                }
            }
            
            float FP[16];
            matrix_multiply_4x4(F, ekf.P, FP);
            matrix_multiply_4x4(FP, FT, P_pred);
            
            // Add process noise
            for (int i = 0; i < 4; i++) {
                P_pred[i*4 + i] += Q_gyro * dt;
            }
            
            // ---- EKF2 UPDATE STEP ----
            // Expected gravity direction in body frame
            float gravity_body[3] = {
                2.0f * (ekf.q[1]*ekf.q[3] - ekf.q[0]*ekf.q[2]),
                2.0f * (ekf.q[2]*ekf.q[3] + ekf.q[0]*ekf.q[1]),
                ekf.q[0]*ekf.q[0] - ekf.q[1]*ekf.q[1] - ekf.q[2]*ekf.q[2] + ekf.q[3]*ekf.q[3]
            };
            
            // Normalize accelerometer measurement
            float accel_norm = sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
            float accel_meas[3] = {accel.x/accel_norm, accel.y/accel_norm, accel.z/accel_norm};
            
            // Measurement residual
            float y[3] = {
                accel_meas[0] - gravity_body[0],
                accel_meas[1] - gravity_body[1], 
                accel_meas[2] - gravity_body[2]
            };
            
            // Measurement Jacobian H (partial derivative of expected gravity w.r.t quaternion)
            float H[12] = {
                -2.0f*ekf.q[2],  2.0f*ekf.q[3], -2.0f*ekf.q[0],  2.0f*ekf.q[1],
                 2.0f*ekf.q[1],  2.0f*ekf.q[0],  2.0f*ekf.q[3],  2.0f*ekf.q[2],
                 2.0f*ekf.q[0], -2.0f*ekf.q[1], -2.0f*ekf.q[2],  2.0f*ekf.q[3]
            };
            
            // Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
            float HPT[12], HPH_T[9];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    HPT[i*4 + j] = H[j*3 + i]; // H^T
                }
            }
            
            float PH_T[12];
            matrix_multiply_4x4(P_pred, HPT, PH_T);
            
            // H * P * H^T (3x3)
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    HPH_T[i*3 + j] = 0.0f;
                    for (int k = 0; k < 4; k++) {
                        HPH_T[i*3 + j] += H[i*4 + k] * PH_T[k*3 + j];
                    }
                }
            }
            
            // Add measurement noise
            for (int i = 0; i < 3; i++) {
                HPH_T[i*3 + i] += R_accel;
            }
            
            // Invert 3x3 matrix (simplified for diagonal dominant case)
            float S_inv[9] = {0};
            for (int i = 0; i < 3; i++) {
                S_inv[i*3 + i] = 1.0f / HPH_T[i*3 + i];
            }
            
            // Kalman gain K = P * H^T * S_inv
            float K[12];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 3; j++) {
                    K[i*3 + j] = 0.0f;
                    for (int k = 0; k < 3; k++) {
                        K[i*3 + j] += PH_T[i*3 + k] * S_inv[k*3 + j];
                    }
                }
            }
            
            // State update: q = q_pred + K * y
            float correction[4] = {0};
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 3; j++) {
                    correction[i] += K[i*3 + j] * y[j];
                }
            }
            
            for (int i = 0; i < 4; i++) {
                ekf.q[i] = q_pred[i] + correction[i];
            }
            quaternion_normalize(ekf.q);
            
            // Covariance update: P = (I - K * H) * P_pred
            float KH[16], I_KH[16];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    KH[i*4 + j] = 0.0f;
                    for (int k = 0; k < 3; k++) {
                        KH[i*4 + j] += K[i*3 + k] * H[k*4 + j];
                    }
                    I_KH[i*4 + j] = (i == j) ? 1.0f - KH[i*4 + j] : -KH[i*4 + j];
                }
            }
            matrix_multiply_4x4(I_KH, P_pred, ekf.P);
            
            // Publish attitude estimate
            quaternion_to_euler(ekf.q, &attitude.roll, &attitude.pitch, &attitude.yaw);
            attitude.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(vehicle_attitude), attitude_pub, &attitude);
            
            initialized = true;
        }

        up_udelay(500); // 2kHz loop
    }

    orb_unsubscribe(accel_sub);
    orb_unsubscribe(gyro_sub);
    return 0;
}

// ---- BOARD INIT ----
__EXPORT int board_app_initialize(uintptr_t arg)
{
    int ret;

    // SPI CS pins — bit-banged, plain GPIO 
    rp2350_gpio_init(BOARD_SPI0_CS_PIN); // IMU CS
    rp2350_gpio_setdir(BOARD_SPI0_CS_PIN, true);
    rp2350_gpio_put(BOARD_SPI0_CS_PIN, true);     

    rp2350_gpio_init(BOARD_SPI1_CS_PIN); // SD CARD CS
    rp2350_gpio_setdir(BOARD_SPI1_CS_PIN, true);
    rp2350_gpio_put(BOARD_SPI1_CS_PIN, true); 

    // IMU INTERRUPT DRDY
    rp2350_gpio_init(BOARD_IMU_DRDY_PIN);
    rp2350_gpio_setdir(BOARD_IMU_DRDY_PIN, false);
    rp2350_gpio_set_pulls(BOARD_IMU_DRDY_PIN, false, false);

    // ARM SAFETY BUTTON
    rp2350_gpio_init(BOARD_SAFETY_BUTTON_PIN);
    rp2350_gpio_setdir(BOARD_SAFETY_BUTTON_PIN, false);
    rp2350_gpio_set_pulls(BOARD_SAFETY_BUTTON_PIN, true, false);

    // SD CARD
    struct spi_dev_s *spi1 = rp2350_spibus_initialize(1);
    if (spi1 == NULL) {
        syslog(LOG_ERR, "board_init: SPI1 init failed\n");
        // non-fatal — continue without SD
    } else {
        ret = mmcsd_spislotinitialize(0, 0, spi1);
        if (ret < 0) {
            syslog(LOG_WARNING, "board_init: mmcsd slot init failed: %d\n", ret);
        } else {
            ret = mount("/dev/mmcsd0", "/fs/microsd", "vfat", 0, NULL);
            if (ret < 0) {
                syslog(LOG_WARNING, "board_init: SD mount failed: %d\n", ret);
            } else {
                syslog(LOG_INFO, "board_init: SD mounted at /fs/microsd\n");
            }
        }
    }

// ---- Core 1 task ----
    pid_t pid = task_create("imu_core1",
                             SCHED_PRIORITY_MAX - 5,
                             2048,
                             imu_core1_task,
                             NULL);
    if (pid > 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);              // explicit Core 1 affinity
        sched_setaffinity(pid, sizeof(cpuset), &cpuset);
        syslog(LOG_INFO, "board_init: imu_core1 pinned to CPU1, pid=%d\n", pid);
    } else {
        syslog(LOG_ERR, "board_init: Core 1 task spawn failed: %d\n", pid);
    }

    return OK;

}