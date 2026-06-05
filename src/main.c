#include <stdio.h>
#include <unistd.h>
#include "pid/pid.h"

typedef struct {
        float x, y, z;
        float vx, vy, vz;
        float roll, pitch, yaw;
        float p, q, r;
} QuadState;

#define GRAVITY         9.81f           // m/s²
#define MASS            0.25f           // kg
#define ARM_LENGTH      0.2f            // meters (center to motor)
#define THRUST_FACTOR   0.005f          // motor_cmd → Newtons
#define TORQUE_FACTOR   0.000001f       // differential → torque
#define Ixx             0.01f           // kg·m²
#define Iyy             0.01f
#define Izz             0.02f

#define DEG_TO_RAD(deg) ((deg) * 0.01745329252f)
#define RAD_TO_DEG(rad) ((rad) * 57.2957795131f)

int main() {
        QuadState quad = {0};
        quad.z  = 0.0f;
        quad.vz = 0.0f;
        quad.roll  = DEG_TO_RAD(0.0f);
        quad.pitch = DEG_TO_RAD(0.0f);
        quad.yaw   = DEG_TO_RAD(0.0f);
        quad.p = 0.0f;
        quad.q = 0.0f;
        quad.r = 0.0f;

        PID_Controller altitude_pid;
        pid_init(&altitude_pid, 1300.0f, 1000.0f, 200.0f, 100.0f, 2000.0f);
        altitude_pid.setpoint = 1.0f;

        FILE* log_file = fopen("flight_log.csv", "w");
        if (log_file == NULL) {
                printf("Error: Could not create flight_log.csv\n");
                return 1;
        }
        fprintf(log_file, "time,z,vz,thrust,setpoint,gust_active\n");

        float dt = 0.005f;
        uint32_t now_ms = 0;
        uint32_t next_print_ms = 0;

        for (int step = 0; step < 4000; step++) {
                now_ms += (uint32_t)(dt * 1000.0f);

                float thrust_cmd = pid_update(&altitude_pid, quad.z, now_ms);
                float thrust = thrust_cmd * THRUST_FACTOR;
                float az = (thrust / MASS) - GRAVITY;

                quad.vz += az * dt;
                quad.z += quad.vz * dt;

                if (quad.z < 0.0f) {
                    quad.z = 0.0f;
                    quad.vz = 0.0f;
                }

                if (now_ms >= 8000 && now_ms < 8100) {
                        quad.vz += 20.0f * dt;  // 20N wind gust for 100 ms
                }

                int gust_active = (now_ms >= 8000 && now_ms < 8100) ? 1 : 0;

                // Log to CSV for live plotting (20Hz)
                static int log_counter = 0;
                if (log_file && (++log_counter % 10 == 0)) {
                    fprintf(log_file, "%.3f,%.6f,%.6f,%.2f,1.0,%d\n",
                            now_ms / 1000.0f, quad.z, quad.vz, thrust_cmd, gust_active);
                    fflush(log_file);
                }

                if (now_ms >= next_print_ms) {
                    printf("t=%.3f | z=%.4f m | vz=%.4f m/s | thrust=%.2f\n",
                           now_ms / 1000.0f, quad.z, quad.vz, thrust);
                    next_print_ms += 50;
                }

                usleep(5000);
        }

        if (log_file) fclose(log_file);

        return 0;
}

