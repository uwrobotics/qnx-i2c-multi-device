#include<inc/i2c_host.h>
#include<iostream>
#include <iomanip>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/neutrino.h>
#include <cmath>
#include <cstring>

#include <inc/json_helper.h>

#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_WHO_AM_I 0x75

template<int N>
struct Vector {
    float data[N];

    Vector() { memset(data, 0, sizeof(data)); }

    float& operator[](int i) { return data[i]; }
    const float& operator[](int i) const { return data[i]; }

    float norm() const {
        float sum = 0;
        for(int i = 0; i < N; i++) sum += data[i] * data[i];
        return sqrtf(sum);
    }
};

template<int ROWS, int COLS>
struct Matrix {
    float data[ROWS][COLS];

    Matrix() { memset(data, 0, sizeof(data)); }

    static Matrix identity() {
        Matrix m;
        for(int i = 0; i < ROWS && i < COLS; i++) m.data[i][i] = 1.0f;
        return m;
    }

    Matrix operator+(const Matrix& other) const {
        Matrix result;
        for(int i = 0; i < ROWS; i++)
            for(int j = 0; j < COLS; j++)
                result.data[i][j] = data[i][j] + other.data[i][j];
        return result;
    }

    Matrix operator-(const Matrix& other) const {
        Matrix result;
        for(int i = 0; i < ROWS; i++)
            for(int j = 0; j < COLS; j++)
                result.data[i][j] = data[i][j] - other.data[i][j];
        return result;
    }

    template<int OTHER_COLS>
    Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS>& other) const {
        Matrix<ROWS, OTHER_COLS> result;
        for(int i = 0; i < ROWS; i++)
            for(int j = 0; j < OTHER_COLS; j++)
                for(int k = 0; k < COLS; k++)
                    result.data[i][j] += data[i][k] * other.data[k][j];
        return result;
    }

    Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> result;
        for(int i = 0; i < ROWS; i++)
            for(int j = 0; j < COLS; j++)
                result.data[j][i] = data[i][j];
        return result;
    }

    Matrix<ROWS, COLS> inverse() const {
        static_assert(ROWS == COLS, "Only square matrices");
        Matrix<ROWS, COLS*2> aug;
        for(int i = 0; i < ROWS; i++) {
            for(int j = 0; j < COLS; j++) aug.data[i][j] = data[i][j];
            aug.data[i][COLS + i] = 1.0f;
        }

        for(int i = 0; i < ROWS; i++) {
            float pivot = aug.data[i][i];
            if(fabsf(pivot) < 1e-10f) pivot = 1e-10f;
            for(int j = 0; j < COLS*2; j++) aug.data[i][j] /= pivot;

            for(int k = 0; k < ROWS; k++) {
                if(k != i) {
                    float factor = aug.data[k][i];
                    for(int j = 0; j < COLS*2; j++)
                        aug.data[k][j] -= factor * aug.data[i][j];
                }
            }
        }

        Matrix result;
        for(int i = 0; i < ROWS; i++)
            for(int j = 0; j < COLS; j++)
                result.data[i][j] = aug.data[i][COLS + j];
        return result;
    }
};

std::variant<direct_access_t, mem_access_t> recv_packet(uint8_t addr ,uint8_t* data, size_t size) {
    mem_access_t send_pck;
    send_pck.addr = addr;
    send_pck.buf = data;
    send_pck.size = size;
    std::variant<direct_access_t, mem_access_t> meta_data = send_pck;
    return meta_data;
}

int16_t combine_bytes(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

struct SensorData {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
    bool data_ready;
};

struct EKFState {
    float position[3];
    float velocity[3];
    float orientation[3];
    Matrix<9, 9> P;
    uint64_t timestamp_ns;
    bool is_stationary;

    EKFState() {
        memset(position, 0, sizeof(position));
        memset(velocity, 0, sizeof(velocity));
        memset(orientation, 0, sizeof(orientation));
        P = Matrix<9, 9>::identity();
        for(int i = 0; i < 9; i++) P.data[i][i] = 0.1f;
        timestamp_ns = 0;
        is_stationary = true;
    }
};

struct CalibrationData {
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    bool calibrated;
};

SensorData g_sensor_data = {0};
EKFState g_ekf_state;
CalibrationData g_calibration = {0};
pthread_mutex_t g_sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_ekf_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile bool g_running = true;

bool detect_zero_velocity(const SensorData& data) {
    const float accel_threshold = 0.05f;
    const float gyro_threshold = 0.5f;

    float accel_mag = sqrtf(data.accel_x_g * data.accel_x_g +
                            data.accel_y_g * data.accel_y_g +
                            data.accel_z_g * data.accel_z_g);
    float gyro_mag = sqrtf(data.gyro_x_dps * data.gyro_x_dps +
                           data.gyro_y_dps * data.gyro_y_dps +
                           data.gyro_z_dps * data.gyro_z_dps);

    float accel_deviation = fabsf(accel_mag - 1.0f);

    return (accel_deviation < accel_threshold) && (gyro_mag < gyro_threshold);
}

void remove_gravity(float accel[3], const float orientation[3]) {
    float roll = orientation[0] * M_PI / 180.0f;
    float pitch = orientation[1] * M_PI / 180.0f;

    float gravity[3];
    gravity[0] = sinf(pitch);
    gravity[1] = -sinf(roll) * cosf(pitch);
    gravity[2] = cosf(roll) * cosf(pitch);

    accel[0] -= gravity[0];
    accel[1] -= gravity[1];
    accel[2] -= gravity[2];
}

void ekf_predict(EKFState& state, const SensorData& sensor, float dt) {
    float accel[3] = {sensor.accel_x_g, sensor.accel_y_g, sensor.accel_z_g};
    remove_gravity(accel, state.orientation);

    if(state.is_stationary) {
        state.velocity[0] = 0;
        state.velocity[1] = 0;
        state.velocity[2] = 0;
    } else {
        state.velocity[0] += accel[0] * 9.81f * dt;
        state.velocity[1] += accel[1] * 9.81f * dt;
        state.velocity[2] += accel[2] * 9.81f * dt;
    }

    state.position[0] += state.velocity[0] * dt;
    state.position[1] += state.velocity[1] * dt;
    state.position[2] += state.velocity[2] * dt;

    state.orientation[0] += sensor.gyro_x_dps * dt;
    state.orientation[1] += sensor.gyro_y_dps * dt;
    state.orientation[2] += sensor.gyro_z_dps * dt;

    Matrix<9, 9> F = Matrix<9, 9>::identity();
    F.data[0][3] = dt;
    F.data[1][4] = dt;
    F.data[2][5] = dt;

    Matrix<9, 9> Q = Matrix<9, 9>::identity();
    float q_pos = 0.001f;
    float q_vel = state.is_stationary ? 0.0001f : 0.1f;
    float q_ori = 0.01f;

    for(int i = 0; i < 3; i++) Q.data[i][i] = q_pos;
    for(int i = 3; i < 6; i++) Q.data[i][i] = q_vel;
    for(int i = 6; i < 9; i++) Q.data[i][i] = q_ori;

    state.P = F * state.P * F.transpose() + Q;
}

void ekf_update_zupt(EKFState& state) {
    Matrix<3, 9> H;
    H.data[0][3] = 1.0f;
    H.data[1][4] = 1.0f;
    H.data[2][5] = 1.0f;

    Matrix<3, 3> R = Matrix<3, 3>::identity();
    for(int i = 0; i < 3; i++) R.data[i][i] = 0.001f;

    Matrix<3, 3> S = H * state.P * H.transpose() + R;
    Matrix<9, 3> K = state.P * H.transpose() * S.inverse();

    float z[3] = {0, 0, 0};
    float innovation[3] = {
        z[0] - state.velocity[0],
        z[1] - state.velocity[1],
        z[2] - state.velocity[2]
    };

    for(int i = 0; i < 9; i++) {
        float correction = 0;
        for(int j = 0; j < 3; j++) {
            correction += K.data[i][j] * innovation[j];
        }
        if(i < 3) state.position[i] += correction;
        else if(i < 6) state.velocity[i-3] += correction;
        else state.orientation[i-6] += correction;
    }

    Matrix<9, 9> I = Matrix<9, 9>::identity();
    state.P = (I - K * H) * state.P;
}

void calibrate_sensor(I2CDevice* device) {
    const int num_samples = 100;
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    JsonEncode json_start;
    json_start.add(std::unordered_map<std::string, int>{{"Type", 0}});
    json_start.add(std::unordered_map<std::string, std::string>{{"SensorID", "MPU6050"}, {"Status", "Calibrating"}});
    json_start.indent("Data");
    json_start.add(std::unordered_map<std::string, int>{{"samples", num_samples}});
    json_start.closeIndent();
    printf("%s\n", json_start.get_string().c_str());

    for(int i = 0; i < num_samples; i++) {
        uint8_t data[14];
        auto packet = recv_packet(MPU6050_REG_ACCEL_XOUT_H, data, 14);
        device->read(packet);

        int16_t accel_x = combine_bytes(data[0], data[1]);
        int16_t accel_y = combine_bytes(data[2], data[3]);
        int16_t accel_z = combine_bytes(data[4], data[5]);
        int16_t gyro_x = combine_bytes(data[8], data[9]);
        int16_t gyro_y = combine_bytes(data[10], data[11]);
        int16_t gyro_z = combine_bytes(data[12], data[13]);

        accel_sum_x += accel_x / 16384.0;
        accel_sum_y += accel_y / 16384.0;
        accel_sum_z += accel_z / 16384.0;
        gyro_sum_x += gyro_x / 131.0;
        gyro_sum_y += gyro_y / 131.0;
        gyro_sum_z += gyro_z / 131.0;

        delay(10);
    }

    g_calibration.accel_offset_x = accel_sum_x / num_samples;
    g_calibration.accel_offset_y = accel_sum_y / num_samples;
    g_calibration.accel_offset_z = (accel_sum_z / num_samples) - 1.0;
    g_calibration.gyro_offset_x = gyro_sum_x / num_samples;
    g_calibration.gyro_offset_y = gyro_sum_y / num_samples;
    g_calibration.gyro_offset_z = gyro_sum_z / num_samples;
    g_calibration.calibrated = true;

    JsonEncode json_result;
    json_result.add(std::unordered_map<std::string, int>{{"Type", 0}});
    json_result.add(std::unordered_map<std::string, std::string>{{"SensorID", "MPU6050"}, {"Status", "Calibrated"}});
    json_result.indent("Data");
    json_result.add(std::unordered_map<std::string, std::vector<double>>{
        {"accel_offset", {g_calibration.accel_offset_x, g_calibration.accel_offset_y, g_calibration.accel_offset_z}},
        {"gyro_offset", {g_calibration.gyro_offset_x, g_calibration.gyro_offset_y, g_calibration.gyro_offset_z}}
    });
    json_result.closeIndent();
    printf("%s\n", json_result.get_string().c_str());
}

void* sensor_reading_thread(void* arg) {
    I2CDevice* device = (I2CDevice*)arg;
    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 20000000; // 50Hz

    uint64_t iteration = 0;

    while(g_running) {
        struct timespec start_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        uint8_t data[14];
        auto packet = recv_packet(MPU6050_REG_ACCEL_XOUT_H, data, 14);
        device->read(packet);

        int16_t accel_x = combine_bytes(data[0], data[1]);
        int16_t accel_y = combine_bytes(data[2], data[3]);
        int16_t accel_z = combine_bytes(data[4], data[5]);

        int16_t temp_raw = combine_bytes(data[6], data[7]);

        int16_t gyro_x = combine_bytes(data[8], data[9]);
        int16_t gyro_y = combine_bytes(data[10], data[11]);
        int16_t gyro_z = combine_bytes(data[12], data[13]);

        pthread_mutex_lock(&g_sensor_mutex);
        g_sensor_data.accel_x_g = (accel_x / 16384.0) - g_calibration.accel_offset_x;
        g_sensor_data.accel_y_g = (accel_y / 16384.0) - g_calibration.accel_offset_y;
        g_sensor_data.accel_z_g = (accel_z / 16384.0) - g_calibration.accel_offset_z;
        g_sensor_data.gyro_x_dps = (gyro_x / 131.0) - g_calibration.gyro_offset_x;
        g_sensor_data.gyro_y_dps = (gyro_y / 131.0) - g_calibration.gyro_offset_y;
        g_sensor_data.gyro_z_dps = (gyro_z / 131.0) - g_calibration.gyro_offset_z;
        g_sensor_data.temp_c = (temp_raw / 340.0) + 36.53;
        g_sensor_data.data_ready = true;
        pthread_mutex_unlock(&g_sensor_mutex);

        if(iteration % 50 == 0) {
            printf("[SENSOR] Accel: X=%7.3f Y=%7.3f Z=%7.3f | Gyro: X=%7.2f Y=%7.2f Z=%7.2f | Temp: %.2f°C\n",
                   g_sensor_data.accel_x_g, g_sensor_data.accel_y_g, g_sensor_data.accel_z_g,
                   g_sensor_data.gyro_x_dps, g_sensor_data.gyro_y_dps, g_sensor_data.gyro_z_dps,
                   g_sensor_data.temp_c);
        }

        iteration++;

        nanosleep(&sleep_time, NULL);
    }

    return NULL;
}

void* ekf_fusion_thread(void* arg) {
    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000; // 1000Hz

    uint64_t iteration = 0;

    while(g_running) {
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        uint64_t timestamp_ns = current_time.tv_sec * 1000000000ULL + current_time.tv_nsec;

        SensorData sensor_snapshot;
        pthread_mutex_lock(&g_sensor_mutex);
        sensor_snapshot = g_sensor_data;
        pthread_mutex_unlock(&g_sensor_mutex);

        if(sensor_snapshot.data_ready) {
            float dt = 0.001;

            pthread_mutex_lock(&g_ekf_mutex);

            bool is_stationary = detect_zero_velocity(sensor_snapshot);
            g_ekf_state.is_stationary = is_stationary;

            ekf_predict(g_ekf_state, sensor_snapshot, dt);

            if(is_stationary) {
                ekf_update_zupt(g_ekf_state);
            }

            g_ekf_state.timestamp_ns = timestamp_ns;

            pthread_mutex_unlock(&g_ekf_mutex);

            if(iteration % 200 == 0) {
                JsonEncode json;

                json.add(std::unordered_map<std::string, int>{{"Type", 1}});
                json.add(std::unordered_map<std::string, std::string>{{"SensorID", "MPU6050"}});

                json.indent("Data");
                json.add(std::unordered_map<std::string, std::vector<double>>{
                    {"position", {g_ekf_state.position[0], g_ekf_state.position[1], g_ekf_state.position[2]}},
                    {"velocity", {g_ekf_state.velocity[0], g_ekf_state.velocity[1], g_ekf_state.velocity[2]}},
                    {"orientation", {g_ekf_state.orientation[0], g_ekf_state.orientation[1], g_ekf_state.orientation[2]}}
                });
                json.add(std::unordered_map<std::string, int>{{"stationary", g_ekf_state.is_stationary ? 1 : 0}});
                json.closeIndent();

                printf("[EKF] %s\n", json.get_string().c_str());
            }
        }

        iteration++;

        nanosleep(&sleep_time, NULL);
    }

    return NULL;
}

int main() {
    std::cout << "=== MPU6050 I2C Driver - Multi-threaded ===" << std::endl;

    I2CDevice device = I2CDevice(1, MPU6050_ADDR, I2C_MEM);

    uint8_t DEVICE_ID;
    auto packet = recv_packet(MPU6050_REG_WHO_AM_I, &DEVICE_ID, 1);
    device.read(packet);
    std::cout << "Device ID: 0x" << std::hex << (int)DEVICE_ID << std::dec << std::endl;
    if (DEVICE_ID != 0x68) {
        printf("  ERROR: Expected 0x68, got 0x%02X\n", DEVICE_ID);
        return -1;
    }

    uint8_t command = 0X00;
    packet = recv_packet(MPU6050_REG_PWR_MGMT_1, &command, 1);
    device.write(packet);
    delay(100);

    command = 0X09;
    packet = recv_packet(MPU6050_REG_SMPLRT_DIV, &command, 1);
    device.write(packet);

    command = 0X00;
    packet = recv_packet(MPU6050_REG_ACCEL_CONFIG, &command, 1);
    device.write(packet);

    command = 0X00;
    packet = recv_packet(MPU6050_REG_GYRO_CONFIG, &command, 1);
    device.write(packet);

    std::cout << "MPU6050 initialized successfully" << std::endl;

    calibrate_sensor(&device);

    pthread_t sensor_thread, ekf_thread;
    pthread_attr_t sensor_attr, ekf_attr;
    struct sched_param sensor_param, ekf_param;

    pthread_attr_init(&sensor_attr);
    pthread_attr_init(&ekf_attr);

    pthread_attr_setschedpolicy(&sensor_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&ekf_attr, SCHED_FIFO);

    sensor_param.sched_priority = 10;
    ekf_param.sched_priority = 15;

    pthread_attr_setschedparam(&sensor_attr, &sensor_param);
    pthread_attr_setschedparam(&ekf_attr, &ekf_param);

    pthread_attr_setdetachstate(&sensor_attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setdetachstate(&ekf_attr, PTHREAD_CREATE_JOINABLE);

    std::cout << "Creating threads..." << std::endl;
    std::cout << "  - Sensor reading thread: 50Hz" << std::endl;
    std::cout << "  - EKF fusion thread: 1000Hz" << std::endl;

    int ret = pthread_create(&sensor_thread, &sensor_attr, sensor_reading_thread, &device);
    if (ret != 0) {
        printf("ERROR: Failed to create sensor thread (errno: %d)\n", ret);
        return -1;
    }

    ret = pthread_create(&ekf_thread, &ekf_attr, ekf_fusion_thread, NULL);
    if (ret != 0) {
        printf("ERROR: Failed to create EKF thread (errno: %d)\n", ret);
        g_running = false;
        pthread_join(sensor_thread, NULL);
        return -1;
    }

    std::cout << "Threads started successfully" << std::endl;
    std::cout << "Press Ctrl+C to stop..." << std::endl;

    pthread_attr_destroy(&sensor_attr);
    pthread_attr_destroy(&ekf_attr);

    pthread_join(sensor_thread, NULL);
    pthread_join(ekf_thread, NULL);

    std::cout << "Threads terminated" << std::endl;

    pthread_mutex_destroy(&g_sensor_mutex);
    pthread_mutex_destroy(&g_ekf_mutex);

    return 0;
}