#ifndef Globals_H
#define Globals_H

struct SensorData
{
    float temp_sense_ch1;
    float temp_sense_ch2;
    float temp_sense_ch3;
    float temp_sense_ch4;
    float temp_sense_ch5;
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;
    float battery_voltage;
};

struct SystemStatus
{
    uint8_t LED_Red_status;
    uint8_t LED_Green_status;
    uint8_t LED_Blue_status;
    uint8_t buzzer_status;
    uint8_t orientation_status;
};

struct WebSystemState
{
    volatile float probe1_temp_readings[3600]; // 3600 = 1 hour of data at 1Hz
    volatile float probe2_temp_readings[3600]; //
    volatile float probe3_temp_readings[3600]; //
    volatile float probe4_temp_readings[3600]; //
    volatile float probe5_temp_readings[3600]; //
    char probe1_seq[20];                       // Sequence for probe 1
    char probe2_seq[20];                       // Sequence for probe 2
    char probe3_seq[20];                       // Sequence for probe 3
    char probe4_seq[20];                       // Sequence for probe 4
    char probe5_seq[20];                       // Sequence for probe 5
    bool probe_states[5];                      // State of each of the 5 probes
    bool sequence_active;                      // Sequence handling flag
    char sequence_status[20];                  // Sequence status
    bool webpage_displaying_graph;             // Whether the webpage is displaying a graph
    bool webpage_load_graph_info;              // Whether graph info should be loaded on page refresh
};

#endif
