#include "kalman.h"
#include "stdint.h"
#include "math.h"

float kalmanVoltage_err_measure;
float kalmanVoltage_err_estimate;
float kalmanVoltage_q;
float kalmanVoltage_current_estimate;
float kalmanVoltage_last_estimate;
float kalmanVoltage_kalman_gain;

float kalmanCurrent_err_measure;
float kalmanCurrent_err_estimate;
float kalmanCurrent_q;
float kalmanCurrent_current_estimate;
float kalmanCurrent_last_estimate;
float kalmanCurrent_kalman_gain;

float kalmanTemperature_err_measure;
float kalmanTemperature_err_estimate;
float kalmanTemperature_q;
float kalmanTemperature_current_estimate;
float kalmanTemperature_last_estimate;
float kalmanTemperature_kalman_gain;

void SimpleKalmanFilter(float mea_e, float est_e, float q, int sensor)
{
    switch(sensor) {
        case 1: // Voltage sensor
            kalmanVoltage_err_measure = mea_e;
            kalmanVoltage_err_estimate = est_e;
            kalmanVoltage_q = q;
            break;
        case 2: // Current sensor
            kalmanCurrent_err_measure = mea_e;
            kalmanCurrent_err_estimate = est_e;
            kalmanCurrent_q = q;
            break;
        case 3: // Temperature sensor
            kalmanTemperature_err_measure = mea_e;
            kalmanTemperature_err_estimate = est_e;
            kalmanTemperature_q = q;
            break;
        default:
            break;
    }
}

float updateEstimate(float mea, int sensor)
{
    float _kalman_gain;
    float _current_estimate;
    float _last_estimate;
    float _err_estimate;
    float _q;
    float _err_measure;

    switch(sensor) {
        case 1: // Voltage sensor
            _kalman_gain = kalmanVoltage_err_estimate / (kalmanVoltage_err_estimate + kalmanVoltage_err_measure);
            _current_estimate = kalmanVoltage_last_estimate + _kalman_gain * (mea - kalmanVoltage_last_estimate);
            _err_estimate =  (1.0 - _kalman_gain) * kalmanVoltage_err_estimate + fabs(kalmanVoltage_last_estimate - _current_estimate) * kalmanVoltage_q;
            kalmanVoltage_last_estimate = _current_estimate;
            return _current_estimate;
        case 2: // Current sensor
            _kalman_gain = kalmanCurrent_err_estimate / (kalmanCurrent_err_estimate + kalmanCurrent_err_measure);
            _current_estimate = kalmanCurrent_last_estimate + _kalman_gain * (mea - kalmanCurrent_last_estimate);
            _err_estimate =  (1.0 - _kalman_gain) * kalmanCurrent_err_estimate + fabs(kalmanCurrent_last_estimate - _current_estimate) * kalmanCurrent_q;
            kalmanCurrent_last_estimate = _current_estimate;
            return _current_estimate;
        case 3: // Temperature sensor
            _kalman_gain = kalmanTemperature_err_estimate / (kalmanTemperature_err_estimate + kalmanTemperature_err_measure);
            _current_estimate = kalmanTemperature_last_estimate + _kalman_gain * (mea - kalmanTemperature_last_estimate);
            _err_estimate =  (1.0 - _kalman_gain) * kalmanTemperature_err_estimate + fabs(kalmanTemperature_last_estimate - _current_estimate) * kalmanTemperature_q;
            kalmanTemperature_last_estimate = _current_estimate;
            return _current_estimate;
        default:
            return 0;
    }
}

void setMeasurementError(float mea_e, int sensor)
{
    switch(sensor) {
        case 1: // Voltage sensor
            kalmanVoltage_err_measure = mea_e;
            break;
        case 2: // Current sensor
            kalmanCurrent_err_measure = mea_e;
            break;
        case 3: // Temperature sensor
            kalmanTemperature_err_measure = mea_e;
            break;
        default:
            break;
    }
}

void setEstimateError(float est_e, int sensor)
{
    switch(sensor) {
        case 1: // Voltage sensor
            kalmanVoltage_err_estimate = est_e;
            break;
        case 2: // Current sensor
            kalmanCurrent_err_estimate = est_e;
            break;
        case 3: // Temperature sensor
            kalmanTemperature_err_estimate = est_e;
            break;
        default:
            break;
    }
}

void setProcessNoise(float q, int sensor)
{
    switch(sensor) {
        case 1: // Voltage sensor
            kalmanVoltage_q = q;
            break;
        case 2: // Current sensor
            kalmanCurrent_q = q;
            break;
        case 3: // Temperature sensor
            kalmanTemperature_q = q;
            break;
        default:
            break;
    }
}

float getKalmanGain(int sensor) {
    switch(sensor) {
        case 1: // Voltage sensor
            return kalmanVoltage_kalman_gain;
        case 2: // Current sensor
            return kalmanCurrent_kalman_gain;
        case 3: // Temperature sensor
            return kalmanTemperature_kalman_gain;
        default:
            return 0;
    }
}

float getEstimateError(int sensor) {
    switch(sensor) {
        case 1: // Voltage sensor
            return kalmanVoltage_err_estimate;
        case 2: // Current sensor
            return kalmanCurrent_err_estimate;
        case 3: // Temperature sensor
            return kalmanTemperature_err_estimate;
        default:
            return 0;
    }
}
