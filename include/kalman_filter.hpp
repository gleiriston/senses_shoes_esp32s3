#pragma once

class KalmanFilter {
public:

    KalmanFilter()
    : q(4.0f), r(400.0f), p(1.0f), x(0.0f) {}

    KalmanFilter(float process_noise, float measurement_noise,
                 float estimation_error, float initial_value) {
        this->q = process_noise;
        this->r = measurement_noise;
        this->p = estimation_error;
        this->x = initial_value;
    }

    float update(float measurement) {
       
        this->p = this->p + this->q;

        
        float k = this->p / (this->p + this->r);
        this->x = this->x + k * (measurement - this->x);
        this->p = (1.0f - k) * this->p;

        return this->x;
    }

private:
    float q; 
    float r; 
    float p; 
    float x; 
};
