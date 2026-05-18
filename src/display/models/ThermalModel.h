#pragma once
#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H

#include <Arduino.h>

struct ThermalModelConfig {
    float ambientTemp          = 20.0f;  // assumed ambient when no prior data available
    float couplingFactor       = 0.7f;   // portafilter equilibrium = ambient + coupling*(boiler-ambient)
    float lagTimeConstant      = 180.0f; // seconds: portafilter thermal lag time constant
    float coolingTimeConstant  = 600.0f; // seconds: portafilter cooling rate after shutdown
    float readyThreshold       = 1.5f;   // Â°C below target to declare portafilter ready
    float correctionWindowSecs = 20.0f;  // seconds of boiler rise observation for startup correction
    float boilerLagEstimate    = 30.0f;  // assumed boiler rise time constant (seconds)
    unsigned long logIntervalMs = 5000UL;
};

// Estimates portafilter temperature using a first-order exponential lag on the boiler temp.
// Call begin() once per session start, update() on every boiler temp tick.
class ThermalModel {
  public:
    ThermalModelConfig config;

    // Initialise from NVS shutdown state. Pass 0/0 for a guaranteed cold-start.
    void begin(float lastShutdownTemp, uint32_t lastShutdownTime);

    // Called each controller tick with the raw (pre-offset) boiler temperature.
    void update(float boilerTemp);

    // Inform the model that standby is being entered so THERMAL READY re-fires on next wakeup.
    void notifyStandby();

    // Set the current brew target temperature used for readiness calculations and logging.
    void setTarget(float targetTemp);

    float getPortafilterEstimate() const { return portafilterEstimate; }

    // True when portafilter estimate >= target - readyThreshold.
    bool isReadyToBrew() const;

    // Seconds until portafilter estimate reaches target - readyThreshold.
    // Returns 0 if already ready, -1 if the boiler cannot heat the portafilter to target.
    int getSecondsToReady() const;

  private:
    float portafilterEstimate   = 20.0f;
    float currentTarget         = 93.0f;
    float lastBoilerTemp        = 0.0f;
    float initialBoilerTemp     = 0.0f;
    bool  begun                 = false;
    bool  initialCaptured       = false;
    bool  correctionApplied     = false;
    bool  readyLogged           = false;
    unsigned long beginMs       = 0;
    unsigned long lastLogMs     = 0;
    unsigned long lastUpdateMs  = 0;
};

#endif // THERMAL_MODEL_H
