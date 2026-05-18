#include <display/models/ThermalModel.h>
#include <cmath>
#include <ctime>

static const char *THERMAL_TAG = "ThermalModel";

void ThermalModel::begin(float lastShutdownTemp, uint32_t lastShutdownTime) {
    correctionApplied = false;
    initialCaptured   = false;
    readyLogged       = false;
    initialBoilerTemp = 0.0f;
    lastBoilerTemp    = config.ambientTemp;

    float initialEstimate = config.ambientTemp;

    if (lastShutdownTemp > config.ambientTemp && lastShutdownTime > 0) {
        time_t now = time(nullptr);
        struct tm tminfo{};
        localtime_r(&now, &tminfo);
        // Only trust the timestamp if NTP has synced (year > 2020).
        bool ntpValid = (tminfo.tm_year > (2020 - 1900));
        if (ntpValid && static_cast<uint32_t>(now) > lastShutdownTime) {
            float elapsed     = static_cast<float>(static_cast<uint32_t>(now) - lastShutdownTime);
            float pfAtShutdown = config.ambientTemp +
                                 config.couplingFactor * (lastShutdownTemp - config.ambientTemp);
            initialEstimate = config.ambientTemp +
                              (pfAtShutdown - config.ambientTemp) *
                              expf(-elapsed / config.coolingTimeConstant);
        }
    }

    portafilterEstimate = initialEstimate;
    beginMs      = millis();
    lastLogMs    = 0;
    lastUpdateMs = millis();
    begun        = true;

    Serial.printf("[%s] begin: pf_init=%.1fÂ°C (shutdown_boiler=%.1fÂ°C)\n",
                  THERMAL_TAG, portafilterEstimate, lastShutdownTemp);
}

void ThermalModel::setTarget(float targetTemp) {
    currentTarget = targetTemp;
}

void ThermalModel::notifyStandby() {
    // Allow THERMAL READY to be logged again after the next wakeup.
    readyLogged = false;
}

void ThermalModel::update(float boilerTemp) {
    if (!begun)
        return;

    unsigned long now = millis();
    float dt = static_cast<float>(now - lastUpdateMs) / 1000.0f;
    lastUpdateMs = now;
    // Guard against first tick or a stale update after a long gap.
    if (dt <= 0.0f || dt > 10.0f)
        dt = 0.1f;

    // Capture initial boiler temp on the first tick for the correction calculation.
    if (!initialCaptured) {
        initialBoilerTemp = boilerTemp;
        initialCaptured   = true;
    }

    // First-order exponential lag: portafilter chases its equilibrium temperature.
    float pfEquilibrium = config.ambientTemp +
                          config.couplingFactor * (boilerTemp - config.ambientTemp);
    float alpha = 1.0f - expf(-dt / config.lagTimeConstant);
    portafilterEstimate += alpha * (pfEquilibrium - portafilterEstimate);

    // One-shot correction at the end of the observation window: compare actual
    // boiler rise rate against the modelled expectation and scale the estimate.
    float elapsedSecs = static_cast<float>(now - beginMs) / 1000.0f;
    if (!correctionApplied && elapsedSecs >= config.correctionWindowSecs) {
        correctionApplied = true;
        float actualRise   = boilerTemp - initialBoilerTemp;
        float expectedRise = (pfEquilibrium - initialBoilerTemp) *
                             (1.0f - expf(-config.correctionWindowSecs / config.boilerLagEstimate));
        if (expectedRise > 0.5f) {
            float factor = constrain(actualRise / expectedRise, 0.5f, 2.0f);
            portafilterEstimate = config.ambientTemp +
                                  factor * (portafilterEstimate - config.ambientTemp);
            Serial.printf("[%s] correction: factor=%.2f  pf_est=%.1fÂ°C\n",
                          THERMAL_TAG, factor, portafilterEstimate);
        }
    }

    lastBoilerTemp = boilerTemp;

    // Periodic serial log every logIntervalMs.
    if (now - lastLogMs >= config.logIntervalMs) {
        lastLogMs = now;
        bool ready = isReadyToBrew();
        int  secs  = getSecondsToReady();
        if (ready && !readyLogged) {
            readyLogged = true;
            Serial.printf("[%s] boiler=%.1fÂ°C  pf_est=%.1fÂ°C  secs_to_ready=0  THERMAL READY\n",
                          THERMAL_TAG, boilerTemp, portafilterEstimate);
        } else {
            Serial.printf("[%s] boiler=%.1fÂ°C  pf_est=%.1fÂ°C  secs_to_ready=%d\n",
                          THERMAL_TAG, boilerTemp, portafilterEstimate, secs);
        }
    }
}

bool ThermalModel::isReadyToBrew() const {
    return portafilterEstimate >= (currentTarget - config.readyThreshold);
}

int ThermalModel::getSecondsToReady() const {
    if (!begun || lastBoilerTemp < config.ambientTemp)
        return -1;

    float threshold = currentTarget - config.readyThreshold;
    if (portafilterEstimate >= threshold)
        return 0;

    float pfEquilibrium = config.ambientTemp +
                          config.couplingFactor * (lastBoilerTemp - config.ambientTemp);
    if (pfEquilibrium < threshold)
        return -1; // boiler cannot bring portafilter to target

    float denom = pfEquilibrium - portafilterEstimate;
    if (denom <= 0.0f)
        return 0;

    float ratio = (pfEquilibrium - threshold) / denom;
    if (ratio <= 0.0f)
        return 0;
    if (ratio >= 1.0f)
        return -1;

    return static_cast<int>(-config.lagTimeConstant * logf(ratio));
}
