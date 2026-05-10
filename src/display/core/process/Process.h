#ifndef PROCESS_H
#define PROCESS_H

constexpr double PREDICTIVE_TIME = 4000.0; // time window for the prediction
// constexpr double PREDICTIVE_TIME_MS = 1000.0;

class Process {
  public:
    Process() = default;
    virtual ~Process() = default;

    virtual bool isRelayActive() = 0;

    virtual bool isAltRelayActive() = 0;

    virtual float getPumpValue() = 0;

    virtual void progress() = 0;

    virtual bool isActive() = 0;

    virtual bool isComplete() = 0;

    virtual int getType() = 0;

    virtual void updateVolume(double volume) = 0;

    // Called unconditionally for every BT scale measurement, regardless of currentVolumetricSource,
    // so that processes can maintain a BT-scale-specific rate tracker even when the active
    // volumetric source is FLOW_ESTIMATION.
    virtual void updateBTVolume(double volume) {}
};

enum class ProcessTarget { VOLUMETRIC, TIME };
enum class ProcessPhase { RUNNING, FINISHED };

#endif // PROCESS_H
