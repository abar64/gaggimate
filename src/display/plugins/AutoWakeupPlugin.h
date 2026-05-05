#ifndef AUTO_WAKEUP_PLUGIN_H
#define AUTO_WAKEUP_PLUGIN_H

#include <ctime>
#include <display/core/Controller.h>
#include <display/core/Plugin.h>
#include <display/core/PluginManager.h>
#include <display/core/Settings.h>

class AutoWakeupPlugin : public Plugin {
  public:
    AutoWakeupPlugin();
    void setup(Controller *controller, PluginManager *pluginManager) override;
    void loop() override;

  private:
    Controller *controller;
    PluginManager *pluginManager;
    Settings *settings;

    unsigned long lastAutoWakeupCheck = 0;
    String lastCheckedTime = "";
    static const unsigned long AUTO_WAKEUP_CHECK_INTERVAL = 30000; // 1 minute

    time_t tempWakeupAt = 0;
    int lastBroadcastedMinutes = -1;

    void checkAutoWakeup();
    void checkTempWakeup();
    bool isTimeValid();
    String getCurrentTimeString();
    int getCurrentDayOfWeek();

  public:
    void setTempWakeup(int minutes);
};

#endif // AUTO_WAKEUP_PLUGIN_H