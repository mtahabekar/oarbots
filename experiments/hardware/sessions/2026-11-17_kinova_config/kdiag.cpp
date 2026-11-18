// Read-only Kinova Jaco2 diagnostics over the USB API, built against Kinova's own headers so the
// struct layouts are exact. Calls only Get* functions (plus InitAPI/SetActiveDevice/CloseAPI).
// Never StartControlAPI, never a Set*, Move*, Send*, Restore* or flash function.
// Usage: kdiag [joystick_watch_seconds]
#include <dlfcn.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include "Kinova.API.USBCommLayerUbuntu.h"
#include "KinovaTypes.h"
#define CODE_VERSION_COUNT 42
#define MAX_INVENTORY 15

template <typename F> F sym(void *h, const char *name) {
    F f = reinterpret_cast<F>(dlsym(h, name));
    if (!f) { std::fprintf(stderr, "missing symbol %s\n", name); std::exit(2); }
    return f;
}

static const char *cfName(int f) {
    static const char *n[] = {"none", "Disable_EnableJoystick", "Retract_ReadyToUse", "Change_TwoAxis_ThreeAxis",
        "Change_DrinkingMode", "Cycle_ModeA", "Cycle_ModeB", "DecreaseSpeed", "IncreaseSpeed"};
    return (f >= 0 && f <= 8) ? n[f] : nullptr;
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    double watch = argc > 1 ? std::atof(argv[1]) : 0.;
    void *h = dlopen("USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) { std::fprintf(stderr, "dlopen: %s\n", dlerror()); return 2; }
    auto InitAPI = sym<int (*)()>(h, "InitAPI");
    auto CloseAPI = sym<int (*)()>(h, "CloseAPI");
    auto GetDevices = sym<int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &)>(h, "GetDevices");
    auto SetActiveDevice = sym<int (*)(KinovaDevice)>(h, "SetActiveDevice");
    auto GetQuickStatus = sym<int (*)(QuickStatus &)>(h, "GetQuickStatus");
    auto GetGeneralInformations = sym<int (*)(GeneralInformations &)>(h, "GetGeneralInformations");
    auto GetClientConfigurations = sym<int (*)(ClientConfigurations &)>(h, "GetClientConfigurations");
    auto GetControlMapping = sym<int (*)(ControlMappingCharts &)>(h, "GetControlMapping");
    auto GetCodeVersion = sym<int (*)(int[CODE_VERSION_COUNT])>(h, "GetCodeVersion");
    auto GetPeripheralInventory = sym<int (*)(PeripheralInfo[MAX_INVENTORY])>(h, "GetPeripheralInventory");
    auto GetSystemErrorCount = sym<int (*)(unsigned int &)>(h, "GetSystemErrorCount");
    auto GetSystemError = sym<int (*)(unsigned int, SystemError &)>(h, "GetSystemError");
    auto GetAngularPosition = sym<int (*)(AngularPosition &)>(h, "GetAngularPosition");
    auto GetJoystickValue = sym<int (*)(JoystickCommand &)>(h, "GetJoystickValue");

    int r = InitAPI();
    std::printf("InitAPI %d (1 = ok)\n", r);
    if (r != 1) return 1;
    KinovaDevice devs[MAX_KINOVA_DEVICE];
    int res = 0, n = GetDevices(devs, res);
    std::printf("devices %d\n", n);
    if (n < 1) { CloseAPI(); return 1; }
    for (int d = 0; d < n; ++d) {
    std::printf("\n================ device %d: model '%s' serial '%s' version %d.%d.%d type %d id %d\n", d,
                devs[d].Model, devs[d].SerialNumber, devs[d].VersionMajor, devs[d].VersionMinor,
                devs[d].VersionRelease, devs[d].DeviceType, devs[d].DeviceID);
    SetActiveDevice(devs[d]);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    QuickStatus q; std::memset(&q, 0, sizeof q);
    r = GetQuickStatus(q);
    std::printf("\nQuickStatus rc %d: fingers %d %d %d, RetractType %d, RetractComplexity %d, ControlEnableStatus %d, "
                "ControlActiveModule %d, ControlFrameType %d, CartesianFaultState %d, ForceControlStatus %d, "
                "CurrentLimitationStatus %d, RobotType %d, RobotEdition %d, TorqueSensorsStatus %d\n", r,
                q.Finger1Status, q.Finger2Status, q.Finger3Status, q.RetractType, q.RetractComplexity,
                q.ControlEnableStatus, q.ControlActiveModule, q.ControlFrameType, q.CartesianFaultState,
                q.ForceControlStatus, q.CurrentLimitationStatus, q.RobotType, q.RobotEdition, q.TorqueSensorsStatus);

    static GeneralInformations g; std::memset(&g, 0, sizeof g);
    r = GetGeneralInformations(g);
    const SystemStatus &s = g.ActualSystemStatus;
    std::printf("\nGeneralInformations rc %d: code %u rev %u, Status %u, Controller %u, ControlMode %u, HandMode %u, "
                "ConnectedActuatorCount %u, PositionType %u, supply %.2f V, current %.3f A, power %.2f W\n", r,
                g.CodeVersion, g.CodeRevision, g.Status, g.Controller, g.ControlMode, g.HandMode,
                g.ConnectedActuatorCount, g.PositionType, g.SupplyVoltage, g.TotalCurrent, g.Power);
    std::printf("  SystemStatus: JoystickActive %u, RetractStatus %u, DrinkingMode %u, ArmLaterality %u, "
                "TranslationActive %u, RotationActive %u, FingersActive %u, WarnForce %u, WarnFingers %u, "
                "WarnLowVoltage %u, MajorErrorOccured %u\n", s.JoystickActive, s.RetractStatus, s.DrinkingMode,
                s.ArmLaterality, s.TranslationActive, s.RotationActive, s.FingersActive, s.WarningOverchargeForce,
                s.WarningOverchargeFingers, s.WarningLowVoltage, s.MajorErrorOccured);
    std::printf("  SPI errors main %u ext %u, CAN errors main %u ext %u\n", g.ErrorsMainSPICount,
                g.ErrorsExternalSPICount, g.ErrorsMainCANCount, g.ErrorsExternalCANCount);
    std::printf("  actuator temps");
    for (int i = 0; i < MAXACTUATORNUMBER; ++i) std::printf(" %.1f", g.ActuatorsTemperatures[i]);
    std::printf("; finger temps %.1f %.1f %.1f\n  actuator comm errors", g.FingersTemperatures[0],
                g.FingersTemperatures[1], g.FingersTemperatures[2]);
    for (int i = 0; i < MAXACTUATORNUMBER; ++i) std::printf(" %d", g.ActuatorsCommErrors[i]);
    std::printf("; finger comm errors %d %d %d\n", g.FingersCommErrors[0], g.FingersCommErrors[1], g.FingersCommErrors[2]);
    std::printf("  peripherals connected 0x%x 0x%x 0x%x 0x%x, device ids 0x%x 0x%x 0x%x 0x%x\n",
                g.PeripheralsConnected[0], g.PeripheralsConnected[1], g.PeripheralsConnected[2], g.PeripheralsConnected[3],
                g.PeripheralsDeviceID[0], g.PeripheralsDeviceID[1], g.PeripheralsDeviceID[2], g.PeripheralsDeviceID[3]);

    static ClientConfigurations c; std::memset(&c, 0, sizeof c);
    r = GetClientConfigurations(c);
    std::printf("\nClientConfigurations rc %d: ClientID '%.20s' ClientName '%.20s' Organization '%.20s' Serial '%.20s' "
                "Model '%.20s'\n  Laterality %d, MaxTransVel %.3f, MaxOrientVel %.3f, MaxTransAcc %.3f, MaxOrientAcc %.3f, "
                "MaxForce %.2f, Sensibility %.2f, ComplexRetractActive %d, RetractedPositionAngle %.2f, "
                "RetractedPositionCount %d, Fingers2and3Inverted %d, DeletePreProgrammedPositionsAtRetract %d, "
                "EnableFlashErrorLog %d, EnableFlashPositionLog %d, RobotConfigSelect %d, TorqueSensorsEnable %d, "
                "FrameType %d\n", r, c.ClientID, c.ClientName, c.Organization, c.Serial, c.Model, (int)c.Laterality,
                c.MaxTranslationVelocity, c.MaxOrientationVelocity, c.MaxTranslationAcceleration,
                c.MaxOrientationAcceleration, c.MaxForce, c.Sensibility, c.ComplexRetractActive,
                c.RetractedPositionAngle, c.RetractedPositionCount, c.Fingers2and3Inverted,
                c.DeletePreProgrammedPositionsAtRetract, c.EnableFlashErrorLog, c.EnableFlashPositionLog,
                c.RobotConfigSelect, c.TorqueSensorsEnable, c.FrameType);

    static ControlMappingCharts m; std::memset(&m, 0, sizeof m);
    r = GetControlMapping(m);
    std::printf("\nControlMapping rc %d: NumOfConfiguredMapping %d, ActualControlMapping %d\n", r,
                m.NumOfConfiguredMapping, m.ActualControlMapping);
    for (int k = 0; k < CONTROL_MAPPING_COUNT; ++k) {
        const ControlMapping &cm = m.Mapping[k];
        int used = 0;
        for (int md = 0; md < MODE_MAP_COUNT; ++md)
            for (const ControlsModeMap *mm : {&cm.ModeControlsA[md], &cm.ModeControlsB[md]}) {
                for (int i = 0; i < STICK_EVENT_COUNT; ++i) used += (mm->ControlSticks[i].Minus != 0) + (mm->ControlSticks[i].Plus != 0);
                for (int i = 0; i < BUTTON_EVENT_COUNT; ++i) {
                    const ButtonEvents &b = mm->ControlButtons[i];
                    used += (b.OneClick != 0) + (b.TwoClick != 0) + (b.HoldOneSec != 0) + (b.HoldTwoSec != 0) +
                            (b.HoldThreeSec != 0) + (b.HoldFourSec != 0) + (b.HoldDown != 0);
                }
            }
        std::printf("  mapping %d: modesA %d modesB %d actualA %d actualB %d mode %d, assigned functions %d\n", k,
                    cm.NumOfModesA, cm.NumOfModesB, cm.ActualModeA, cm.ActualModeB, (int)cm.Mode, used);
        if (k < m.NumOfConfiguredMapping || k == m.ActualControlMapping) {
            const ControlsModeMap &mm = cm.ModeControlsA[0];
            for (int i = 0; i < BUTTON_EVENT_COUNT; ++i) {
                const ButtonEvents &b = mm.ControlButtons[i];
                unsigned char ev[7] = {b.OneClick, b.TwoClick, b.HoldOneSec, b.HoldTwoSec, b.HoldThreeSec, b.HoldFourSec, b.HoldDown};
                const char *en[7] = {"click", "2click", "hold1s", "hold2s", "hold3s", "hold4s", "holddown"};
                for (int e = 0; e < 7; ++e)
                    if (ev[e]) std::printf("    modeA0 button %2d %-8s -> %d %s\n", i, en[e], ev[e], cfName(ev[e]) ? cfName(ev[e]) : "");
            }
        }
    }

    int code[CODE_VERSION_COUNT] = {0};
    r = GetCodeVersion(code);
    std::printf("\nGetCodeVersion rc %d:", r);
    for (int i = 0; i < CODE_VERSION_COUNT; ++i) std::printf(" %d", code[i]);
    PeripheralInfo inv[MAX_INVENTORY]; std::memset(inv, 0, sizeof inv);
    r = GetPeripheralInventory(inv);
    std::printf("\nPeripheralInventory rc %d:\n", r);
    for (int i = 0; i < MAX_INVENTORY; ++i)
        if (inv[i].Type || inv[i].Handle)
            std::printf("  handle %u type %u port %u address %u code 0x%x\n", inv[i].Handle, inv[i].Type, inv[i].Port,
                        inv[i].Address, inv[i].CodeVersion);
    unsigned int cnt = 0;
    GetSystemErrorCount(cnt);
    std::printf("System errors: %u\n", cnt);
    for (unsigned int i = (cnt > 30 ? cnt - 30 : 0); i < cnt; ++i) {
        SystemError e; std::memset(&e, 0, sizeof e);
        GetSystemError(i, e);
        std::printf("  [%u] type %d time %u life %d data", i, (int)e.ErrorType, e.SystemTime, e.LifeTime);
        for (int k = 0; k < e.DataCount && k < 8; ++k) std::printf(" %u", e.Data[k]);
        std::printf("\n");
    }
    AngularPosition a; std::memset(&a, 0, sizeof a);
    GetAngularPosition(a);
    std::printf("Angles (deg) %.2f %.2f %.2f %.2f %.2f %.2f; fingers %.1f %.1f %.1f\n", a.Actuators.Actuator1,
                a.Actuators.Actuator2, a.Actuators.Actuator3, a.Actuators.Actuator4, a.Actuators.Actuator5,
                a.Actuators.Actuator6, a.Fingers.Finger1, a.Fingers.Finger2, a.Fingers.Finger3);

    // Byte copies of the stored settings: a backup before anyone changes anything, and for diffing arms.
    char path[128];
    std::snprintf(path, sizeof path, "%s_mapping.bin", devs[d].SerialNumber);
    if (FILE *f = std::fopen(path, "wb")) { std::fwrite(&m, sizeof m, 1, f); std::fclose(f); }
    std::snprintf(path, sizeof path, "%s_client.bin", devs[d].SerialNumber);
    if (FILE *f = std::fopen(path, "wb")) { std::fwrite(&c, sizeof c, 1, f); std::fclose(f); }
    std::printf("Saved %s_mapping.bin (%zu bytes) and %s_client.bin (%zu bytes)\n", devs[d].SerialNumber, sizeof m,
                devs[d].SerialNumber, sizeof c);
    }  // devices

    int sel = argc > 2 ? std::atoi(argv[2]) : 0;
    if (watch > 0 && sel < n) {
        SetActiveDevice(devs[sel]);
        std::printf("\nJoystick of device %d (%s)\n", sel, devs[sel].SerialNumber);
        std::printf("\nWatching the joystick for %.0f s: press buttons, move the stick\n", watch);
        std::fflush(stdout);
        JoystickCommand prev; std::memset(&prev, 0, sizeof prev);
        auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(watch);
        while (std::chrono::steady_clock::now() < end) {
            JoystickCommand j; std::memset(&j, 0, sizeof j);
            GetJoystickValue(j);
            if (std::memcmp(&j, &prev, sizeof j)) {
                std::printf("  buttons");
                for (int i = 0; i < JOYSTICK_BUTTON_COUNT; ++i) std::printf(" %d", j.ButtonValue[i]);
                std::printf(" | axes %.2f %.2f %.2f %.2f %.2f %.2f\n", j.InclineLeftRight, j.InclineForwardBackward,
                            j.Rotate, j.MoveLeftRight, j.MoveForwardBackward, j.PushPull);
                std::fflush(stdout);
                prev = j;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    std::printf("CloseAPI %d\n", CloseAPI());
    return 0;
}
