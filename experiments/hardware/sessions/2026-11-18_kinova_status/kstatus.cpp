// Read-only Kinova Jaco2 status for "why does the arm ignore Cartesian velocity here": quick status
// (CartesianFaultState), singularity vector, protection zones, Cartesian and angular position.
// Calls only Get* functions (plus InitAPI/GetDevices/SetActiveDevice/CloseAPI), like kdiag.cpp
// (2026-11-17_kinova_config). Never StartControlAPI, Set*, Move*, Send*, Erase* or flash functions.
// Run only while no arm driver holds the USB (the fabric session stopped on that helper).
// Build on a helper: g++ -O1 -o kstatus kstatus.cpp -I ~/fabric_ws/src/kinova-ros2/kinova_driver/include/kinova -ldl
// Run: LD_LIBRARY_PATH=~/fabric_ws/src/kinova-ros2/kinova_driver/lib/x86_64-linux-gnu ./kstatus
#include <dlfcn.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include "Kinova.API.USBCommLayerUbuntu.h"
#include "KinovaTypes.h"

template <typename F> F sym(void *h, const char *name) {
    F f = reinterpret_cast<F>(dlsym(h, name));
    if (!f) { std::fprintf(stderr, "missing symbol %s\n", name); std::exit(2); }
    return f;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    void *h = dlopen("USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) { std::fprintf(stderr, "dlopen: %s\n", dlerror()); return 2; }
    auto InitAPI = sym<int (*)()>(h, "InitAPI");
    auto CloseAPI = sym<int (*)()>(h, "CloseAPI");
    auto GetDevices = sym<int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &)>(h, "GetDevices");
    auto SetActiveDevice = sym<int (*)(KinovaDevice)>(h, "SetActiveDevice");
    auto GetQuickStatus = sym<int (*)(QuickStatus &)>(h, "GetQuickStatus");
    auto GetSingularityVector = sym<int (*)(SingularityVector &)>(h, "GetSingularityVector");
    auto GetProtectionZone = sym<int (*)(ZoneList &)>(h, "GetProtectionZone");
    auto GetCartesianPosition = sym<int (*)(CartesianPosition &)>(h, "GetCartesianPosition");
    auto GetAngularPosition = sym<int (*)(AngularPosition &)>(h, "GetAngularPosition");
    int r = InitAPI();
    std::printf("InitAPI %d (1 = ok)\n", r);
    if (r != 1) return 1;
    KinovaDevice devs[MAX_KINOVA_DEVICE];
    int res = 0, n = GetDevices(devs, res);
    if (n < 1) { std::printf("no device\n"); CloseAPI(); return 1; }
    SetActiveDevice(devs[0]);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::printf("device: %s serial %s\n", devs[0].Model, devs[0].SerialNumber);
    QuickStatus q; std::memset(&q, 0, sizeof q);
    r = GetQuickStatus(q);
    std::printf("QuickStatus rc %d: CartesianFaultState %d, ControlEnableStatus %d, ControlActiveModule %d, "
                "ControlFrameType %d, RetractType %d, ForceControlStatus %d, CurrentLimitationStatus %d\n", r,
                q.CartesianFaultState, q.ControlEnableStatus, q.ControlActiveModule, q.ControlFrameType,
                q.RetractType, q.ForceControlStatus, q.CurrentLimitationStatus);
    SingularityVector s; std::memset(&s, 0, sizeof s);
    r = GetSingularityVector(s);
    std::printf("SingularityVector rc %d: translation count %d distance %.4f, orientation count %d distance %.4f, "
                "repulsion (%.4f %.4f %.4f | %.4f %.4f %.4f)\n", r, s.TranslationSingularityCount,
                s.TranslationSingularityDistance, s.OrientationSingularityCount, s.OrientationSingularityDistance,
                s.RepulsionVector.X, s.RepulsionVector.Y, s.RepulsionVector.Z, s.RepulsionVector.ThetaX,
                s.RepulsionVector.ThetaY, s.RepulsionVector.ThetaZ);
    ZoneList z; std::memset(&z, 0, sizeof z);
    r = GetProtectionZone(z);
    std::printf("ProtectionZone rc %d: %d zone(s)\n", r, z.NbZones);
    for (int i = 0; i < z.NbZones && i < LEGACY_CONFIG_NB_ZONES_MAX; ++i) {
        const Zone &zz = z.Zones[i];
        std::printf("  zone %d id %d shape %d speeds %.3f %.3f %.3f\n", i, zz.ID, (int)zz.zoneShape.shapeType,
                    zz.zoneLimitation.speedParameter1, zz.zoneLimitation.speedParameter2, zz.zoneLimitation.speedParameter3);
        for (int k = 0; k < LEGACY_CONFIG_NB_POINTS_COUNT; ++k) {
            const CartesianInfo &p = zz.zoneShape.Points[k];
            std::printf("    point %d: %.3f %.3f %.3f\n", k, p.X, p.Y, p.Z);
        }
    }
    CartesianPosition c; std::memset(&c, 0, sizeof c);
    r = GetCartesianPosition(c);
    std::printf("CartesianPosition rc %d: %.4f %.4f %.4f | %.4f %.4f %.4f\n", r, c.Coordinates.X, c.Coordinates.Y,
                c.Coordinates.Z, c.Coordinates.ThetaX, c.Coordinates.ThetaY, c.Coordinates.ThetaZ);
    AngularPosition a; std::memset(&a, 0, sizeof a);
    r = GetAngularPosition(a);
    std::printf("AngularPosition rc %d: %.2f %.2f %.2f %.2f %.2f %.2f\n", r, a.Actuators.Actuator1, a.Actuators.Actuator2,
                a.Actuators.Actuator3, a.Actuators.Actuator4, a.Actuators.Actuator5, a.Actuators.Actuator6);
    CloseAPI();
    return 0;
}
