#ifndef PROPELLER_H
#define PROPELLER_H

#include <cmath>
#include <cstdlib>

#include "CP_data.h"
#include "CT_data.h"

#define CONSTANTS_DEFAULT_TAU 0.05

struct PropellerParams
{
    double diameter = 0.0;
    double rho = 1.22;
} __attribute__((aligned(16))) __attribute__((packed));

struct OperPoint
{
    double omega;
    double airspeed;
    double pitch;
} __attribute__((aligned(32)));

struct OperPointRPM
{
    double rpm;
    double airspeed;
    double pitch;
} __attribute__((aligned(32)));


class Propeller
{
    template <typename T>
    static T constrain(T value, T min, T max)
    {
        return std::min(std::max(value, min), max);
    }

    template <typename T>
    static T mapValue(T value, T fromMin, T fromMax, T toMin, T toMax)
    {
        // Constrain value between fromMin and fromMax
        value = constrain(value, fromMin, fromMax);
        // Perform linear mapping
        return (value - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
    }

    template <int s1, int s2, int s3>
    static double interp_c(const OperPointRPM& point, const double arr[s1][s2][s3])
    {
        double rpm = point.rpm;
        double airvel = point.airspeed;
        double pitch = point.pitch;

        // Get floating-point mappings from input values to floating-point data indices
        double rpmIdxDouble = mapValue(rpm, 3000.0, 6000.0, 0.0, 30.0);
        double airvelIdxDouble = mapValue(airvel, 0.0, 100.0, 0.0, 100.0);
        double pitchIdxDouble = mapValue(pitch, -4.0, 40.0, 0.0, 44.0);

        int rpmStartIdx = static_cast<int>(std::round(rpmIdxDouble));
        rpmStartIdx = constrain(rpmStartIdx, 0, 30);
        int airvelStartIdx = static_cast<int>(std::round(airvelIdxDouble));
        airvelStartIdx = constrain(airvelStartIdx, 0, 100);
        int pitchStartIdx = static_cast<int>(std::round(pitchIdxDouble));
        pitchStartIdx = constrain(pitchStartIdx, 0, 44);

        int rpmEndIdx = rpmStartIdx + (rpmIdxDouble > rpmStartIdx ? 1 : -1);
        rpmEndIdx = constrain(rpmEndIdx, 0, 30);
        int airvelEndIdx = airvelStartIdx + (airvelIdxDouble > airvelStartIdx ? 1 : -1);
        airvelEndIdx = constrain(airvelEndIdx, 0, 100);
        int pitchEndIdx = pitchStartIdx + (pitchIdxDouble > pitchStartIdx ? 1 : -1);
        pitchEndIdx = constrain(pitchEndIdx, 0, 44);

        double C1_ = arr[airvelStartIdx][pitchStartIdx][rpmStartIdx];
        double C2_ = arr[airvelEndIdx][pitchEndIdx][rpmEndIdx];

        double dist = std::sqrt((rpmStartIdx - rpmIdxDouble) * (rpmStartIdx - rpmIdxDouble) +
                                (airvelStartIdx - airvelIdxDouble) * (airvelStartIdx - airvelIdxDouble) +
                                (pitchStartIdx - pitchIdxDouble) * (pitchStartIdx - pitchIdxDouble));

        return C1_+ (C2_ - C1_) * dist / 1.73205080757;
    }

    static double interp_Cp(const OperPointRPM& point)
    {
        return interp_c<101, 45, 31>(point, CPData::cpArray);
    }

    static double interp_Ct(const OperPointRPM& point)
    {
        return interp_c<101, 45, 31>(point, CTData::ct);
    }

    double D;
    double rho;

public:
    Propeller(const PropellerParams& params) : D(params.diameter), rho(params.rho)
    {
    }

    [[nodiscard]] double getThrust(const OperPoint& point) const
    {
        // Input velocity may be negative due to CCW rotation
        double omega = std::abs(point.omega);
        double rpm = omega * 60.0 / (2.0 * M_PI);
        double Ct_ = interp_Ct({
            .rpm = rpm,
            .airspeed = point.airspeed,
            .pitch = point.pitch
        });
        return Ct_ * 0.5 * rho * (omega * D / 2.0) * (omega * D / 2.0) * M_PI * (D / 2) * (D / 2);
    }

    [[nodiscard]] double getTorque(const OperPoint& point) const
    {
        double omega = std::abs(point.omega);
        double rpm = omega * 60.0 / (2.0 * M_PI);
        // Eventough name is CP, after examining QProp documentation, it is clear
        // that it is actually torque coefficient
        double Cp_ = interp_Cp({
            .rpm = rpm,
            .airspeed = point.airspeed,
            .pitch = point.pitch
        });
        return Cp_ * 0.5 * rho * (omega * D / 2.0) * (omega * D / 2.0) * M_PI * (D / 2) * (D / 2) * (D / 2);
    }

    [[nodiscard]] double getDiameter() const
    {
        return this->D;
    }

    void setRho(double rho)
    {
        this->rho = rho;
    }

    void setDiameterInch(double diameter)
    {
        this->D = diameter*2.54/100.0;
    }
};

#endif // PROPELLER_H
