#include "HistogramData.h"

float HistogramData::getTempC()
{
    return -45 + 175 * (temperature / (pow(2, 16) - 1));
}
float HistogramData::getTempF()
{
    return -49 + 347 * (temperature / (pow(2, 16) - 1));
}
float HistogramData::getHumidity()
{
    return 100 * (humidity / (pow(2, 16) - 1));
}

String HistogramData::toString() {
    
    String info = "";
   // Bin Counts
    for (int i = 0; i < 24; i++) {
        info += binCounts[i];
        info += ",";
    }

    // Time of flight
    info += bin1TimeToCross;
    info += ",";

    info += bin3TimeToCross;
    info += ",";

    info += bin5TimeToCross;
    info += ",";

    info += bin7TimeToCross;
    info += ",";

    // Sampling Period (s)
    info += samplingPeriod / 100.0;
    info += ",";

    // Sample Flow Rate (ml/s)
    info += sampleFlowRate / 100.0;
    info += ",";

    // Temperature (degC)
    info += -45 + 175 * (temperature / (pow(2, 16) - 1));
    info += ",";

    // Relative humidity (%)
    info += 100 * (humidity / (pow(2, 16) - 1));
    info += ",";

    // rejections (Glitch, Long ToF, Ratio, Out of Range)
    info += rejectCountGlitch;
    info += ",";
    info += rejectCountLongTOF;
    info += ",";
    info += rejectCountRatio;
    info += ",";
    info += rejectCountOutOfRange;
    info += ",";

    // Fan Revolutions
    info += fanRevCount;
    info += ",";

    // Laser Status
    info += laserStatus;
    info += ",";
    info += checkSum;
    info += ",";

    // PM Concentrations
    info += pm1;
    info += ",";
    info += pm2_5;
    info += ",";
    info += pm10;

    return info;
}