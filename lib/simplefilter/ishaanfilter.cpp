// Ishaan Takrani

#include "IshaanFilter.h"


IshaanFilter::IshaanFilter(float xCalp, float yCalp, float zCalp) {
    xCal = xCalp;
    yCal = yCalp;
    zCal = zCalp;
}

IshaanFilter::~IshaanFilter() {}

void IshaanFilter::updateCalValues(float x, float y, float z) {
    xCal = x;
    yCal = y;
    zCal = z;
}

float IshaanFilter::filterLowAcc(float d) {
    if (d < 0.07 && d > -0.07){
        d = 0.0;
    }
    return d;
}



bool IshaanFilter::isAccZero(float ax,float ay,float az){
    if((ax == 0.0) & (ay == 0.0) & (az == 0.0)){
        return 1 ;
    }
    return 0;

}


void IshaanFilter::updateV() {
    float last, secondlast, currInt;
    float thresh = 0.3;

    xBufferMean = 0;
    for (int i = 0; i < xBuffer.size(); i++) xBufferMean += xBuffer[i];
    xBufferMean /= xBuffer.size();

    last = xBuffer[xBuffer.size() - 1];
    secondlast = xBuffer[xBuffer.size() - 2];
    currInt = (last + secondlast) / 2;
    float fxV = xV + currInt;
    xV = (fxV < thresh && fxV > -thresh) ? 0 : fxV - xBufferMean;

    yBufferMean = 0;
    for (int i = 0; i < yBuffer.size(); i++) yBufferMean += yBuffer[i];
    yBufferMean /= yBuffer.size();

    last = yBuffer[yBuffer.size() - 1];
    secondlast = yBuffer[yBuffer.size() - 2];
    currInt = (last + secondlast) / 2;
    float fyV = yV + currInt;
    yV = (fyV < thresh && fyV > -thresh) ? 0 : fyV - yBufferMean;

    zBufferMean = 0;
    for (int i = 0; i < zBuffer.size(); i++) zBufferMean += zBuffer[i];
    zBufferMean /= zBuffer.size();

    last = zBuffer[zBuffer.size() - 1];
    secondlast = zBuffer[zBuffer.size() - 2];
    currInt = (last + secondlast) / 2;
    float fzV = zV + currInt;
    zV = (fzV < thresh && fzV > -thresh) ? 0 : fzV - zBufferMean;
}


void IshaanFilter::updateFilter(float ax, float ay, float az) {
    ax -= xCal;
    ay -= yCal;
    az -= zCal;

    ax = filterLowAcc(ax);
    ay = filterLowAcc(ay);
    az = filterLowAcc(az);

    if(isAccZero(ax, ay, az)){
        xV = 0;
        yV = 0;
        zV = 0;
        // xBuffer.clear();
        // yBuffer.clear();
        // zBuffer.clear();
    }    

    xBuffer.push(ax);
    yBuffer.push(ay);
    zBuffer.push(az);

    updateV();
}




// bool IshaanFilter::isStable() {

//     int lastN = 15;

//     if (buffer.size() < lastN) return false;

//     float minVal = buffer[buffer.size() - lastN];
//     float maxVal = minVal;

//     for (int i = buffer.size() - lastN; i < buffer.size(); ++i) {
//         float v = buffer[i];
//         if (v < minVal) minVal = v;
//         if (v > maxVal) maxVal = v;
//     }

//     return (maxVal - minVal) < threshold;
// }
