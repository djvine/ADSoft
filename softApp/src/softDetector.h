/* softDriver.h
 * 
 * This is a driver that allows any software to send images to area detector.
 * Based on the simDetector driver by Mark Rivers.
 * 
 * Author: David J. Vine
 * Berkeley National Lab
 * 
 * Created: September 28, 2016
 *
 */


#include <epicsEvent.h>
#include "ADDriver.h"

class epicsShareClass softDetector : public ADDriver{
public:
    softDetector(const char *portName, int maxSizeX, int maxSizeY, int maxSizeZ,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);
    
    virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void setShutter(int open);
    void startTask();

protected:
    int arrayMode;                 /* 0: Overwrite, 1: Append                */
    int arrayMode_RBV;             /* Array mode readback                    */
    int partialArrayCallbacks;     /* 0: Disable, 1: Enable                  */
    int partialArrayCallbacks_RBV; /* Partial array callbacks readback.      */
    int numElements;               /* Num elements to append in Append mode. */
    int numElements_RBV;           /* Num elements readback.                 */
    int currentPixel;              /* Append beginning at this pixel.        */
    int currentPixel_RBV;          /* Append beginning at this pixel.        */
    int sizeZ;                     /* z dimension size.                      */
    int sizeZ_RBV;                 /* z dimensions size readback.            */
    int maxSizeZ_RBV;              /* Maximum z dimension size readback.     */
    int arrayIn;                   /* Input array of Float64                 */

#define FIRST_SOFT_DETECTOR_PARAM arrayMode
#define LAST_SOFT_DETECTOR_PARAM arrayIn

private:
    int updateImage();
    template <typename epicsType> int computeImage();
    epicsEventId startEventId;
    epicsEventId stopEventId;
    epicsEventId imageEventId;
    epicsFloat64* pRaw;
};

#define arrayModeString                "ARRAY_MODE"                  /* (asynInt32,        r/w) Overwrite or append         */
#define arrayModeRBVString             "ARRAY_MODE_RBV"              /* (asynInt32,        ro ) Read only                   */
#define partialArrayCallbacksString    "PARTIAL_ARRAY_CALLBACKS"     /* (asynInt32,        r/w) Disable or Enable           */
#define partialArrayCallbacksRBVString "PARTIAL_ARRAY_CALLBACKS_RBV" /* (asynInt32,        ro ) Read only                   */
#define numElementsString              "NUM_ELEMENTS"                /* (asynInt32,        r/w) Number of pixels to append  */
#define numElementsRBVString           "NUM_ELEMENTS_RBV"            /* (asynInt32,        ro ) Read only                   */
#define currentPixelString             "CURRENT_PIXEL"               /* (asynInt32,        r/w) Append beginning this pixel */
#define currentPixelRBVString          "CURRENT_PIXEL_RBV"           /* (asynInt32,        ro ) Read only                   */
#define sizeZString                    "SIZE_Z"                      /* (asynInt32,        r/w) Z dimension size            */
#define sizeZRBVString                 "SIZE_Z_RBV"                  /* (asynInt32,        ro ) Read only                   */
#define maxSizeZRBVString              "MAX_SIZE_Z_RBV"              /* (asynInt32,        ro ) Read only                   */
#define arrayInString                  "ARRAY_IN"                    /* (asynFloat64Array, r/w) holds image data            */

#define NUM_SOFT_DETECTOR_PARAMS ((int)(&LAST_SOFT_DETECTOR_PARAM - &FIRST_SOFT_DETECTOR_PARAM + 1))
