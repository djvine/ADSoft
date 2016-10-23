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
    softDetector(const char *portName, int maxSizeX, int maxSizeY,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);
    
    virtual asynStatus writeInt8Array(asynUser *pasynUser, epicsInt8 *value, size_t nElements);
    virtual asynStatus writeUInt8Array(asynUser *pasynUser, epicsUInt8 *value, size_t nElements);
    virtual asynStatus writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements);
    virtual asynStatus writeUInt16Array(asynUser *pasynUser, epicsUInt16 *value, size_t nElements);
    virtual asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    virtual asynStatus writeUInt32Array(asynUser *pasynUser, epicsUInt32 *value, size_t nElements);
    virtual asynStatus writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements);
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
    int arrayInInt8;               /* Input array of Int8                    */
    int arrayInUInt8;              /* Input array of UInt8                   */
    int arrayInInt16;              /* Input array of Int16                   */
    int arrayInUInt16;             /* Input array of UInt16                  */
    int arrayInInt32;              /* Input array of Int32                   */
    int arrayInUInt32;             /* Input array of UInt32                  */
    int arrayInFloat32;            /* Input array of Float32                 */
    int arrayIn;                   /* Input array of Float64                 */

#define FIRST_SOFT_DETECTOR_PARAM arrayMode
#define LAST_SOFT_DETECTOR_PARAM arrayInFloat64

private:
    template <typename epicsType> int updateImage(asynUser*& pasynUser, epicsType*& value, size_t nElements);
    template <typename epicsType> int computeImage(asynUser*& pasynUser, epicsType*& value, size_t nElements);
    epicsEventId startEventId;
    epicsEventId stopEventId;
    epicsEventId imageEventId;

};

#define arrayModeString                "ARRAY_MODE"                  /* (asynInt32,        r/w) Overwrite or append         */
#define arrayModeRBVString             "ARRAY_MODE_RBV"              /* (asynInt32,        ro ) Read only                   */
#define partialArrayCallbacksString    "PARTIAL_ARRAY_CALLBACKS"     /* (asynInt32,        r/w) Disable or Enable           */
#define partialArrayCallbacksRBVString "PARTIAL_ARRAY_CALLBACKS_RBV" /* (asynInt32,        ro ) Read only                   */
#define numElementsString              "NUM_ELEMENTS"                /* (asynInt32,        r/w) Number of pixels to append  */
#define numElementsRBVString           "NUM_ELEMENTS_RBV"            /* (asynInt32,        ro ) Read only                   */
#define currentPixelString             "CURRENT_PIXEL"               /* (asynInt32,        r/w) Append beginning this pixel */
#define currentPixelRBVString          "CURRENT_PIXEL_RBV"           /* (asynInt32,        r/w) Read only                   */
#define arrayInInt8String              "ARRAY_IN_INT8"               /* (asynInt8Array,    r/w) holds image data            */
#define arrayInUInt8String             "ARRAY_IN_UINT8"              /* (asynInt8Array,    r/w) holds image data            */
#define arrayInInt16String             "ARRAY_IN_INT16"              /* (asynInt16Array,   r/w) holds image data            */
#define arrayInUInt16String            "ARRAY_IN_UINT16"             /* (asynInt16Array,   r/w) holds image data            */
#define arrayInInt32String             "ARRAY_IN_INT32"              /* (asynInt32Array,   r/w) holds image data            */
#define arrayInUInt32String            "ARRAY_IN_UINT32"             /* (asynInt32Array,   r/w) holds image data            */
#define arrayInFloat32String           "ARRAY_IN_FLOAT32"            /* (asynFloat32Array, r/w) holds image data            */
#define arrayInString                  "ARRAY_IN"                    /* (asynFloat64Array, r/w) holds image data            */

#define NUM_SOFT_DETECTOR_PARAMS ((int)(&LAST_SOFT_DETECTOR_PARAM - &FIRST_SOFT_DETECTOR_PARAM + 1))
