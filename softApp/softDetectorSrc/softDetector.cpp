/* softDriver.cpp
 * 
 * This is a driver that allows any software to send images to area detector.
 * 
 * Author: David J. Vine
 * Berkeley National Lab
 * 
 * Created: September 28, 2016
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>
#include "softDetector.h"

static const char *driverName = "softDetector";

void softDetector::setShutter(int open)
{
    int shutterMode;
    getIntegerParam(ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector){
        setIntegerParam(ADShutterStatus, open);
    } else {
        ADDriver::setShutter(open);
    }
}


static void startTaskC(void *drvPvt)
{
    softDetector *pPvt = (softDetector *)drvPvt;
    pPvt->startTask();
}

void softDetector::startTask()
{
    int status = asynSuccess;
    int acquire;
    const char* functionName = "startTask";
    printf("%s:%s: arrived in function.\n", driverName, functionName);

    this->lock();
    /* Loop forever */
    while(1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is
         * started. */
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            /* Release lock while we wait for an event that says acquire has started, then lock
             * again. */
             printf("%s:%s: waiting for acquire to start\n", driverName, functionName);
             asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
             this->unlock();
             epicsEventWait(this->startEventId);
             printf("%s:%s: now we're acquiring again.\n", driverName, functionName);
             this->lock();
             acquire = 1;
             setStringParam(ADStatusMessage, "Acquiring Data");
             setIntegerParam(ADNumImagesCounter, 0);
        }

        setIntegerParam(ADStatus, ADStatusAcquire);

        /* Open the shutter. */
        setShutter(ADShutterOpen);

        /* Call the callbacks to update any changes. */
        callParamCallbacks();

        this->unlock();
        status = epicsEventWaitWithTimeout(this->stopEventId, 1.0);
        this->lock();
        
        setIntegerParam(ADStatus, ADStatusReadout);

        /* Close the shutter. */
        setShutter(ADShutterClosed);

        /* Call the callbacks to update any changes. */
        callParamCallbacks();

        setStringParam(ADStatusMessage, "Waiting for acquisition.");
        setIntegerParam(ADStatus, ADStatusIdle);
        callParamCallbacks();

        acquire=0;
        setIntegerParam(ADAcquire, acquire);

        callParamCallbacks();

        
    }
}

asynStatus softDetector::writeInt8Array(asynUser *pasynUser, epicsInt8 *value, size_t nElements)
{
    printf("%s:int8", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeUInt8Array(asynUser *pasynUser, epicsUInt8 *value, size_t nElements)
{
    printf("%s:uint8", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements)
{
    printf("%s: int16", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeUInt16Array(asynUser *pasynUser, epicsUInt16 *value, size_t nElements)
{
    printf("%s: uint16", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    printf("%s: int32", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeUInt32Array(asynUser *pasynUser, epicsUInt32 *value, size_t nElements)
{
    printf("%s: uint32", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements)
{

        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:writeInt32 \n",
            driverName);
    printf("%s: float32", driverName);
    return asynSuccess;
}
asynStatus softDetector::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    printf("%s: float64", driverName);
    return asynSuccess;
}
/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire.
  * \param[in] pasynUser pasynUser structure that encodes reason and address.
  * \param[in] value Value to write
  */
asynStatus softDetector::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    asynStatus status = asynSuccess;

    status = setIntegerParam(function, value);

    if (function==ADAcquire)
    {
        getIntegerParam(ADStatus, &adstatus);
        if (value && (adstatus==ADStatusIdle))
        {
            printf("Sending wakeup signal\n");
            /* Wake up acquisition task. */
            epicsEventSignal(this->startEventId);
        }
        if (!value && (adstatus != ADStatusIdle))
        {
            printf("Sending stop recording signal.\n");
            epicsEventSignal(this->stopEventId);
        }
    } else {
        status = ADDriver::writeInt32(pasynUser, value);
    }

    callParamCallbacks();

    if (status)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:writeInt32 error, status=%d function=%d, value=%d\n",
            driverName, status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:writeInt32: function=%d, value=%d\n",
            driverName, function, value);
    }
    return status;
}

/** Constructor for softDetector; most parameters are simply passed to ADDriver::ADDriver.
  * \param[in] portName The name of the asyn port to be created
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver
  *            is allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            is allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set
  *            in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in
  *            asynFlags.
  **/

softDetector::softDetector(const char *portName,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize)

    : ADDriver(portName, 1, NUM_SOFT_DETECTOR_PARAMS,
               maxBuffers, maxMemory,
               asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat32ArrayMask | asynFloat64ArrayMask, 
               asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat32ArrayMask | asynFloat64ArrayMask, 
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize)

{

    int status = asynSuccess;
    const char *functionName = "softDetector";

    /* Create the epicsEvents for signaling to the acquisition task when acquisition starts and
     * stops. */
     this->startEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->startEventId)
     {
         printf("%s:%s: epicsEventCreate failure for start event\n",
            driverName, functionName);
         return;
     }
     this->stopEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->stopEventId)
     {
         printf("%s:%s: epicsEventCreate failure for stop event\n",
            driverName, functionName);
         return;
     }

    createParam(arrayModeString,       asynParamInt32,        &arrayMode);
    createParam(arrayInInt8String,     asynParamInt8Array,    &arrayInInt8);
    createParam(arrayInUInt8String,    asynParamInt8Array,    &arrayInUInt8);
    createParam(arrayInInt16String,    asynParamInt16Array,   &arrayInInt16);
    createParam(arrayInUInt16String,   asynParamInt16Array,   &arrayInUInt16);
    createParam(arrayInInt32String,    asynParamInt32Array,   &arrayInInt32);
    createParam(arrayInUInt32String,   asynParamInt32Array,   &arrayInUInt32);
    createParam(arrayInFloat32String,  asynParamFloat32Array, &arrayInFloat32);
    createParam(arrayInFloat64String,  asynParamFloat64Array, &arrayInFloat64);

    status  = setStringParam (ADManufacturer, "Soft Detector");
    status |= setStringParam (ADModel, "Software Detector");
    status |= setIntegerParam(ADMaxSizeX, 2000);
    status |= setIntegerParam(ADMaxSizeY, 2000);
    status |= setIntegerParam(ADMinX, 0); 
    status |= setIntegerParam(ADMinY, 0); 
    status |= setIntegerParam(ADBinX, 1); 
    status |= setIntegerParam(ADBinY, 1); 
    status |= setIntegerParam(ADReverseX, 0); 
    status |= setIntegerParam(ADReverseY, 0); 
    status |= setIntegerParam(ADSizeX, 1000);
    status |= setIntegerParam(ADSizeY, 1000);
    status |= setIntegerParam(NDArraySizeX, 1000);
    status |= setIntegerParam(NDArraySizeY, 1000);
    status |= setIntegerParam(NDArraySize, 0); 
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setDoubleParam (ADAcquireTime, .001);
    status |= setDoubleParam (ADAcquirePeriod, .005);
    status |= setIntegerParam(ADNumImages, 100);
    status |= setIntegerParam(arrayMode, 0);


    if (status) {
	printf("%s: Unable to set camera prameters.", functionName);
	return;
    }

    /* Create the thread that updates the images */
    status = (epicsThreadCreate("softDetectorTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)startTaskC,
                                this)==NULL);

    if (status)
    {
        printf("%s:%s: epicsThreadCreate failure for image task\n",
            driverName, functionName);
        return;
    }
}

extern "C" int softDetectorConfig(const char *portName, int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new softDetector(portName,
                    (maxBuffers < 0) ? 0 : maxBuffers,
                    (maxMemory < 0) ? 0 : maxMemory,
                    priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg softDetectorConfigArg0 = {"Port name", iocshArgString};
static const iocshArg softDetectorConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg softDetectorConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg softDetectorConfigArg3 = {"priority", iocshArgInt};
static const iocshArg softDetectorConfigArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const softDetectorConfigArgs[] =  {&softDetectorConfigArg0,
                                                           &softDetectorConfigArg1,
                                                           &softDetectorConfigArg2,
                                                           &softDetectorConfigArg3,
                                                           &softDetectorConfigArg4};
static const iocshFuncDef configsoftDetector = {"softDetectorConfig", 5, softDetectorConfigArgs};
static void configsoftDetectorCallFunc(const iocshArgBuf *args)
{
    softDetectorConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

static void softDetectorRegister(void)
{

    iocshRegister(&configsoftDetector, configsoftDetectorCallFunc);
}

extern "C" {
epicsExportRegistrar(softDetectorRegister);
}

