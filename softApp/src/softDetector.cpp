/* softDriver.cpp
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

template <typeName epicsType> int softDetector::computeImage()
{
    epicsType *pMono=NULL, *pRed=NULL, *pBlue=NULL, *pGreen=NULL;
    int columnStep=0, rowStep=0, colorMode;
    int status = asynSuccess;
    int i, j;
    int itemp;
    NDDataType_t dataType;

    status = getIntegerParam(NDColorMode, *colorMode);
    status |= getIntegerParam(ADDataType, *itemp); dataType = (NDDataType_t) itemp;
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting parameters",
                        driverName, functionName);

    if (epicsType != ADDataType)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:computeImage: waveform put doesn't match AD data type",
                driverName);
    }

    switch (colorMode)
    {
        case NDColorModeMono:
            pMono = (epicsType *)pRaw->pData;
            break;
        case NDColorModeRGB1:
            columnStep = 3;
            rowStep = 0;
            pRed   = (epicsType *)pRaw->pData;
            pGreen = (epicsType *)pRaw->pData+1;
            pBlue  = (epicsType *)pRaw->pData+2;
            break;
        case NDColorModeRGB2:
            columnStep = 1;
            rowStep = 2*sizeX;
            pRed   = (epicsType *)pRaw->pData;
            pGreen = (epicsType *)pRaw->pData+sizeX;
            pBlue  = (epicsType *)pRaw->pData+2*sizeX;
            break;
        case NDColorModeRGB3:
            columnStep = 1;
            rowStep = 0;
            pRed   = (epicsType *)pRaw->pData;
            pGreen = (epicsType *)pRaw->pData+sizeX*sizeY;
            pBlue  = (epicsType *)pRaw->pData+2*sizeX*sizeY;
            break;
    }
    pRaw->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);

    for (i=0;i<sizeY;i++)
    {
        switch(colorMode)
        {
            case NDColorModeMono:
                for (j=0;j<sizeX;j++)
                {
                    rndm = 2.*(rand()/(double)RAND_MAX-0.5);
                    *pMono++ = (epicsType) rndm;
                }
                break;
            case NDColorModeRGB1:
            case NDColorModeRGB2:
            case NDColorModeRGB3:
                for (j=0;j<sizeX;j++)
                {
                    rndm = 2.*(rand()/(double)RAND_MAX-0.5);
                    *pRed = (epicsType)rndm;
                    rndm = 2.*(rand()/(double)RAND_MAX-0.5);
                    *pBlue = (epicsType)rndm;
                    rndm = 2.*(rand()/(double)RAND_MAX-0.5);
                    *pGreen = (epicsType)rndm;
                    pRed  += columnStep;
                    pBlue += columnStep;
                    pBlue += columnStep;
                }
                pRed   += rowStep;
                pGreen += rowStep;
                pBlue  += rowStep;
                break;
        }
    }
    
    epicsEventSignal(this->imageEventID);
    return(status);
}

/* Template function for reassembling an NDArray from a 1-D waveform. */
int softDetector::processArray()
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int itemp;
    int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
    int xDim=0, yDim=1, colorDim=-1;
    int arrayModeVal = 0 /* Overwrite or append */
    int maxSizeX, maxSizeY;
    int colorMode;
    int ndims=0;
    NDDimension_t dimsOut[3];
    size_t dims[3];
    NDArrayInfo_t arrayInfo;
    NDArray *pImage;
    const char* functionName = "reconstructArray";

    /* Get parameters from ADBase */
    status  = getIntegerParam(ADBinX,      &binX);
    status |= getIntegerParam(ADBinY,      &binY);
    status |= getIntegerParam(ADMinX,      &minX);
    status |= getIntegerParam(ADMinY,      &minY);
    status |= getIntegerParam(ADSizeX,     &sizeX);
    status |= getIntegerParam(ADSizeY,     &sizeY);
    status |= getIntegerParam(ADMaxSizeX,  &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY,  &maxSizeY);
    status |= getIntegerParam(ADReverseX,  &reverseX);
    status |= getIntegerParam(ADReverseY,  &reverseY);
    status |= getIntegerParam(ADDataType,  &itemp); dataType = (NDDataType_t) itemp;
    status |= getIntegerParam(ADColorMode, &colorMode);
    status |= getIntegerParam(arrayMode, &arrayModeVal);

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting parameters",
                        driverName, functionName);

    /* Make sure parameters are consistent, fix them if they're not. */
    if (binX<1){
        binX=1;
        status |= setIntegerParam(ADBinx, binX);
    }
    if (binY<1){
        binY=1;
        status |= setIntegerParam(ADBinY, binY);
    }
    if (minX < 0) {
        minX = 0;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY < 0) {
        minY = 0;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX-minX;
        status |= setIntegerParam(ADSizeX, sizeX);
    }
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY-minY;
        status |= setIntegerParam(ADSizeY, sizeY);
    }


    if (pRaw_) pRaw_->release();
    /* Allocate the raw buffer we use to compute images. */
    dims[xDim] = maxSizeX;
    dims[yDim] = maxSizeY;
    if (ndims > 2) dims[colorDim] = 3;
    pRaw_ = this->pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);

    if (!pRaw_) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error allocating raw buffer\n",
                  driverName, functionName);
        return(status);
    }

    /* This switch is used to determine the pixel order in the array */
    switch (colorMode) {
        case NDColorModeMono:
            ndims = 2;
            xDim = 0;
            yDim = 1;
            break;
        case NDColorModeRGB1:
            ndims = 3;
            colorDim = 0;
            xDim     = 1;
            yDim     = 2;
            break;
        case NDColorModeRGB2:
            ndims = 3;
            colorDim = 1;
            xDim     = 0;
            yDim     = 2;
            break;
        case NDColorModeRGB3:
            ndims = 3;
            colorDim = 2;
            xDim     = 0;
            yDim     = 1;
            break;
    }

    /* Stop here and wait for an array to be written */
    epicsEventWait(this->imageEventId);

    
    /* Extract the region of interest with binning.
     * If the entire image is being used (no ROI or binning) that's OK because
     * convertImage detects that case and is very efficient */
    pRaw->initDimension(&dimsOut[xDim], sizeX);
    pRaw->initDimension(&dimsOut[yDim], sizeY);
    if (ndims > 2) pRaw->initDimension(&dimsOut[colorDim], 3);
    dimsOut[xDim].binning = binX;
    dimsOut[xDim].offset  = minX;
    dimsOut[xDim].reverse = reverseX;
    dimsOut[yDim].binning = binY;
    dimsOut[yDim].offset  = minY;
    dimsOut[yDim].reverse = reverseY;
    /* We save the most recent image buffer so it can be used in the read() function.
     * Now release it before getting a new version. */
    if (this->pArrays[0]) this->pArrays[0]->release();
    status = this->pNDArrayPool->convert(pRaw,
                                         &this->pArrays[0],
                                         dataType,
                                         dimsOut);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error allocating buffer in convert()\n",
                    driverName, functionName);
        return(status);
    }
    pImage = this->pArrays[0];
    pImage->getInfo(&arrayInfo);
    status = asynSuccess;
    status |= setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
    status |= setIntegerParam(NDArraySizeX, (int)pImage->dims[xDim].size);
    status |= setIntegerParam(NDArraySizeY, (int)pImage->dims[yDim].size);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error setting parameters\n",
                    driverName, functionName);
    return(status);
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
             asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
             this->unlock();
             epicsEventWait(this->startEventId);
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

        setIntegerParam(ADStatus, ADStatusReadout);

        status |= processArray()

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
    computeImage<epicsInt8>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeUInt8Array(asynUser *pasynUser, epicsUInt8 *value, size_t nElements)
{
    computeImage<epicsUInt8>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements)
{
    computeImage<epicsInt16>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeUInt16Array(asynUser *pasynUser, epicsUInt16 *value, size_t nElements)
{
    computeImage<epicsUInt16>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    computeImage<epicsInt32>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeUInt32Array(asynUser *pasynUser, epicsUInt32 *value, size_t nElements)
{
    computeImage<epicsUInt32>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements)
{
    computeImage<epicsFloat32>(pasynUser, value, nElements, epicsInt8);
    return asynSuccess;
}
asynStatus softDetector::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    computeImage<epicsFloat64>(pasynUser, value, nElements, epicsInt8);
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
        if (function < FIRST_SOFT_DETECTOR_PARAM) status = ADDriver::writeInt32(pasynUser, value);
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

/** Constructor for softDetector; most parameters are passed to ADDriver::ADDriver.
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
        pRaw(NULL)

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
     this->imageEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->imageEventId)
     {
         printf("%s:%s: epicsEventCreate failure for image event\n",
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
static const iocshArg softDetectorConfigArg1 = {"Max X size", iocshArgString};
static const iocshArg softDetectorConfigArg2 = {"Max Y size", iocshArgString};
static const iocshArg softDetectorConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg softDetectorConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg softDetectorConfigArg5 = {"priority", iocshArgInt};
static const iocshArg softDetectorConfigArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const softDetectorConfigArgs[] =  {&softDetectorConfigArg0,
                                                           &softDetectorConfigArg1,
                                                           &softDetectorConfigArg2,
                                                           &softDetectorConfigArg3,
                                                           &softDetectorConfigArg4,
                                                           &softDetectorConfigArg5,
                                                           &softDetectorConfigArg6};
static const iocshFuncDef configsoftDetector = {"softDetectorConfig", 7, softDetectorConfigArgs};
static void configsoftDetectorCallFunc(const iocshArgBuf *args)
{
    softDetectorConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

static void softDetectorRegister(void)
{

    iocshRegister(&configsoftDetector, configsoftDetectorCallFunc);
}

extern "C" {
epicsExportRegistrar(softDetectorRegister);
}

