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

template <typename epicsType> int softDetector::computeImage(asynUser*& pasynUser, epicsType*& array, size_t nElements)
{
    epicsType *pMono=NULL, *pRed=NULL, *pBlue=NULL, *pGreen=NULL;
    int columnStep=0, rowStep=0, colorMode;
    int status = asynSuccess;
    int sizeX, sizeY;
    int i, j, k=0;
    int i0, j0;
    int arrayModeVal; /* Overwrite (0) or append (1) */
    int numElementsVal; /* Number of elements to write in append mode */
    int currentPixelVal; /* Append beginning at this pixel */
    int pixelCount; /* How many pixels have been written */

    status = getIntegerParam(NDColorMode, &colorMode);
    status |= getIntegerParam(ADSizeX, &sizeX);
    status |= getIntegerParam(ADSizeY, &sizeY);
    status |= getIntegerParam(numElements, &numElementsVal);
    status |= getIntegerParam(arrayMode, &arrayModeVal);
    status |= getIntegerParam(currentPixel, &currentPixelVal);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:computeImage: error getting parameters",
                        driverName);

    switch (colorMode)
    {
        case NDColorModeMono:
            pMono = (epicsType *)this->pArrays[0]->pData;
            break;
        case NDColorModeRGB1:
            columnStep = 3;
            rowStep = 0;
            pRed   = (epicsType *)this->pArrays[0]->pData;
            pGreen = (epicsType *)this->pArrays[0]->pData+1;
            pBlue  = (epicsType *)this->pArrays[0]->pData+2;
            break;
        case NDColorModeRGB2:
            columnStep = 1;
            rowStep = 2*sizeX;
            pRed   = (epicsType *)this->pArrays[0]->pData;
            pGreen = (epicsType *)this->pArrays[0]->pData+sizeX;
            pBlue  = (epicsType *)this->pArrays[0]->pData+2*sizeX;
            break;
        case NDColorModeRGB3:
            columnStep = 1;
            rowStep = 0;
            pRed   = (epicsType *)this->pArrays[0]->pData;
            pGreen = (epicsType *)this->pArrays[0]->pData+sizeX*sizeY;
            pBlue  = (epicsType *)this->pArrays[0]->pData+2*sizeX*sizeY;
            break;
    }
    this->pArrays[0]->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);

    if (arrayModeVal==0){
        i0=0;
        j0=0;
    } else {
        i0 = floor(currentPixelVal/sizeX);
        j0 = currentPixelVal % sizeX;
    }


    pixelCount = 0;
    for (i=i0;i<sizeY;i++)
    {
        if (arrayModeVal==1){
            if (i<i0){
                continue;
            }
        }
        switch(colorMode)
        {
            case NDColorModeMono:
                for (j=0;j<sizeX;j++)
                {
                    if (arrayModeVal==1){
                        if ((i==i0)&&(j<j0)){
                            continue;
                        }
                    }
                    *(pMono+i*sizeX+j) = (epicsType) *(array+i*sizeX+j);
                    pixelCount += 1;
                    currentPixelVal += 1;
                    if ((currentPixelVal>=sizeX*sizeY) || /* written one full array */
                        (pixelCount>=numElementsVal)){ /* Written all requested elements */
                        /* currentPixelVal is used to determine if acquisition is completed
                           and is reset to zero in startTask */
                        goto exceed_pixel_count;
                    }
                }
                break;
            case NDColorModeRGB1:
            case NDColorModeRGB2:
            case NDColorModeRGB3:
                for (j=j0;j<sizeX;j++)
                {
                    if (arrayMode==1){
                        if ((i==i0)&&(j<j0)){
                            continue;
                        }
                    }
                    *pRed = (epicsType) *(array + k++);
                    *pBlue = (epicsType) *(array + k++);
                    *pGreen = (epicsType) *(array + k++);
                    pRed  += columnStep;
                    pBlue += columnStep;
                    pBlue += columnStep;
                    pixelCount += 1;
                    currentPixelVal += 1;
                    if ((currentPixelVal>=sizeX*sizeY) || /* written one full array */
                        (pixelCount>=numElementsVal)){ /* Written all requested elements */
                        /* currentPixelVal is used to determine if acquisition is completed
                           and is reset to zero in startTask */
                        goto exceed_pixel_count;
                    }
                }
                pRed   += rowStep;
                pGreen += rowStep;
                pBlue  += rowStep;
                break;
        }
    }
exceed_pixel_count:
    if (arrayModeVal==1){
        setIntegerParam(currentPixel, currentPixelVal);
        setIntegerParam(currentPixel_RBV, currentPixelVal);
    }
    callParamCallbacks();
    return(status);
}

template <typename epicsType> int softDetector::updateImage(asynUser*& pasynUser, epicsType*& value, size_t nElements)
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int itemp;
    int xDim=0, yDim=1, colorDim=-1;
    int arrayModeVal = 0; /* Overwrite or append */
    NDArrayInfo arrayInfo;
    int sizeX, sizeY;
    int maxSizeX, maxSizeY;
    NDColorMode_t colorMode;
    int ndims=0;
    int currentPixelVal;
    size_t dims[3];
    NDArray *pImage = this->pArrays[0];
    const char* functionName = "updateImage";

    /* Get parameters from ADBase */
    status |= getIntegerParam(ADSizeX,     &sizeX);
    status |= getIntegerParam(ADSizeY,     &sizeY);
    status |= getIntegerParam(ADMaxSizeX,  &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY,  &maxSizeY);
    status |= getIntegerParam(NDDataType,  &itemp); dataType = (NDDataType_t) itemp;
    status |= getIntegerParam(NDColorMode, &itemp); colorMode = (NDColorMode_t) itemp;
    status |= getIntegerParam(arrayMode, &arrayModeVal);
    status |= getIntegerParam(currentPixel, &currentPixelVal);

    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting parameters",
                        driverName, functionName);

    /* Make sure parameters are consistent, fix them if they're not. */
    if (sizeX > maxSizeX) {
        sizeX = maxSizeX;
        status |= setIntegerParam(ADSizeX, sizeX);
    }
    if (sizeY > maxSizeY) {
        sizeY = maxSizeY;
        status |= setIntegerParam(ADSizeY, sizeY);
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
    if (arrayModeVal==0 || currentPixelVal==0){ /* Overwrite */
        if (this->pArrays[0]) this->pArrays[0]->release();
        /* Allocate the raw buffer we use to compute images. */
        dims[xDim] = sizeX;
        dims[yDim] = sizeY;
        if (ndims > 2) dims[colorDim] = 3;
        this->pArrays[0] = this->pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);

        if (!this->pArrays[0]) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error allocating raw buffer\n",
                      driverName, functionName);
            return(status);
        }
    }

    status |= computeImage<epicsType>(pasynUser, value, nElements);
    
    pImage = this->pArrays[0];
    pImage->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    pImage->getInfo(&arrayInfo);
    status = asynSuccess;
    status |= setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
    status |= setIntegerParam(NDArraySizeX, (int)pImage->dims[xDim].size);
    status |= setIntegerParam(NDArraySizeY, (int)pImage->dims[yDim].size);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error setting parameters\n",
                    driverName, functionName);
    callParamCallbacks();
    return(status);
}

static void startTaskC(void *drvPvt)
{
    softDetector *pPvt = (softDetector *)drvPvt;
    pPvt->startTask();
}

void softDetector::startTask()
{
    int imageStatus;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int partialArrayCallbacksVal;
    int arrayModeVal;
    int currentPixelVal;
    int sizeX, sizeY;
    int arrayCallbacks;
    int acquire;
    double acquirePeriod, delay;
    epicsTimeStamp startTime, endTime;
    double elapsedTime;
    const char* functionName = "startTask";
    NDArray *pImage;

    this->lock();
    /* Loop forever */
    while(1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is
         * started. */
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
            /* Release lock while we wait for an event that says acquire has started, then lock
             * again. */
             asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
             this->unlock();
             epicsEventWait(this->startEventId);
             this->lock();
             setStringParam(ADStatusMessage, "Acquiring Data");
             setIntegerParam(ADNumImagesCounter, 0);
        }

        /* We are acquiring. */
        /* Get the current time. */
        epicsTimeGetCurrent(&startTime);

        /* Get exposure parameters. */
        getDoubleParam(ADAcquirePeriod, &acquirePeriod);

        setIntegerParam(ADStatus, ADStatusAcquire);

        /* Open the shutter. */
        setShutter(ADShutterOpen);

        /* Call the callbacks to update any changes. */
        callParamCallbacks();

        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: waiting for user to put waveform data\n", driverName, functionName);

        this->unlock();
        imageStatus = epicsEventWait(this->imageEventId);
        this->lock();
        getIntegerParam(ADAcquire, &acquire);
        if (!acquire){
            continue;
        }

        /* Close the shutter. */
        setShutter(ADShutterClosed);

        /* Call the callbacks to update any changes. */
        callParamCallbacks();
        
        setIntegerParam(ADStatus, ADStatusReadout);
        callParamCallbacks();

        getIntegerParam(partialArrayCallbacks, &partialArrayCallbacksVal);
        if ((partialArrayCallbacksVal==1) ||
            (arrayModeVal==0) || 
            ((arrayModeVal==1) && 
             (currentPixelVal>=sizeX*sizeY)))
        {
            pImage = this->pArrays[0];
            /* Get current parameters. */
            getIntegerParam(NDArrayCounter, &imageCounter);
            getIntegerParam(ADNumImages, &numImages);
            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            getIntegerParam(ADImageMode, &imageMode);
            getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
            imageCounter++;
            numImagesCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            setIntegerParam(ADNumImagesCounter, numImagesCounter);

            /* put the frame number and timestamp into the current buffer. */
            pImage->uniqueId = imageCounter;
            pImage->timeStamp = startTime.secPastEpoch+startTime.nsec/1.e9;
            updateTimeStamp(&pImage->epicsTS);

            
            this->getAttributes(pImage->pAttributeList);
            if (arrayCallbacks) {
                this->unlock();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: calling imageData callback\n", driverName, functionName);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
            }
        }
        
        getIntegerParam(ADSizeX, &sizeX);
        getIntegerParam(ADSizeY, &sizeY);
        getIntegerParam(arrayMode, &arrayModeVal);
        getIntegerParam(currentPixel_RBV, &currentPixelVal);
        /* Check if acquisition is done. */
        if ((imageMode==ADImageSingle) ||
            ((imageMode==ADImageMultiple) &&
            (numImagesCounter>=numImages))) 
        {
            /* Check if we are appending values to an array. If so
               we want to collect one whole array before acquisition is completed. */
            if ((arrayModeVal==0) || 
                ((arrayModeVal==1) && 
                (currentPixelVal>=sizeX*sizeY)))
            {
                setIntegerParam(ADAcquire, 0);
                setIntegerParam(ADStatus, ADStatusIdle);
                setIntegerParam(currentPixel, 0);
                setIntegerParam(currentPixel_RBV, 0);
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: acquisition completes\n", driverName, functionName);
            }
        }

        callParamCallbacks();
        getIntegerParam(ADAcquire, &acquire);

        if (acquire)
        {
            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            delay = acquirePeriod - elapsedTime;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                     "%s:%s: delay=%f\n",
                     driverName, functionName, delay);
            if (delay>=0.0)
            {
                /* Set the status to readout to indicate we are in delay period  */
                setIntegerParam(ADStatus, ADStatusWaiting);
                callParamCallbacks();
                this->unlock();
                epicsEventWaitWithTimeout(this->stopEventId, delay);
                this->lock();
            }
        }

    }
}

asynStatus softDetector::writeInt8Array(asynUser *pasynUser, epicsInt8 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeInt8Array: Ignore array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsInt8>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeUInt8Array(asynUser *pasynUser, epicsUInt8 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeUInt8Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsUInt8>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeInt16Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsInt16>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeUInt16Array(asynUser *pasynUser, epicsUInt16 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeUInt16Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsUInt16>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeInt32Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsInt32>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeUInt32Array(asynUser *pasynUser, epicsUInt32 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeUInt32Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsUInt32>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeFloat32Array(asynUser *pasynUser, epicsFloat32 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeFloat32Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsFloat32>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
}
asynStatus softDetector::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements)
{
    int adstatus;
    asynStatus status = asynSuccess;
    status = getIntegerParam(ADStatus, &adstatus);

    if (!adstatus){
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:writeFloat64Array: Ignore input array while not acquiring.\n", driverName);
    } else {
        this->lock();
        updateImage<epicsFloat64>(pasynUser, value, nElements);
        this->unlock();
        epicsEventSignal(this->imageEventId);
    }
    return status;
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
            /* Wake up acquisition task. */
            epicsEventSignal(this->startEventId);
        }
        if (!value && (adstatus != ADStatusIdle))
        {
            if (adstatus == ADStatusAcquire)
            {
                epicsEventSignal(this->imageEventId);
            } else if (adstatus == ADStatusWaiting) {
                epicsEventSignal(this->stopEventId);
            }
        }
    } else if (function==arrayMode) {
        setIntegerParam(arrayMode_RBV, value);
    } else if (function==partialArrayCallbacks) {
        setIntegerParam(partialArrayCallbacks_RBV, value);   
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

softDetector::softDetector(const char *portName, int maxSizeX, int maxSizeY,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize)

    : ADDriver(portName, 1, NUM_SOFT_DETECTOR_PARAMS,
               maxBuffers, maxMemory,
               asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat32ArrayMask | asynFloat64ArrayMask | asynDrvUserMask, 
               asynInt8ArrayMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat32ArrayMask | asynFloat64ArrayMask, 
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize)
{

    int status = asynSuccess;
    const char *functionName = "softDetector";
    /* In append mode we need to store the pixel number to write to next.  */

    /* Create the epicsEvents for signaling to the acquisition task when acquisition starts and
     * stops. */
     this->startEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->startEventId)
     {
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: epicsEventCreate failure for start event\n",
            driverName, functionName);
         return;
     }
     this->stopEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->stopEventId)
     {
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: epicsEventCreate failure for stop event\n",
            driverName, functionName);
         return;
     }
     this->imageEventId = epicsEventCreate(epicsEventEmpty);
     if (!this->imageEventId)
     {
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: epicsEventCreate failure for image event\n",
            driverName, functionName);
         return;
     }

    createParam(arrayModeString,                asynParamInt32,        &arrayMode);
    createParam(arrayModeRBVString,             asynParamInt32,        &arrayMode_RBV);
    createParam(partialArrayCallbacksString,    asynParamInt32,        &partialArrayCallbacks);
    createParam(partialArrayCallbacksRBVString, asynParamInt32,        &partialArrayCallbacks_RBV);
    createParam(numElementsString,              asynParamInt32,        &numElements);
    createParam(numElementsRBVString,           asynParamInt32,        &numElements_RBV);
    createParam(currentPixelString,             asynParamInt32,        &currentPixel);
    createParam(currentPixelRBVString,          asynParamInt32,        &currentPixel_RBV);
    createParam(arrayInInt8String,              asynParamInt8Array,    &arrayInInt8);
    createParam(arrayInUInt8String,             asynParamInt8Array,    &arrayInUInt8);
    createParam(arrayInInt16String,             asynParamInt16Array,   &arrayInInt16);
    createParam(arrayInUInt16String,            asynParamInt16Array,   &arrayInUInt16);
    createParam(arrayInInt32String,             asynParamInt32Array,   &arrayInInt32);
    createParam(arrayInUInt32String,            asynParamInt32Array,   &arrayInUInt32);
    createParam(arrayInFloat32String,           asynParamFloat32Array, &arrayInFloat32);
    createParam(arrayInString,                  asynParamFloat64Array, &arrayIn);

    status  = setStringParam (ADManufacturer, "Soft Detector");
    status |= setStringParam (ADModel, "Software Detector");
    status |= setIntegerParam(ADMaxSizeX, 2000);
    status |= setIntegerParam(ADMaxSizeY, 2000);
    status |= setIntegerParam(ADSizeX, 256);
    status |= setIntegerParam(ADSizeY, 256);
    status |= setIntegerParam(NDArraySizeX, 256);
    status |= setIntegerParam(NDArraySizeY, 256);
    status |= setIntegerParam(NDArraySize, 65536); 
    status |= setIntegerParam(ADImageMode, ADImageSingle);
    status |= setDoubleParam (ADAcquireTime, .001);
    status |= setDoubleParam (ADAcquirePeriod, .005);
    status |= setIntegerParam(ADNumImages, 100);
    status |= setIntegerParam(arrayMode, 0);
    status |= setIntegerParam(arrayMode_RBV, 0);
    status |= setIntegerParam(partialArrayCallbacks, 1);
    status |= setIntegerParam(partialArrayCallbacks_RBV, 1);
    status |= setIntegerParam(numElements, 1);
    status |= setIntegerParam(numElements_RBV, 1);
    status |= setIntegerParam(currentPixel, 0);
    status |= setIntegerParam(currentPixel_RBV, 0);


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

extern "C" int softDetectorConfig(const char *portName, int maxSizeX, int maxSizeY, int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new softDetector(portName, maxSizeX, maxSizeY,
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

