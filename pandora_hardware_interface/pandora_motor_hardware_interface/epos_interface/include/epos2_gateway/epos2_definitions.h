/*************************************************************************************************************************************
**                  maxon motor ag, CH-6072 Sachseln
**************************************************************************************************************************************
**
** FILE:            Definitions.h
**
** Summary:         Functions for Linux shared library
**
** Date:            10.10.2014
** Target:          x86, x86_64, arm sf/hf 
** Written by:      maxon motor ag, CH-6072 Sachseln
**
** Changes:        4.8.1.0    (15.12.10): initial version
** 			       4.8.2.0    (14.03.11): usb interface related bugfix's
** 			       4.9.1.0    (27.07.12): ipm mode bugfix, kernel 2.6 support, ftdi driver update 
** 			       4.9.2.0    (26.04.13): rs232 baudrate bugfix, new functions: VCS_GetHomingState, VCS_WaitForHomingAttained, 	**                                        VCS_GetVelocityIsAveraged, VCS_GetCurrentIsAveraged
** 			       5.0.1.0    (10.10.14): x86_64, arm sf/hf support, new functions: VCS_GetDriverInfo, bugfix: VCS_GetErrorInfo
*************************************************************************************************************************************/

#ifndef _H_LINUX_EPOSCMD_
#define _H_LINUX_EPOSCMD_

//Communication
    int CreateCommunication();
    int DeleteCommunication();

// INITIALISATION FUNCTIONS
    #define Initialisation_DllExport            extern "C"
    #define HelpFunctions_DllExport             extern "C"
// CONFIGURATION FUNCTIONS
    #define Configuration_DllExport             extern "C"
// OPERATION FUNCTIONS
    #define Status_DllExport                    extern "C"
    #define StateMachine_DllExport              extern "C"
    #define ErrorHandling_DllExport             extern "C"
    #define MotionInfo_DllExport                extern "C"
    #define ProfilePositionMode_DllExport       extern "C"
    #define ProfileVelocityMode_DllExport       extern "C"
    #define HomingMode_DllExport                extern "C"
    #define InterpolatedPositionMode_DllExport  extern "C"
    #define PositionMode_DllExport              extern "C"
    #define VelocityMode_DllExport              extern "C"
    #define CurrentMode_DllExport               extern "C"
    #define MasterEncoderMode_DllExport         extern "C"
    #define StepDirectionMode_DllExport         extern "C"
    #define InputsOutputs_DllExport             extern "C"
// LOW LAYER FUNCTIONS
    #define CanLayer_DllExport                  extern "C"

/*************************************************************************************************************************************
* INITIALISATION FUNCTIONS
*************************************************************************************************************************************/

//Communication
    Initialisation_DllExport void*  VCS_OpenDevice(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, unsigned int* pErrorCode);
    Initialisation_DllExport int  VCS_SetProtocolStackSettings(void* KeyHandle, unsigned int Baudrate, unsigned int Timeout, unsigned int* pErrorCode);
    Initialisation_DllExport int  VCS_GetProtocolStackSettings(void* KeyHandle, unsigned int* pBaudrate, unsigned int* pTimeout, unsigned int* pErrorCode);
    Initialisation_DllExport int  VCS_CloseDevice(void* KeyHandle, unsigned int* pErrorCode);
    Initialisation_DllExport int  VCS_CloseAllDevices(unsigned int* pErrorCode);

//Info
    HelpFunctions_DllExport int  VCS_GetVersion(void* KeyHandle, uint16_t NodeId, uint16_t* pHardwareVersion, uint16_t* pSoftwareVersion, uint16_t* pApplicationNumber, uint16_t* pApplicationVersion, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetErrorInfo(unsigned int ErrorCodeValue, char* pErrorInfo, uint16_t MaxStrSize);
    HelpFunctions_DllExport int  VCS_GetDriverInfo(char* p_pszLibraryName, uint16_t p_usMaxLibraryNameStrSize,char* p_pszLibraryVersion, uint16_t p_usMaxLibraryVersionStrSize, unsigned int* p_pErrorCode);

//Advanced Functions
    HelpFunctions_DllExport int  VCS_GetDeviceNameSelection(int StartOfSelection, char* pDeviceNameSel, uint16_t MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetProtocolStackNameSelection(char* DeviceName, int StartOfSelection, char* pProtocolStackNameSel, uint16_t MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetInterfaceNameSelection(char* DeviceName, char* ProtocolStackName, int StartOfSelection, char* pInterfaceNameSel, uint16_t MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetPortNameSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, int StartOfSelection, char* pPortSel, uint16_t MaxStrSize, int* pEndOfSelection, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetBaudrateSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, int StartOfSelection, unsigned int* pBaudrateSel, int* pEndOfSelection, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetKeyHandle(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, void** pKeyHandle, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetDeviceName(void* KeyHandle, char* pDeviceName, uint16_t MaxStrSize, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetProtocolStackName(void* KeyHandle, char* pProtocolStackName, uint16_t MaxStrSize, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetInterfaceName(void* KeyHandle, char* pInterfaceName, uint16_t MaxStrSize, unsigned int* pErrorCode);
    HelpFunctions_DllExport int  VCS_GetPortName(void* KeyHandle, char* pPortName, uint16_t MaxStrSize, unsigned int* pErrorCode);

/*************************************************************************************************************************************
* CONFIGURATION FUNCTIONS
*************************************************************************************************************************************/

//General
    Configuration_DllExport int  VCS_SetObject(void* KeyHandle, uint16_t NodeId, uint16_t ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned int NbOfBytesToWrite, unsigned int* pNbOfBytesWritten, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetObject(void* KeyHandle, uint16_t NodeId, uint16_t ObjectIndex, unsigned char ObjectSubIndex, void* pData, unsigned int NbOfBytesToRead, unsigned int* pNbOfBytesRead, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_Restore(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_Store(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//Advanced Functions
    //Motor
    Configuration_DllExport int  VCS_SetMotorType(void* KeyHandle, uint16_t NodeId, uint16_t MotorType, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetDcMotorParameter(void* KeyHandle, uint16_t NodeId, uint16_t NominalCurrent, uint16_t MaxOutputCurrent, uint16_t ThermalTimeConstant, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetEcMotorParameter(void* KeyHandle, uint16_t NodeId, uint16_t NominalCurrent, uint16_t MaxOutputCurrent, uint16_t ThermalTimeConstant, unsigned char NbOfPolePairs, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetMotorType(void* KeyHandle, uint16_t NodeId, uint16_t* pMotorType, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetDcMotorParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pNominalCurrent, uint16_t* pMaxOutputCurrent, uint16_t* pThermalTimeConstant, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetEcMotorParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pNominalCurrent, uint16_t* pMaxOutputCurrent, uint16_t* pThermalTimeConstant, unsigned char* pNbOfPolePairs, unsigned int* pErrorCode);

    //Sensor
    Configuration_DllExport int  VCS_SetSensorType(void* KeyHandle, uint16_t NodeId, uint16_t SensorType, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetIncEncoderParameter(void* KeyHandle, uint16_t NodeId, unsigned int EncoderResolution, int InvertedPolarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetHallSensorParameter(void* KeyHandle, uint16_t NodeId, int InvertedPolarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetSsiAbsEncoderParameter(void* KeyHandle, uint16_t NodeId, uint16_t DataRate, uint16_t NbOfMultiTurnDataBits, uint16_t NbOfSingleTurnDataBits, int InvertedPolarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetSensorType(void* KeyHandle, uint16_t NodeId, uint16_t* pSensorType, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetIncEncoderParameter(void* KeyHandle, uint16_t NodeId, unsigned int* pEncoderResolution, int* pInvertedPolarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetHallSensorParameter(void* KeyHandle, uint16_t NodeId, int* pInvertedPolarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetSsiAbsEncoderParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pDataRate, uint16_t* pNbOfMultiTurnDataBits, uint16_t* pNbOfSingleTurnDataBits, int* pInvertedPolarity, unsigned int* pErrorCode);

    //Safety
    Configuration_DllExport int  VCS_SetMaxFollowingError(void* KeyHandle, uint16_t NodeId, unsigned int MaxFollowingError, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxFollowingError(void* KeyHandle, uint16_t NodeId, unsigned int* pMaxFollowingError, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetMaxProfileVelocity(void* KeyHandle, uint16_t NodeId, unsigned int MaxProfileVelocity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxProfileVelocity(void* KeyHandle, uint16_t NodeId, unsigned int* pMaxProfileVelocity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetMaxAcceleration(void* KeyHandle, uint16_t NodeId, unsigned int MaxAcceleration, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetMaxAcceleration(void* KeyHandle, uint16_t NodeId, unsigned int* pMaxAcceleration, unsigned int* pErrorCode);

    //Position Regulator
    Configuration_DllExport int  VCS_SetPositionRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t P, uint16_t I, uint16_t D, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetPositionRegulatorFeedForward(void* KeyHandle, uint16_t NodeId, uint16_t VelocityFeedForward, uint16_t AccelerationFeedForward, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetPositionRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t* pP, uint16_t* pI, uint16_t* pD, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetPositionRegulatorFeedForward(void* KeyHandle, uint16_t NodeId, uint16_t* pVelocityFeedForward, uint16_t* pAccelerationFeedForward, unsigned int* pErrorCode);

    //Velocity Regulator
    Configuration_DllExport int  VCS_SetVelocityRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t P, uint16_t I, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_SetVelocityRegulatorFeedForward(void* KeyHandle, uint16_t NodeId, uint16_t VelocityFeedForward, uint16_t AccelerationFeedForward, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetVelocityRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t* pP, uint16_t* pI, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetVelocityRegulatorFeedForward(void* KeyHandle, uint16_t NodeId, uint16_t* pVelocityFeedForward, uint16_t* pAccelerationFeedForward, unsigned int* pErrorCode);

    //Current Regulator
    Configuration_DllExport int  VCS_SetCurrentRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t P, uint16_t I, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetCurrentRegulatorGain(void* KeyHandle, uint16_t NodeId, uint16_t* pP, uint16_t* pI, unsigned int* pErrorCode);

    //Inputs/Outputs
    Configuration_DllExport int  VCS_DigitalInputConfiguration(void* KeyHandle, uint16_t NodeId, uint16_t DigitalInputNb, uint16_t Configuration, int Mask, int Polarity, int ExecutionMask, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_DigitalOutputConfiguration(void* KeyHandle, uint16_t NodeId, uint16_t DigitalOutputNb, uint16_t Configuration, int State, int Mask, int Polarity, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_AnalogInputConfiguration(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNb, uint16_t Configuration, int ExecutionMask, unsigned int* pErrorCode);

    //Units
    Configuration_DllExport int  VCS_SetVelocityUnits(void* KeyHandle, uint16_t NodeId, unsigned char VelDimension, signed char VelNotation, unsigned int* pErrorCode);
    Configuration_DllExport int  VCS_GetVelocityUnits(void* KeyHandle, uint16_t NodeId, unsigned char* pVelDimension, char* pVelNotation, unsigned int* pErrorCode);

/*************************************************************************************************************************************
* OPERATION FUNCTIONS
*************************************************************************************************************************************/

//OperationMode
    Status_DllExport int  VCS_SetOperationMode(void* KeyHandle, uint16_t NodeId, char OperationMode, unsigned int* pErrorCode);
    Status_DllExport int  VCS_GetOperationMode(void* KeyHandle, uint16_t NodeId, char* pOperationMode, unsigned int* pErrorCode);

//StateMachine
    StateMachine_DllExport int  VCS_ResetDevice(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_SetState(void* KeyHandle, uint16_t NodeId, uint16_t State, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_SetEnableState(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_SetDisableState(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_SetQuickStopState(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_ClearFault(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_GetState(void* KeyHandle, uint16_t NodeId, uint16_t* pState, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_GetEnableState(void* KeyHandle, uint16_t NodeId, int* pIsEnabled, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_GetDisableState(void* KeyHandle, uint16_t NodeId, int* pIsDisabled, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_GetQuickStopState(void* KeyHandle, uint16_t NodeId, int* pIsQuickStopped, unsigned int* pErrorCode);
    StateMachine_DllExport int  VCS_GetFaultState(void* KeyHandle, uint16_t NodeId, int* pIsInFault, unsigned int* pErrorCode);

//ErrorHandling
    ErrorHandling_DllExport int  VCS_GetNbOfDeviceError(void* KeyHandle, uint16_t NodeId, unsigned char *pNbDeviceError, unsigned int *pErrorCode);
    ErrorHandling_DllExport int  VCS_GetDeviceErrorCode(void* KeyHandle, uint16_t NodeId, unsigned char DeviceErrorNumber, unsigned int *pDeviceErrorCode, unsigned int *pErrorCode);

//Motion Info
    MotionInfo_DllExport int  VCS_GetMovementState(void* KeyHandle, uint16_t NodeId, int* pTargetReached, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_GetPositionIs(void* KeyHandle, uint16_t NodeId, int* pPositionIs, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_GetVelocityIs(void* KeyHandle, uint16_t NodeId, int* pVelocityIs, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_GetVelocityIsAveraged(void* KeyHandle, uint16_t NodeId, int* pVelocityIsAveraged, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_GetCurrentIs(void* KeyHandle, uint16_t NodeId, int16_t* pCurrentIs, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_GetCurrentIsAveraged(void* KeyHandle, uint16_t NodeId, int16_t* pCurrentIsAveraged, unsigned int* pErrorCode);
    MotionInfo_DllExport int  VCS_WaitForTargetReached(void* KeyHandle, uint16_t NodeId, unsigned int Timeout, unsigned int* pErrorCode);

//Profile Position Mode
    ProfilePositionMode_DllExport int  VCS_ActivateProfilePositionMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_SetPositionProfile(void* KeyHandle, uint16_t NodeId, unsigned int ProfileVelocity, unsigned int ProfileAcceleration, unsigned int ProfileDeceleration, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_GetPositionProfile(void* KeyHandle, uint16_t NodeId, unsigned int* pProfileVelocity, unsigned int* pProfileAcceleration, unsigned int* pProfileDeceleration, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_MoveToPosition(void* KeyHandle, uint16_t NodeId, long TargetPosition, int Absolute, int Immediately, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_GetTargetPosition(void* KeyHandle, uint16_t NodeId, long* pTargetPosition, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_HaltPositionMovement(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

    //Advanced Functions
    ProfilePositionMode_DllExport int  VCS_EnablePositionWindow(void* KeyHandle, uint16_t NodeId, unsigned int PositionWindow, uint16_t PositionWindowTime, unsigned int* pErrorCode);
    ProfilePositionMode_DllExport int  VCS_DisablePositionWindow(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//Profile Velocity Mode
    ProfileVelocityMode_DllExport int  VCS_ActivateProfileVelocityMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_SetVelocityProfile(void* KeyHandle, uint16_t NodeId, unsigned int ProfileAcceleration, unsigned int ProfileDeceleration, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_GetVelocityProfile(void* KeyHandle, uint16_t NodeId, unsigned int* pProfileAcceleration, unsigned int* pProfileDeceleration, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_MoveWithVelocity(void* KeyHandle, uint16_t NodeId, long TargetVelocity, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_GetTargetVelocity(void* KeyHandle, uint16_t NodeId, long* pTargetVelocity, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_HaltVelocityMovement(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

    //Advanced Functions
    ProfileVelocityMode_DllExport int  VCS_EnableVelocityWindow(void* KeyHandle, uint16_t NodeId, unsigned int VelocityWindow, uint16_t VelocityWindowTime, unsigned int* pErrorCode);
    ProfileVelocityMode_DllExport int  VCS_DisableVelocityWindow(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//Homing Mode
    HomingMode_DllExport int  VCS_ActivateHomingMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_SetHomingParameter(void* KeyHandle, uint16_t NodeId, unsigned int HomingAcceleration, unsigned int SpeedSwitch, unsigned int SpeedIndex, int HomeOffset, uint16_t CurrentTreshold, int HomePosition, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_GetHomingParameter(void* KeyHandle, uint16_t NodeId, unsigned int* pHomingAcceleration, unsigned int* pSpeedSwitch, unsigned int* pSpeedIndex, int* pHomeOffset, uint16_t* pCurrentTreshold, int* pHomePosition, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_FindHome(void* KeyHandle, uint16_t NodeId, signed char HomingMethod, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_StopHoming(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_DefinePosition(void* KeyHandle, uint16_t NodeId, int HomePosition, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_WaitForHomingAttained(void* KeyHandle, uint16_t NodeId, int Timeout, unsigned int* pErrorCode);
    HomingMode_DllExport int  VCS_GetHomingState(void* KeyHandle, uint16_t NodeId, int* pHomingAttained, int* pHomingError, unsigned int* pErrorCode);

//Interpolated Position Mode
    InterpolatedPositionMode_DllExport int  VCS_ActivateInterpolatedPositionMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_SetIpmBufferParameter(void* KeyHandle, uint16_t NodeId, uint16_t UnderflowWarningLimit, uint16_t OverflowWarningLimit, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetIpmBufferParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pUnderflowWarningLimit, uint16_t* pOverflowWarningLimit, unsigned int* pMaxBufferSize, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_ClearIpmBuffer(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetFreeIpmBufferSize(void* KeyHandle, uint16_t NodeId, unsigned int* pBufferSize, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_AddPvtValueToIpmBuffer(void* KeyHandle, uint16_t NodeId, long Position, long Velocity, unsigned char Time, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_StartIpmTrajectory(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_StopIpmTrajectory(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InterpolatedPositionMode_DllExport int  VCS_GetIpmStatus(void* KeyHandle, uint16_t NodeId, int* pTrajectoryRunning, int* pIsUnderflowWarning, int* pIsOverflowWarning, int* pIsVelocityWarning, int* pIsAccelerationWarning, int* pIsUnderflowError, int* pIsOverflowError, int* pIsVelocityError, int* pIsAccelerationError, unsigned int* pErrorCode);

//Position Mode
    PositionMode_DllExport int  VCS_ActivatePositionMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    PositionMode_DllExport int  VCS_SetPositionMust(void* KeyHandle, uint16_t NodeId, long PositionMust, unsigned int* pErrorCode);
    PositionMode_DllExport int  VCS_GetPositionMust(void* KeyHandle, uint16_t NodeId, long* pPositionMust, unsigned int* pErrorCode);

    //Advanced Functions
    PositionMode_DllExport int  VCS_ActivateAnalogPositionSetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, float Scaling, long Offset, unsigned int* pErrorCode);
    PositionMode_DllExport int  VCS_DeactivateAnalogPositionSetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, unsigned int* pErrorCode);
    PositionMode_DllExport int  VCS_EnableAnalogPositionSetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    PositionMode_DllExport int  VCS_DisableAnalogPositionSetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//Velocity Mode
    VelocityMode_DllExport int  VCS_ActivateVelocityMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    VelocityMode_DllExport int  VCS_SetVelocityMust(void* KeyHandle, uint16_t NodeId, long VelocityMust, unsigned int* pErrorCode);
    VelocityMode_DllExport int  VCS_GetVelocityMust(void* KeyHandle, uint16_t NodeId, long* pVelocityMust, unsigned int* pErrorCode);

    //Advanced Functions
    VelocityMode_DllExport int  VCS_ActivateAnalogVelocitySetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, float Scaling, long Offset, unsigned int* pErrorCode);
    VelocityMode_DllExport int  VCS_DeactivateAnalogVelocitySetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, unsigned int* pErrorCode);
    VelocityMode_DllExport int  VCS_EnableAnalogVelocitySetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    VelocityMode_DllExport int  VCS_DisableAnalogVelocitySetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//Current Mode
    CurrentMode_DllExport int  VCS_ActivateCurrentMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    CurrentMode_DllExport int  VCS_SetCurrentMust(void* KeyHandle, uint16_t NodeId, int16_t CurrentMust, unsigned int* pErrorCode);
    CurrentMode_DllExport int  VCS_GetCurrentMust(void* KeyHandle, uint16_t NodeId, int16_t* pCurrentMust, unsigned int* pErrorCode);

    //Advanced Functions
    CurrentMode_DllExport int  VCS_ActivateAnalogCurrentSetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, float Scaling, int16_t Offset, unsigned int* pErrorCode);
    CurrentMode_DllExport int  VCS_DeactivateAnalogCurrentSetpoint(void* KeyHandle, uint16_t NodeId, uint16_t AnalogInputNumber, unsigned int* pErrorCode);
    CurrentMode_DllExport int  VCS_EnableAnalogCurrentSetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    CurrentMode_DllExport int  VCS_DisableAnalogCurrentSetpoint(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

//MasterEncoder Mode
    MasterEncoderMode_DllExport int  VCS_ActivateMasterEncoderMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    MasterEncoderMode_DllExport int  VCS_SetMasterEncoderParameter(void* KeyHandle, uint16_t NodeId, uint16_t ScalingNumerator, uint16_t ScalingDenominator, unsigned char Polarity, unsigned int MaxVelocity, unsigned int MaxAcceleration, unsigned int* pErrorCode);
    MasterEncoderMode_DllExport int  VCS_GetMasterEncoderParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pScalingNumerator, uint16_t* pScalingDenominator, unsigned char* pPolarity, unsigned int* pMaxVelocity, unsigned int* pMaxAcceleration, unsigned int* pErrorCode);

//StepDirection Mode
    StepDirectionMode_DllExport int  VCS_ActivateStepDirectionMode(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    StepDirectionMode_DllExport int  VCS_SetStepDirectionParameter(void* KeyHandle, uint16_t NodeId, uint16_t ScalingNumerator, uint16_t ScalingDenominator, unsigned char Polarity, unsigned int MaxVelocity, unsigned int MaxAcceleration, unsigned int* pErrorCode);
    StepDirectionMode_DllExport int  VCS_GetStepDirectionParameter(void* KeyHandle, uint16_t NodeId, uint16_t* pScalingNumerator, uint16_t* pScalingDenominator, unsigned char* pPolarity, unsigned int* pMaxVelocity, unsigned int* pMaxAcceleration, unsigned int* pErrorCode);

//Inputs Outputs
    //General
    InputsOutputs_DllExport int  VCS_GetAllDigitalInputs(void* KeyHandle, uint16_t NodeId, uint16_t* pInputs, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetAllDigitalOutputs(void* KeyHandle, uint16_t NodeId, uint16_t* pOutputs, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetAllDigitalOutputs(void* KeyHandle, uint16_t NodeId, uint16_t Outputs, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetAnalogInput(void* KeyHandle, uint16_t NodeId, uint16_t InputNumber, uint16_t* pAnalogValue, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetAnalogOutput(void* KeyHandle, uint16_t NodeId, uint16_t OutputNumber, uint16_t AnalogValue, unsigned int* pErrorCode);

    //Position Compare
    InputsOutputs_DllExport int  VCS_SetPositionCompareParameter(void* KeyHandle, uint16_t NodeId, unsigned char OperationalMode, unsigned char IntervalMode, unsigned char DirectionDependency, uint16_t IntervalWidth, uint16_t IntervalRepetitions, uint16_t PulseWidth, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetPositionCompareParameter(void* KeyHandle, uint16_t NodeId, unsigned char* pOperationalMode, unsigned char* pIntervalMode, unsigned char* pDirectionDependency, uint16_t* pIntervalWidth, uint16_t* pIntervalRepetitions, uint16_t* pPulseWidth, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_ActivatePositionCompare(void* KeyHandle, uint16_t NodeId, uint16_t DigitalOutputNumber, int Polarity, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_DeactivatePositionCompare(void* KeyHandle, uint16_t NodeId, uint16_t DigitalOutputNumber, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_EnablePositionCompare(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_DisablePositionCompare(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_SetPositionCompareReferencePosition(void* KeyHandle, uint16_t NodeId, long ReferencePosition, unsigned int* pErrorCode);

    //Position Marker
    InputsOutputs_DllExport int  VCS_SetPositionMarkerParameter(void* KeyHandle, uint16_t NodeId, unsigned char PositionMarkerEdgeType, unsigned char PositionMarkerMode, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_GetPositionMarkerParameter(void* KeyHandle, uint16_t NodeId, unsigned char* pPositionMarkerEdgeType, unsigned char* pPositionMarkerMode, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_ActivatePositionMarker(void* KeyHandle, uint16_t NodeId, uint16_t DigitalInputNumber, int Polarity, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_DeactivatePositionMarker(void* KeyHandle, uint16_t NodeId, uint16_t DigitalInputNumber, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_ReadPositionMarkerCounter(void* KeyHandle, uint16_t NodeId, uint16_t* pCount, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_ReadPositionMarkerCapturedPosition(void* KeyHandle, uint16_t NodeId, uint16_t CounterIndex, long* pCapturedPosition, unsigned int* pErrorCode);
    InputsOutputs_DllExport int  VCS_ResetPositionMarkerCounter(void* KeyHandle, uint16_t NodeId, unsigned int* pErrorCode);

/*************************************************************************************************************************************
* LOW LAYER FUNCTIONS
*************************************************************************************************************************************/

//CanLayer Functions
    CanLayer_DllExport int  VCS_SendCANFrame(void* KeyHandle, uint16_t CobID, uint16_t Length, void* pData, unsigned int* pErrorCode);
    CanLayer_DllExport int  VCS_ReadCANFrame(void* KeyHandle, uint16_t CobID, uint16_t Length, void* pData, unsigned int Timeout, unsigned int* pErrorCode);
    CanLayer_DllExport int  VCS_RequestCANFrame(void* KeyHandle, uint16_t CobID, uint16_t Length, void* pData, unsigned int* pErrorCode);
    CanLayer_DllExport int  VCS_SendNMTService(void* KeyHandle, uint16_t NodeId, uint16_t CommandSpecifier, unsigned int* pErrorCode);

/*************************************************************************************************************************************
* TYPE DEFINITIONS
*************************************************************************************************************************************/
//Communication
	//Dialog Mode
	const int DM_PROGRESS_DLG					= 0;
	const int DM_PROGRESS_AND_CONFIRM_DLG		= 1;
	const int DM_CONFIRM_DLG					= 2;
	const int DM_NO_DLG							= 3;

//Configuration
    //MotorType
    const uint16_t MT_DC_MOTOR                      = 1;
    const uint16_t MT_EC_SINUS_COMMUTATED_MOTOR     = 10;
    const uint16_t MT_EC_BLOCK_COMMUTATED_MOTOR     = 11;

    //SensorType
    const uint16_t ST_UNKNOWN                       = 0;
    const uint16_t ST_INC_ENCODER_3CHANNEL          = 1;
    const uint16_t ST_INC_ENCODER_2CHANNEL          = 2;
    const uint16_t ST_HALL_SENSORS                  = 3;
    const uint16_t ST_SSI_ABS_ENCODER_BINARY        = 4;
    const uint16_t ST_SSI_ABS_ENCODER_GREY          = 5;

//In- and outputs
    //Digital input configuration
    const uint16_t DIC_NEGATIVE_LIMIT_SWITCH        = 0;
    const uint16_t DIC_POSITIVE_LIMIT_SWITCH        = 1;
    const uint16_t DIC_HOME_SWITCH                  = 2;
    const uint16_t DIC_POSITION_MARKER              = 3;
    const uint16_t DIC_DRIVE_ENABLE                 = 4;
    const uint16_t DIC_QUICK_STOP                   = 5;
    const uint16_t DIC_GENERAL_PURPOSE_J            = 6;
    const uint16_t DIC_GENERAL_PURPOSE_I            = 7;
    const uint16_t DIC_GENERAL_PURPOSE_H            = 8;
    const uint16_t DIC_GENERAL_PURPOSE_G            = 9;
    const uint16_t DIC_GENERAL_PURPOSE_F            = 10;
    const uint16_t DIC_GENERAL_PURPOSE_E            = 11;
    const uint16_t DIC_GENERAL_PURPOSE_D            = 12;
    const uint16_t DIC_GENERAL_PURPOSE_C            = 13;
    const uint16_t DIC_GENERAL_PURPOSE_B            = 14;
    const uint16_t DIC_GENERAL_PURPOSE_A            = 15;

    //Digital output configuration
    const uint16_t DOC_READY_FAULT                  = 0;
    const uint16_t DOC_POSITION_COMPARE             = 1;
    const uint16_t DOC_GENERAL_PURPOSE_H            = 8;
    const uint16_t DOC_GENERAL_PURPOSE_G            = 9;
    const uint16_t DOC_GENERAL_PURPOSE_F            = 10;
    const uint16_t DOC_GENERAL_PURPOSE_E            = 11;
    const uint16_t DOC_GENERAL_PURPOSE_D            = 12;
    const uint16_t DOC_GENERAL_PURPOSE_C            = 13;
    const uint16_t DOC_GENERAL_PURPOSE_B            = 14;
    const uint16_t DOC_GENERAL_PURPOSE_A            = 15;

    //Analog input configuration
    const uint16_t AIC_ANALOG_CURRENT_SETPOINT      = 0;
    const uint16_t AIC_ANALOG_VELOCITY_SETPOINT     = 1;
    const uint16_t AIC_ANALOG_POSITION_SETPOINT     = 2;

//Units
    //VelocityDimension
    const unsigned char VD_RPM                               = 0xA4;

    //VelocityNotation
    const signed char VN_STANDARD                          = 0;
    const signed char VN_DECI                              = -1;
    const signed char VN_CENTI                             = -2;
    const signed char VN_MILLI                             = -3;

//Operational mode 
    const signed char OMD_PROFILE_POSITION_MODE          = 1;
    const signed char OMD_PROFILE_VELOCITY_MODE          = 3;
    const signed char OMD_HOMING_MODE                    = 6;
    const signed char OMD_INTERPOLATED_POSITION_MODE     = 7;
    const signed char OMD_POSITION_MODE                  = -1;
    const signed char OMD_VELOCITY_MODE                  = -2;
    const signed char OMD_CURRENT_MODE                   = -3;
    const signed char OMD_MASTER_ENCODER_MODE            = -5;
    const signed char OMD_STEP_DIRECTION_MODE            = -6;

//States
    const uint16_t ST_DISABLED                         = 0;
    const uint16_t ST_ENABLED                          = 1;
    const uint16_t ST_QUICKSTOP                        = 2;
    const uint16_t ST_FAULT                            = 3;

//Homing mode
    //Homing method
    const char HM_ACTUAL_POSITION                               = 35;
    const char HM_NEGATIVE_LIMIT_SWITCH                         = 17;
    const char HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX               = 1;
    const char HM_POSITIVE_LIMIT_SWITCH                         = 18;
    const char HM_POSITIVE_LIMIT_SWITCH_AND_INDEX               = 2;
    const char HM_HOME_SWITCH_POSITIVE_SPEED                    = 23;
    const char HM_HOME_SWITCH_POSITIVE_SPEED_AND_INDEX          = 7;
    const char HM_HOME_SWITCH_NEGATIVE_SPEED                    = 27;
    const char HM_HOME_SWITCH_NEGATIVE_SPEED_AND_INDEX          = 11;
    const char HM_CURRENT_THRESHOLD_POSITIVE_SPEED              = -3;
    const char HM_CURRENT_THRESHOLD_POSITIVE_SPEED_AND_INDEX    = -1;
    const char HM_CURRENT_THRESHOLD_NEGATIVE_SPEED              = -4;
    const char HM_CURRENT_THRESHOLD_NEGATIVE_SPEED_AND_INDEX    = -2;
    const char HM_INDEX_POSITIVE_SPEED                          = 34;
    const char HM_INDEX_NEGATIVE_SPEED                          = 33;

//Input Output PositionMarker
    //PositionMarkerEdgeType
    const unsigned char PET_BOTH_EDGES                   = 0;
    const unsigned char PET_RISING_EDGE                  = 1;
    const unsigned char PET_FALLING_EDGE                 = 2;

    //PositionMarkerMode
    const unsigned char PM_CONTINUOUS                    = 0;
    const unsigned char PM_SINGLE                        = 1;
    const unsigned char PM_MULTIPLE                      = 2;

//Input Output PositionCompare
    //PositionCompareOperationalMode
    const unsigned char PCO_SINGLE_POSITION_MODE         = 0;
    const unsigned char PCO_POSITION_SEQUENCE_MODE       = 1;

    //PositionCompareIntervalMode
    const unsigned char PCI_NEGATIVE_DIR_TO_REFPOS       = 0;
    const unsigned char PCI_POSITIVE_DIR_TO_REFPOS       = 1;
    const unsigned char PCI_BOTH_DIR_TO_REFPOS           = 2;

    //PositionCompareDirectionDependency
    const unsigned char PCD_MOTOR_DIRECTION_NEGATIVE     = 0;
    const unsigned char PCD_MOTOR_DIRECTION_POSITIVE     = 1;
    const unsigned char PCD_MOTOR_DIRECTION_BOTH         = 2;

//Data recorder
    //Trigger type
    const uint16_t DR_MOVEMENT_START_TRIGGER        = 1;    //bit 1
    const uint16_t DR_ERROR_TRIGGER                 = 2;    //bit 2
    const uint16_t DR_DIGITAL_INPUT_TRIGGER         = 4;    //bit 3
    const uint16_t DR_MOVEMENT_END_TRIGGER          = 8;    //bit 4

//CanLayer Functions
    const uint16_t NCS_START_REMOTE_NODE            = 1;
    const uint16_t NCS_STOP_REMOTE_NODE             = 2;
    const uint16_t NCS_ENTER_PRE_OPERATIONAL        = 128;
    const uint16_t NCS_RESET_NODE                   = 129;
    const uint16_t NCS_RESET_COMMUNICATION          = 130;

#endif //_H_LINUX_EPOSCMD_


