/**
 * @file    can_type_def.h
 * @brief   ZCU / Motor ECU / Sensor ECU CAN Protocol Type Definition
 *
 * ECU 구성:
 *  - ZCU
 *  - Motor ECU
 *  - Sensor ECU
 *
 * CAN Interface:
 *  - ZCU -> Motor ECU  : Gear / Vehicle Control / Vehicle State
 *  - ZCU -> Sensor ECU : Vehicle State / OTA Request
 *  - Sensor ECU -> ZCU : IMU / TOF / Speed / OTA Response
 *
 * 규칙:
 *  - 11-bit Standard CAN ID
 *  - MessageName: CamelCase
 *  - FieldName: lowerCamelCase
 *  - 0x080 ~ 0x202: Classical CAN
 *  - 0x600 ~ 0x601: CAN FD + CAN TP
 */

#ifndef CAN_TYPE_DEF_H
#define CAN_TYPE_DEF_H

#include <stdint.h>

/* ============================================================
   CAN ID 정의
   낮은 ID = 높은 우선순위
   ============================================================ */

/* ZCU -> Motor ECU / Sensor ECU */
#define CAN_ID_VEHICLE_STATE          0x080U  /* VehicleState      | Classical CAN | Event */

/* ZCU -> Motor ECU */
#define CAN_ID_VEHICLE_CONTROL_CMD    0x100U  /* VehicleControlCmd | Classical CAN | 10ms  */

/* Sensor ECU -> ZCU */
#define CAN_ID_IMU_DATA               0x200U  /* ImuData           | Classical CAN | 10ms  */
#define CAN_ID_TOF_DISTANCE_DATA      0x201U  /* TofDistanceData   | Classical CAN | 10ms  */
#define CAN_ID_SPEED_DATA             0x202U  /* SpeedData         | Classical CAN | 50ms  */

/* ZCU <-> Sensor ECU OTA */
#define CAN_ID_OTA_REQUEST            0x600U  /* OtaRequest        | CAN FD + CAN TP | Event / MultiFrame */
#define CAN_ID_OTA_RESPONSE           0x601U  /* OtaResponse       | CAN FD + CAN TP | Response / MultiFrame */


/* ============================================================
   DLC 정의
   ============================================================ */

#define CAN_DLC_VEHICLE_STATE         1U
#define CAN_DLC_VEHICLE_CONTROL_CMD   3U
#define CAN_DLC_IMU_DATA              4U
#define CAN_DLC_TOF_DISTANCE_DATA     2U
#define CAN_DLC_SPEED_DATA            2U

#define CAN_CLASSIC_MAX_DLC           8U
#define CANFD_MAX_DLC                 64U

/* OTA TP Payload 기준 */
#define OTA_FIRMWARE_VERSION_LEN      10U
#define OTA_MAX_BLOCK_DATA_SIZE       256U


/* ============================================================
   주기 정의
   ============================================================ */

#define CAN_PERIOD_VEHICLE_CONTROL_CMD_MS   10U
#define CAN_PERIOD_IMU_DATA_MS              10U
#define CAN_PERIOD_TOF_DISTANCE_DATA_MS     10U
#define CAN_PERIOD_SPEED_DATA_MS            50U


/* ============================================================
   공용 자료형
   uint8 enum 형태는 실제 payload 크기를 1 byte로 고정하기 위해
   typedef uint8_t + enum 상수 방식으로 정의
   ============================================================ */

/* GearState: 0x080 VehicleState */
typedef uint8_t GearState_t;
enum {
    GEAR_STATE_P = 0x00U,
    GEAR_STATE_D = 0x01U
};

/* DriveCmd: 0x100 VehicleControlCmd */
typedef uint8_t DriveCmd_t;
/*
 * 0 ~ 255 : 속도 명령
 * 127     : 정지 기준값
 */
#define DRIVE_CMD_STOP_VALUE          127U

/* SteeringCmd: 0x100 VehicleControlCmd */
typedef uint8_t SteeringCmd_t;
/*
 * 0 ~ 255 : 조향 명령
 * 127     : 중앙 기준값
 */
#define STEERING_CMD_CENTER_VALUE     127U

/* StopCmd: 0x100 VehicleControlCmd */
typedef uint8_t StopCmd_t;
enum {
    STOP_CMD_GO   = 0x00U,
    STOP_CMD_STOP = 0x01U
};

/* OTA Service ID */
typedef uint8_t OtaServiceId_t;
enum {
    OTA_SERVICE_START          = 0x01U,
    OTA_SERVICE_TRANSFER_DATA  = 0x02U,
    OTA_SERVICE_TRANSFER_EXIT  = 0x03U,
    OTA_SERVICE_RESET          = 0x04U
};

/* OTA ApplyRequest */
typedef uint8_t OtaApplyRequest_t;
enum {
    OTA_APPLY_VERIFY_ONLY      = 0x00U,
    OTA_APPLY_VERIFY_AND_APPLY = 0x01U
};

/* OTA ResetType */
typedef uint8_t OtaResetType_t;
enum {
    OTA_RESET_SOFT             = 0x00U,
    OTA_RESET_JUMP_TO_APP      = 0x01U
};

/* OTA ResultCode */
typedef uint8_t OtaResultCode_t;
enum {
    OTA_RESULT_OK               = 0x00U,
    OTA_RESULT_REJECT           = 0x01U,
    OTA_RESULT_BUSY             = 0x02U,
    OTA_RESULT_INVALID_CONDITION= 0x03U,
    OTA_RESULT_SEQUENCE_ERROR   = 0x04U,
    OTA_RESULT_CRC_ERROR        = 0x05U,
    OTA_RESULT_FLASH_ERROR      = 0x06U,
    OTA_RESULT_TIMEOUT          = 0x07U
};

/* OTA Ready */
typedef uint8_t OtaReady_t;
enum {
    OTA_READY_NOT_READY         = 0x00U,
    OTA_READY_READY             = 0x01U
};

/* OTA Start ErrorCode */
typedef uint8_t OtaStartErrorCode_t;
enum {
    OTA_START_ERR_NONE          = 0x00U,
    OTA_START_ERR_INVALID_SIZE  = 0x01U,
    OTA_START_ERR_VERSION_ERROR = 0x02U,
    OTA_START_ERR_MEMORY_ERROR  = 0x03U
};

/* OTA TransferData ErrorCode */
typedef uint8_t OtaTransferErrorCode_t;
enum {
    OTA_TRANSFER_ERR_NONE              = 0x00U,
    OTA_TRANSFER_ERR_SEQUENCE_MISMATCH = 0x01U,
    OTA_TRANSFER_ERR_WRITE_FAIL        = 0x02U,
    OTA_TRANSFER_ERR_BUFFER_FULL       = 0x03U
};

/* OTA VerifyResult */
typedef uint8_t OtaVerifyResult_t;
enum {
    OTA_VERIFY_FAIL             = 0x00U,
    OTA_VERIFY_SUCCESS          = 0x01U
};

/* OTA ResetAccepted */
typedef uint8_t OtaResetAccepted_t;
enum {
    OTA_RESET_NOT_ACCEPTED      = 0x00U,
    OTA_RESET_ACCEPTED          = 0x01U
};


/* ============================================================
   Classical CAN 메시지 구조체
   CAN 수신 버퍼와 memcpy해서 사용
   ============================================================ */

/**
 * 0x080 VehicleState
 * ZCU -> Motor ECU, Sensor ECU
 * Classical CAN
 * DLC = 1
 *
 * Motor ECU:
 *  - GearState == P이면 모터 제어 제한
 *
 * Sensor ECU:
 *  - GearState == P이면 센서 값 읽기 중지
 *
 * B0: GearState
 */
typedef struct __attribute__((packed)) {
    GearState_t gearState;          /* B0 */
} VehicleState_t;


/**
 * 0x100 VehicleControlCmd
 * ZCU -> Motor ECU
 * Classical CAN
 * DLC = 3
 *
 * B0: DriveCmd
 * B1: SteeringCmd
 * B2: StopCmd
 */
typedef struct __attribute__((packed)) {
    DriveCmd_t    driveCmd;         /* B0: 0~255, stop 기준 127 */
    SteeringCmd_t steeringCmd;      /* B1: 0~255, center 기준 127 */
    StopCmd_t     stopCmd;          /* B2: 0x00 Go, 0x01 Stop */
} VehicleControlCmd_t;


/**
 * 0x200 ImuData
 * Sensor ECU -> ZCU
 * Classical CAN
 * DLC = 4
 *
 * B0~B1: YawAngle
 * B2~B3: YawRate
 *
 * YawAngle:
 *  - BNO055 Euler Heading
 *
 * YawRate:
 *  - BNO055 Gyro Z
 *  - Factor 0.0625
 */
typedef struct __attribute__((packed)) {
    uint16_t yawAngle;              /* B0~B1 */
    int16_t  yawRate;               /* B2~B3 */
} ImuData_t;


/**
 * 0x201 TofDistanceData
 * Sensor ECU -> ZCU
 * Classical CAN
 * DLC = 2
 *
 * B0~B1: distanceMm
 */
typedef struct __attribute__((packed)) {
    uint16_t distanceMm;            /* B0~B1 */
} TofDistanceData_t;


/**
 * 0x202 SpeedData
 * Sensor ECU -> ZCU
 * Classical CAN
 * DLC = 2
 *
 * B0~B1: VehicleSpeed
 * Factor 0.01
 */
typedef struct __attribute__((packed)) {
    uint16_t vehicleSpeed;          /* B0~B1, Factor 0.01 */
} SpeedData_t;


/* ============================================================
   OTA Request Payload 구조체
   CAN ID: 0x600
   ZCU -> Sensor ECU
   CAN FD + CAN TP
   ============================================================ */

/**
 * 0x600 OtaRequest - OtaStart
 *
 * P0     : OtaServiceId = 0x01
 * P1~P4 : FirmwareSize
 * P5~P8 : FirmwareCrc32
 * P9~P18: FirmwareVersion[10]
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t otaServiceId;                         /* P0 */
    uint32_t       firmwareSize;                         /* P1~P4 */
    uint32_t       firmwareCrc32;                        /* P5~P8 */
    char           firmwareVersion[OTA_FIRMWARE_VERSION_LEN]; /* P9~P18 */
} OtaStartRequest_t;


/**
 * 0x600 OtaRequest - OtaTransferData
 *
 * P0                  : OtaServiceId = 0x02
 * P1~P2              : BlockSequence
 * P3~P4              : DataLength
 * P5~P(4+dataLength) : FirmwareData
 *
 * FirmwareData 최대 256 byte
 * 실제 CAN FD 한 프레임은 64 byte이므로 CAN TP 계층에서 분할 전송
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t otaServiceId;                          /* P0 */
    uint16_t       blockSequence;                         /* P1~P2 */
    uint16_t       dataLength;                            /* P3~P4 */
    uint8_t        firmwareData[OTA_MAX_BLOCK_DATA_SIZE]; /* P5~ */
} OtaTransferDataRequest_t;


/**
 * 0x600 OtaRequest - OtaTransferExit
 *
 * P0    : OtaServiceId = 0x03
 * P1~P4: TotalCrc32
 * P5    : ApplyRequest
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t    otaServiceId;       /* P0 */
    uint32_t          totalCrc32;         /* P1~P4 */
    OtaApplyRequest_t applyRequest;       /* P5 */
} OtaTransferExitRequest_t;


/**
 * 0x600 OtaRequest - OtaReset
 *
 * P0: OtaServiceId = 0x04
 * P1: ResetType
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t otaServiceId;          /* P0 */
    OtaResetType_t resetType;             /* P1 */
} OtaResetRequest_t;


/* ============================================================
   OTA Response Payload 구조체
   CAN ID: 0x601
   Sensor ECU -> ZCU
   CAN FD + CAN TP
   ============================================================ */

/**
 * 0x601 OtaResponse - OtaStart Response
 *
 * P0    : OtaServiceId = 0x01
 * P1    : ResultCode
 * P2~P3: MaxBlockSize
 * P4    : OtaReady
 * P5    : ErrorCode
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t       otaServiceId;     /* P0 */
    OtaResultCode_t      resultCode;       /* P1 */
    uint16_t             maxBlockSize;     /* P2~P3 */
    OtaReady_t           otaReady;         /* P4 */
    OtaStartErrorCode_t  errorCode;        /* P5 */
} OtaStartResponse_t;


/**
 * 0x601 OtaResponse - OtaTransferData Response
 *
 * P0    : OtaServiceId = 0x02
 * P1    : ResultCode
 * P2~P3: BlockSequence
 * P4    : Progress
 * P5    : ErrorCode
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t          otaServiceId;   /* P0 */
    OtaResultCode_t         resultCode;     /* P1 */
    uint16_t                blockSequence;  /* P2~P3 */
    uint8_t                 progress;       /* P4, 0~100% */
    OtaTransferErrorCode_t  errorCode;      /* P5 */
} OtaTransferDataResponse_t;


/**
 * 0x601 OtaResponse - OtaTransferExit Response
 *
 * P0    : OtaServiceId = 0x03
 * P1    : ResultCode
 * P2    : VerifyResult
 * P3~P6: CalculatedCrc32
 * P7    : Progress
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t      otaServiceId;       /* P0 */
    OtaResultCode_t     resultCode;         /* P1 */
    OtaVerifyResult_t   verifyResult;       /* P2 */
    uint32_t            calculatedCrc32;    /* P3~P6 */
    uint8_t             progress;           /* P7, 0~100% */
} OtaTransferExitResponse_t;


/**
 * 0x601 OtaResponse - OtaReset Response
 *
 * P0: OtaServiceId = 0x04
 * P1: ResultCode
 * P2: ResetAccepted
 */
typedef struct __attribute__((packed)) {
    OtaServiceId_t      otaServiceId;       /* P0 */
    OtaResultCode_t     resultCode;         /* P1 */
    OtaResetAccepted_t  resetAccepted;      /* P2 */
} OtaResetResponse_t;


/* ============================================================
   Classical CAN 프레임 수신용 union
   raw[]에 memcpy 후 fields로 접근
   ============================================================ */

#define CAN_FRAME_UNION(TypeName, StructType, DLC) \
    typedef union __attribute__((packed)) {         \
        uint8_t    raw[(DLC)];                      \
        StructType fields;                          \
    } TypeName

CAN_FRAME_UNION(VehicleState_Frame_t,       VehicleState_t,       CAN_DLC_VEHICLE_STATE);
CAN_FRAME_UNION(VehicleControlCmd_Frame_t,  VehicleControlCmd_t,  CAN_DLC_VEHICLE_CONTROL_CMD);
CAN_FRAME_UNION(ImuData_Frame_t,            ImuData_t,            CAN_DLC_IMU_DATA);
CAN_FRAME_UNION(TofDistanceData_Frame_t,    TofDistanceData_t,    CAN_DLC_TOF_DISTANCE_DATA);
CAN_FRAME_UNION(SpeedData_Frame_t,          SpeedData_t,          CAN_DLC_SPEED_DATA);


/* ============================================================
   OTA Payload 크기 정의
   CAN TP 계층에서 payload length 검증할 때 사용
   ============================================================ */

#define OTA_START_REQUEST_SIZE              ((uint16_t)sizeof(OtaStartRequest_t))
#define OTA_TRANSFER_DATA_HEADER_SIZE       5U
#define OTA_TRANSFER_DATA_REQUEST_MAX_SIZE  ((uint16_t)sizeof(OtaTransferDataRequest_t))
#define OTA_TRANSFER_EXIT_REQUEST_SIZE      ((uint16_t)sizeof(OtaTransferExitRequest_t))
#define OTA_RESET_REQUEST_SIZE              ((uint16_t)sizeof(OtaResetRequest_t))

#define OTA_START_RESPONSE_SIZE             ((uint16_t)sizeof(OtaStartResponse_t))
#define OTA_TRANSFER_DATA_RESPONSE_SIZE     ((uint16_t)sizeof(OtaTransferDataResponse_t))
#define OTA_TRANSFER_EXIT_RESPONSE_SIZE     ((uint16_t)sizeof(OtaTransferExitResponse_t))
#define OTA_RESET_RESPONSE_SIZE             ((uint16_t)sizeof(OtaResetResponse_t))


/* ============================================================
   OTA Payload Union
   서비스 ID 확인 후 해당 fields로 접근
   ============================================================ */

typedef union __attribute__((packed)) {
    uint8_t                     raw[OTA_TRANSFER_DATA_REQUEST_MAX_SIZE];
    OtaStartRequest_t           start;
    OtaTransferDataRequest_t    transferData;
    OtaTransferExitRequest_t    transferExit;
    OtaResetRequest_t           reset;
} OtaRequestPayload_t;

typedef union __attribute__((packed)) {
    uint8_t                       raw[OTA_TRANSFER_EXIT_RESPONSE_SIZE];
    OtaStartResponse_t            start;
    OtaTransferDataResponse_t     transferData;
    OtaTransferExitResponse_t     transferExit;
    OtaResetResponse_t            reset;
} OtaResponsePayload_t;


/* ============================================================
   컴파일 타임 크기 확인
   C11 지원 시에만 동작
   ============================================================ */

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)

_Static_assert(sizeof(VehicleState_t)      == CAN_DLC_VEHICLE_STATE,       "VehicleState_t size mismatch");
_Static_assert(sizeof(VehicleControlCmd_t) == CAN_DLC_VEHICLE_CONTROL_CMD, "VehicleControlCmd_t size mismatch");
_Static_assert(sizeof(ImuData_t)           == CAN_DLC_IMU_DATA,            "ImuData_t size mismatch");
_Static_assert(sizeof(TofDistanceData_t)   == CAN_DLC_TOF_DISTANCE_DATA,   "TofDistanceData_t size mismatch");
_Static_assert(sizeof(SpeedData_t)         == CAN_DLC_SPEED_DATA,          "SpeedData_t size mismatch");

_Static_assert(sizeof(OtaStartRequest_t)          == 19U, "OtaStartRequest_t size mismatch");
_Static_assert(sizeof(OtaTransferExitRequest_t)   == 6U,  "OtaTransferExitRequest_t size mismatch");
_Static_assert(sizeof(OtaResetRequest_t)          == 2U,  "OtaResetRequest_t size mismatch");

_Static_assert(sizeof(OtaStartResponse_t)         == 6U,  "OtaStartResponse_t size mismatch");
_Static_assert(sizeof(OtaTransferDataResponse_t)  == 6U,  "OtaTransferDataResponse_t size mismatch");
_Static_assert(sizeof(OtaTransferExitResponse_t)  == 8U,  "OtaTransferExitResponse_t size mismatch");
_Static_assert(sizeof(OtaResetResponse_t)         == 3U,  "OtaResetResponse_t size mismatch");

#endif


#endif /* CAN_TYPE_DEF_H */
