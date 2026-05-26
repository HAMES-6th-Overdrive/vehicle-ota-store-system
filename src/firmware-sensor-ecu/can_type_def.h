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
 *  - ZCU -> Motor ECU  : VehicleState / VehicleControlCmd
 *  - ZCU -> Sensor ECU : VehicleState / UDS OTA Request
 *  - Sensor ECU -> ZCU : TOF / Speed / UDS OTA Response
 *
 * 규칙:
 *  - 11-bit Standard CAN ID
 *  - MessageName: CamelCase
 *  - FieldName: lowerCamelCase
 *  - 0x080 ~ 0x202: Classical CAN
 *  - 0x600 ~ 0x601: CAN FD 기반 UDS-style OTA
 *
 * 주의:
 *  - 현재 구현은 CAN FD Raw Frame 기반 UDS-style OTA이다.
 *  - ISO-TP / CAN TP는 추후 확장 가능하다.
 *  - IMU 센서는 현재 프로젝트에서 사용하지 않으므로 0x200은 Reserved로 둔다.
 */

#ifndef CAN_TYPE_DEF_H
#define CAN_TYPE_DEF_H

#include <stdint.h>

/* ============================================================
   Compiler compatibility
   ============================================================ */

/*
 * TASKING 컴파일러는 GCC식 __attribute__((packed))를 경고로 처리할 수 있다.
 * 현재 Classical CAN payload 구조체는 padding 영향이 없는 형태로 구성되어 있고,
 * UDS payload는 실제 코드에서 raw byte buffer 기반으로 직접 파싱한다.
 */
#if defined(__TASKING__)
#define CAN_PACKED
#else
#define CAN_PACKED __attribute__((packed))
#endif


/* ============================================================
   CAN ID 정의
   낮은 ID = 높은 우선순위
   ============================================================ */

/* ZCU -> Motor ECU / Sensor ECU */
#define CAN_ID_VEHICLE_STATE          0x080U  /* VehicleState      | Classical CAN | Event */

/* ZCU -> Motor ECU */
#define CAN_ID_VEHICLE_CONTROL_CMD    0x100U  /* VehicleControlCmd | Classical CAN | 10ms  */

/* Sensor ECU -> ZCU */
/*
 * 0x200 was previously used for ImuData.
 * IMU is no longer used in the current project.
 * Keep 0x200 reserved to avoid CAN ID confusion.
 */
#define CAN_ID_RESERVED_0x200         0x200U  /* Reserved          | Not used */

#define CAN_ID_TOF_DISTANCE_DATA      0x201U  /* TofDistanceData   | Classical CAN | 10ms  */
#define CAN_ID_SPEED_DATA             0x202U  /* SpeedData         | Classical CAN | 100ms */

/*
 * ZCU <-> Sensor ECU OTA / UDS
 *
 * CAN_ID_OTA_REQUEST / CAN_ID_OTA_RESPONSE 이름은 기존 MCMCAN.c와의 호환을 위해 유지.
 * 새 UDS 코드에서는 CAN_ID_UDS_REQUEST / CAN_ID_UDS_RESPONSE 이름을 사용해도 됨.
 */
#define CAN_ID_OTA_REQUEST            0x600U  /* UDS Request  | ZCU -> Sensor ECU | CAN FD */
#define CAN_ID_OTA_RESPONSE           0x601U  /* UDS Response | Sensor ECU -> ZCU | CAN FD */

#define CAN_ID_UDS_REQUEST            CAN_ID_OTA_REQUEST
#define CAN_ID_UDS_RESPONSE           CAN_ID_OTA_RESPONSE


/* ============================================================
   DLC 정의
   ============================================================ */

#define CAN_DLC_VEHICLE_STATE         1U
#define CAN_DLC_VEHICLE_CONTROL_CMD   3U
#define CAN_DLC_TOF_DISTANCE_DATA     2U
#define CAN_DLC_SPEED_DATA            2U

#define CAN_CLASSIC_MAX_DLC           8U
#define CANFD_MAX_DLC                 64U


/* ============================================================
   주기 정의
   ============================================================ */

#define CAN_PERIOD_VEHICLE_CONTROL_CMD_MS   10U
#define CAN_PERIOD_TOF_DISTANCE_DATA_MS     10U
#define CAN_PERIOD_SPEED_DATA_MS            100U


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


/* ============================================================
   UDS on CAN FD - Sensor ECU OTA 정의
   ============================================================ */

/*
 * 현재 목표:
 *  - ZCU -> Sensor ECU : CAN FD 0x600
 *  - Sensor ECU -> ZCU : CAN FD 0x601
 *
 * UDS-style OTA 흐름:
 *  1. DiagnosticSessionControl 0x10
 *  2. RequestDownload          0x34
 *  3. TransferData             0x36
 *  4. RequestTransferExit      0x37
5. RequestTransferExit에서 CRC 확인 및 OTA flag 저장
6. Reset 후 Bootloader가 SOTA/UCB_SWAP 수행
 */

/* UDS Service ID */
typedef uint8_t UdsServiceId_t;
enum {
    UDS_SID_DIAGNOSTIC_SESSION_CONTROL = 0x10U,
    UDS_SID_ECU_RESET                  = 0x11U,
    UDS_SID_ROUTINE_CONTROL            = 0x31U,
    UDS_SID_REQUEST_DOWNLOAD           = 0x34U,
    UDS_SID_TRANSFER_DATA              = 0x36U,
    UDS_SID_REQUEST_TRANSFER_EXIT      = 0x37U,
    UDS_SID_NEGATIVE_RESPONSE          = 0x7FU
};

/* UDS Positive Response = Request SID + 0x40 */
#define UDS_POSITIVE_RESPONSE_OFFSET   0x40U

/* UDS Positive Response SID */
#define UDS_RSID_DIAGNOSTIC_SESSION_CONTROL  (UDS_SID_DIAGNOSTIC_SESSION_CONTROL + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x50 */
#define UDS_RSID_ECU_RESET                   (UDS_SID_ECU_RESET                  + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x51 */
#define UDS_RSID_ROUTINE_CONTROL             (UDS_SID_ROUTINE_CONTROL            + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x71 */
#define UDS_RSID_REQUEST_DOWNLOAD            (UDS_SID_REQUEST_DOWNLOAD           + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x74 */
#define UDS_RSID_TRANSFER_DATA               (UDS_SID_TRANSFER_DATA              + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x76 */
#define UDS_RSID_REQUEST_TRANSFER_EXIT       (UDS_SID_REQUEST_TRANSFER_EXIT      + UDS_POSITIVE_RESPONSE_OFFSET) /* 0x77 */

/* Diagnostic Session Type */
typedef uint8_t UdsSessionType_t;
enum {
    UDS_SESSION_DEFAULT                = 0x01U,
    UDS_SESSION_PROGRAMMING            = 0x02U,
    UDS_SESSION_EXTENDED               = 0x03U
};

/* ECU Reset Type */
typedef uint8_t UdsResetType_t;
enum {
    UDS_RESET_HARD_RESET               = 0x01U,
    UDS_RESET_KEY_OFF_ON_RESET         = 0x02U,
    UDS_RESET_SOFT_RESET               = 0x03U,

    /*
     * 프로젝트 데모용:
     * 현재 구현에서는 0x01 요청 시 HardReset 대신 Application Jump로 처리 가능.
     */
    UDS_RESET_JUMP_TO_APP              = 0x01U
};

/* RoutineControl Type */
typedef uint8_t UdsRoutineControlType_t;
enum {
    UDS_ROUTINE_START                  = 0x01U,
    UDS_ROUTINE_STOP                   = 0x02U,
    UDS_ROUTINE_REQUEST_RESULT         = 0x03U
};

/* Project-specific Routine ID */
#define UDS_ROUTINE_ID_CHECK_CRC32     0x0202U

/* Negative Response Code */
typedef uint8_t UdsNrc_t;
enum {
    UDS_NRC_GENERAL_REJECT                      = 0x10U,
    UDS_NRC_SERVICE_NOT_SUPPORTED               = 0x11U,
    UDS_NRC_SUBFUNCTION_NOT_SUPPORTED           = 0x12U,
    UDS_NRC_INCORRECT_MESSAGE_LENGTH_OR_FORMAT  = 0x13U,
    UDS_NRC_CONDITIONS_NOT_CORRECT              = 0x22U,
    UDS_NRC_REQUEST_OUT_OF_RANGE                = 0x31U,
    UDS_NRC_SECURITY_ACCESS_DENIED              = 0x33U,
    UDS_NRC_GENERAL_PROGRAMMING_FAILURE         = 0x72U,
    UDS_NRC_WRONG_BLOCK_SEQUENCE_COUNTER        = 0x73U,
    UDS_NRC_REQUEST_CORRECTLY_RECEIVED_PENDING  = 0x78U
};

/* UDS OTA 설정 */
#define UDS_CANFD_MAX_PAYLOAD_SIZE     64U
#define UDS_TRANSFER_DATA_SIZE         32U
#define UDS_MAX_BLOCK_LENGTH           32U

/*
 * RequestDownload simplified format
 *
 * B0      : 0x34
 * B1      : DataFormatIdentifier = 0x00
 * B2      : AddressAndLengthFormatIdentifier = 0x44
 * B3~B6   : MemoryAddress, little endian
 * B7~B10  : MemorySize, little endian
 */
#define UDS_DOWNLOAD_DATA_FORMAT_ID    0x00U
#define UDS_DOWNLOAD_ADDR_LEN_FORMAT   0x44U

/*
 * OTA Download Target
 *
 * 기존 Single Slot OTA:
 *   0x80040000 = Active App 영역
 *
 * Dual Slot / Bank B OTA:
 *   0x80300000 = Inactive Bank B 영역
 *
 * 주행 중에는 이 영역에 새 firmware를 저장하고 CRC 검증까지만 수행한다.
 */
#define UDS_APP_START_ADDR 0x80320000U

/* UDS Request 최소 길이 */
#define UDS_REQ_LEN_DIAGNOSTIC_SESSION_CONTROL   2U
#define UDS_REQ_LEN_ECU_RESET                    2U
#define UDS_REQ_LEN_REQUEST_DOWNLOAD             11U
#define UDS_REQ_LEN_TRANSFER_DATA_MIN            3U
#define UDS_REQ_LEN_REQUEST_TRANSFER_EXIT        1U
#define UDS_REQ_LEN_ROUTINE_CONTROL_CHECK_CRC32  8U

/* UDS Response 기본 길이 */
#define UDS_RESP_LEN_NEGATIVE_RESPONSE           3U
#define UDS_RESP_LEN_DIAGNOSTIC_SESSION_CONTROL  2U
#define UDS_RESP_LEN_ECU_RESET                   2U
#define UDS_RESP_LEN_REQUEST_DOWNLOAD            4U
#define UDS_RESP_LEN_TRANSFER_DATA               2U
#define UDS_RESP_LEN_REQUEST_TRANSFER_EXIT       1U
#define UDS_RESP_LEN_ROUTINE_CONTROL_CHECK_CRC32 8U


/* ============================================================
   UDS-style Request/Response 구조체
   실제 CAN FD payload는 64 byte raw buffer로 처리하는 것을 권장.
   아래 구조체는 payload format 문서화 및 memcpy 보조용.
   ============================================================ */

/**
 * 0x600 UDS Request - DiagnosticSessionControl
 *
 * P0: SID = 0x10
 * P1: SessionType
 */
typedef struct CAN_PACKED {
    UdsServiceId_t    sid;
    UdsSessionType_t  sessionType;
} UdsDiagnosticSessionControlRequest_t;

/**
 * 0x601 UDS Response - DiagnosticSessionControl
 *
 * P0: RSID = 0x50
 * P1: SessionType
 */
typedef struct CAN_PACKED {
    UdsServiceId_t    responseSid;
    UdsSessionType_t  sessionType;
} UdsDiagnosticSessionControlResponse_t;

/**
 * 0x600 UDS Request - RequestDownload
 *
 * P0      : SID = 0x34
 * P1      : DataFormatIdentifier
 * P2      : AddressAndLengthFormatIdentifier
 * P3~P6   : MemoryAddress
 * P7~P10  : MemorySize
 */
typedef struct CAN_PACKED {
    UdsServiceId_t sid;
    uint8_t        dataFormatIdentifier;
    uint8_t        addressAndLengthFormatIdentifier;
    uint32_t       memoryAddress;
    uint32_t       memorySize;
} UdsRequestDownloadRequest_t;

/**
 * 0x601 UDS Response - RequestDownload
 *
 * P0      : RSID = 0x74
 * P1      : LengthFormatIdentifier style value
 * P2~P3   : MaxNumberOfBlockLength = 32
 */
typedef struct CAN_PACKED {
    UdsServiceId_t responseSid;
    uint8_t        lengthFormatIdentifier;
    uint16_t       maxNumberOfBlockLength;
} UdsRequestDownloadResponse_t;

/**
 * 0x600 UDS Request - TransferData
 *
 * P0    : SID = 0x36
 * P1    : BlockSequenceCounter
 * P2~   : TransferRequestParameterRecord = firmware data
 */
typedef struct CAN_PACKED {
    UdsServiceId_t sid;
    uint8_t        blockSequenceCounter;
    uint8_t        data[UDS_TRANSFER_DATA_SIZE];
} UdsTransferDataRequest_t;

/**
 * 0x601 UDS Response - TransferData
 *
 * P0: RSID = 0x76
 * P1: BlockSequenceCounter
 */
typedef struct CAN_PACKED {
    UdsServiceId_t responseSid;
    uint8_t        blockSequenceCounter;
} UdsTransferDataResponse_t;

/**
 * 0x600 UDS Request - RequestTransferExit
 *
 * P0: SID = 0x37
 */
typedef struct CAN_PACKED {
    UdsServiceId_t sid;
} UdsRequestTransferExitRequest_t;

/**
 * 0x601 UDS Response - RequestTransferExit
 *
 * P0: RSID = 0x77
 */
typedef struct CAN_PACKED {
    UdsServiceId_t responseSid;
} UdsRequestTransferExitResponse_t;

/**
 * 0x600 UDS Request - RoutineControl CRC32
 *
 * P0      : SID = 0x31
 * P1      : RoutineControlType = 0x01
 * P2~P3   : RoutineIdentifier = 0x0202
 * P4~P7   : ExpectedCrc32
 */
typedef struct CAN_PACKED {
    UdsServiceId_t             sid;
    UdsRoutineControlType_t    routineControlType;
    uint16_t                   routineIdentifier;
    uint32_t                   expectedCrc32;
} UdsRoutineControlCheckCrc32Request_t;

/**
 * 0x601 UDS Response - RoutineControl CRC32
 *
 * P0      : RSID = 0x71
 * P1      : RoutineControlType = 0x01
 * P2~P3   : RoutineIdentifier = 0x0202
 * P4~P7   : CalculatedCrc32
 */
typedef struct CAN_PACKED {
    UdsServiceId_t             responseSid;
    UdsRoutineControlType_t    routineControlType;
    uint16_t                   routineIdentifier;
    uint32_t                   calculatedCrc32;
} UdsRoutineControlCheckCrc32Response_t;

/**
 * 0x600 UDS Request - ECUReset
 *
 * P0: SID = 0x11
 * P1: ResetType
 */
typedef struct CAN_PACKED {
    UdsServiceId_t sid;
    UdsResetType_t resetType;
} UdsEcuResetRequest_t;

/**
 * 0x601 UDS Response - ECUReset
 *
 * P0: RSID = 0x51
 * P1: ResetType
 */
typedef struct CAN_PACKED {
    UdsServiceId_t responseSid;
    UdsResetType_t resetType;
} UdsEcuResetResponse_t;

/**
 * 0x601 UDS Negative Response
 *
 * P0: 0x7F
 * P1: Request SID
 * P2: NRC
 */
typedef struct CAN_PACKED {
    UdsServiceId_t negativeResponseSid;
    UdsServiceId_t requestSid;
    UdsNrc_t       nrc;
} UdsNegativeResponse_t;


/* ============================================================
   LEGACY Custom OTA 정의
   기존 코드 호환용.
   새 UDS OTA 코드에서는 아래 OTA_SERVICE_* 대신 UDS_SID_* 사용 권장.
   ============================================================ */

#define OTA_FIRMWARE_VERSION_LEN      10U
#define OTA_MAX_BLOCK_DATA_SIZE       256U

/* OTA Service ID - Legacy */
typedef uint8_t OtaServiceId_t;
enum {
    OTA_SERVICE_START          = 0x01U,
    OTA_SERVICE_TRANSFER_DATA  = 0x02U,
    OTA_SERVICE_TRANSFER_EXIT  = 0x03U,
    OTA_SERVICE_RESET          = 0x04U
};

/* OTA ApplyRequest - Legacy */
typedef uint8_t OtaApplyRequest_t;
enum {
    OTA_APPLY_VERIFY_ONLY      = 0x00U,
    OTA_APPLY_VERIFY_AND_APPLY = 0x01U
};

/* OTA ResetType - Legacy */
typedef uint8_t OtaResetType_t;
enum {
    OTA_RESET_SOFT             = 0x00U,
    OTA_RESET_JUMP_TO_APP      = 0x01U
};

/* OTA ResultCode - Legacy */
typedef uint8_t OtaResultCode_t;
enum {
    OTA_RESULT_OK                = 0x00U,
    OTA_RESULT_REJECT            = 0x01U,
    OTA_RESULT_BUSY              = 0x02U,
    OTA_RESULT_INVALID_CONDITION = 0x03U,
    OTA_RESULT_SEQUENCE_ERROR    = 0x04U,
    OTA_RESULT_CRC_ERROR         = 0x05U,
    OTA_RESULT_FLASH_ERROR       = 0x06U,
    OTA_RESULT_TIMEOUT           = 0x07U
};

/* OTA Ready - Legacy */
typedef uint8_t OtaReady_t;
enum {
    OTA_READY_NOT_READY         = 0x00U,
    OTA_READY_READY             = 0x01U
};

/* OTA Start ErrorCode - Legacy */
typedef uint8_t OtaStartErrorCode_t;
enum {
    OTA_START_ERR_NONE          = 0x00U,
    OTA_START_ERR_INVALID_SIZE  = 0x01U,
    OTA_START_ERR_VERSION_ERROR = 0x02U,
    OTA_START_ERR_MEMORY_ERROR  = 0x03U
};

/* OTA TransferData ErrorCode - Legacy */
typedef uint8_t OtaTransferErrorCode_t;
enum {
    OTA_TRANSFER_ERR_NONE              = 0x00U,
    OTA_TRANSFER_ERR_SEQUENCE_MISMATCH = 0x01U,
    OTA_TRANSFER_ERR_WRITE_FAIL        = 0x02U,
    OTA_TRANSFER_ERR_BUFFER_FULL       = 0x03U
};

/* OTA VerifyResult - Legacy */
typedef uint8_t OtaVerifyResult_t;
enum {
    OTA_VERIFY_FAIL             = 0x00U,
    OTA_VERIFY_SUCCESS          = 0x01U
};

/* OTA ResetAccepted - Legacy */
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
 *  - GearState == P이면 OTA 허용 조건 판단에 사용
 *
 * B0: GearState
 */
typedef struct CAN_PACKED {
    GearState_t gearState;
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
typedef struct CAN_PACKED {
    DriveCmd_t    driveCmd;
    SteeringCmd_t steeringCmd;
    StopCmd_t     stopCmd;
} VehicleControlCmd_t;


/**
 * 0x201 TofDistanceData
 * Sensor ECU -> ZCU
 * Classical CAN
 * DLC = 2
 *
 * B0~B1: distanceMm
 */
typedef struct CAN_PACKED {
    uint16_t distanceMm;
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
typedef struct CAN_PACKED {
    uint16_t vehicleSpeed;
} SpeedData_t;


/* ============================================================
   LEGACY OTA Request Payload 구조체
   CAN ID: 0x600
   ZCU -> Sensor ECU
   기존 커스텀 OTA 호환용
   ============================================================ */

typedef struct CAN_PACKED {
    OtaServiceId_t otaServiceId;
    uint32_t       firmwareSize;
    uint32_t       firmwareCrc32;
    char           firmwareVersion[OTA_FIRMWARE_VERSION_LEN];
} OtaStartRequest_t;

typedef struct CAN_PACKED {
    OtaServiceId_t otaServiceId;
    uint16_t       blockSequence;
    uint16_t       dataLength;
    uint8_t        firmwareData[OTA_MAX_BLOCK_DATA_SIZE];
} OtaTransferDataRequest_t;

typedef struct CAN_PACKED {
    OtaServiceId_t    otaServiceId;
    uint32_t          totalCrc32;
    OtaApplyRequest_t applyRequest;
} OtaTransferExitRequest_t;

typedef struct CAN_PACKED {
    OtaServiceId_t otaServiceId;
    OtaResetType_t resetType;
} OtaResetRequest_t;


/* ============================================================
   LEGACY OTA Response Payload 구조체
   CAN ID: 0x601
   Sensor ECU -> ZCU
   기존 커스텀 OTA 호환용
   ============================================================ */

typedef struct CAN_PACKED {
    OtaServiceId_t       otaServiceId;
    OtaResultCode_t      resultCode;
    uint16_t             maxBlockSize;
    OtaReady_t           otaReady;
    OtaStartErrorCode_t  errorCode;
} OtaStartResponse_t;

typedef struct CAN_PACKED {
    OtaServiceId_t          otaServiceId;
    OtaResultCode_t         resultCode;
    uint16_t                blockSequence;
    uint8_t                 progress;
    OtaTransferErrorCode_t  errorCode;
} OtaTransferDataResponse_t;

typedef struct CAN_PACKED {
    OtaServiceId_t      otaServiceId;
    OtaResultCode_t     resultCode;
    OtaVerifyResult_t   verifyResult;
    uint32_t            calculatedCrc32;
    uint8_t             progress;
} OtaTransferExitResponse_t;

typedef struct CAN_PACKED {
    OtaServiceId_t      otaServiceId;
    OtaResultCode_t     resultCode;
    OtaResetAccepted_t  resetAccepted;
} OtaResetResponse_t;


/* ============================================================
   Classical CAN 프레임 수신용 union
   raw[]에 memcpy 후 fields로 접근
   ============================================================ */

#define CAN_FRAME_UNION(TypeName, StructType, DLC) \
    typedef union CAN_PACKED {                      \
        uint8_t    raw[(DLC)];                      \
        StructType fields;                          \
    } TypeName

CAN_FRAME_UNION(VehicleState_Frame_t,       VehicleState_t,       CAN_DLC_VEHICLE_STATE);
CAN_FRAME_UNION(VehicleControlCmd_Frame_t,  VehicleControlCmd_t,  CAN_DLC_VEHICLE_CONTROL_CMD);
CAN_FRAME_UNION(TofDistanceData_Frame_t,    TofDistanceData_t,    CAN_DLC_TOF_DISTANCE_DATA);
CAN_FRAME_UNION(SpeedData_Frame_t,          SpeedData_t,          CAN_DLC_SPEED_DATA);


/* ============================================================
   UDS / OTA Payload 크기 정의
   ============================================================ */

/* UDS-style sizes */
#define UDS_DIAGNOSTIC_SESSION_CONTROL_REQUEST_SIZE   ((uint16_t)sizeof(UdsDiagnosticSessionControlRequest_t))
#define UDS_DIAGNOSTIC_SESSION_CONTROL_RESPONSE_SIZE  ((uint16_t)sizeof(UdsDiagnosticSessionControlResponse_t))

#define UDS_REQUEST_DOWNLOAD_REQUEST_SIZE             ((uint16_t)sizeof(UdsRequestDownloadRequest_t))
#define UDS_REQUEST_DOWNLOAD_RESPONSE_SIZE            ((uint16_t)sizeof(UdsRequestDownloadResponse_t))

#define UDS_TRANSFER_DATA_REQUEST_HEADER_SIZE         2U
#define UDS_TRANSFER_DATA_RESPONSE_SIZE               ((uint16_t)sizeof(UdsTransferDataResponse_t))

#define UDS_REQUEST_TRANSFER_EXIT_REQUEST_SIZE        ((uint16_t)sizeof(UdsRequestTransferExitRequest_t))
#define UDS_REQUEST_TRANSFER_EXIT_RESPONSE_SIZE       ((uint16_t)sizeof(UdsRequestTransferExitResponse_t))

#define UDS_ROUTINE_CONTROL_CHECK_CRC32_REQUEST_SIZE  ((uint16_t)sizeof(UdsRoutineControlCheckCrc32Request_t))
#define UDS_ROUTINE_CONTROL_CHECK_CRC32_RESPONSE_SIZE ((uint16_t)sizeof(UdsRoutineControlCheckCrc32Response_t))

#define UDS_ECU_RESET_REQUEST_SIZE                    ((uint16_t)sizeof(UdsEcuResetRequest_t))
#define UDS_ECU_RESET_RESPONSE_SIZE                   ((uint16_t)sizeof(UdsEcuResetResponse_t))

#define UDS_NEGATIVE_RESPONSE_SIZE                    ((uint16_t)sizeof(UdsNegativeResponse_t))


/* Legacy custom OTA sizes */
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
   UDS Payload Union
   CAN FD 64 byte raw buffer 기반
   ============================================================ */

typedef union CAN_PACKED {
    uint8_t                                      raw[CANFD_MAX_DLC];

    UdsDiagnosticSessionControlRequest_t         diagnosticSessionControl;
    UdsRequestDownloadRequest_t                  requestDownload;
    UdsTransferDataRequest_t                     transferData;
    UdsRequestTransferExitRequest_t              requestTransferExit;
    UdsRoutineControlCheckCrc32Request_t         routineControlCheckCrc32;
    UdsEcuResetRequest_t                         ecuReset;
} UdsRequestPayload_t;

typedef union CAN_PACKED {
    uint8_t                                      raw[CANFD_MAX_DLC];

    UdsDiagnosticSessionControlResponse_t        diagnosticSessionControl;
    UdsRequestDownloadResponse_t                 requestDownload;
    UdsTransferDataResponse_t                    transferData;
    UdsRequestTransferExitResponse_t             requestTransferExit;
    UdsRoutineControlCheckCrc32Response_t        routineControlCheckCrc32;
    UdsEcuResetResponse_t                        ecuReset;
    UdsNegativeResponse_t                        negativeResponse;
} UdsResponsePayload_t;


/* ============================================================
   LEGACY OTA Payload Union
   기존 코드 호환용
   ============================================================ */

typedef union CAN_PACKED {
    uint8_t                     raw[OTA_TRANSFER_DATA_REQUEST_MAX_SIZE];
    OtaStartRequest_t           start;
    OtaTransferDataRequest_t    transferData;
    OtaTransferExitRequest_t    transferExit;
    OtaResetRequest_t           reset;
} OtaRequestPayload_t;

typedef union CAN_PACKED {
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
_Static_assert(sizeof(TofDistanceData_t)   == CAN_DLC_TOF_DISTANCE_DATA,   "TofDistanceData_t size mismatch");
_Static_assert(sizeof(SpeedData_t)         == CAN_DLC_SPEED_DATA,          "SpeedData_t size mismatch");

_Static_assert(sizeof(UdsDiagnosticSessionControlRequest_t)  == 2U,  "UDS DSC request size mismatch");
_Static_assert(sizeof(UdsDiagnosticSessionControlResponse_t) == 2U,  "UDS DSC response size mismatch");
_Static_assert(sizeof(UdsRequestDownloadRequest_t)           == 11U, "UDS RequestDownload request size mismatch");
_Static_assert(sizeof(UdsRequestDownloadResponse_t)          == 4U,  "UDS RequestDownload response size mismatch");
_Static_assert(sizeof(UdsTransferDataResponse_t)             == 2U,  "UDS TransferData response size mismatch");
_Static_assert(sizeof(UdsRequestTransferExitRequest_t)       == 1U,  "UDS TransferExit request size mismatch");
_Static_assert(sizeof(UdsRequestTransferExitResponse_t)      == 1U,  "UDS TransferExit response size mismatch");
_Static_assert(sizeof(UdsRoutineControlCheckCrc32Request_t)  == 8U,  "UDS RoutineControl request size mismatch");
_Static_assert(sizeof(UdsRoutineControlCheckCrc32Response_t) == 8U,  "UDS RoutineControl response size mismatch");
_Static_assert(sizeof(UdsEcuResetRequest_t)                  == 2U,  "UDS ECUReset request size mismatch");
_Static_assert(sizeof(UdsEcuResetResponse_t)                 == 2U,  "UDS ECUReset response size mismatch");
_Static_assert(sizeof(UdsNegativeResponse_t)                 == 3U,  "UDS NegativeResponse size mismatch");

_Static_assert(sizeof(OtaStartRequest_t)          == 19U, "OtaStartRequest_t size mismatch");
_Static_assert(sizeof(OtaTransferExitRequest_t)   == 6U,  "OtaTransferExitRequest_t size mismatch");
_Static_assert(sizeof(OtaResetRequest_t)          == 2U,  "OtaResetRequest_t size mismatch");

_Static_assert(sizeof(OtaStartResponse_t)         == 6U,  "OtaStartResponse_t size mismatch");
_Static_assert(sizeof(OtaTransferDataResponse_t)  == 6U,  "OtaTransferDataResponse_t size mismatch");
_Static_assert(sizeof(OtaTransferExitResponse_t)  == 8U,  "OtaTransferExitResponse_t size mismatch");
_Static_assert(sizeof(OtaResetResponse_t)         == 3U,  "OtaResetResponse_t size mismatch");

#endif


#endif /* CAN_TYPE_DEF_H */
