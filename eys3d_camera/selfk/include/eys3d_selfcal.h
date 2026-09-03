#ifndef EYS3D_SELFCAL_H
#define EYS3D_SELFCAL_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
  #if defined(EYS3D_SELFCAL_BUILD_SHARED)
    #if defined(EYS3D_SELFCAL_EXPORTS)
      #define EYS3D_SC_API __declspec(dllexport)
    #else
      #define EYS3D_SC_API __declspec(dllimport)
    #endif
  #else
    #define EYS3D_SC_API
  #endif
#else
  #if defined(EYS3D_SELFCAL_BUILD_SHARED)
    #define EYS3D_SC_API __attribute__((visibility("default")))
  #else
    #define EYS3D_SC_API
  #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EYS3D_SC_API_VERSION_MAJOR 1u
#define EYS3D_SC_API_VERSION_MINOR 3u
#define EYS3D_SC_API_VERSION_PATCH 0u
#define EYS3D_SC_MESSAGE_CAPACITY 256u
#define EYS3D_SC_PROFILE_NAME_CAPACITY 64u
#define EYS3D_SC_DEVICE_ID_CAPACITY 128u

typedef struct EYS3D_SC_Context* EYS3D_SC_Handle;

typedef enum EYS3D_SC_Result {
    EYS3D_SC_OK = 0,
    EYS3D_SC_STATUS_NO_UPDATE = 1,
    EYS3D_SC_STATUS_COMPLETED = 2,

    EYS3D_SC_ERROR_INVALID_ARGUMENT = -1,
    EYS3D_SC_ERROR_INVALID_STATE = -2,
    EYS3D_SC_ERROR_CONFIG = -3,
    EYS3D_SC_ERROR_DEVICE_BUSY = -4,
    EYS3D_SC_ERROR_DEVICE_IO = -5,
    EYS3D_SC_ERROR_FRAME_FORMAT = -6,
    EYS3D_SC_ERROR_NOT_READY = -7,
    EYS3D_SC_ERROR_NO_RESULT = -8,
    EYS3D_SC_ERROR_FLASH_DISABLED = -9,
    EYS3D_SC_ERROR_FLASH_WRITE = -10,
    EYS3D_SC_ERROR_INTERNAL = -99
} EYS3D_SC_Result;

typedef enum EYS3D_SC_SessionMode {
    EYS3D_SC_SESSION_ONE_SHOT = 0,
    EYS3D_SC_SESSION_RUNTIME = 1
} EYS3D_SC_SessionMode;

typedef enum EYS3D_SC_State {
    EYS3D_SC_STATE_CREATED = 0,
    EYS3D_SC_STATE_CONFIGURED = 1,
    EYS3D_SC_STATE_RUNNING = 2,
    EYS3D_SC_STATE_COMPLETED = 3,
    EYS3D_SC_STATE_STOPPED = 4,
    EYS3D_SC_STATE_ERROR = 5
} EYS3D_SC_State;

typedef enum EYS3D_SC_Phase {
    EYS3D_SC_PHASE_NONE = 0,
    EYS3D_SC_PHASE_INITIAL_SEARCH = 1,
    EYS3D_SC_PHASE_REFINEMENT = 2,
    EYS3D_SC_PHASE_RUNTIME_TRACKING = 3,
    EYS3D_SC_PHASE_COMPLETED = 4
} EYS3D_SC_Phase;

typedef enum EYS3D_SC_DepthFormat {
    EYS3D_SC_DEPTH_U8 = 1,
    EYS3D_SC_DEPTH_U16 = 2
} EYS3D_SC_DepthFormat;

typedef enum EYS3D_SC_Outcome {
    EYS3D_SC_OUTCOME_NOT_AVAILABLE = 0,
    EYS3D_SC_OUTCOME_SUCCESS = 1,
    EYS3D_SC_OUTCOME_NO_CHANGE = 2,
    EYS3D_SC_OUTCOME_INSUFFICIENT_INPUT = 3,
    EYS3D_SC_OUTCOME_TIMEOUT = 4,
    EYS3D_SC_OUTCOME_FAILED = 5
} EYS3D_SC_Outcome;

typedef enum EYS3D_SC_LogLevel {
    EYS3D_SC_LOG_DEBUG = 0,
    EYS3D_SC_LOG_INFO = 1,
    EYS3D_SC_LOG_WARNING = 2,
    EYS3D_SC_LOG_ERROR = 3
} EYS3D_SC_LogLevel;

typedef void (*EYS3D_SC_LogCallback)(
    EYS3D_SC_LogLevel level,
    const char* message,
    void* user_data);

/*
 * The application already owns these objects through eSPDI.
 * espdi_device_selection must point to DEVSELINFO.
 * espdi_device_information must point to DEVINFORMATIONEX.
 * All three objects must remain valid until EYS3D_SC_Destroy().
 */
typedef struct EYS3D_SC_CreateInfo {
    uint32_t struct_size;
    void* espdi_handle;
    void* espdi_device_selection;
    void* espdi_device_information;
    EYS3D_SC_LogCallback log_callback;
    void* log_user_data;

    /*
     * Added in API 1.2. Optional stable device identifier used by the persistent
     * calibration-history log. Pass the eSPDI serial number when available.
     * The SDK copies this string during EYS3D_SC_Create(); the caller does not
     * need to keep the pointer alive after Create returns.
     */
    const char* device_id;
} EYS3D_SC_CreateInfo;

/*
 * The depth buffer is borrowed only for the duration of EYS3D_SC_ProcessFrame().
 * stride_bytes may be larger than width * bytes_per_pixel; the SDK copies rows
 * into an internal contiguous buffer when necessary.
 * temperature_c may be NaN when temperature is unavailable.
 */
typedef struct EYS3D_SC_Frame {
    uint32_t struct_size;
    const void* depth_data;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    EYS3D_SC_DepthFormat format;
    float temperature_c;
    uint64_t timestamp_us;
} EYS3D_SC_Frame;

typedef struct EYS3D_SC_Status {
    uint32_t struct_size;
    EYS3D_SC_State state;
    EYS3D_SC_SessionMode session_mode;
    float progress;                 /* 0.0 to 1.0; runtime mode normally reports 0.0 */
    float elapsed_time_sec;
    uint64_t processed_frame_count;
    uint32_t optimization_iteration_count;
    uint8_t result_available;
    uint8_t can_commit_to_flash;
    uint8_t reserved_u8[2];
    int32_t last_result;
    char profile_name[EYS3D_SC_PROFILE_NAME_CAPACITY];
    char message[EYS3D_SC_MESSAGE_CAPACITY];

    /* Added in API 1.1. Appended for ABI compatibility. */
    EYS3D_SC_Phase phase;
    uint32_t phase_index;            /* 1-based while active; 0 when unavailable */
    uint32_t phase_count;            /* 1 or 2 for one-shot; 0 for runtime/idle */
    float phase_progress;            /* 0.0 to 1.0 when a phase budget is available */
} EYS3D_SC_Status;

typedef struct EYS3D_SC_CalibrationResult {
    uint32_t struct_size;
    EYS3D_SC_Outcome outcome;
    uint64_t session_id;
    float input_valid_ratio_first;  /* first accepted frame; diagnostic only */
    float input_valid_ratio_latest; /* latest submitted frame; diagnostic only */
    float input_valid_ratio_delta;  /* latest minus first */
    float correction_level;         /* normalized 0.0 to 1.0; implementation-defined */
    uint8_t correction_applied;
    uint8_t committed_to_flash;
    uint8_t reserved_u8[2];
    char summary[EYS3D_SC_MESSAGE_CAPACITY];

    /* Added in API 1.1. Appended for ABI compatibility. */
    uint8_t refinement_enabled;
    uint8_t refinement_completed;
    uint8_t used_coarse_fallback;
    uint8_t reserved_v11_u8;
} EYS3D_SC_CalibrationResult;

/* Returns the runtime API version encoded as major*10000 + minor*100 + patch. */
EYS3D_SC_API uint32_t EYS3D_SC_GetApiVersion(void);

/* Returns a static English name for a result code. Never returns NULL. */
EYS3D_SC_API const char* EYS3D_SC_ResultToString(EYS3D_SC_Result result);

/* Returns a static English name for a public session state. Never returns NULL. */
EYS3D_SC_API const char* EYS3D_SC_StateToString(EYS3D_SC_State state);

/* Returns a static English name for a calibration outcome. Never returns NULL. */
EYS3D_SC_API const char* EYS3D_SC_OutcomeToString(EYS3D_SC_Outcome outcome);

/* Returns a static English name for the current calibration phase. Never returns NULL. */
EYS3D_SC_API const char* EYS3D_SC_PhaseToString(EYS3D_SC_Phase phase);

/* Creates a context bound to an existing eSPDI device. Does not open/close video. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_Create(
    const EYS3D_SC_CreateInfo* create_info,
    EYS3D_SC_Handle* out_handle);

/* Loads and validates a JSON profile. May only be called while not running. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_LoadConfig(
    EYS3D_SC_Handle handle,
    const char* config_path);

/* Loads the same JSON profile from memory. Useful on embedded/read-only systems. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_LoadConfigJson(
    EYS3D_SC_Handle handle,
    const char* json_text,
    size_t json_size);

/* Starts a new session and binds the internal SelfK2 backend to the eSPDI device. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_Start(EYS3D_SC_Handle handle);

/* Synchronously processes one depth frame. No background capture thread is created. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_ProcessFrame(
    EYS3D_SC_Handle handle,
    const EYS3D_SC_Frame* frame);

/* Retrieves a snapshot of the current session status. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_GetStatus(
    EYS3D_SC_Handle handle,
    EYS3D_SC_Status* out_status);

/* Retrieves the latest completed candidate/result. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_GetResult(
    EYS3D_SC_Handle handle,
    EYS3D_SC_CalibrationResult* out_result);

/* Writes the latest valid candidate to flash. Never accepts a raw calibration value. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_CommitToFlash(EYS3D_SC_Handle handle);

/* Restarts optimization from the value present at the beginning of the session. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_Reset(EYS3D_SC_Handle handle);

/* Stops processing and restores the most recent stable value if a probe was active. */
EYS3D_SC_API EYS3D_SC_Result EYS3D_SC_Stop(EYS3D_SC_Handle handle);

/* Returns the last context-local diagnostic string. Pointer remains valid until next API call. */
EYS3D_SC_API const char* EYS3D_SC_GetLastError(EYS3D_SC_Handle handle);

/* Stops if needed and releases the context. It never closes the customer's eSPDI device. */
EYS3D_SC_API void EYS3D_SC_Destroy(EYS3D_SC_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* EYS3D_SELFCAL_H */
