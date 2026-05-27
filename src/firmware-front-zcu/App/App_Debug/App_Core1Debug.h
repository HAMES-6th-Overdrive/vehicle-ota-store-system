#ifndef APP_CORE1DEBUG_H
#define APP_CORE1DEBUG_H

#include "Ifx_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CORE1DEBUG_MSG_MAX_LEN     (96u)
#define APP_CORE1DEBUG_QUEUE_DEPTH     (16u)

/*
 * Core0에서 1회 초기화.
 * 반드시 AppDebug_Init() 이후 또는 직전에 호출 가능.
 */
void AppCore1Debug_Init(void);

/*
 * Core1에서 호출하는 안전 로그 함수.
 * FreeRTOS API 사용 안 함.
 * UART 직접 접근 안 함.
 */
void AppCore1Debug_Push(const char *msg);

/*
 * 숫자 하나를 같이 찍고 싶을 때 사용.
 * Core1에서 snprintf/printf 계열을 쓰지 않기 위한 간단 함수.
 */
void AppCore1Debug_PushU32(const char *prefix, uint32 value);

/*
 * Core0 FreeRTOS task 생성.
 */
void AppCore1Debug_StartTask(void);

/*
 * Core0 task에서 주기적으로 호출해도 됨.
 * xTaskCreate를 쓰기 싫으면 기존 task 안에서 이 함수만 호출 가능.
 */
void AppCore1Debug_DrainOnce(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CORE1DEBUG_H */