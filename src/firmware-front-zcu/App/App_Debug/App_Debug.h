#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAXCHARS 256

void AppDebug_Init(void);
void AppDebug_Print(const char *format, ...);
void AppDebug_Print_C1(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEBUG_H */