#ifndef __DIGIBESTOSPORTING_H
#define __DIGIBESTOSPORTING_H

#ifdef __cplusplus
extern "C" {
#endif

extern BOOL OSPorting_Frontend_Initialization();
extern VOID OSPorting_Frontend_Uninitialization();
extern VOID OSPorting_Frontend_Sleep(ULONG Milliseconds); 
extern BOOL OSPorting_Frontend_MutualExclusionKeyCreate(PULONG pMutualExclusionKeyID);
extern VOID OSPorting_Frontend_MutualExclusionKeyDelete(ULONG MutualExclusionKeyID);
extern VOID OSPorting_Frontend_MutualExclusionKeyLock(ULONG MutualExclusionKeyID);
extern VOID OSPorting_Frontend_MutualExclusionKeyRelease(ULONG MutualExclusionKeyID);
extern VOID OSPorting_Frontend_failflag(unsigned int mask, unsigned int status);

#ifdef __cplusplus
}
#endif

#endif





