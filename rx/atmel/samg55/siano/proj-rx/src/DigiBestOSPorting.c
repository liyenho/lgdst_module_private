#include "DigiBestTypedef.h"
#include "delay.h"

extern unsigned int statusflags;

BOOL OSPorting_Frontend_Initialization()
{
// please implement your own OS portig initialization here !

     return TRUE;
}
VOID OSPorting_Frontend_Uninitialization()
{
// please implement your own OS portig uninitialization here !
}
VOID OSPorting_Frontend_Sleep(ULONG Milliseconds)
{
// please implement your own milliseconds delay or sleep functionality here !
delay_ms(Milliseconds);

}
BOOL OSPorting_Frontend_MutualExclusionKeyCreate(PULONG pMutualExclusionKeyID)
{
// please implement your own Mutual Exclusion Key Creation here !

	 return TRUE;
}
VOID OSPorting_Frontend_MutualExclusionKeyDelete(ULONG MutualExclusionKeyID)
{ 
// please implement your own Mutual Exclusion Key deletion here !

}
VOID OSPorting_Frontend_MutualExclusionKeyLock(ULONG MutualExclusionKeyID)
{
// please implement your own Mutual Exclusion Key lock here !

}
VOID OSPorting_Frontend_MutualExclusionKeyRelease(ULONG MutualExclusionKeyID)
{
// please implement your own Mutual Exclusion Key unlock here !

}

VOID OSPorting_Frontend_failflag(unsigned int mask, unsigned int status)
{
// please implement your failflag setup
  statusflags = (statusflags & (~mask)) | status;

}
