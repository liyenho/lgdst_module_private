#ifndef __DIGIBESTTYPEDEF_H
#define __DIGIBESTTYPEDEF_H

typedef void                  VOID;
typedef void*                 PVOID;
typedef unsigned int          UINT;
typedef unsigned long         ULONG;
typedef unsigned short        UWORD;
typedef unsigned char         UBYTE;
typedef int                   INT;
typedef long                  LONG;
typedef short                 WORD;
typedef char                  BYTE;
typedef int                   SINT;
typedef long                  SLONG;
typedef short                 SWORD;
typedef char                  SBYTE;
typedef char                  CHAR;
typedef char*                 PSTRING;
typedef unsigned long*        PULONG;
typedef unsigned short*       PUWORD;
typedef char*                 PBYTE;
typedef unsigned char*        PUBYTE;
typedef unsigned char**       PPUBYTE;
typedef unsigned long long    UINT64;
typedef unsigned long long*   PUINT64;
typedef long long             INT64;
typedef long long*            PINT64;
#ifndef DEFINED_BOOL
#define DEFINED_BOOL
typedef int                   BOOL;
#endif
#ifndef NULL
#define NULL ((void*)0)
#endif
#ifndef TRUE
#define TRUE (1 == 1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif

#endif
