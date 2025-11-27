/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\version.h                                                  #
 #  Created Date: Sat, 22nd Feb 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 27th Nov 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#define MAGIS_IDENTIFIER          "MAGIS V2"

#define API_VERSION_MAJOR         0     // increment when major changes are made
#define API_VERSION_MINOR         28    // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH        2

#define FC_FW_VERSION_MAJOR       2    // increment when a major release is made (big new feature, etc)
#define FC_FW_VERSION_MINOR       8    // increment when a minor release is made (small new feature, change etc)
#define FC_FW_VERSION_PATCH_LEVEL 0    // increment when a bug is fixed

#define STR_HELPER( x )           #x
#define STR( x )                  STR_HELPER ( x )
#define FC_FW_VERSION_STRING      STR ( FC_FW_VERSION_MAJOR ) "." STR ( FC_FW_VERSION_MINOR ) "." STR ( FC_FW_VERSION_PATCH_LEVEL )

#define MW_VERSION                231

extern const char *const targetName;

#define GIT_SHORT_REVISION_LENGTH 7    // lower case hexadecimal digits.
extern const char *const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char *const buildDate;    // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char *const buildTime;    // "HH:MM:SS"

extern const char *const FwVersion;
extern const char *const ApiVersion;
extern const char *const FwName;

#ifdef __cplusplus
}
#endif
