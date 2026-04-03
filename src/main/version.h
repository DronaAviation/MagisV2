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
 #  Last Modified: Wed, 21st Jan 2026                                          #
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

#define FW_RELEASE_TYPE           "C"
#define FW_VERSION_LENGTH         5
#define API_VERSION_LENGTH        6
#define PROJECT_LENGTH            8    // lower case hexadecimal digits.
#define BUILD_DATE_LENGTH 11
#define BUILD_TIME_LENGTH 8

#define STR_HELPER( x )           #x
#define STR( x )                  STR_HELPER ( x )

#define MW_VERSION                231

#define GIT_SHORT_REVISION_LENGTH 7    // lower case hexadecimal digits.

extern const char *const targetName;
extern const char *const shortGitRevision;
extern const char *const buildDate;    // "MMM DD YYYY" MMM = Jan/Feb/...
extern const char *const buildTime;    // "HH:MM:SS"
extern const char *const FwVersion;
extern const char *const ApiVersion;
extern const char *const FwName;
extern const char *const Project;

#ifdef __cplusplus
}
#endif
