/*

 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define MAGIS_IDENTIFIER       "MAGIS V2"

#define API_VERSION_MAJOR      0    // increment when major changes are made
#define API_VERSION_MINOR      9    // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH     2

#define FC_FW_VERSION_MAJOR       2    // increment when a major release is made (big new feature, etc)
#define FC_FW_VERSION_MINOR       0    // increment when a minor release is made (small new feature, change etc)
#define FC_FW_VERSION_PATCH_LEVEL 0    // increment when a bug is fixed

#define STR_HELPER( x )        #x
#define STR( x )               STR_HELPER ( x )
#define FC_FW_VERSION_STRING      STR ( FC_FW_VERSION_MAJOR ) "." STR ( FC_FW_VERSION_MINOR ) "." STR ( FC_FW_VERSION_PATCH_LEVEL )

#define MW_VERSION             231

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
