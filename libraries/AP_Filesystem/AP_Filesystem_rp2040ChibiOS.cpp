/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ArduPilot filesystem interface for rp2040 systems
 */
#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>

#if HAVE_FILESYSTEM_SUPPORT
#if CONFIG_HAL_BOARD == HAL_BOARD_RP2040CHIBIOS

#define FSDEBUG 0

#include <utime.h>

extern const AP_HAL::HAL& hal;

int AP_Filesystem_rp2040ChibiOS::open(const char *fname, int flags, bool allow_absolute_paths)
{
// #if FSDEBUG
//     printf("DO open %s \n", fname);
// #endif
//     // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
//     return ::open(fname, flags | O_TRUNC | O_CLOEXEC, 0666);
    return 0;
}

int AP_Filesystem_rp2040ChibiOS::close(int fd)
{
// #if FSDEBUG
//     printf("DO close \n");
// #endif
//     return ::close(fd);
    return 0;
}

int32_t AP_Filesystem_rp2040ChibiOS::read(int fd, void *buf, uint32_t count)
{
// #if FSDEBUG
//     printf("DO read \n");
// #endif
//     return ::read(fd, buf, count);
    return 0;
}

int32_t AP_Filesystem_rp2040ChibiOS::write(int fd, const void *buf, uint32_t count)
{
// #if FSDEBUG
//     printf("DO write \n");
// #endif
//     return ::write(fd, buf, count);
    return 0;
}

int AP_Filesystem_rp2040ChibiOS::fsync(int fd)
{
// #if FSDEBUG
//     printf("DO fsync \n");
// #endif
//     return ::fsync(fd);
    return 0;
}

int32_t AP_Filesystem_rp2040ChibiOS::lseek(int fd, int32_t offset, int seek_from)
{
// #if FSDEBUG
//     printf("DO lseek \n");
// #endif
//     return ::lseek(fd, offset, seek_from);
    return 0;
}

int AP_Filesystem_rp2040ChibiOS::stat(const char *pathname, struct stat *stbuf)
{
// #if FSDEBUG
//     printf("DO stat %s \n", pathname);
// #endif
//     return ::stat(pathname, stbuf);
    return 0;
}

int AP_Filesystem_rp2040ChibiOS::unlink(const char *pathname)
{
// #if FSDEBUG
//     printf("DO unlink %s \n", pathname);
// #endif
//     return ::unlink(pathname);
    return 0;
}

int AP_Filesystem_rp2040ChibiOS::mkdir(const char *pathname)
{
// #if FSDEBUG
//     printf("DO mkdir %s \n", pathname);
// #endif
//     return ::mkdir(pathname, 0777);
    return 0;
}

void* AP_Filesystem_rp2040ChibiOS::opendir(const char *pathname)
{
// #if FSDEBUG
//     printf("DO opendir %s \n", pathname);
// #endif

//     return (void*)::opendir(pathname);
    //	return NULL;
    return NULL;
}

struct dirent *AP_Filesystem_rp2040ChibiOS::readdir(void *dirp)
{
// #if FSDEBUG
//     printf("DO readdir \n");
// #endif
//     return ::readdir((DIR*)dirp);
//     	return NULL;
    return NULL;
}

int AP_Filesystem_rp2040ChibiOS::closedir(void *dirp)
{
// #if FSDEBUG
//     printf("DO closedir \n");
// #endif

//     return ::closedir((DIR*)dirp);
    //	return 0;
    return 0;
}

// return free disk space in bytes
int64_t AP_Filesystem_rp2040ChibiOS::disk_free(const char *path)
{

// #if FSDEBUG
//     printf("DO free disk %s \n", path);
// #endif
//     FATFS *fs;
//     DWORD fre_clust, fre_sect;

//     /* Get volume information and free clusters of sdcard */
//     auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
//     if (res) {
//         return -1;
//     }

//     /* Get total sectors and free sectors */
//     fre_sect = fre_clust * fs->csize;

//     int64_t tmp_free_bytes = fre_sect * FF_SS_SDCARD;

//     return tmp_free_bytes;
    return 0;
}

// return total disk space in bytes
int64_t AP_Filesystem_rp2040ChibiOS::disk_space(const char *path)
{
// #if FSDEBUG
//     printf("DO usage disk %s \n", path);
// #endif
//     FATFS *fs;
//     DWORD fre_clust, tot_sect;

//     /* Get volume information and free clusters of sdcard */
//     auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
//     if (res) {
//         return -1;
//     }

//     /* Get total sectors and free sectors */
//     tot_sect = (fs->n_fatent - 2) * fs->csize;

//     int64_t tmp_total_bytes = tot_sect * FF_SS_SDCARD;

//     return tmp_total_bytes;
    return 0;
}

/*
  set mtime on a file
 */
bool AP_Filesystem_rp2040ChibiOS::set_mtime(const char *filename, const uint32_t mtime_sec)
{

// #if FSDEBUG
//     printf("DO time %s \n", filename);
// #endif
//     struct utimbuf times {};
//     times.actime = mtime_sec;
//     times.modtime = mtime_sec;

//     return utime(filename, &times) == 0;
    return true;
}

#endif // CONFIG_HAL_BOARD
#endif
