/*
  vfs.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Virtual File System handler

  Part of grblHAL

  Copyright (c) 2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_VFS_H
#define INCLUDE_VFS_H

#define GRBL_VFS

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define vfs_load_plugin(x)
#define bcopy(src, dest, len) memmove(dest, src, len)

#if !(defined(__time_t_defined) || defined(__MSP432P401R__) || defined(PART_TM4C123GH6PM))
typedef struct {
    short date;
    short time;
} time_t;

struct tm {
  int tm_year;
  int tm_mon;
  int tm_mday;
  int tm_hour;
  int tm_min;
  int tm_sec;
};
#endif // __time_t_defined

typedef union
{
    uint8_t mode;
    struct {
        uint8_t read_only :1,
                hidden    :1,
                system    :1,
                unused    :1,
                directory :1,
                archive   :1;
    };
} vfs_st_mode_t;

typedef struct {
    size_t st_size;
    vfs_st_mode_t st_mode;
#ifdef ESP_PLATFORM // some versions of ESP-IDF/Compiler combos are fcked up
    time_t st_mtim;
#else
    time_t st_mtime;
#endif
} vfs_stat_t;

typedef struct {
    const void *fs;
    size_t size;
    uint8_t handle; // first byte of file handle structure
} vfs_file_t;

typedef struct {
    const void *fs;
    uint8_t handle;
} vfs_dir_t;

typedef struct {
    char name[255];
    size_t size;
    vfs_st_mode_t st_mode;
} vfs_dirent_t;

typedef struct {
    size_t size;
    size_t used;
} vfs_free_t;

typedef struct {
    const char *name;
    size_t size;
    const uint8_t data[];
} embedded_file_t;

typedef vfs_file_t *(*vfs_open_ptr)(const char *filename, const char *mode);
typedef char *(*vfs_getcwd_ptr)(char *buf, size_t size);
typedef size_t (*vfs_read_ptr)(void *buffer, size_t size, size_t count, vfs_file_t *file);
typedef size_t (*vfs_write_ptr)(const void *buffer, size_t size, size_t count, vfs_file_t *file);
typedef void (*vfs_close_ptr)(vfs_file_t *file);
typedef size_t (*vfs_ftell_ptr)(vfs_file_t *file);
typedef int (*vfs_fseek_ptr)(vfs_file_t *file, size_t offset);
typedef bool (*vfs_eof_ptr)(vfs_file_t *file);
typedef int (*vfs_rename_ptr)(const char *from, const char *to);
typedef int (*vfs_unlink_ptr)(const char *filename);

typedef int (*vfs_mkdir_ptr)(const char *path);
typedef int (*vfs_chdir_ptr)(const char *path);
typedef int (*vfs_rmdir_ptr)(const char *path);
typedef vfs_dir_t *(*vfs_opendir_ptr)(const char *path);
typedef char *(*vfs_readdir_ptr)(vfs_dir_t *dir, vfs_dirent_t *dirent);
typedef void (*vfs_closedir_ptr)(vfs_dir_t *dir);
typedef int (*vfs_stat_ptr)(const char *filename, vfs_stat_t *st);
typedef int (*vfs_utime_ptr)(const char *filename, struct tm *modified);

typedef bool (*vfs_getfree_ptr)(vfs_free_t *free);
typedef int (*vfs_format_ptr)(void);

typedef struct
{
    const char *fs_name;
    vfs_st_mode_t mode;
    vfs_open_ptr fopen;
    vfs_close_ptr fclose;
    vfs_read_ptr fread;
    vfs_write_ptr fwrite;
    vfs_ftell_ptr ftell;
    vfs_fseek_ptr fseek;
    vfs_eof_ptr feof;
    vfs_rename_ptr frename;
    vfs_unlink_ptr funlink;
    vfs_mkdir_ptr fmkdir;
    vfs_chdir_ptr fchdir;
    vfs_rmdir_ptr frmdir;
    vfs_opendir_ptr fopendir;
    vfs_readdir_ptr readdir;
    vfs_closedir_ptr fclosedir;
    vfs_stat_ptr fstat;
    vfs_utime_ptr futime;
    vfs_getcwd_ptr fgetcwd;
    vfs_getfree_ptr fgetfree;
    vfs_format_ptr format;
} vfs_t;

typedef struct vfs_mount
{
    char path[64];
    const vfs_t *vfs;
    struct vfs_mount *next;
} vfs_mount_t;

typedef struct {
    vfs_mount_t *mount;
} vfs_drives_t;

typedef struct {
    const char *name;
    const char *path;
    vfs_st_mode_t mode;
    const void *fs;
} vfs_drive_t;

extern int vfs_errno;

char *vfs_fixpath (char *path);

bool vfs_mount (const char *path, const vfs_t *fs);
bool vfs_unmount (const char *path);
vfs_file_t *vfs_open (const char *filename, const char *mode);
void vfs_close (vfs_file_t *file);
size_t vfs_read (void *buffer, size_t size, size_t count, vfs_file_t *file);
size_t vfs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file);
int vfs_puts (const char *s, vfs_file_t *file);
size_t vfs_tell (vfs_file_t *file);
int vfs_seek (vfs_file_t *file, size_t offset);
bool vfs_eof (vfs_file_t *file);
int vfs_rename (const char *from, const char *to);
int vfs_unlink (const char *filename);
int vfs_mkdir (const char *path);
int vfs_rmdir (const char *path);
int vfs_chdir (const char *path);
vfs_dir_t *vfs_opendir (const char *path);
vfs_dirent_t *vfs_readdir (vfs_dir_t *dir);
void vfs_closedir (vfs_dir_t *dir);
char *vfs_getcwd (char *buf, size_t len);
int vfs_stat (const char *filename, vfs_stat_t *st);
int vfs_utime (const char *filename, struct tm *modified);
vfs_free_t *vfs_fgetfree (const char *path);

vfs_drives_t *vfs_drives_open (void);
vfs_drive_t *vfs_drives_read (vfs_drives_t *handle);
void vfs_drives_close (vfs_drives_t *handle);
vfs_free_t *vfs_drive_getfree (vfs_drive_t *drive);
int vfs_drive_format (vfs_drive_t *drive);
vfs_drive_t *vfs_get_drive (const char *path);

#endif // INCLUDE_VFS_H
