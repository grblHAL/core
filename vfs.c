/*
  vfs.c - An embedded CNC Controller with rs274/ngc (g-code) support

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

#include <string.h>
#include <stdlib.h>

#ifndef ARDUINO_SAM_DUE

#include "hal.h"
#include "vfs.h"
//#include <errno.h>

// NULL file system

static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    return NULL;
}

static void fs_close (vfs_file_t *file)
{
}

static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

static size_t fs_tell (vfs_file_t *file)
{
    return 0;
}

static int fs_seek (vfs_file_t *file, size_t offset)
{
    return -1;
}

static bool fs_eof (vfs_file_t *file)
{
    return true;
}

static int fs_unlink (const char *filename)
{
    return -1;
}

static int fs_dirop (const char *path)
{
    return -1;
}

static vfs_dir_t *fs_opendir (const char *path)
{
    return NULL;
}

static void fs_closedir (vfs_dir_t *dir)
{
}

static int fs_stat (const char *filename, vfs_stat_t *st)
{
    return -1;
}

static int fs_chdir (const char *path)
{
    return !strcmp(path, "/") ? 0 : -1;
}

static char *fs_getcwd (char *buf, size_t size)
{
    return "/";
}

static const vfs_t fs_null = {
    .mode.directory = true,
    .mode.hidden = true,
    .mode.read_only = true,
    .fopen = fs_open,
    .fclose = fs_close,
    .fread = fs_read,
    .fwrite = fs_write,
    .ftell = fs_tell,
    .fseek = fs_seek,
    .feof = fs_eof,
    .funlink = fs_unlink,
    .fmkdir = fs_dirop,
    .fchdir = fs_chdir,
    .frmdir = fs_dirop,
    .fopendir = fs_opendir,
    .fclosedir = fs_closedir,
    .fstat = fs_stat,
    .fgetcwd = fs_getcwd
};

// End NULL file system

static vfs_mount_t root = {
    .path = "/",
    .vfs = &fs_null,
    .next = NULL
};
static vfs_mount_t *cwdmount = &root;
static char cwd[100] = "/";

int vfs_errno = 0;

// Strip trailing directory separator, FatFS dont't like it (WinSCP adds it)
char *vfs_fixpath (char *path)
{
    char *s = path + strlen(path) - 1;
    if(s > path && *s == '/' && s != strchr(path, '/'))
        *s = '\0';

    return path;
}

static inline vfs_mount_t *path_is_mount_dir (const char *path)
{
    size_t plen = strlen(path);

    if(*path == '/' && plen == 1)
        return &root;

    size_t mlen;
    vfs_mount_t *mount = root.next;

    if(mount) do {
        mlen = strlen(mount->path) - 1;
        if(mlen == plen && !strncmp(mount->path, path, mlen))
            break;
    } while((mount = mount->next));

    return mount;
}

static vfs_mount_t *get_mount (const char *path)
{
    vfs_errno = 0;

    if(*path != '/')
        return cwdmount;

    vfs_mount_t *mount;

    if((mount = path_is_mount_dir(path)) == NULL) {

        mount = root.next;

        if(mount) do {
            if(!strncmp(mount->path, path, strlen(mount->path)))
                break;
        } while((mount = mount->next));

        if(mount == NULL)
            mount = &root;
    }

    return mount;
}

static const char *get_filename (vfs_mount_t *mount, const char *filename)
{
    if(*filename == '/') {
        size_t len = strlen(mount->path);
        return filename + (len == 1 ? 0 : len);
    } else
        return filename;
}

vfs_file_t *vfs_open (const char *filename, const char *mode)
{
    vfs_file_t *file = NULL;
    vfs_mount_t *mount = get_mount(filename);

    if(mount && (file = mount->vfs->fopen(get_filename(mount, filename), mode)))
        file->fs = mount->vfs;

    return file;
}

void vfs_close (vfs_file_t *file)
{
    vfs_errno = 0;

    ((vfs_t *)(file->fs))->fclose(file);
}

size_t vfs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fread(buffer, size, count, file);
}

size_t vfs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fwrite(buffer, size, count, file);
}

int vfs_puts (const char *s, vfs_file_t *file)
{
    size_t count = strlen(s), ret;

    vfs_errno = 0;

    if((ret = ((vfs_t *)(file->fs))->fwrite(s, sizeof(char), count, file)) != count)
        return -1;

    return (int)ret;
}

size_t vfs_tell (vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->ftell(file);
}

int vfs_seek (vfs_file_t *file, size_t offset)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fseek(file, offset);
}

bool vfs_eof (vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->feof(file);
}

int vfs_rename (const char *from, const char *to)
{
    vfs_mount_t *mount_from = get_mount(from), *mount_to = get_mount(to);

    return mount_from == mount_to ? mount_from->vfs->frename(get_filename(mount_from, from), get_filename(mount_to, to)) : -1;
}

int vfs_unlink (const char *filename)
{
    vfs_mount_t *mount = get_mount(filename); // TODO: test for dir?

    return mount ? mount->vfs->funlink(get_filename(mount, filename)) : -1;
}

int vfs_mkdir (const char *path)
{
    vfs_mount_t *mount = get_mount(path);

    return mount ? mount->vfs->fmkdir(get_filename(mount, path)) : -1;
}

int vfs_rmdir (const char *path)
{
    vfs_mount_t *mount = get_mount(path);

    return mount ? mount->vfs->frmdir(get_filename(mount, path)) : -1;
}

int vfs_chdir (const char *path)
{
    int ret;

    vfs_errno = 0;

    if(*path != '/') {
        if(strcmp(path, "..")) {
            if(strlen(cwd) > 1)
                strcat(cwd, "/");
            strcat(cwd, path);
        } else {
            char *s = strrchr(cwd, '/');
            if(s)
                *s = '\0';
        }
    } else {
        strcpy(cwd, path);
        cwdmount = get_mount(cwd);
    }

    if((ret = cwdmount ? cwdmount->vfs->fchdir(path) : -1) != 0) { // + strlen(mount->path));))
        char *s = strrchr(cwd, '/');
        if(s)
            *s = '\0';
    }

    return ret;
}

vfs_dir_t *vfs_opendir (const char *path)
{
    vfs_dir_t *dir = NULL;
    vfs_mount_t *mount = get_mount(path);

    if(mount && (dir = mount->vfs->fopendir(get_filename(mount, path))))
        dir->fs = mount->vfs;

    return dir;
}

vfs_dirent_t *vfs_readdir (vfs_dir_t *dir)
{
    static vfs_dirent_t dirent;

    vfs_errno = 0;

    ((vfs_t *)dir->fs)->readdir(dir, &dirent);

    return *dirent.name == '\0' ? NULL : &dirent;
}

void vfs_closedir (vfs_dir_t *dir)
{
    vfs_errno = 0;

    ((vfs_t *)dir->fs)->fclosedir(dir);
}

char *vfs_getcwd (char *buf, size_t len)
{
    char *cwd = root.vfs->fgetcwd(NULL, len);

    vfs_errno = 0;

    if(buf == NULL)
        buf = (char *)malloc(strlen(cwd) + 1);

    if(buf)
        strcpy(buf, cwd);

    return buf ? buf : cwd;
}

int vfs_stat (const char *filename, vfs_stat_t *st)
{
    vfs_mount_t *mount = get_mount(filename);

    return mount ? mount->vfs->fstat(get_filename(mount, filename), st) : -1;
}

int vfs_utime (const char *filename, struct tm *modified)
{
    vfs_mount_t *mount = get_mount(filename);

    return mount && mount->vfs->futime ? mount->vfs->futime(get_filename(mount, filename), modified) : -1;
}

vfs_free_t *vfs_fgetfree (const char *path)
{
    static vfs_free_t free;

    vfs_mount_t *mount = get_mount(path);

    if(mount && mount->vfs->fgetfree && mount->vfs->fgetfree(&free))
        return &free;

    return NULL;
}

bool vfs_mount (const char *path, const vfs_t *fs)
{
    vfs_mount_t *vfs;

    if(!strcmp(path, "/"))
        root.vfs = fs;

    else if((vfs = (vfs_mount_t *)malloc(sizeof(vfs_mount_t)))) {

        strcpy(vfs->path, path);
        if(vfs->path[strlen(path) - 1] != '/')
            strcat(vfs->path, "/");

        vfs->vfs = fs;
        vfs->next = NULL;

        vfs_mount_t *mount = &root;

        while(mount->next)
            mount = mount->next;

        mount->next = vfs;
    }

    return fs != NULL;
}

bool vfs_unmount (const char *path)
{
    // TODO: close open files?

    if(!strcmp(path, "/"))
        root.vfs = &fs_null;

    else {

        vfs_mount_t *mount = get_mount(path);
        if(mount) {

            vfs_mount_t *pmount = &root;

            while(pmount->next && pmount->next != mount)
                pmount = pmount->next;

            if(pmount->next == mount)
                pmount->next = mount->next;

            free(mount);
        }
    }

    return true;
}

vfs_drive_t *vfs_get_drive (const char *path)
{
    static vfs_drive_t drive;

    vfs_mount_t *mount = get_mount(path);
    drive.name = mount->vfs->fs_name;
    drive.path = (const char *)mount->path;
    drive.mode = mount->vfs->mode;
    drive.fs = mount->vfs;

    return &drive;
}

vfs_drives_t *vfs_drives_open (void)
{
    vfs_drives_t *handle;
    vfs_mount_t *mount = &root;

    if((handle = malloc(sizeof(vfs_drives_t)))) {

        handle->mount = NULL;
        do {
            if(mount->vfs->mode.hidden)
                mount = mount->next;
            else
                handle->mount = mount;
        } while(mount && handle->mount == NULL);

        if(handle->mount == NULL) {
            free(handle);
            handle = NULL;
        }
    }

    return handle;
}

vfs_drive_t *vfs_drives_read (vfs_drives_t *handle)
{
    static vfs_drive_t drive;

    bool ok;

    if((ok = handle->mount != NULL)) {

        drive.name = handle->mount->vfs->fs_name;
        drive.path = (const char *)handle->mount->path;
        drive.mode = handle->mount->vfs->mode;
        drive.fs = handle->mount->vfs;

        handle->mount = handle->mount->next;

        if(handle->mount) do {
            if(!handle->mount->vfs->mode.hidden && handle->mount->vfs->fs_name)
                break;
        } while((handle->mount = handle->mount->next));
    }

    return ok ? &drive : NULL;
}

void vfs_drives_close (vfs_drives_t *handle)
{
    free(handle);
}

vfs_free_t *vfs_drive_getfree (vfs_drive_t *drive)
{
    static vfs_free_t free;

    const vfs_t *fs = drive->fs;

    return fs->fgetfree && fs->fgetfree(&free) ? &free : NULL;
}

int vfs_drive_format (vfs_drive_t *drive)
{
    const vfs_t *fs = drive->fs;

    return fs->format ? fs->format() : -1;
}

#endif
