/*
  fs_embedded.c - flash based read-only file system

  Part of grblHAL

  Copyright (c) 2022-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "grbl.h"
#include "vfs.h"

typedef struct {
    const embedded_file_t *file;
    size_t remaining;
} embedded_filehandle_t;

static struct embedded_files {
    const embedded_file_t **files;
    struct embedded_files *next;
} embedded = {0};

typedef struct {
    uint_fast16_t idx;
    struct embedded_files *files;
} embedded_dir_handle_t;

static size_t fs_size = 0;
static bool all_hidden = true;

FLASHMEM static const embedded_file_t *find_file (const char *filename)
{
    uint_fast16_t idx = 0;
    const embedded_file_t *file = NULL;
    struct embedded_files *root = &embedded;

    if(*filename == '/')
        filename++;

    if(root) do {
        do {
            if(!strcmp(root->files[idx]->name, filename))
                file = root->files[idx];
        } while(file == NULL && root->files[++idx]);
    } while(file == NULL && (root = root->next));

    return file;
}

FLASHMEM static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    vfs_file_t *fileh = NULL;

    if(strchr(mode, 'r')) {

        const embedded_file_t *file = NULL;

        if((file = find_file(filename)) && (fileh = malloc(sizeof(vfs_file_t) - VFS_HANDLE_SIZE + sizeof(embedded_filehandle_t)))) {
            embedded_filehandle_t *f = (embedded_filehandle_t *)&fileh->handle;
            f->file = file;
            fileh->size = f->remaining = file->size;
        }
    }

    return fileh;
}

FLASHMEM static void fs_close (vfs_file_t *file)
{
    free(file);
}

FLASHMEM static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    size_t rcount = 0;
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    if(fileh->remaining) {
        rcount = size * count > fileh->remaining ? fileh->remaining : size * count;
        memcpy(buffer, &fileh->file->data[fileh->file->size - fileh->remaining], rcount);
    }

    fileh->remaining -= rcount;

    return rcount;
}

FLASHMEM static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

FLASHMEM static size_t fs_tell (vfs_file_t *file)
{
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    return fileh->file->size - fileh->remaining;
}

FLASHMEM static int fs_seek (vfs_file_t *file, size_t offset)
{
    embedded_filehandle_t *fileh = (embedded_filehandle_t *)&file->handle;

    if(fileh->file->size <= offset)
        fileh->remaining = fileh->file->size - offset;

    return fileh->file->size < offset ? fileh->remaining : -1;
}

FLASHMEM static bool fs_eof (vfs_file_t *file)
{
    return ((embedded_filehandle_t *)&file->handle)->remaining == 0;
}

FLASHMEM static int fs_unlink (const char *filename)
{
    return -1;
}

FLASHMEM static int fs_dirop (const char *path)
{
    return -1;
}

FLASHMEM static int fs_chdir (const char *path)
{
    return *path == '\0' && !all_hidden ? 0 : -1;
}

FLASHMEM static vfs_dir_t *fs_opendir (const char *path)
{
    vfs_dir_t *dir = NULL;
    embedded_dir_handle_t *dirh;

    if(embedded.files && (dirh = malloc(sizeof(embedded_dir_handle_t)))) {
        if((dir = malloc(sizeof(vfs_dir_t) - VFS_HANDLE_SIZE + sizeof(embedded_dir_handle_t)))) {
            dirh->idx = 0;
            dirh->files = &embedded;
            dir->handle = dirh;
        } else
            free(dirh);
    }

    return dir;
}

FLASHMEM static char *fs_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    const embedded_file_t *f;
    embedded_dir_handle_t *dirh = dir->handle;

    *dirent->name = '\0';

    while(dirh->files && (f = dirh->files->files[dirh->idx])) {
        if(dirh->files->files[++dirh->idx] == NULL) {
            dirh->idx = 0;
            dirh->files = dirh->files->next;
        }
        if(!f->st_mode.hidden) {
            vfs_errno = 0;
            dirent->size = f->size;
            dirent->st_mode = f->st_mode;
            strcpy(dirent->name, f->name);
            break;
        }
    }

    return *dirent->name ? dirent->name : NULL;
}

FLASHMEM static void fs_closedir (vfs_dir_t *dir)
{
    free(dir->handle);
    free(dir);
}

FLASHMEM static int fs_stat (const char *filename, vfs_stat_t *st)
{
    int ret = -1;
    const embedded_file_t *file;

    memset(st, 0, sizeof(vfs_stat_t));

    if(*filename == '\0') {
        ret = 0;
        st->st_size = -1;
        st->st_mode.directory = st->st_mode.read_only= On;
    } else if((file = find_file(filename))) {
        ret = 0;
        st->st_size = file->size;
        st->st_mode.mode = file->st_mode.mode;
        st->st_mode.read_only = On;
    }

    return ret;
}

FLASHMEM static bool fs_getfree (vfs_free_t *free)
{
    free->size = free->used = fs_size;

    return true;
}

FLASHMEM void fs_embedded_mount (const embedded_file_t **files)
{
    PROGMEM static const vfs_t fs = {
        .fs_name = "embedded",
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
        .readdir = fs_readdir,
        .fclosedir = fs_closedir,
        .fstat = fs_stat,
        .fgetfree = fs_getfree
    };

    uint_fast16_t idx = 0;

    while(files[idx]) {
        fs_size += files[idx]->size;
        if(!files[idx++]->st_mode.hidden)
            all_hidden = false;
    }

    if(embedded.files == NULL) {

        embedded.files = files;
        vfs_mount(NULL, "/embedded", &fs, (vfs_st_mode_t){ .directory = On, .read_only = On, .hidden = On });

    } else {

        struct embedded_files *add = &embedded;

        while(add->next)
            add = add->next;

        if((add->next = malloc(sizeof(struct embedded_files)))) {
            add->next->files = files;
            add->next->next = NULL;
        }
    }

    if(!all_hidden)
        vfs_mount_set_mode("/embedded", (vfs_st_mode_t){ .directory = On, .read_only = On });
}
