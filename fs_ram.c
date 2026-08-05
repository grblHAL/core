/*
  fs_ram.c - heap based in memory filing system

  NOTE: files will be deleted on first close after opened for reading.

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#include "grbl/hal.h"
#include "grbl/platform.h"
#include "grbl/vfs.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FNAME_LEN 32

typedef union
{
    uint8_t mode;
    struct {
        uint8_t write  :1,
                stream :1,
                unused :1;
    };
} ram_file_flags_t;

typedef struct ram_file {
    char name[MAX_FNAME_LEN + 1];
    const uint8_t *data;
    const uint8_t *pos;
    ram_file_flags_t mode;
    uint8_t refs;
    size_t len;
    size_t remaining;
    stream_block_tx_buffer_t txbuf;
    struct ram_file *next;
} ram_file_t;

#define ram_fp(p) (*((ram_file_t **)(&p)))

static ram_file_t *files = NULL;
static driver_reset_ptr driver_reset = NULL;

static bool ram_write (ram_file_t *file, const uint8_t *s, size_t length)
{
    uint8_t *data;

    if((data = realloc((void *)file->data, file->len + length))) {
        memcpy((void *)(data + file->len), s, length);
        file->data = data;
        file->len += length;
        file->remaining = file->len;
        file->pos = data + file->len;
    } else {
        if(file->data != NULL)
            free((void *)file->data);
        file->data = file->pos = NULL;
        file->len = file->remaining = 0;
    }

    file->txbuf.s = file->txbuf.data;
    file->txbuf.length = 0;

    return !!data;
}

FLASHMEM static ram_file_t *find_file (const char *name)
{
    ram_file_t *file = files;

    while(file && strcmp(name, file->name))
        file = file->next;

    return file;
}

FLASHMEM static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    vfs_file_t *file;
    ram_file_t *rfile = find_file(filename);

    if(strlen(filename) > MAX_FNAME_LEN || (strchr(mode, 'r') && rfile == NULL) || (rfile && rfile->refs))
        return NULL;

    if((file = malloc(sizeof(vfs_file_t) + sizeof(ram_file_t *) - 1))) {

        if(rfile == NULL && (rfile = calloc(sizeof(ram_file_t), 1))) {

            if(rfile->txbuf.s == NULL) {
                rfile->txbuf.s = rfile->txbuf.data;
                rfile->txbuf.length = 0;
                rfile->txbuf.max_length = sizeof(rfile->txbuf.data);
                strcpy(rfile->name, filename);
            }

            if(files == NULL)
                files = rfile;
            else {
                ram_file_t *add = files;
                while(add->next)
                    add = add->next;
                add->next = rfile;
            }
        }

        if(rfile) {
            rfile->refs++;
            rfile->mode.write = !!strchr(mode, 'w');
            rfile->pos = rfile->data;
            file->size = rfile->remaining = rfile->len;
            file->status.is_temporary = On;
            memcpy(&file->handle, &rfile, sizeof(ram_file_t *));
        } else {
            free(file);
            file = NULL;
        }
    }

    return file;
}

FLASHMEM static bool unlink (ram_file_t *file)
{
    bool unlink;

    if((unlink = !file->refs || --file->refs == 0)) {

        if(file == files)
            files = file->next;
        else {
            ram_file_t *prev = files;
            while(prev->next && prev->next != file)
                prev = prev->next;
            if(prev)
                prev->next = file->next;
        }
    }

    if(unlink) {
        if(file->data)
            free((void *)file->data);
        free(file);
    }

    return unlink;
}

FLASHMEM static void fs_close (vfs_file_t *file)
{
    ram_file_t *rfile = ram_fp(file->handle);

    if(rfile->mode.write) {
        if(rfile->txbuf.length)
            ram_write(rfile, rfile->txbuf.data, rfile->txbuf.length);
        rfile->mode.write = false;
        rfile->refs--;
    } else
        unlink(rfile);

    free(file);
}

FLASHMEM static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    size_t rcount = 0;
    ram_file_t *rfile = ram_fp(file->handle);

    if(rfile->pos) {
        rcount = size * count > rfile->remaining ? rfile->remaining : size * count;
        memcpy(buffer, rfile->pos, rcount);
        rfile->pos += rcount;
    } else
        rfile->remaining = rcount = 0;

    rfile->remaining -= rcount;

    return rcount;
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    uint8_t *s = (uint8_t *)buffer;
    size_t length = size * count;
    ram_file_t *rfile;

    if(length == 0 || (rfile = ram_fp(file->handle)) == NULL)
        return 0;

    if(rfile->txbuf.length && (rfile->txbuf.length + length) > rfile->txbuf.max_length) {
        if(!ram_write(rfile, rfile->txbuf.data, rfile->txbuf.length))
            return 0;
    }

    if(rfile->txbuf.length == 0 && length > rfile->txbuf.max_length)
        return ram_write(rfile, s, length) ? length : 0;

    memcpy(rfile->txbuf.s, s, length);
    rfile->txbuf.length += length;
    rfile->txbuf.s += length;

    return length;
}

FLASHMEM static size_t fs_tell (vfs_file_t *file)
{
   return ram_fp(file->handle)->len - ram_fp(file->handle)->remaining;
}

FLASHMEM static int fs_seek (vfs_file_t *file, size_t offset)
{
    ram_file_t *rfile = NULL;

    vfs_errno = ENOENT;

    if((rfile = ram_fp(file->handle)) && (vfs_errno = offset <= rfile->len ? 0 : EINVAL) == 0) {
        rfile->pos = rfile->data + offset;
        rfile->remaining = rfile->len - offset;
    }

    return vfs_errno == 0 ? 0 : -1;
}

FLASHMEM static int fs_truncate (vfs_file_t *file, size_t length)
{
    ram_file_t *rfile = NULL;

    if(length == 0 && (rfile = ram_fp(file->handle))) {
        if(rfile->data) {
            free((void *)rfile->data);
            rfile->data = rfile->pos = NULL;
            rfile->len = rfile->remaining = 0;
        }
    }

    return rfile && rfile->len == length ? 0 : -1;
}

FLASHMEM static bool fs_eof (vfs_file_t *file)
{
    return ram_fp(file->handle)->remaining == 0;
}

FLASHMEM static int fs_rename (const char *from, const char *to)
{
    ram_file_t *file = NULL;

    if(strlen(to) <= MAX_FNAME_LEN && (file = find_file(from)))
        strcpy(file->name, to);

    return file ? 0 : -1;
}

FLASHMEM static int fs_unlink (const char *filename)
{
    ram_file_t *file;

    if((file = find_file(filename)) && file->refs == 0)
        unlink(file);
    else
        return -1;

    return 0;
}

FLASHMEM static int fs_dirop (const char *path)
{
    return -1;
}

FLASHMEM static vfs_dir_t *fs_opendir (const char *path)
{
    vfs_dir_t *dir;

    if((dir = files ? calloc(1, sizeof(vfs_dir_t) + sizeof(ram_file_t *) - 1) : NULL))
        memcpy(&dir->handle, &files, sizeof(ram_file_t *));

    return dir;
}

FLASHMEM static char *fs_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    ram_file_t *f;

    *dirent->name = '\0';

    if((f = ram_fp(dir->handle))) {
        vfs_errno = 0;
        dirent->size = f->len;
        dirent->st_mode.mode = 0;
        strcpy(dirent->name, f->name);
        memcpy(&dir->handle, &f->next, sizeof(ram_file_t *));
    }

    return *dirent->name ? dirent->name : NULL;
}

FLASHMEM static void fs_closedir (vfs_dir_t *dir)
{
    if(dir)
        free(dir);
}

FLASHMEM static int fs_stat (const char *filename, vfs_stat_t *st)
{
    ram_file_t *file;

    if((file = find_file(filename)))
        st->st_size = file->len;

    return file ? 0 : -1;
}

FLASHMEM static void fs_reset (void)
{
    driver_reset();

    ram_file_t *file = files, *next;

    if(file) do {
        next = file->next;
        if(file->data)
            free((void *)file->data);
        free(file);
    } while((file = next));
}

FLASHMEM static bool fs_getfree (vfs_free_t *free)
{
    ram_file_t *file = files;

    free->used = 0;

    if(file) do {
        free->used += file->len;
    } while ((file = file->next));

    free->size = free->used; // available memory?

    return true;
}

/*

Code for redirecting output stream to file, replaces networking/fs_stream.c

static vfs_file_t *file; -> move to ram_file_t?
static stream_write_ptr wrptr;

fdopen()?

static void stream_write (const char *s)
{
    size_t length = strlen(s);

    if(length == 0)
        return;

    fs_write(s, length, 1, file);
}

FLASHMEM bool redirect_output (const char *filename)
{
    if(hal.stream.write == stream_write)
        return NULL;

    if((file = fs_open(filename, "w"))) {
        file->mode.stream = On;
        wrptr = hal.stream.write;
        hal.stream.write = stream_write;
    }

    return !!file;
}

FLASHMEM bool redirect_close (void)
{
    bool ok;

    if(hal.stream.write == stream_write)
        hal.stream.write = wrptr;

    if((ok = !!file)) {
        fs_close(file);
        file = NULL;
    }

    return ok;
}

*/

FLASHMEM void fs_ram_mount (void)
{
    PROGMEM static const vfs_t fs = {
        .fs_name = "RAM",
        .fopen = fs_open,
        .fclose = fs_close,
        .fread = fs_read,
        .fwrite = fs_write,
        .ftell = fs_tell,
        .fseek = fs_seek,
        .ftruncate = fs_truncate,
        .feof = fs_eof,
        .frename = fs_rename,
        .funlink = fs_unlink,
        .fmkdir = fs_dirop,
        .fchdir = fs_dirop,
        .frmdir = fs_dirop,
        .fopendir = fs_opendir,
        .readdir = fs_readdir,
        .fclosedir = fs_closedir,
        .fstat = fs_stat,
        .fgetfree = fs_getfree
    };

    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = fs_reset;
        vfs_mount(NULL, "/ram", &fs, (vfs_st_mode_t){ .directory = true, .hidden = true });
    }
}
