/*
  fs_device.c - redirector for file i/o to stream (device)

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#include <stdlib.h>
#include <string.h>

#include "vfs.h"
#include "stream.h"
#include "nuts_bolts.h"

struct fs_device {
    io_stream_t io_stream;
    stream_claim_ptr open;
    char name[10];
    bool write;
    vfs_file_t file;
    struct fs_device *next;
};
typedef struct fs_device fs_device_t;

static fs_device_t *root = NULL;

bool device_fs_add_stream (io_stream_properties_t *stream)
{
    if(!stream->flags.claimable || !stream->flags.modbus_ready)
        return false;

    fs_device_t *add = calloc(sizeof(fs_device_t), 1);
    if(add) {

        switch(stream->type) {

            case StreamType_Serial:
                strcpy(add->name, /*stream->flags.is_usb*/ 0 ? "usb" : "uart");
                break;

            case StreamType_Bluetooth:
                strcpy(add->name, "bt");
                break;

            default: break;
        }

        add->open = stream->claim;
        strcat(add->name, uitoa(stream->instance));

        if(root == NULL)
            root = add;
        else {
            fs_device_t *fss = root;
            while(fss->next)
                fss = fss->next;
            fss->next = add;
        }
    }

    return add != NULL;
}

static fs_device_t *find (const char *filename)
{
    fs_device_t *device = root;

    if(device) do {
        if(!strcmp(device->name, filename + 1))
            break;
    } while((device = device->next));

    return device;
}

static inline io_stream_t *iostream (io_stream_t **io_stream)
{
    return (io_stream_t *)*io_stream;
}

static inline io_stream_t *iostream_ptr (io_stream_t *io_stream)
{
    return io_stream;
}

static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    vfs_file_t *file = NULL;
    fs_device_t *device = find(filename);

    if(device && (device->write = strchr(mode, 'w') != NULL)) {

        const io_stream_t *io_stream;

        if(device->open) {
            if((io_stream = device->open(115200))) {
                memcpy(&device->io_stream, io_stream, sizeof(io_stream_t));
                device->open = NULL;
            } else
                return NULL;
        }

        if(device->io_stream.state.passthru)
            return NULL;

        device->io_stream.state.passthru = On; // flag open

        if((file = calloc(sizeof(vfs_file_t) + sizeof(io_stream_t *), 1))) {
            io_stream = &device->io_stream;
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsizeof-pointer-memaccess"
#endif
            memcpy(iostream_ptr((io_stream_t *)&file->handle), &io_stream, sizeof(io_stream_t *));
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
        }
    }

    return file;
}

static void fs_close (vfs_file_t *file)
{
    iostream((io_stream_t **)&file->handle)->state.passthru = Off;

    free(file);
}

static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    uint8_t *pos = (uint8_t *)buffer;
    size_t rcount = iostream((io_stream_t **)&file->handle)->get_rx_buffer_count();

    if((rcount = min(rcount, size * count))) {
        while(rcount--)
            *pos++ = (uint8_t)iostream((io_stream_t **)&file->handle)->read();
    }

    return rcount;
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    iostream((io_stream_t **)&file->handle)->write_n((uint8_t *)buffer, size * count);

    return size * count;
}

static size_t fs_tell (vfs_file_t *file)
{
    return 0;
}

static bool fs_eof (vfs_file_t *file)
{
    return iostream((io_stream_t **)&file->handle)->get_rx_buffer_count() == 0;
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
    fs_device_t *device = find(filename);

    st->st_size = device ? device->io_stream.get_rx_buffer_count() : 0;

    return device ? 0 : -1;
}

void fs_device_mount (void)
{
    PROGMEM static const vfs_t fs = {
        .fopen = fs_open,
        .fclose = fs_close,
        .fread = fs_read,
        .fwrite = fs_write,
        .ftell = fs_tell,
        .feof = fs_eof,
        .funlink = fs_unlink,
        .fmkdir = fs_dirop,
        .fchdir = fs_dirop,
        .frmdir = fs_dirop,
        .fopendir = fs_opendir,
        .fclosedir = fs_closedir,
        .fstat = fs_stat
    };
    static bool ok = false;

    if(!ok)
        ok = vfs_mount("/dev", &fs, (vfs_st_mode_t){ .directory = true, .hidden = true });
}
