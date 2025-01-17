/*
  stream_file.c - stream redirector for file input

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

#include "hal.h"
#include "stream_file.h"

typedef struct rd_stream {
    vfs_file_t *file_new;
    vfs_file_t *file;
    stream_read_ptr read;
    stream_type_t type;
    status_message_ptr status_handler;
    on_file_end_ptr eof_handler;
    struct rd_stream *next;
} rd_stream_t;

static rd_stream_t *rd_streams = NULL;
static status_message_ptr status_message;
static on_file_end_ptr on_file_end;
static on_report_handlers_init_ptr on_report_handlers_init;

// File stream input function.
// Reads character by character from a file and returns them when
// requested by the foreground process.
static int16_t stream_read_file (void)
{
    static bool eol_ok = false;

    char c;

    if(hal.stream.file) {
        if(vfs_read(&c, 1, 1, hal.stream.file) == 1) {
            if(c == ASCII_CR || c == ASCII_LF) {
                if(eol_ok)
                    return SERIAL_NO_DATA;
                eol_ok = true;
            } else
                eol_ok = false;
        } else if(eol_ok) {
            eol_ok = false;
            return ASCII_EOF;   // Return end-of-file. grbl.on_file_end() event generated in protocol.c.
        } else {
            eol_ok = true;
            return ASCII_LF;    // Return a linefeed if the last character was not a linefeed.
        }
    } else
        return SERIAL_NO_DATA; // TODO: close all streams?

    return (int16_t)c;
}

static status_code_t onFileEnd (vfs_file_t *file, status_code_t status)
{
    rd_stream_t *stream;

    if((stream = rd_streams)) {

        while(stream->next && (stream = stream->next));

        if(stream->eof_handler)
            stream->eof_handler(file, status);
    }

    if(on_file_end)
        on_file_end(file, status);

    return true;
}

// This code will be executed after each command is sent to the parser,
// If an error is detected reading from file(s) will be stopped and the
// status_code reported, if not a "ok" status reply will not be output.
static status_code_t trap_status_messages (status_code_t status)
{
    gc_state.last_error = status;

    if(rd_streams == NULL)
        status = status_message(status);

    else if(!(status == Status_OK || status == Status_Unhandled)) {

        rd_stream_t *stream;

        if((stream = rd_streams)) {

            while(stream->next && (stream = stream->next));

            if(stream->status_handler)
                status = stream->status_handler(status);
        }
    }

    return status;
}

static void onReportHandlersInit (void)
{
    if(on_report_handlers_init)
        on_report_handlers_init();

    status_message = grbl.report.status_message;
    grbl.report.status_message = trap_status_messages;
}

void stream_set_type (stream_type_t type, vfs_file_t *file)
{
    hal.stream.type = type;
    if(!(hal.stream.file = file))
        gc_state.file_stream = false;
}

bool stream_is_file (void)
{
    return hal.stream.type == StreamType_File;
}

vfs_file_t *stream_redirect_read (char *filename, status_message_ptr status_handler, on_file_end_ptr eof_handler)
{
    static bool error_handler_ok = false;

    vfs_file_t *file;

    if((file = vfs_open(filename, "r"))) {
        rd_stream_t *rd_stream, *streams = rd_streams;
        if((rd_stream = malloc(sizeof(rd_stream_t)))) {
            rd_stream->file = hal.stream.file;
            rd_stream->type = hal.stream.type;
            rd_stream->file_new = file;
            rd_stream->read = hal.stream.read;
            rd_stream->eof_handler = eof_handler;
            rd_stream->status_handler = status_handler;
            rd_stream->next = NULL;
            hal.stream.read = stream_read_file;
            stream_set_type(StreamType_File, file);
            if(streams == NULL)
                rd_streams = rd_stream;
            else do {
                if(streams->next == NULL) {
                    streams->next = rd_stream;
                    break;
                }
            } while((streams == streams->next));
        } else {
            vfs_close(file);
            file = NULL;
        }
    }

    if(file && !error_handler_ok) {
        error_handler_ok = true;

        on_report_handlers_init = grbl.on_report_handlers_init;
        grbl.on_report_handlers_init = onReportHandlersInit;

        status_message = grbl.report.status_message;
        grbl.report.status_message = trap_status_messages;

        on_file_end = grbl.on_file_end;
        grbl.on_file_end = onFileEnd;
    }

    return file;
}

void stream_redirect_close (vfs_file_t *file)
{
    rd_stream_t *stream = rd_streams, *prev_stream = NULL;

    if(stream) do {
        if(stream->file_new == file) {
            vfs_close(file);
            hal.stream.read = stream->read;
            stream_set_type(stream->type, stream->file);
            if(stream == rd_streams)
                rd_streams = stream->next;
            else
                prev_stream->next = stream->next;
            free(stream);
            break;
        }
        prev_stream = stream;
    } while((stream = stream->next));
}
