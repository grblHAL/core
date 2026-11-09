/*
  vfs.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Virtual File System handler

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

#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "hal.h"
#include "strutils.h"
#include "vfs.h"

#ifdef ARDUINO_SAM_DUE
#undef feof
#endif

typedef void (*vfs_ptr)(void);

static vfs_mount_t *get_rootfs (void);

static inline vfs_mount_t *path_is_mount_dir (const char *path)
{
    size_t plen = strlen(path);

    vfs_mount_t *mount = get_rootfs();

    if(*path == '/' && plen == 1)
        return mount;

    size_t mlen;

    if(*(path + plen - 1) == '/')
        plen--;

    if((mount = mount->next)) do {
        mlen = strlen(mount->path) - 1;
        if(mlen == plen && !strncmp(mount->path, path, mlen))
            break;
    } while((mount = mount->next));

    return mount;
}

// NULL file system

FLASHMEM static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    return NULL;
}

FLASHMEM static void fs_close (vfs_file_t *file)
{
}

FLASHMEM static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

FLASHMEM static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return 0;
}

FLASHMEM static size_t fs_tell (vfs_file_t *file)
{
    return 0;
}

FLASHMEM static int fs_seek (vfs_file_t *file, size_t offset)
{
    return -1;
}

FLASHMEM static bool fs_eof (vfs_file_t *file)
{
    return true;
}

FLASHMEM static int fs_unlink (const char *filename)
{
    return -1;
}

FLASHMEM static int fs_dirop (const char *path)
{
    return -1;
}

FLASHMEM static vfs_dir_t *fs_opendir (const char *path)
{
    static vfs_dir_t *dir = NULL;

    if(dir == NULL)
        dir = calloc(1, sizeof(vfs_dir_t) + 3);

    if(dir) {
        vfs_mount_t **mount = (vfs_mount_t **)&dir->handle;
        *mount = get_rootfs();
    }

    return !strcmp(path, "/") ? dir : NULL;
}

FLASHMEM static char *fs_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    *dirent->name = '\0';

    return NULL;
}

FLASHMEM static void fs_closedir (vfs_dir_t *dir)
{
}

FLASHMEM static int fs_stat (const char *filename, vfs_stat_t *st)
{
    memset(st, 0, sizeof(vfs_stat_t));

    if(*filename++ == '/' && *filename == '\0') {
        st->st_size = -1;
        st->st_mode.directory = On;
        st->st_mode.read_only = get_rootfs()->next == NULL;
        vfs_errno = 0;
    } else
        vfs_errno = ENOTDIR;

    return vfs_errno ? -1 : 0;
}

FLASHMEM static int fs_chdir (const char *path)
{
    return !strcmp(path, "/") ? 0 : -1;
}

FLASHMEM static char *fs_getcwd (char *buf, size_t size)
{
    return "/";
}

PROGMEM static const vfs_t fs_null = {
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
    .fgetcwd = fs_getcwd
};

// End NULL file system

static struct {
    uint8_t root     :1,
            embedded :1,
            littlefs :1,
            unused   :5;
} fsmounted = {0};
static vfs_mount_t root = {
    .path = "/",
    .vfs = &fs_null,
    .mode = { .directory = true, .read_only = true, .hidden = true },
    .next = NULL
};
static vfs_mount_t *cwdmount = &root;
static char _cwd[101] = "/";
static string_t cwdb = { .name = _cwd, .len = sizeof(_cwd) - 1 }, cwd = { .name = _cwd, .len = sizeof(_cwd) - 1, .base = &cwdb };

volatile int vfs_errno = 0;
vfs_events_t vfs = {0};

static int _vfs_stat (vfs_mount_t *mount, const char *filename, vfs_stat_t *st);

FLASHMEM static void mount_fs_null (void)
{
    root.vfs = &fs_null;
    root.mode = (vfs_st_mode_t){ .directory = true, .read_only = true, .hidden = true };
    fsmounted.root = Off;
    if(cwdmount == &root)
        strcpy(cwd.name, "/");
}

FLASHMEM static vfs_mount_t *get_rootfs (void)
{
    return &root;
}

// Strip trailing directory separator, FatFS dont't like it (WinSCP adds it)
FLASHMEM char *vfs_fixpath (char *path)
{
    char *s = path + strlen(path) - 1;
    if(s > path && *s == '/' && s != strchr(path, '/'))
        *s = '\0';

    return path;
}

// NOTE: only call once from public call
FLASHMEM static const char *parse_path (const char *path)
{
    static string_t abspath = {0};

    char *fullpath;

    if((fullpath = strset_(&abspath, strlen(cwd.name) + strlen(path) + 1, *path == '/' ? "/" : cwd.name))) {

        char *newpath;

        if((newpath = malloc(strlen(path) + 1))) {

            strcpy(newpath, path);
            strcpy(fullpath, *path == '/' ? "/" : cwd.name);

            char *p, *el = strtok(newpath, "/");

            while(el) {
                if(!strcmp("..", el)) {
                    if((p = strrchr(fullpath, '/')))
                        *(p + (p == fullpath ? 1 : 0)) = '\0';
                } else if(*el && strcmp(el, ".")) {
                    if(strlen(fullpath) == 1)
                        strcat(fullpath, el);
                    else
                        strcat(strcat(fullpath, "/"), el);
                }
                el = strtok(NULL, "/");
            }

            free(newpath);
        } else
            fullpath = NULL;
    }

    vfs_errno = fullpath ? 0 : ENOMEM;

    return fullpath;
}

FLASHMEM static vfs_mount_t *get_mount (const char *path)
{
    if(path == NULL)
        return NULL;

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

FLASHMEM static const char *get_filename (vfs_mount_t *mount, const char *filename)
{
    if(*filename == '/') {
        size_t len = strlen(mount->path);
        return filename + (len == 1 ? 0 : len - 1);
    } else
        return filename;
}

FLASHMEM static vfs_mount_t *mount_modifiable (const char *path, size_t op_fn)
{
    vfs_mount_t *mount = get_mount(path);

    if(mount == NULL || mount->mode.hidden)
        vfs_errno = EFAULT;
    else if(mount->mode.read_only)
        vfs_errno = EROFS;
    else if(*((vfs_ptr *)((uint8_t *)mount->vfs + op_fn)) == NULL)
        vfs_errno = EPERM;
    else if(!strncmp(mount->path, path, strlen(path)))
        vfs_errno = EISDIR;
    else
        vfs_errno = 0;

    return vfs_errno ? NULL : mount;
}

FLASHMEM vfs_file_t *vfs_open (const char *filename, const char *mode)
{
    vfs_file_t *file = NULL;
    vfs_mount_t *mount = get_mount(filename);

    if(mount && (file = mount->vfs->fopen(get_filename(mount, filename), mode))) {
        file->fs = mount->vfs;
        file->status.update = !mount->mode.hidden && !!strchr(mode, 'w');
    }

    return file;
}

FLASHMEM void vfs_close (vfs_file_t *file)
{
    vfs_t *fs;
    bool update;

    vfs_errno = 0;

    if((update = file->status.update))
        fs = (vfs_t *)file->fs;

    ((vfs_t *)(file->fs))->fclose(file);

    if(update && vfs.on_fs_changed)
        vfs.on_fs_changed(fs);
}

FLASHMEM size_t vfs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fread(buffer, size, count, file);
}

FLASHMEM size_t vfs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fwrite(buffer, size, count, file);
}

FLASHMEM int vfs_puts (const char *s, vfs_file_t *file)
{
    size_t count = strlen(s), ret;

    vfs_errno = 0;

    if((ret = ((vfs_t *)(file->fs))->fwrite(s, sizeof(char), count, file)) != count)
        return -1;

    return (int)ret;
}

FLASHMEM size_t vfs_tell (vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->ftell(file);
}

FLASHMEM int vfs_seek (vfs_file_t *file, size_t offset)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->fseek(file, offset);
}

FLASHMEM int vfs_truncate (vfs_file_t *file, size_t offset)
{
    vfs_errno = ((vfs_t *)(file->fs))->ftruncate ? 0 : EPERM;

    return vfs_errno ? -1 : ((vfs_t *)(file->fs))->ftruncate(file, offset);
}

FLASHMEM bool vfs_eof (vfs_file_t *file)
{
    vfs_errno = 0;

    return ((vfs_t *)(file->fs))->feof(file);
}

FLASHMEM int vfs_rename (const char *from, const char *to)
{
    int ret;

    vfs_mount_t *mount_from = get_mount(from), *mount_to = get_mount(to);

    if((ret = mount_from == mount_to ? mount_from->vfs->frename(get_filename(mount_from, from), get_filename(mount_to, to)) : -1) != -1 &&
               vfs.on_fs_changed && !mount_from->mode.hidden) {
        vfs.on_fs_changed(mount_from->vfs);
        if(mount_from != mount_to && !mount_to->mode.hidden)
            vfs.on_fs_changed(mount_to->vfs);
    }

    return ret;
}

FLASHMEM int vfs_unlink (const char *filename)
{
    int ret = -1;
    vfs_mount_t *mount; // TODO: test for dir?

    if((filename = parse_path(filename)) && (mount = mount_modifiable(filename, offsetof(vfs_t, funlink)))) {
        if((ret = mount->vfs->funlink(get_filename(mount, filename))) != -1 && vfs.on_fs_changed)
            vfs.on_fs_changed(mount->vfs);
    }

    return ret;
}

FLASHMEM int vfs_mkdir (const char *path)
{
    int ret = -1;
    vfs_mount_t *mount;

    if((path = parse_path(path)) && (mount = mount_modifiable(path, offsetof(vfs_t, fmkdir)))) {
        if((ret = mount->vfs->fmkdir(get_filename(mount, path))) != -1 && vfs.on_fs_changed)
            vfs.on_fs_changed(mount->vfs);
    }

    return ret;
}

FLASHMEM int vfs_rmdir (const char *path)
{
    int ret = -1;
    vfs_mount_t *mount;

    if((path = parse_path(path)) && (mount = mount_modifiable(path, offsetof(vfs_t, frmdir)))) {

        vfs_stat_t st;

        if(_vfs_stat(mount, path, &st) == 0 && !st.st_mode.directory)
            vfs_errno = ENOTDIR;
        else if((ret = mount->vfs->frmdir(get_filename(mount, path))) != -1 && vfs.on_fs_changed)
            vfs.on_fs_changed(mount->vfs);
    }

    return ret;
}

FLASHMEM int vfs_chdir (const char *path)
{
    char *cdir = NULL;
    vfs_mount_t *mount;
    string_t tmp;

    if((vfs_errno = (tmp.name = malloc((tmp.len = strlen(cwd.name) + 1))) && (path = parse_path(path)) ? 0 : ENOMEM))
        return -1;

    if((mount = get_mount(path))) {
        vfs_stat_t st;
        strcpy(tmp.name, cwd.name);
        if(mount->mode.hidden || (mount == cwdmount && !mount->vfs->fchdir && strcmp(path, cwd.name)))
            vfs_errno = EPERM;
        else if(!(mount->vfs->fstat(get_filename(mount, path), &st) == 0 && st.st_mode.directory))
            vfs_errno = ENOTDIR;
    } else
        vfs_errno = ENOTDIR;

    if(vfs_errno == 0 && mount->vfs->fchdir) {
        if(mount == cwdmount)
            cdir = mount->vfs->fgetcwd(NULL, 0);
        if(mount->vfs->fchdir(get_filename(mount, path))) {
            vfs_errno = EAGAIN;
            if(cdir && mount->vfs->fchdir(cdir)) {
                cwdmount = &root;
                strcpy(cwd.name, (path = root.path));
                vfs_errno = EIO;
            } else
                vfs_errno = EAGAIN;
        }
    }

    if(vfs_errno == 0) {
        if(strset_(&cwd, strlen(path) + 1, path)) {
            if(cwdmount != mount && cwdmount->vfs->fchdir)
                cwdmount->vfs->fchdir("/");
            cwdmount = mount;
        } else {
            if(cdir && mount->vfs->fchdir(cdir)) {
                cwdmount = &root;
                strcpy(cwd.name, (path = root.path));
                vfs_errno = EIO;
            } else {
                cwd.len = tmp.len;
                cwd.name = tmp.name;
                vfs_errno = ENOMEM;
            }
        }
    }

    if(tmp.name != cwd.name)
        free(tmp.name);

    return vfs_errno ? -1 : 0;
}

FLASHMEM vfs_dir_t *vfs_opendir (const char *path)
{
    vfs_dir_t *dir = NULL;
    vfs_mount_t *mount = get_mount(path), *add_mount;
    vfs_mount_ll_entry_t *ml = NULL, *mln;

    if(mount && (dir = mount->vfs->fopendir(get_filename(mount, path)))) {

        dir->fs = mount->vfs;
        dir->mounts = NULL;

        if((add_mount = root.next)) do {
            if(add_mount != mount && !strncmp(add_mount->path, path, strlen(path))) {
                if(!add_mount->mode.hidden && (mln = malloc(sizeof(vfs_mount_ll_entry_t)))) {
                    mln->mount = add_mount;
                    mln->next = NULL;
                    if(dir->mounts == NULL)
                        dir->mounts = ml = mln;
                    else {
                        ml->next = mln;
                        ml = mln;
                    }
                }
            }
        } while((add_mount = add_mount->next));
    }

    return dir;
}

FLASHMEM vfs_dirent_t *vfs_readdir (vfs_dir_t *dir)
{
    static vfs_dirent_t dirent;

    vfs_errno = 0;

    ((vfs_t *)dir->fs)->readdir(dir, &dirent);

    if(*dirent.name == '\0' && dir->mounts) {

        char *s;
        vfs_mount_ll_entry_t *ml = dir->mounts;

        strcpy(dirent.name, ml->mount->path + 1);
        if((s = strrchr(dirent.name, '/')))
            *s = '\0';

        dirent.st_mode = ml->mount->mode;
        dirent.st_mode.directory = true;
        dir->mounts = ml->next;
        free(ml);
    }

    return *dirent.name == '\0' ? NULL : &dirent;
}

FLASHMEM void vfs_closedir (vfs_dir_t *dir)
{
    vfs_errno = 0;

    while(dir->mounts) {
        vfs_mount_ll_entry_t *ml = dir->mounts;
        dir->mounts = ml->next;
        free(ml);
    }

    ((vfs_t *)dir->fs)->fclosedir(dir);
}

FLASHMEM char *vfs_getcwd (char *buf, size_t len)
{
    char *cwds = cwdmount->vfs->fgetcwd ? cwdmount->vfs->fgetcwd(NULL, len) : "/", *cwd;
    size_t cwdlen;

    vfs_errno = 0;

    if((cwd = (char *)malloc(strlen(cwdmount->path) + strlen(cwds) + 3))) {
        strcat(strcpy(cwd, cwdmount->path), cwds + 1);
        vfs_fixpath(cwd);
        cwdlen = strlen(cwd);
        if(buf) {
            if(cwdlen < len)
                strcpy(buf, cwd);
            else {
                buf = NULL;
                vfs_errno = ERANGE;
            }
            free(cwd);
        } else if(len == 0 || cwdlen < len)
            buf = cwd;
        else {
            free(cwd);
            vfs_errno = ERANGE;
        }
    } else {
        buf = NULL;
        vfs_errno = ENOMEM;
    }

    return buf;
}

FLASHMEM int vfs_chmod (const char *filename, vfs_st_mode_t attr, vfs_st_mode_t mask)
{
    vfs_mount_t *mount = get_mount(filename);

    return mount && mount->vfs->fchmod ? mount->vfs->fchmod(get_filename(mount, filename), attr, mask) : -1;
}

FLASHMEM static int _vfs_stat (vfs_mount_t *mount, const char *filename, vfs_stat_t *st)
{
    return mount ? mount->vfs->fstat(get_filename(mount, filename), st) : -1;
}

FLASHMEM int vfs_stat (const char *filename, vfs_stat_t *st)
{
    return (filename = parse_path(filename)) ? _vfs_stat(get_mount(filename), filename, st) : -1;
}

FLASHMEM int vfs_utime (const char *filename, struct tm *modified)
{
    vfs_mount_t *mount = get_mount((filename = parse_path(filename)));

    return mount && mount->vfs->futime ? mount->vfs->futime(get_filename(mount, filename), modified) : -1;
}

FLASHMEM vfs_free_t *vfs_fgetfree (const char *path)
{
    static vfs_free_t free;

    vfs_mount_t *mount = get_mount(path);

    if(mount && mount->vfs->fgetfree && mount->vfs->fgetfree(&free))
        return &free;

    return NULL;
}

FLASHMEM static bool vfs_get_time (struct tm *time)
{
    memset(time, 0, sizeof(struct tm));

 // 2026-01-01:00:00:00
    time->tm_year = 2026 - 1900;
    time->tm_mday = 1;

    return true;
}

FLASHMEM bool vfs_mount (const void *device, const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    vfs_mount_t *mount;

    if(fs->fchdir && !fs->fgetcwd)
        return false;

    if((mount = path_is_mount_dir(path)) && mount->vfs != &fs_null)
        return mount->vfs == fs;

    if(!strcmp(path, "/")) {
        root.vfs = fs;
        root.mode = mode;
        root.device = device;
        fsmounted.root = On;
    } else if((mount = (vfs_mount_t *)calloc(1, sizeof(vfs_mount_t)))) {

        struct tm tm;

        strcpy(mount->path, path);
        if(mount->path[strlen(path) - 1] != '/')
            strcat(mount->path, "/");

        mount->vfs = fs;
        mount->mode = mode;
        mount->next = NULL;
        mount->device = device;
        if(hal.rtc.get_datetime && hal.rtc.get_datetime(&tm)) {
#ifdef ESP_PLATFORM
            mount->st_mtim = mktime(&tm);
#else
            mount->st_mtime = mktime(&tm);
#endif
        }

        if(!strcmp(path, "/embedded"))
            fsmounted.embedded = On;
        else if(!strcmp(path, "/littlefs"))
            fsmounted.littlefs = On;

        vfs_mount_t *lmount = &root;

        while(lmount->next)
            lmount = lmount->next;

        lmount->next = mount;
    } else
        fs = NULL;

    if(hal.rtc.get_datetime == NULL)
        hal.rtc.get_datetime = vfs_get_time;

    if(fs && vfs.on_mount)
        vfs.on_mount(path, fs, mode);

    return fs != NULL;
}

FLASHMEM bool vfs_unmount (const void *device, const char *path)
{
    // TODO: close open files?

    vfs_mount_t *mount;

    if((mount = get_mount(path)) && mount->vfs != &fs_null) {

        if(vfs.on_unmount)
            vfs.on_unmount(path);

        if(mount->vfs == root.vfs)
            mount_fs_null();
        else {

            if(mount == cwdmount) {
                cwdmount = &root;
                strcpy(cwd.name, "/");
            }

            vfs_mount_t *pmount = &root;

            while(pmount->next && pmount->next != mount)
                pmount = pmount->next;

            if(pmount->next == mount)
                pmount->next = mount->next;

            free(mount);
        }
    }

    return !!mount;
}

FLASHMEM vfs_drive_t *vfs_get_drive (const char *path)
{
    static vfs_drive_t drive;
    static char mpath[VFS_MOUNT_PATH_LEN];

    vfs_mount_t *mount;

    if((mount = get_mount(path))) {
        strcpy(mpath, mount->path);
        drive.name = mount->vfs->fs_name;
        drive.path = mpath;
        drive.mode = mount->mode;
        drive.removable = mount->vfs->removable;
        drive.fs = mount->vfs;
    } else
        memset(&drive, 0, sizeof(vfs_drive_t));

    return &drive; // TODO: return NULL if not mounted?
}

FLASHMEM vfs_drives_t *vfs_drives_open (void)
{
    vfs_drives_t *handle;
    vfs_mount_t *mount = &root;

    if((handle = malloc(sizeof(vfs_drives_t)))) {

        handle->mount = NULL;
        do {
            if(mount->mode.hidden)
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

FLASHMEM vfs_drive_t *vfs_drives_read (vfs_drives_t *handle, bool add_hidden)
{
    static vfs_drive_t drive;
    static char path[VFS_MOUNT_PATH_LEN];

    bool ok;

    if((ok = handle->mount != NULL)) {

        strcpy(path, handle->mount->path);
        drive.name = handle->mount->vfs->fs_name;
        drive.path = path;
        drive.mode = handle->mount->mode;
        drive.removable = handle->mount->vfs->removable;
        drive.fs = handle->mount->vfs;

        handle->mount = handle->mount->next;

        if(handle->mount) do {
            if((!handle->mount->mode.hidden || add_hidden) && handle->mount->vfs->fs_name)
                break;
        } while((handle->mount = handle->mount->next));
    }

    return ok ? &drive : NULL;
}

FLASHMEM void vfs_drives_close (vfs_drives_t *handle)
{
    free(handle);
}

FLASHMEM vfs_free_t *vfs_drive_getfree (vfs_drive_t *drive)
{
    static vfs_free_t free;

    const vfs_t *fs = drive->fs;

    return fs->fgetfree && fs->fgetfree(&free) ? &free : NULL;
}

FLASHMEM int vfs_drive_format (vfs_drive_t *drive)
{
    const vfs_t *fs = drive->fs;
    vfs_mount_t *mount = path_is_mount_dir(drive->path);

    stream_await_tx_clear(&hal.stream);

    if(mount && fs->format) {

        int ferrno;

        if(mount == cwdmount) {
            strcpy(cwd.name, drive->path);
            vfs_fixpath(cwd.name);
        }

        if(vfs.on_unmount)
            vfs.on_unmount(drive->path);

        if((ferrno = fs->format()) == 0) {
            if(fs->device_mount && !fs->device_mount(mount->device, true))
                ferrno = -1;
            if(ferrno == 0 && vfs.on_mount)
                vfs.on_mount(drive->path, drive->fs, drive->mode);
        }

        if((vfs_errno = ferrno) && fs->device_mount) {
            if(mount == cwdmount) {
                if(mount->vfs == root.vfs)
                    mount_fs_null();
                else {
                    cwdmount = &root;
                    strcpy(cwd.name, root.path);
                }
            }
            fs->device_mount(mount->device, false); // dismount drive on error
        }
    } else
        vfs_errno = -1;

    return vfs_errno;
}

FLASHMEM bool vfs_mount_set_mode (const char *path, vfs_st_mode_t mode)
{
    vfs_mount_t *mount;
    
    if((mount = get_mount(path)) && !strncmp(path, mount->path, strlen(mount->path) - 1))
        mount->mode = mode;

    return !!mount;
}

FLASHMEM const char *vfs_locate_file (const char *filename)
{
    static char _name[32];
    static string_t pathb = { .name = _name, .len = sizeof(_name) - 1 }, path = { .name = _name, .len = sizeof(_name) - 1, .base = &pathb };

    vfs_stat_t st;

    if(*filename == '/')
        return fsmounted.root && vfs_stat(filename, &st) == 0 ? filename : NULL;

    if(strset_(&path, strlen(filename) + 11, NULL)) {

        if(fsmounted.root) {
            strcat(strcpy(path.name, "/"), filename);
            if(vfs_stat(path.name, &st) != 0)
                *path.name = '\0';
        }

        if(!*path.name && fsmounted.littlefs) {
            strcat(strcpy(path.name, "/littlefs/"), filename);
            if(vfs_stat(path.name, &st) != 0)
                *path.name = '\0';
        }

        if(!*path.name && fsmounted.embedded) {
            strcat(strcpy(path.name, "/embedded/"), filename);
            if(vfs_stat(path.name, &st) != 0)
                *path.name = '\0';
        }
    }

    return path.name && *path.name ? (const char *)path.name : NULL;
}
