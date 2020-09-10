//
//  serial.c
//  serial
//
//  Created by Albin Stigö on 2019-11-10.
//  Copyright © 2019 Albin Stigo. All rights reserved.
//

#include <erl_nif.h>

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <errno.h>

#include "serial.h"

/* handy macros */
#define TUPLE_ATOM_STRING(atom, reason) enif_make_tuple2(env, atom, enif_make_string(env, reason, ERL_NIF_LATIN1));

#define TUPLE_ATOM_ATOM(atom0, atom1) enif_make_tuple2(env, atom0, atom1);

#define RW_MAX 1024

#define PARITY_NONE 0
#define PARITY_ODD  1
#define PARITY_EVEN 2

/* serial_t */
typedef struct {
    int fd;
    uint8_t buffer[RW_MAX];
    char path[PATH_MAX];
} serial_t;

typedef struct {
    ERL_NIF_TERM atom_ok;
    ERL_NIF_TERM atom_undefined;
    ERL_NIF_TERM atom_error;
    ERL_NIF_TERM atom_none;
    ERL_NIF_TERM atom_odd;
    ERL_NIF_TERM atom_even;
    ERL_NIF_TERM atom_ewouldblock;
} serial_priv_t;

/* select callbacks */
static void serial_rt_destructor(ErlNifEnv *env, void *obj) {
    printf("serial_rt_destructor\n");
}

static void serial_rt_stop(ErlNifEnv *env, void *obj, int fd, int is_direct_call) {
    serial_t *serial = (serial_t *)obj;
    printf("serial_rt_stop %d\n", fd);
    // TODO: maybe schedule call?
    close(fd);
}

static void serial_rt_down(ErlNifEnv* env, void* obj, ErlNifPid* pid, ErlNifMonitor* serial_mon) {
    printf("serial_rt_down\n");
    serial_t *serial = (serial_t *)obj;
    ERL_NIF_TERM undefined;
    enif_make_existing_atom(env, "undefined", &undefined, ERL_NIF_LATIN1);
    enif_select(env, serial->fd, ERL_NIF_SELECT_STOP, serial, NULL, undefined);
}

static ErlNifResourceType *serial_rt;
static ErlNifResourceTypeInit serial_rt_init = {serial_rt_destructor, serial_rt_stop, serial_rt_down};

/* Serial helpers */
static int set_interface_attribs(int fd, int speed, int parity, int databits, int stopbits)
{
    int ret;
    struct termios tty;
    
    ret = tcgetattr(fd, &tty);
    if (ret < 0) {
        // perror("tcgetattr");
        return ret;
    }
    
    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);
    
    // ignore modem controls
    tty.c_cflag |= (CLOCAL | CREAD);
    
    // Databits 8/7/6/5
    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            errno = EINVAL;
            return -1;
    }
    // Parity none/odd/event
    switch (parity) {
        case PARITY_NONE:
            tty.c_cflag &= ~PARENB; // no parity bit
            break;
        case PARITY_ODD:
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD; // odd parity, else even
            break;
        case PARITY_EVEN:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD; // odd parity, else even
            break;
        default:
            errno = EINVAL;
            return -1;
    }
    // Stopbits 1/2
    switch (stopbits) {
        case 1: tty.c_cflag &= ~CSTOPB; break;
        case 2: tty.c_cflag |= CSTOPB; break;
        default:
            errno = EINVAL;
            return -1;
    }
    
    // No hardware flowcontrol
    tty.c_cflag &= ~CRTSCTS;
    
    // Setup for non-canonical mode
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    
    // Fetch bytes as they become available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    ret = tcsetattr(fd, TCSANOW, &tty);
    if (ret != 0) {
        // perror("tcsetattr");
        return ret;
    }
    
    return 0;
}

static int set_mincount(int fd, int mcount)
{
    int ret;
    struct termios tty;
    
    ret = tcgetattr(fd, &tty);
    if (ret < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return ret;
    }
    
    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */
    
    ret = tcsetattr(fd, TCSANOW, &tty);
    if (ret < 0) {
        printf("Error tcsetattr: %s\n", strerror(errno));
    }
    
    return 0;
}

static ERL_NIF_TERM open_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    int ret;
    ERL_NIF_TERM res;
    serial_priv_t* priv = enif_priv_data(env);
    ErlNifPid controlling_process;
    serial_t *serial;
    
    char path[PATH_MAX];
    int baudrate;
    int parity;
    int databits;
    int stopbits;
    
    // Path
    if(!enif_get_string(env, argv[0], path, sizeof(path), ERL_NIF_LATIN1)) {
        return enif_make_badarg(env);
    }
    
    // Baud rate
    if(!enif_get_int(env, argv[1], &baudrate)) {
        return enif_make_badarg(env);
    }
    
    // Parity
    if(enif_compare(argv[2], priv->atom_none) == 0) {
        parity = PARITY_NONE;
    } else if (enif_compare(argv[2], priv->atom_odd) == 0) {
        parity = PARITY_ODD;
    } else if (enif_compare(argv[2], priv->atom_even) == 0) {
        parity = PARITY_EVEN;
    } else {
        return enif_make_badarg(env);
    }
    
    // Data bits
    if(!enif_get_int(env, argv[3], &databits)) {
        return enif_make_badarg(env);
    }
    
    // Stop bits
    if(!enif_get_int(env, argv[4], &stopbits)) {
        return enif_make_badarg(env);
    }
    
    // Allocate resource
    serial = enif_alloc_resource(serial_rt, sizeof(serial_t));
    
    //serial->fd = open(path, O_RDWR | O_NOCTTY |  | O_NONBLOCK);
    serial->fd = open(path, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (serial->fd < 0) {
        enif_release_resource((void*) serial);
        return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
    }
    strncpy(serial->path, path, PATH_MAX);
    
    ret = set_interface_attribs(serial->fd, baudrate, parity, databits, stopbits);
    if (ret < 0) {
        enif_release_resource((void*) serial);
        return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
    }
    
    /*ret = set_mincount(serial->fd, 1);
     if (ret < 0) {
     close(serial->fd);
     return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
     }*/
    
    /* Starts monitoring a process from a resource.
     When a process is monitored, a process exit
     results in a call to the provided down callback
     associated with the resource type.*/
    enif_self(env, &controlling_process);
    enif_monitor_process(env, serial, &controlling_process, NULL);
    
    res = enif_make_resource(env, serial);
    // Erlang will handle this resource
    enif_release_resource((void*) serial);

    return enif_make_tuple2(env,
                            priv->atom_ok,
                            res);
}

static ERL_NIF_TERM poll_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    int ret;
    serial_t *serial;
    serial_priv_t* priv = enif_priv_data(env);
    
    if(!enif_get_resource(env, argv[0], serial_rt, (void **)&serial)) {
        return enif_make_badarg(env);
    }
    if (serial->fd < 0) {
        return enif_make_badarg(env);
    }
    
    // this is a one shot
    ret = enif_select(env,
                      serial->fd,
                      ERL_NIF_SELECT_READ,
                      serial,
                      NULL,
                      priv->atom_undefined);
    if (ret < 0) {
        close(serial->fd);
        return TUPLE_ATOM_STRING(priv->atom_error, "enif_select");
    }
    
    return priv->atom_ok;
}

static ERL_NIF_TERM close_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    int ret;
    serial_t *serial;
    serial_priv_t* priv = enif_priv_data(env);
    
    if(!enif_get_resource(env, argv[0], serial_rt, (void **)&serial)) {
        return enif_make_badarg(env);
    }
    
    // This will also close the fd
    ret = enif_select(env, serial->fd, ERL_NIF_SELECT_STOP, serial, NULL, priv->atom_undefined);
    if (ret < 0) {
        return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
    }
    
    printf("close!\n");
    
    return priv->atom_ok;
}

static ERL_NIF_TERM read_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    serial_t *serial;
    ssize_t n;
    serial_priv_t* priv = enif_priv_data(env);
    
    if(!enif_get_resource(env, argv[0], serial_rt, (void **)&serial)) {
        return enif_make_badarg(env);
    }
    
    n = read(serial->fd, serial->buffer, RW_MAX);
    if (n < 0) {
        printf("ERROR %d\n", errno);
        switch (errno) {
            case EWOULDBLOCK:
                return TUPLE_ATOM_ATOM(priv->atom_error, priv->atom_ewouldblock);
            default:
                return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
        }
    }
    
    // Move read data to binary
    ERL_NIF_TERM binary;
    unsigned char* bin_buffer = enif_make_new_binary(env, n, &binary);
    memcpy(bin_buffer, serial->buffer, n);
    
    return enif_make_tuple2(env, priv->atom_ok, binary);
}

static ERL_NIF_TERM write_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    serial_t *serial;
    ssize_t n;
    serial_priv_t* priv = enif_priv_data(env);
    ErlNifBinary bin;
    
    if(!enif_get_resource(env, argv[0], serial_rt, (void **)&serial)) {
        return enif_make_badarg(env);
    }
    
    if(!enif_inspect_iolist_as_binary(env, argv[1], &bin)) {
        return enif_make_badarg(env);
    }
    // TODO: have to think about this
    if (bin.size > RW_MAX) {
        return enif_make_badarg(env);
    }
    
    n = write(serial->fd, bin.data, bin.size);
    if (n < 0) {
        switch (errno) {
            case EWOULDBLOCK:
                return TUPLE_ATOM_ATOM(priv->atom_error, priv->atom_ewouldblock);
            default:
                return TUPLE_ATOM_STRING(priv->atom_error, strerror(errno));
        }
    }
    
    return priv->atom_ok;
}

static int on_load(ErlNifEnv* env, void** priv_data, ERL_NIF_TERM load_info) {
    printf("on load\n");

    serial_priv_t* data = enif_alloc(sizeof(serial_priv_t));
    if (data == NULL) {
        return -1;
    }
    
    // Some predefined atoms
    data->atom_ok = enif_make_atom(env, "ok");
    data->atom_error = enif_make_atom(env, "error");
    data->atom_undefined = enif_make_atom(env, "undefined");
    data->atom_none = enif_make_atom(env, "none");
    data->atom_odd = enif_make_atom(env, "odd");
    data->atom_even = enif_make_atom(env, "even");
    data->atom_ewouldblock = enif_make_atom(env, "ewouldblock");
    
    // Set priv_data pointer to point to this
    *priv_data = (void*) data;
    
    // Resource type for select
    serial_rt = enif_open_resource_type_x(env,
                                          "serial",
                                          &serial_rt_init,
                                          ERL_NIF_RT_CREATE | ERL_NIF_RT_TAKEOVER,
                                          NULL);
    if (serial_rt == NULL) {
        return -1;
    }
    
    return 0;
}

static int on_upgrade(ErlNifEnv* env, void** priv_data, void** old_priv_data, ERL_NIF_TERM load_info) {
    printf("on upgrade\n");
    return on_load(env, priv_data, load_info);
}

static void on_unload(ErlNifEnv* env, void* priv_data) {
    printf("onload\n");
    enif_free(priv_data);
}

static ErlNifFunc nif_funcs[] = {
    {"open", 5, open_nif},
    {"poll", 1, poll_nif},
    {"close", 1, close_nif},
    {"read", 1, read_nif},
    {"write", 2, write_nif}
};

// name, funcs, load, (deprecated )reload, upgrade, unload
ERL_NIF_INIT(serial, nif_funcs, &on_load, NULL, &on_upgrade, &on_unload)
